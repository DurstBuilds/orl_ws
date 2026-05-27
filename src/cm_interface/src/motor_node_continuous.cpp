// motor_node_continuous: MIT commands for continuous (delta position) firmware.
// Position field bit 15: 0 = hold, 1 = apply new command. Lower 15 bits encode delta.
// Hold when p_delta, v_des, and t_ff are all zero. Sends every motor_command (no dedup).

#include <chrono>
#include <cmath>
#include <cerrno>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <string>

#include <poll.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "cm_interface/motor_mit_profile.hpp"
#include "rclcpp/rclcpp.hpp"
#include "motor_interfaces/msg/motor_command.hpp"
#include "motor_interfaces/msg/motor_state.hpp"
#include "std_msgs/msg/bool.hpp"

namespace
{

constexpr int kDefaultCanId = 0;
constexpr float kCmdZeroEps = 1e-6f;

// Bit 15 of the 16-bit position field in the MIT command
constexpr int kPositionApplyBit = 0x8000;

const char * mit_error_string(int code)
{
  switch (code) {
    case 0: return "No fault";
    case 1: return "Motor over-temperature";
    case 2: return "Over-current";
    case 3: return "Over-voltage";
    case 4: return "Under-voltage";
    case 5: return "Encoder fault";
    case 6: return "MOSFET over-temperature";
    case 7: return "Motor stall";
    default: return "Unknown fault";
  }
}

int float_to_uint(float x, float x_min, float x_max, int bits)
{
  if (x < x_min) {
    x = x_min;
  }
  if (x > x_max) {
    x = x_max;
  }
  const float span = x_max - x_min;
  return static_cast<int>((x - x_min) * static_cast<float>((1 << bits) - 1) / span);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
  const float span = x_max - x_min;
  return static_cast<float>(x_int) * span / static_cast<float>((1 << bits) - 1) + x_min;
}

// SocketCAN arbitration ID (standard or extended, flags stripped).
int arbitration_id_from_frame(const struct can_frame & frame)
{
  const canid_t raw = frame.can_id;
  if (raw & CAN_EFF_FLAG) {
    return static_cast<int>(raw & CAN_EFF_MASK);
  }
  return static_cast<int>(raw & CAN_SFF_MASK);
}

// Motor drive ID encoded in MIT feedback DATA[0] (full byte or low nibble + ERR high nibble).
int motor_id_from_feedback_data(uint8_t data0, int expected_drive_id)
{
  const int id_low_nibble = static_cast<int>(data0 & 0x0F);
  if (id_low_nibble == expected_drive_id) {
    return id_low_nibble;
  }
  return static_cast<int>(data0);
}

// True when this frame is MIT feedback for expected_drive_id.
// Commands use the motor CAN ID; feedback may use the same ID (AK70-style) or the
// master ID (default 0) with the motor ID in DATA[0] (GL / AK gimbal-style manual §5.4).
bool feedback_frame_matches_drive(const struct can_frame & frame, int expected_drive_id)
{
  if (frame.can_dlc < 7) {
    return false;
  }

  const int arb_id = arbitration_id_from_frame(frame);
  const int motor_id = motor_id_from_feedback_data(frame.data[0], expected_drive_id);
  if (motor_id != expected_drive_id) {
    return false;
  }

  constexpr int kDefaultMasterCanId = 0;
  return arb_id == expected_drive_id || arb_id == kDefaultMasterCanId;
}

struct MitFeedback
{
  int drive_id{0};
  uint32_t can_id{0};
  float position_rad{0.0f};
  float velocity_rad_s{0.0f};
  float torque_nm{0.0f};
  float temperature_c{0.0f};
  int error_code{0};

  bool unpack_reply(
    const struct can_frame & frame,
    int expected_drive_id,
    const cm_interface::MotorMitProfile & profile)
  {
    if (!feedback_frame_matches_drive(frame, expected_drive_id)) {
      return false;
    }

    can_id = static_cast<uint32_t>(arbitration_id_from_frame(frame));
    drive_id = motor_id_from_feedback_data(frame.data[0], expected_drive_id);

    const int p_int = (frame.data[1] << 8) | frame.data[2];
    const int v_int = (frame.data[3] << 4) | (frame.data[4] >> 4);
    const int i_int = ((frame.data[4] & 0x0F) << 8) | frame.data[5];
    const int t_int = frame.data[6];

    position_rad = uint_to_float(p_int, profile.p_min, profile.p_max, 16);
    velocity_rad_s = uint_to_float(v_int, profile.v_min, profile.v_max, 12);
    torque_nm = uint_to_float(i_int, profile.t_min, profile.t_max, 12);
    temperature_c = static_cast<float>(t_int) - 40.0f;
    error_code = frame.can_dlc >= 8 ? static_cast<int>(frame.data[7]) : 0;

    return true;
  }
};

}  // namespace

class MotorNodeContinuous : public rclcpp::Node
{
public:
  MotorNodeContinuous()
  : Node("motor_node_continuous")
  {
    const std::string motor_model = declare_parameter<std::string>(
      "motor_model", "ak70_10");
    can_id_ = declare_parameter<int>("can_id", kDefaultCanId);
    if (!get_parameter("can_id", can_id_)) {
      throw std::runtime_error("Failed to read can_id parameter");
    }

    if (can_id_ < 0 || can_id_ > 0x7FF) {
      throw std::invalid_argument("can_id must be in [0, 2047] for standard CAN");
    }

    try {
      profile_ = cm_interface::get_motor_mit_profile(motor_model);
    } catch (const std::exception & e) {
      RCLCPP_FATAL(get_logger(), "%s", e.what());
      throw;
    }

    RCLCPP_INFO(
      get_logger(),
      "Motor model: %s | can_id: %d | position [%.3f, %.3f] rad | velocity [%.1f, %.1f] rad/s | "
      "torque [%.1f, %.1f] Nm",
      profile_.name, can_id_, profile_.p_min, profile_.p_max,
      profile_.v_min, profile_.v_max, profile_.t_min, profile_.t_max);

    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to create CAN socket.");
      return;
    }

    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, "can0", IFNAMSIZ - 1);

    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
      RCLCPP_ERROR(get_logger(), "can0 not found.");
      close(can_socket_);
      can_socket_ = -1;
      return;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(can_socket_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
      RCLCPP_ERROR(get_logger(), "Bind failed.");
      close(can_socket_);
      can_socket_ = -1;
      return;
    }

    subscription_ = create_subscription<motor_interfaces::msg::MotorCommand>(
      "motor_command",
      10,
      std::bind(&MotorNodeContinuous::motor_command_callback, this, std::placeholders::_1));

    soft_mode_sub_ = create_subscription<std_msgs::msg::Bool>(
      "soft_mode",
      10,
      std::bind(&MotorNodeContinuous::soft_mode_callback, this, std::placeholders::_1));

    state_publisher_ = create_publisher<motor_interfaces::msg::MotorState>("motor_state", 10);

    enable_motor();
    set_motor_origin();
  }

  ~MotorNodeContinuous() override
  {
    disable_motor();
    if (can_socket_ >= 0) {
      close(can_socket_);
    }
  }

private:
  int pack_position_continuous(float p_delta, float v_des, float t_ff, bool force_apply = false) const
  {
    const bool hold = std::fabs(p_delta) < kCmdZeroEps &&
      std::fabs(v_des) < kCmdZeroEps &&
      std::fabs(t_ff) < kCmdZeroEps;

    int p_int = float_to_uint(p_delta, profile_.p_min, profile_.p_max, 15) & 0x7FFF;
    if (force_apply || !hold) {
      p_int |= kPositionApplyBit;
    }
    return p_int;
  }

  void enable_motor()
  {
    if (can_socket_ < 0) {
      return;
    }

    struct can_frame frame{};
    frame.can_id = static_cast<canid_t>(can_id_);
    frame.can_dlc = 8;
    frame.data[0] = 0xFF;
    frame.data[1] = 0xFF;
    frame.data[2] = 0xFF;
    frame.data[3] = 0xFF;
    frame.data[4] = 0xFF;
    frame.data[5] = 0xFF;
    frame.data[6] = 0xFF;
    frame.data[7] = 0xFC;

    write(can_socket_, &frame, sizeof(frame));
  }

  void disable_motor()
  {
    if (can_socket_ < 0) {
      return;
    }

    struct can_frame frame{};
    frame.can_id = static_cast<canid_t>(can_id_);
    frame.can_dlc = 8;
    frame.data[0] = 0xFF;
    frame.data[1] = 0xFF;
    frame.data[2] = 0xFF;
    frame.data[3] = 0xFF;
    frame.data[4] = 0xFF;
    frame.data[5] = 0xFF;
    frame.data[6] = 0xFF;
    frame.data[7] = 0xFD;

    write(can_socket_, &frame, sizeof(frame));
  }

  void set_motor_origin()
  {
    if (can_socket_ < 0) {
      return;
    }

    struct can_frame frame{};
    frame.can_id = static_cast<canid_t>(can_id_);
    frame.can_dlc = 8;
    frame.data[0] = 0xFF;
    frame.data[1] = 0xFF;
    frame.data[2] = 0xFF;
    frame.data[3] = 0xFF;
    frame.data[4] = 0xFF;
    frame.data[5] = 0xFF;
    frame.data[6] = 0xFF;
    frame.data[7] = 0xFE;

    if (write(can_socket_, &frame, sizeof(frame)) == static_cast<ssize_t>(sizeof(frame))) {
      RCLCPP_INFO(get_logger(), "Set motor origin (current position = 0).");
      read_mit_feedback();
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to set motor origin.");
    }
  }

  void send_mit_command(
    float p_delta, float v_des, float kp, float kd, float t_ff, bool force_apply = false)
  {
    const int p_int = pack_position_continuous(p_delta, v_des, t_ff, force_apply);
    const int v_int = float_to_uint(v_des, profile_.v_min, profile_.v_max, 12);
    const int kp_int = float_to_uint(kp, profile_.kp_min, profile_.kp_max, 12);
    const int kd_int = float_to_uint(kd, profile_.kd_min, profile_.kd_max, 12);
    const int t_int = float_to_uint(t_ff, profile_.t_min, profile_.t_max, 12);

    struct can_frame frame{};
    frame.can_id = static_cast<canid_t>(can_id_);
    frame.can_dlc = 8;

    frame.data[0] = static_cast<uint8_t>(p_int >> 8);
    frame.data[1] = p_int & 0xFF;
    frame.data[2] = v_int >> 4;
    frame.data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
    frame.data[4] = kp_int & 0xFF;
    frame.data[5] = kd_int >> 4;
    frame.data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
    frame.data[7] = t_int & 0xFF;

    if (write(can_socket_, &frame, sizeof(frame)) != static_cast<ssize_t>(sizeof(frame))) {
      RCLCPP_ERROR(get_logger(), "Failed to send MIT command.");
      return;
    }

    const int apply_bit = (p_int & kPositionApplyBit) ? 1 : 0;
    RCLCPP_INFO(
      get_logger(),
      "Sent MIT cmd: dP=%.3f apply=%d p_packed=0x%04X V=%.3f KP=%.3f KD=%.3f T=%.3f",
      p_delta, apply_bit, p_int & 0xFFFF, v_des, kp, kd, t_ff);

    read_mit_feedback();
  }

  void read_mit_feedback()
  {
    constexpr int kPollTimeoutMs = 50;
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::milliseconds(kPollTimeoutMs);

    bool saw_any_frame = false;
    int last_arb_id = -1;
    uint8_t last_data0 = 0;

    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      const auto remaining_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - std::chrono::steady_clock::now()).count();

      if (remaining_ms <= 0) {
        break;
      }

      struct pollfd pfd{};
      pfd.fd = can_socket_;
      pfd.events = POLLIN;

      const int poll_result = poll(&pfd, 1, static_cast<int>(remaining_ms));
      if (poll_result < 0) {
        if (errno == EINTR) {
          continue;
        }
        RCLCPP_WARN(
          get_logger(), "poll() while waiting for MIT feedback: %s",
          std::strerror(errno));
        return;
      }
      if (poll_result == 0) {
        break;
      }

      struct can_frame frame{};
      const ssize_t nbytes = read(can_socket_, &frame, sizeof(frame));
      if (nbytes < 0) {
        RCLCPP_WARN(
          get_logger(), "read() while waiting for MIT feedback: %s",
          std::strerror(errno));
        return;
      }
      if (nbytes != static_cast<ssize_t>(sizeof(frame))) {
        continue;
      }

      saw_any_frame = true;
      last_arb_id = arbitration_id_from_frame(frame);
      last_data0 = frame.data[0];

      MitFeedback fb;
      if (fb.unpack_reply(frame, can_id_, profile_)) {
        publish_motor_state(fb);
        log_mit_feedback(fb);
        return;
      }
    }

    if (!saw_any_frame) {
      RCLCPP_WARN(
        get_logger(),
        "No MIT feedback within %d ms for can_id=%d (no CAN frames on bus).",
        kPollTimeoutMs, can_id_);
    } else {
      RCLCPP_WARN(
        get_logger(),
        "No MIT feedback within %d ms for can_id=%d. Last frame arb_id=%d data[0]=0x%02X "
        "(feedback may use master ID 0 with motor ID in data[0] low nibble).",
        kPollTimeoutMs, can_id_, last_arb_id, last_data0);
    }
  }

  void publish_motor_state(const MitFeedback & fb)
  {
    motor_interfaces::msg::MotorState msg;
    msg.position = fb.position_rad;
    msg.velocity = fb.velocity_rad_s;
    msg.torque = fb.torque_nm;
    msg.temperature = fb.temperature_c;
    msg.error_code = fb.error_code;
    msg.drive_id = static_cast<uint8_t>(fb.drive_id);
    state_publisher_->publish(msg);
  }

  void log_mit_feedback(const MitFeedback & fb)
  {
    const double position_deg = fb.position_rad * 180.0 / M_PI;

    std::ostringstream out;
    out << std::fixed << std::setprecision(4);
    out << "\n========== " << profile_.name << " MIT Feedback ==========\n";
    out << "Drive ID:      " << fb.drive_id << '\n';
    out << "Position:      " << fb.position_rad << " rad  (" << position_deg << " deg)\n";
    out << "Velocity:      " << fb.velocity_rad_s << " rad/s\n";
    out << "Torque:        " << fb.torque_nm << " Nm\n";
    out << "Temperature:   " << fb.temperature_c << " C\n";
    out << "Error code:    " << fb.error_code << " (" << mit_error_string(fb.error_code) << ")\n";
    out << "==========================================";

    RCLCPP_INFO(get_logger(), "%s", out.str().c_str());
  }

  void motor_command_callback(const motor_interfaces::msg::MotorCommand::SharedPtr msg)
  {
    if (soft_mode_) {
      return;
    }
    send_mit_command(msg->position, msg->velocity, msg->kp, msg->kd, msg->torque);
  }

  void soft_mode_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (soft_mode_ != msg->data) {
      soft_mode_ = msg->data;
      if (soft_mode_) {
        send_soft_mode_command();
      }
      RCLCPP_WARN(
        get_logger(),
        "soft_mode=%s; %s",
        soft_mode_ ? "true" : "false",
        soft_mode_ ? "ignoring motor_command and sending KD-max only" : "normal command flow resumed");
    }
  }

  void send_soft_mode_command()
  {
    // Force apply so the KD-only frame is latched even with zero delta/velocity/torque.
    send_mit_command(0.0f, 0.0f, 0.0f, profile_.kd_max, 0.0f, true);
  }

  cm_interface::MotorMitProfile profile_{cm_interface::kAk70_10};
  int can_id_{kDefaultCanId};
  rclcpp::Subscription<motor_interfaces::msg::MotorCommand>::SharedPtr subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr soft_mode_sub_;
  rclcpp::Publisher<motor_interfaces::msg::MotorState>::SharedPtr state_publisher_;
  int can_socket_{-1};
  bool soft_mode_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<MotorNodeContinuous>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("motor_node_continuous"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
