// motor_node_continuous: MIT commands for continuous (delta position) firmware.
// Position field MSB (bit 15): 0 = drive (apply delta), 1 = hold. Lower 15 bits encode
// position change magnitude. Sends every motor_command message (no deduplication).

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

#include "rclcpp/rclcpp.hpp"
#include "motor_interfaces/msg/motor_command.hpp"

namespace
{

constexpr float kPMin = -12.5f;
constexpr float kPMax = 12.5f;
constexpr float kVMin = -50.0f;
constexpr float kVMax = 50.0f;
constexpr float kTMin = -25.0f;
constexpr float kTMax = 25.0f;
constexpr float kKpMin = 0.0f;
constexpr float kKpMax = 500.0f;
constexpr float kKdMin = 0.0f;
constexpr float kKdMax = 5.0f;

constexpr int kDriveId = 0;

// Bit 15 of the 16-bit position field: 0 = drive (apply delta), 1 = hold
constexpr int kPositionHoldBit = 0x8000;

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

// Pack position for continuous firmware: MSB 0 = drive, MSB 1 = hold; bits 14:0 = delta.
int pack_position_continuous(float p_delta)
{
  if (p_delta == 0.0f) {
    return kPositionHoldBit;
  }
  return float_to_uint(p_delta, kPMin, kPMax, 15) & 0x7FFF;
}

struct Ak70MitFeedback
{
  int drive_id{0};
  uint32_t can_id{0};
  float position_rad{0.0f};
  float velocity_rad_s{0.0f};
  float torque_nm{0.0f};
  float temperature_c{0.0f};
  int error_code{0};

  bool unpack_reply(const struct can_frame & frame, int expected_drive_id)
  {
    if (frame.can_dlc < 7) {
      return false;
    }

    can_id = frame.can_id & CAN_SFF_MASK;
    drive_id = frame.data[0];

    if (static_cast<int>(can_id) != expected_drive_id ||
        drive_id != expected_drive_id)
    {
      return false;
    }

    const int p_int = (frame.data[1] << 8) | frame.data[2];
    const int v_int = (frame.data[3] << 4) | (frame.data[4] >> 4);
    const int i_int = ((frame.data[4] & 0x0F) << 8) | frame.data[5];
    const int t_int = frame.data[6];

    position_rad = uint_to_float(p_int, kPMin, kPMax, 16);
    velocity_rad_s = uint_to_float(v_int, kVMin, kVMax, 12);
    torque_nm = uint_to_float(i_int, kTMin, kTMax, 12);
    temperature_c = static_cast<float>(t_int) - 40.0f;
    error_code = frame.can_dlc >= 8 ? static_cast<int>(frame.data[7]) : 0;

    return true;
  }
};

}  // namespace

class MotorNodeContinuous : public rclcpp::Node
{
public:
  MotorNodeContinuous() : Node("motor_node_continuous")
  {
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create CAN socket.");
      return;
    }

    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, "can0", IFNAMSIZ - 1);

    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
      RCLCPP_ERROR(this->get_logger(), "can0 not found.");
      close(can_socket_);
      can_socket_ = -1;
      return;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(can_socket_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Bind failed.");
      close(can_socket_);
      can_socket_ = -1;
      return;
    }

    subscription_ = this->create_subscription<motor_interfaces::msg::MotorCommand>(
      "motor_command",
      10,
      std::bind(&MotorNodeContinuous::motor_command_callback, this, std::placeholders::_1));

    enable_motor();
  }

  ~MotorNodeContinuous() override
  {
    disable_motor();
    if (can_socket_ >= 0) {
      close(can_socket_);
    }
  }

private:
  void enable_motor()
  {
    if (can_socket_ < 0) {
      return;
    }

    struct can_frame frame{};
    frame.can_id = kDriveId;
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
    frame.can_id = kDriveId;
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

  void send_mit_command(float p_delta, float v_des, float kp, float kd, float t_ff)
  {
    const int p_int = pack_position_continuous(p_delta);
    const int v_int = float_to_uint(v_des, kVMin, kVMax, 12);
    const int kp_int = float_to_uint(kp, kKpMin, kKpMax, 12);
    const int kd_int = float_to_uint(kd, kKdMin, kKdMax, 12);
    const int t_int = float_to_uint(t_ff, kTMin, kTMax, 12);

    struct can_frame frame{};
    frame.can_id = kDriveId;
    frame.can_dlc = 8;

    frame.data[0] = p_int >> 8;
    frame.data[1] = p_int & 0xFF;
    frame.data[2] = v_int >> 4;
    frame.data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
    frame.data[4] = kp_int & 0xFF;
    frame.data[5] = kd_int >> 4;
    frame.data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
    frame.data[7] = t_int & 0xFF;

    if (write(can_socket_, &frame, sizeof(frame)) != static_cast<ssize_t>(sizeof(frame))) {
      RCLCPP_ERROR(this->get_logger(), "Failed to send MIT command.");
      return;
    }

    const int hold_bit = (p_int & kPositionHoldBit) ? 1 : 0;
    RCLCPP_INFO(
      this->get_logger(),
      "Sent MIT cmd: dP=%.3f hold=%d p_packed=0x%04X V=%.3f KP=%.3f KD=%.3f T=%.3f",
      p_delta, hold_bit, p_int & 0xFFFF, v_des, kp, kd, t_ff);

    read_mit_feedback();
  }

  void read_mit_feedback()
  {
    constexpr int kPollTimeoutMs = 50;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(kPollTimeoutMs);

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
        RCLCPP_WARN(this->get_logger(), "poll() while waiting for MIT feedback: %s",
          std::strerror(errno));
        return;
      }
      if (poll_result == 0) {
        break;
      }

      struct can_frame frame{};
      const ssize_t nbytes = read(can_socket_, &frame, sizeof(frame));
      if (nbytes < 0) {
        RCLCPP_WARN(this->get_logger(), "read() while waiting for MIT feedback: %s",
          std::strerror(errno));
        return;
      }
      if (nbytes != static_cast<ssize_t>(sizeof(frame))) {
        continue;
      }

      Ak70MitFeedback fb;
      if (fb.unpack_reply(frame, kDriveId)) {
        log_mit_feedback(fb);
        return;
      }
    }

    RCLCPP_WARN(this->get_logger(), "No MIT feedback received within %d ms.", kPollTimeoutMs);
  }

  void log_mit_feedback(const Ak70MitFeedback & fb)
  {
    const double position_deg = fb.position_rad * 180.0 / M_PI;

    std::ostringstream out;
    out << std::fixed << std::setprecision(4);
    out << "\n========== AK70-10 MIT Feedback ==========\n";
    out << "Drive ID:      " << fb.drive_id << '\n';
    out << "Position:      " << fb.position_rad << " rad  (" << position_deg << " deg)\n";
    out << "Velocity:      " << fb.velocity_rad_s << " rad/s\n";
    out << "Torque:        " << fb.torque_nm << " Nm\n";
    out << "Temperature:   " << fb.temperature_c << " C\n";
    out << "Error code:    " << fb.error_code << " (" << mit_error_string(fb.error_code) << ")\n";
    out << "==========================================";

    RCLCPP_INFO(this->get_logger(), "%s", out.str().c_str());
  }

  void motor_command_callback(const motor_interfaces::msg::MotorCommand::SharedPtr msg)
  {
    send_mit_command(msg->position, msg->velocity, msg->kp, msg->kd, msg->torque);
  }

  rclcpp::Subscription<motor_interfaces::msg::MotorCommand>::SharedPtr subscription_;
  int can_socket_{-1};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorNodeContinuous>());
  rclcpp::shutdown();
  return 0;
}
