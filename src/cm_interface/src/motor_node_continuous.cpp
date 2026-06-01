// motor_node_continuous: MIT commands for continuous (delta position) firmware.
// Position field bit 15: 0 = hold, 1 = apply new command. Lower 15 bits encode delta.
// CAN RX runs on a dedicated thread with kernel filters; TX is rate-limited and uses the
// latest motor_command only (no per-callback blocking read or RX drain).

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cerrno>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>

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
constexpr int kDefaultMasterCanId = 0;
constexpr float kCmdZeroEps = 1e-6f;
constexpr float kSoftModeKd = 0.025f;
constexpr float kSoftReleaseKp = 0.5f;
constexpr float kDefaultMaxTorqueNm = 10.0f;
constexpr double kDefaultTxRateHz = 200.0;
constexpr int kDefaultFeedbackTimeoutMs = 250;
constexpr int kDefaultStartupFeedbackTimeoutMs = 2500;
constexpr int kCanRxSocketTimeoutMs = 100;
constexpr int kDefaultCanRxBufferBytes = 1 << 20;
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

int arbitration_id_from_frame(const struct can_frame & frame)
{
  const canid_t raw = frame.can_id;
  if (raw & CAN_EFF_FLAG) {
    return static_cast<int>(raw & CAN_EFF_MASK);
  }
  return static_cast<int>(raw & CAN_SFF_MASK);
}

int motor_id_from_feedback_data(uint8_t data0, int expected_drive_id)
{
  const int id_low_nibble = static_cast<int>(data0 & 0x0F);
  if (id_low_nibble == expected_drive_id) {
    return id_low_nibble;
  }
  return static_cast<int>(data0);
}

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

using SteadyClock = std::chrono::steady_clock;
using SteadyTime = SteadyClock::time_point;

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
    can_interface_ = declare_parameter<std::string>("can_interface", "can0");
    max_torque_nm_ = static_cast<float>(declare_parameter<double>(
      "max_torque", static_cast<double>(kDefaultMaxTorqueNm)));
    tx_rate_hz_ = declare_parameter<double>("tx_rate_hz", kDefaultTxRateHz);
    feedback_timeout_ms_ = declare_parameter<int>(
      "feedback_timeout_ms", kDefaultFeedbackTimeoutMs);
    startup_feedback_timeout_ms_ = declare_parameter<int>(
      "startup_feedback_timeout_ms", kDefaultStartupFeedbackTimeoutMs);
    use_can_filters_ = declare_parameter<bool>("use_can_filters", false);
    if (!get_parameter("can_id", can_id_)) {
      throw std::runtime_error("Failed to read can_id parameter");
    }

    if (can_id_ < 0 || can_id_ > 0x7FF) {
      throw std::invalid_argument("can_id must be in [0, 2047] for standard CAN");
    }
    if (max_torque_nm_ <= 0.0f) {
      throw std::invalid_argument("max_torque must be > 0");
    }
    if (tx_rate_hz_ <= 0.0) {
      throw std::invalid_argument("tx_rate_hz must be > 0");
    }
    if (feedback_timeout_ms_ <= 0) {
      throw std::invalid_argument("feedback_timeout_ms must be > 0");
    }
    if (startup_feedback_timeout_ms_ <= 0) {
      throw std::invalid_argument("startup_feedback_timeout_ms must be > 0");
    }

    try {
      profile_ = cm_interface::get_motor_mit_profile(motor_model);
    } catch (const std::exception & e) {
      RCLCPP_FATAL(get_logger(), "%s", e.what());
      throw;
    }

    RCLCPP_INFO(
      get_logger(),
      "Motor model: %s | %s | can_id: %d | tx_rate: %.0f Hz | feedback_timeout: %d ms | "
      "startup_feedback_timeout: %d ms | can_rx_filter=%s | max_torque=%.2f Nm",
      profile_.name, can_interface_.c_str(), can_id_, tx_rate_hz_, feedback_timeout_ms_,
      startup_feedback_timeout_ms_, use_can_filters_ ? "on" : "off", max_torque_nm_);

    if (!open_can_socket()) {
      return;
    }

    const auto cmd_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    subscription_ = create_subscription<motor_interfaces::msg::MotorCommand>(
      "motor_command",
      cmd_qos,
      std::bind(&MotorNodeContinuous::motor_command_callback, this, std::placeholders::_1));

    soft_mode_sub_ = create_subscription<std_msgs::msg::Bool>(
      "soft_mode",
      10,
      std::bind(&MotorNodeContinuous::soft_mode_callback, this, std::placeholders::_1));

    state_publisher_ = create_publisher<motor_interfaces::msg::MotorState>("motor_state", 10);

    rx_running_ = true;
    rx_thread_ = std::thread(&MotorNodeContinuous::can_rx_thread_main, this);

    enable_motor();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    set_motor_origin();

    if (wait_for_feedback(std::chrono::milliseconds(startup_feedback_timeout_ms_))) {
      drive_ready_ = true;
      RCLCPP_INFO(get_logger(), "can_id=%d initial MIT feedback OK.", can_id_);
    } else {
      RCLCPP_WARN(
        get_logger(),
        "can_id=%d no MIT feedback within %d ms at startup; motion TX deferred until "
        "feedback is received.",
        can_id_, startup_feedback_timeout_ms_);
    }

    const auto tx_period = std::chrono::duration<double>(1.0 / tx_rate_hz_);
    tx_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(tx_period),
      std::bind(&MotorNodeContinuous::tx_timer_callback, this));
  }

  ~MotorNodeContinuous() override
  {
    rx_running_ = false;
    if (can_socket_ >= 0) {
      shutdown(can_socket_, SHUT_RDWR);
    }
    if (rx_thread_.joinable()) {
      rx_thread_.join();
    }
    disable_motor();
    if (can_socket_ >= 0) {
      close(can_socket_);
      can_socket_ = -1;
    }
  }

private:
  bool open_can_socket()
  {
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to create CAN socket.");
      return false;
    }

    const int recv_buf = kDefaultCanRxBufferBytes;
    setsockopt(
      can_socket_, SOL_SOCKET, SO_RCVBUF, &recv_buf, sizeof(recv_buf));

    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);

    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
      RCLCPP_ERROR(get_logger(), "%s not found.", can_interface_.c_str());
      close(can_socket_);
      can_socket_ = -1;
      return false;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(can_socket_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
      RCLCPP_ERROR(get_logger(), "Bind failed on %s.", can_interface_.c_str());
      close(can_socket_);
      can_socket_ = -1;
      return false;
    }

    if (use_can_filters_) {
      if (!install_can_filters()) {
        close(can_socket_);
        can_socket_ = -1;
        return false;
      }
    }

    return true;
  }

  bool install_can_filters()
  {
    struct can_filter filters[2];
    filters[0].can_id = static_cast<canid_t>(can_id_);
    filters[0].can_mask = CAN_SFF_MASK;
    filters[1].can_id = static_cast<canid_t>(kDefaultMasterCanId);
    filters[1].can_mask = CAN_SFF_MASK;

    if (setsockopt(
        can_socket_, SOL_CAN_RAW, CAN_RAW_FILTER, filters, sizeof(filters)) < 0)
    {
      RCLCPP_ERROR(
        get_logger(), "Failed to set CAN filters for can_id=%d: %s",
        can_id_, std::strerror(errno));
      return false;
    }
    return true;
  }

  void can_rx_thread_main()
  {
    while (rx_running_) {
      struct pollfd pfd{};
      pfd.fd = can_socket_;
      pfd.events = POLLIN;

      const int poll_result = poll(&pfd, 1, kCanRxSocketTimeoutMs);
      if (poll_result < 0) {
        if (errno == EINTR) {
          continue;
        }
        if (rx_running_) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "CAN RX poll error on can_id=%d: %s", can_id_, std::strerror(errno));
        }
        continue;
      }
      if (poll_result == 0) {
        continue;
      }

      struct can_frame frame{};
      const ssize_t nbytes = read(can_socket_, &frame, sizeof(frame));
      if (nbytes < 0) {
        if (errno == EINTR || !rx_running_) {
          continue;
        }
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "CAN RX read error on can_id=%d: %s", can_id_, std::strerror(errno));
        continue;
      }
      if (nbytes != static_cast<ssize_t>(sizeof(frame))) {
        continue;
      }

      MitFeedback fb;
      if (!fb.unpack_reply(frame, can_id_, profile_)) {
        continue;
      }

      if (fb.error_code != 0) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "can_id=%d MIT fault: %s (code %d)",
          can_id_, mit_error_string(fb.error_code), fb.error_code);
      }

      {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        last_feedback_time_ = SteadyClock::now();
        has_feedback_ = true;
        comm_fault_ = false;
        last_position_rad_ = fb.position_rad;
        has_last_position_ = true;
      }

      if (!drive_ready_.load()) {
        drive_ready_ = true;
        RCLCPP_INFO(get_logger(), "can_id=%d MIT feedback active.", can_id_);
      }

      publish_motor_state(fb);
      feedback_cv_.notify_all();
    }
  }

  bool feedback_is_fresh()
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    if (!has_feedback_) {
      return false;
    }
    const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      SteadyClock::now() - last_feedback_time_).count();
    return age_ms <= feedback_timeout_ms_;
  }

  void mark_comm_fault()
  {
    if (!drive_ready_.load()) {
      return;
    }
    bool was_fault = false;
    {
      std::lock_guard<std::mutex> lock(feedback_mutex_);
      was_fault = comm_fault_;
      comm_fault_ = true;
    }
    if (!was_fault) {
      RCLCPP_ERROR(
        get_logger(),
        "can_id=%d comm fault: no fresh MIT feedback for %d ms; holding motor command at zero",
        can_id_, feedback_timeout_ms_);
    }
  }

  bool wait_for_feedback(std::chrono::milliseconds timeout)
  {
    std::unique_lock<std::mutex> lock(feedback_mutex_);
    return feedback_cv_.wait_for(lock, timeout, [this]() {
      return has_feedback_;
    });
  }

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

  bool write_mit_frame(
    float p_delta, float v_des, float kp, float kd, float t_ff, bool force_apply = false)
  {
    if (can_socket_ < 0) {
      return false;
    }

    const float p_delta_limited = clamp_position_delta_for_torque(p_delta, kp);

    const int p_int = pack_position_continuous(p_delta_limited, v_des, t_ff, force_apply);
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
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Failed to send MIT command on can_id=%d.", can_id_);
      return false;
    }
    return true;
  }

  void send_mit_command(
    float p_delta, float v_des, float kp, float kd, float t_ff, bool force_apply = false)
  {
    write_mit_frame(p_delta, v_des, kp, kd, t_ff, force_apply);
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
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to set motor origin.");
    }
  }

  float clamp_position_delta_for_torque(float p_delta, float kp) const
  {
    if (kp <= kCmdZeroEps) {
      return p_delta;
    }
    const float max_abs_delta = max_torque_nm_ / kp;
    if (p_delta > max_abs_delta) {
      return max_abs_delta;
    }
    if (p_delta < -max_abs_delta) {
      return -max_abs_delta;
    }
    return p_delta;
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

  void store_pending_command(const motor_interfaces::msg::MotorCommand & msg)
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    pending_command_ = msg;
    has_pending_command_ = true;
  }

  bool get_latest_command(motor_interfaces::msg::MotorCommand & out)
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (!has_pending_command_) {
      return false;
    }
    out = pending_command_;
    return true;
  }

  void tx_timer_callback()
  {
    if (can_socket_ < 0) {
      return;
    }

    if (soft_mode_) {
      return;
    }

    if (!drive_ready_.load()) {
      return;
    }

    if (!feedback_is_fresh()) {
      mark_comm_fault();
      write_mit_frame(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false);
      return;
    }

    motor_interfaces::msg::MotorCommand cmd;
    if (!get_latest_command(cmd)) {
      return;
    }

    write_mit_frame(cmd.position, cmd.velocity, cmd.kp, cmd.kd, cmd.torque, false);
  }

  void motor_command_callback(const motor_interfaces::msg::MotorCommand::SharedPtr msg)
  {
    if (soft_mode_) {
      return;
    }
    store_pending_command(*msg);
  }

  void soft_mode_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (soft_mode_ != msg->data) {
      soft_mode_ = msg->data;
      if (soft_mode_) {
        {
          std::lock_guard<std::mutex> lock(feedback_mutex_);
          soft_mode_on_position_rad_ = last_position_rad_;
          has_soft_mode_on_position_ = has_last_position_;
        }
        {
          std::lock_guard<std::mutex> lock(command_mutex_);
          has_pending_command_ = false;
        }
        send_soft_mode_command();
      } else {
        send_zero_mit_command();
        float p_delta = 0.0f;
        bool use_delta = false;
        {
          std::lock_guard<std::mutex> lock(feedback_mutex_);
          use_delta = has_soft_mode_on_position_ && has_last_position_;
          if (use_delta) {
            p_delta = last_position_rad_ - soft_mode_on_position_rad_;
          }
        }
        send_soft_release_hold_command(use_delta ? p_delta : 0.0f);
      }
      RCLCPP_INFO(
        get_logger(),
        "soft_mode=%s; %s",
        soft_mode_ ? "true" : "false",
        soft_mode_ ? "ignoring motor_command and sending KD only" : "normal command flow resumed");
    }
  }

  void send_soft_mode_command()
  {
    write_mit_frame(0.0f, 0.0f, 0.0f, kSoftModeKd, 0.0f, false);
  }

  void send_zero_mit_command()
  {
    write_mit_frame(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false);
  }

  void send_soft_release_hold_command(float p_delta)
  {
    write_mit_frame(p_delta, 0.0f, kSoftReleaseKp, 0.0f, 0.0f, true);
  }

  cm_interface::MotorMitProfile profile_{cm_interface::kAk70_10};
  int can_id_{kDefaultCanId};
  std::string can_interface_{"can0"};
  double tx_rate_hz_{kDefaultTxRateHz};
  int feedback_timeout_ms_{kDefaultFeedbackTimeoutMs};
  int startup_feedback_timeout_ms_{kDefaultStartupFeedbackTimeoutMs};
  bool use_can_filters_{false};
  std::atomic<bool> drive_ready_{false};

  rclcpp::Subscription<motor_interfaces::msg::MotorCommand>::SharedPtr subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr soft_mode_sub_;
  rclcpp::Publisher<motor_interfaces::msg::MotorState>::SharedPtr state_publisher_;
  rclcpp::TimerBase::SharedPtr tx_timer_;

  int can_socket_{-1};
  std::atomic<bool> rx_running_{false};
  std::thread rx_thread_;

  mutable std::mutex command_mutex_;
  motor_interfaces::msg::MotorCommand pending_command_;
  bool has_pending_command_{false};

  std::mutex feedback_mutex_;
  std::condition_variable feedback_cv_;
  SteadyTime last_feedback_time_{};
  bool has_feedback_{false};
  bool comm_fault_{false};
  float last_position_rad_{0.0f};
  bool has_last_position_{false};
  float soft_mode_on_position_rad_{0.0f};
  bool has_soft_mode_on_position_{false};

  bool soft_mode_{false};
  float max_torque_nm_{kDefaultMaxTorqueNm};
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
