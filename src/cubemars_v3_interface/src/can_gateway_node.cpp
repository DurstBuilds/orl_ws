// can_gateway_node: V3 firmware MIT gateway over a single SocketCAN interface.
//
// Parameters:
//   can_interface          — SocketCAN device (e.g. can0)
//   namespaces,motor_models,can_ids — comma-separated lists, same length
//   loop_rate_hz           — MIT command TX rate (default 200)
//   feedback_timeout_ms    — stale streaming feedback → comm fault zero-hold
//   bus_warmup_ms          — delay after bind before first MIT command
//
// V3 drives stream MIT feedback at a fixed rate (not request-response). A background
// RX thread publishes motor_state on every feedback frame; the TX loop only sends
// motor_command. MotorCommand.position is absolute position (rad), not per-tick delta.

#include <atomic>
#include <chrono>
#include <cerrno>
#include <cstring>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <poll.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "cubemars_v3_interface/mit_can_codec.hpp"
#include "cubemars_v3_interface/motor_mit_profile.hpp"
#include "rclcpp/rclcpp.hpp"
#include "motor_interfaces/msg/motor_command.hpp"
#include "motor_interfaces/msg/motor_state.hpp"

namespace
{

constexpr double kDefaultLoopRateHz = 200.0;
constexpr int kDefaultFeedbackTimeoutMs = 250;
constexpr int kDefaultBusWarmupMs = 100;
constexpr int kRxPollTimeoutMs = 5;
constexpr int kDefaultCanRxBufferBytes = 1 << 20;

using SteadyClock = std::chrono::steady_clock;
using SteadyTime = SteadyClock::time_point;

std::vector<std::string> split_csv(const std::string & value)
{
  std::vector<std::string> parts;
  std::stringstream stream(value);
  std::string item;
  while (std::getline(stream, item, ',')) {
    const auto start = item.find_first_not_of(" \t");
    if (start == std::string::npos) {
      continue;
    }
    const auto end = item.find_last_not_of(" \t");
    parts.push_back(item.substr(start, end - start + 1));
  }
  return parts;
}

}  // namespace

class CanGatewayNode : public rclcpp::Node
{
public:
  struct DriveChannel
  {
    std::string ns;
    int drive_id{0};
    cubemars_v3_interface::MotorMitProfile profile{cubemars_v3_interface::kAk60_6};

    rclcpp::Publisher<motor_interfaces::msg::MotorState>::SharedPtr state_pub;
    rclcpp::Subscription<motor_interfaces::msg::MotorCommand>::SharedPtr cmd_sub;

    motor_interfaces::msg::MotorCommand pending_command;
    bool has_pending_command{false};

    SteadyTime last_feedback_time{};
    bool has_feedback{false};
    bool comm_fault{false};
  };

  CanGatewayNode()
  : Node("can_gateway_node")
  {
    can_interface_ = declare_parameter<std::string>("can_interface", "can0");
    loop_rate_hz_ = declare_parameter<double>("loop_rate_hz", kDefaultLoopRateHz);
    feedback_timeout_ms_ = declare_parameter<int>(
      "feedback_timeout_ms", kDefaultFeedbackTimeoutMs);
    bus_warmup_ms_ = declare_parameter<int>("bus_warmup_ms", kDefaultBusWarmupMs);

    const auto namespaces = split_csv(
      declare_parameter<std::string>("namespaces", "motor"));
    const auto motor_models = split_csv(
      declare_parameter<std::string>("motor_models", "ak60_6"));
    const auto drive_ids_str = split_csv(
      declare_parameter<std::string>("can_ids", "1"));

    if (namespaces.size() != motor_models.size() || namespaces.size() != drive_ids_str.size()) {
      throw std::invalid_argument(
        "namespaces, motor_models, and can_ids must have the same number of entries");
    }
    if (namespaces.empty()) {
      throw std::invalid_argument("At least one drive is required");
    }
    if (loop_rate_hz_ <= 0.0) {
      throw std::invalid_argument("loop_rate_hz must be > 0");
    }

    drives_.reserve(namespaces.size());
    std::vector<int> seen_drive_ids;
    seen_drive_ids.reserve(namespaces.size());
    for (size_t i = 0; i < namespaces.size(); ++i) {
      DriveChannel ch;
      ch.ns = namespaces[i];
      try {
        ch.drive_id = std::stoi(drive_ids_str[i]);
      } catch (const std::exception &) {
        throw std::invalid_argument(
          "can_ids[" + std::to_string(i) + "] for namespace '" + ch.ns +
          "': invalid integer '" + drive_ids_str[i] + "'");
      }
      if (ch.drive_id < 0 || ch.drive_id > 0xFF) {
        throw std::invalid_argument(
          "can_ids[" + std::to_string(i) + "] for namespace '" + ch.ns +
          "': must be in [0, 255] for V3 extended drive ID");
      }
      for (size_t j = 0; j < seen_drive_ids.size(); ++j) {
        if (seen_drive_ids[j] == ch.drive_id) {
          throw std::invalid_argument(
            "Duplicate can_id " + std::to_string(ch.drive_id) + " for namespaces '" +
            drives_[j].ns + "' and '" + ch.ns + "'");
        }
      }
      seen_drive_ids.push_back(ch.drive_id);
      ch.profile = cubemars_v3_interface::get_motor_mit_profile(motor_models[i]);
      drives_.push_back(std::move(ch));
    }

    if (!open_can_socket()) {
      throw std::runtime_error(
        "Failed to open CAN interface '" + can_interface_ + "'; gateway cannot start");
    }

    const auto cmd_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    for (auto & ch : drives_) {
      const std::string prefix = "/" + ch.ns;
      ch.state_pub = create_publisher<motor_interfaces::msg::MotorState>(
        prefix + "/motor_state", 10);

      ch.cmd_sub = create_subscription<motor_interfaces::msg::MotorCommand>(
        prefix + "/motor_command",
        cmd_qos,
        [this, drive_id = ch.drive_id](const motor_interfaces::msg::MotorCommand::SharedPtr msg) {
          on_motor_command(drive_id, msg);
        });

      RCLCPP_INFO(
        get_logger(), "Drive %s: %s drive_id=%d arb=0x%03X",
        ch.ns.c_str(), ch.profile.name, ch.drive_id,
        cubemars_v3_interface::make_mit_arbitration_id(ch.drive_id));
    }

    RCLCPP_INFO(
      get_logger(),
      "%s | %zu drives | TX %.0f Hz | streaming feedback | feedback_timeout %d ms",
      can_interface_.c_str(), drives_.size(), loop_rate_hz_, feedback_timeout_ms_);

    start_rx_thread();
    startup_all_drives();

    const auto period = std::chrono::duration<double>(1.0 / loop_rate_hz_);
    loop_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&CanGatewayNode::loop_timer_callback, this));
  }

  ~CanGatewayNode() override
  {
    stop_rx_thread();
    if (can_socket_ >= 0) {
      close(can_socket_);
      can_socket_ = -1;
    }
  }

private:
  DriveChannel * find_drive_by_id(int drive_id)
  {
    for (auto & ch : drives_) {
      if (ch.drive_id == drive_id) {
        return &ch;
      }
    }
    return nullptr;
  }

  bool open_can_socket()
  {
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to create CAN socket.");
      return false;
    }

    const int recv_buf = kDefaultCanRxBufferBytes;
    setsockopt(can_socket_, SOL_SOCKET, SO_RCVBUF, &recv_buf, sizeof(recv_buf));

    struct ifreq ifr{};
    std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
      RCLCPP_ERROR(get_logger(), "%s not found.", can_interface_.c_str());
      close(can_socket_);
      can_socket_ = -1;
      return false;
    }

    struct sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(can_socket_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
      RCLCPP_ERROR(get_logger(), "Bind failed on %s.", can_interface_.c_str());
      close(can_socket_);
      can_socket_ = -1;
      return false;
    }

    RCLCPP_INFO(get_logger(), "CAN gateway bound to %s (V3 extended frames).",
      can_interface_.c_str());
    return true;
  }

  bool write_frame(const struct can_frame & frame)
  {
    if (can_socket_ < 0) {
      return false;
    }
    return write(can_socket_, &frame, sizeof(frame)) == static_cast<ssize_t>(sizeof(frame));
  }

  void start_rx_thread()
  {
    rx_running_ = true;
    rx_thread_ = std::thread(&CanGatewayNode::rx_thread_main, this);
  }

  void stop_rx_thread()
  {
    rx_running_ = false;
    if (rx_thread_.joinable()) {
      rx_thread_.join();
    }
  }

  void rx_thread_main()
  {
    struct pollfd pfd{};
    pfd.fd = can_socket_;
    pfd.events = POLLIN;

    while (rx_running_ && rclcpp::ok()) {
      const int poll_result = poll(&pfd, 1, kRxPollTimeoutMs);
      if (poll_result < 0) {
        if (errno == EINTR) {
          continue;
        }
        break;
      }
      if (poll_result == 0) {
        continue;
      }

      while (rx_running_) {
        struct can_frame frame{};
        const ssize_t nbytes = read(can_socket_, &frame, sizeof(frame));
        if (nbytes != static_cast<ssize_t>(sizeof(frame))) {
          break;
        }
        route_feedback_frame(frame);
      }
    }
  }

  bool route_feedback_frame(const struct can_frame & frame)
  {
    for (auto & ch : drives_) {
      cubemars_v3_interface::MitFeedback fb;
      if (!fb.unpack_reply(frame, ch.drive_id)) {
        continue;
      }

      if (fb.error_code != 0) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "[%s] drive_id=%d MIT fault: %s (code %d)",
          ch.ns.c_str(), ch.drive_id,
          cubemars_v3_interface::mit_error_string(fb.error_code), fb.error_code);
      }

      bool first = false;
      {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        first = !ch.has_feedback;
        ch.last_feedback_time = SteadyClock::now();
        ch.has_feedback = true;
        ch.comm_fault = false;
      }

      if (first) {
        RCLCPP_INFO(
          get_logger(), "[%s] drive_id=%d streaming MIT feedback active.",
          ch.ns.c_str(), ch.drive_id);
      }

      publish_state(ch, fb);
      return true;
    }
    return false;
  }

  void publish_state(const DriveChannel & ch, const cubemars_v3_interface::MitFeedback & fb)
  {
    motor_interfaces::msg::MotorState msg;
    msg.position = fb.position_rad;
    msg.velocity = fb.velocity_rad_s;
    msg.torque = fb.torque_nm;
    msg.temperature = fb.temperature_c;
    msg.error_code = fb.error_code;
    msg.drive_id = static_cast<uint8_t>(fb.drive_id);
    ch.state_pub->publish(msg);
  }

  bool send_mit(
    const DriveChannel & ch,
    float p_des, float v_des, float kp, float kd, float t_ff)
  {
    struct can_frame frame{};
    cubemars_v3_interface::pack_mit_command_frame(
      frame, ch.drive_id, p_des, v_des, kp, kd, t_ff, ch.profile);
    if (!write_frame(frame)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "[%s] MIT TX failed on drive_id=%d.", ch.ns.c_str(), ch.drive_id);
      return false;
    }
    return true;
  }

  void startup_all_drives()
  {
    if (bus_warmup_ms_ > 0) {
      RCLCPP_INFO(
        get_logger(), "[STARTUP] Waiting %d ms for %s to settle.",
        bus_warmup_ms_, can_interface_.c_str());
      std::this_thread::sleep_for(std::chrono::milliseconds(bus_warmup_ms_));
    }

    for (auto & ch : drives_) {
      send_mit(ch, 0.0f, 0.0f, 0.0f, ch.profile.mit_kd, 0.0f);
    }

    const auto deadline = SteadyClock::now() +
      std::chrono::milliseconds(feedback_timeout_ms_);
    while (rclcpp::ok() && SteadyClock::now() < deadline) {
      bool all_have_feedback = true;
      {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        for (const auto & ch : drives_) {
          if (!ch.has_feedback) {
            all_have_feedback = false;
            break;
          }
        }
      }
      if (all_have_feedback) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    bool any_feedback = false;
    {
      std::lock_guard<std::mutex> lock(feedback_mutex_);
      for (const auto & ch : drives_) {
        if (ch.has_feedback) {
          any_feedback = true;
        } else {
          RCLCPP_WARN(
            get_logger(),
            "[STARTUP] [%s] drive_id=%d no streaming MIT feedback within %d ms.",
            ch.ns.c_str(), ch.drive_id, feedback_timeout_ms_);
        }
      }
    }

    if (!any_feedback && !drives_.empty()) {
      RCLCPP_ERROR(
        get_logger(),
        "[STARTUP] No MIT feedback; check power, CAN wiring, drive ID, MIT mode, "
        "and streaming feedback setting.");
    } else if (any_feedback) {
      RCLCPP_INFO(get_logger(), "[STARTUP] Gateway ready.");
    }

    comm_fault_checks_armed_ = true;
  }

  bool feedback_is_fresh(const DriveChannel & ch) const
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    if (!ch.has_feedback) {
      return false;
    }
    const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      SteadyClock::now() - ch.last_feedback_time).count();
    return age_ms <= feedback_timeout_ms_;
  }

  void mark_comm_fault(DriveChannel & ch)
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    if (!ch.has_feedback || ch.comm_fault) {
      return;
    }
    ch.comm_fault = true;
    RCLCPP_ERROR(
      get_logger(),
      "[%s] drive_id=%d comm fault: no fresh MIT feedback for %d ms; zero hold",
      ch.ns.c_str(), ch.drive_id, feedback_timeout_ms_);
  }

  void service_drive_tx(DriveChannel & ch)
  {
    bool comm_fault = false;
    bool has_feedback = false;
    {
      std::lock_guard<std::mutex> lock(feedback_mutex_);
      comm_fault = ch.comm_fault;
      has_feedback = ch.has_feedback;
    }

    if (comm_fault) {
      send_mit(ch, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
      return;
    }

    if (comm_fault_checks_armed_ && has_feedback && !feedback_is_fresh(ch)) {
      mark_comm_fault(ch);
      send_mit(ch, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
      return;
    }

    if (!has_feedback) {
      send_mit(ch, 0.0f, 0.0f, 0.0f, ch.profile.mit_kd, 0.0f);
      return;
    }

    if (ch.has_pending_command) {
      const auto & cmd = ch.pending_command;
      send_mit(ch, cmd.position, cmd.velocity, cmd.kp, cmd.kd, cmd.torque);
      return;
    }

    send_mit(ch, 0.0f, 0.0f, 0.0f, ch.profile.mit_kd, 0.0f);
  }

  void loop_timer_callback()
  {
    if (can_socket_ < 0) {
      return;
    }

    for (auto & ch : drives_) {
      service_drive_tx(ch);
    }
  }

  void on_motor_command(
    int drive_id, const motor_interfaces::msg::MotorCommand::SharedPtr msg)
  {
    DriveChannel * ch = find_drive_by_id(drive_id);
    if (ch == nullptr) {
      return;
    }
    ch->pending_command = *msg;
    ch->has_pending_command = true;
  }

  std::string can_interface_{"can0"};
  double loop_rate_hz_{kDefaultLoopRateHz};
  int feedback_timeout_ms_{kDefaultFeedbackTimeoutMs};
  int bus_warmup_ms_{kDefaultBusWarmupMs};

  std::vector<DriveChannel> drives_;
  int can_socket_{-1};
  rclcpp::TimerBase::SharedPtr loop_timer_;
  bool comm_fault_checks_armed_{false};

  std::atomic<bool> rx_running_{false};
  std::thread rx_thread_;
  mutable std::mutex feedback_mutex_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<CanGatewayNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("can_gateway_node"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
