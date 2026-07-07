// can_gateway_node: one SocketCAN interface, multiple MIT drives.
// Replaces one motor_node_continuous per namespace for multi-motor stacks.
//
// Parameters (TWEAK via launch or ros2 param):
//   can_interface          — SocketCAN device (e.g. can0)
//   namespaces,motor_models,can_ids — comma-separated lists, same length
//   loop_rate_hz           — TX/RX service rate (default 200)
//   feedback_timeout_ms    — stale feedback → comm fault zero-hold
//   feedback_poll_ms       — blocking RX poll each loop iteration
//   startup_stagger_ms     — delay between per-drive enable sequences
//   enable_settle_ms       — post-enable wait (non-AK80 drives)
//   ak80_enable_settle_ms  — post-enable wait for AK80-64 knee
//   startup_origin_poll_ms — RX wait after set-origin (0xFE)
//   bus_warmup_ms          — delay after bind before first enable
//
// Startup: enables drives sorted by can_id (AK80/knee last). Retries enable if
// no feedback. Soft-mode off triggers set-origin + Kd hold on that drive.
// loop_timer_callback: drain RX, poll for feedback, then service_drive_tx (MIT TX).
// Comm-fault uses last_feedback_time from the pre-service poll, not from TX this tick.
// refresh_all_drive_feedback() pings each drive sequentially (with retries) so
// early-enabled drives are not starved by batch MIT TX on a shared bus.

#include <algorithm>
#include <numeric>
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

#include "cm_interface/mit_can_codec.hpp"
#include "cm_interface/motor_mit_profile.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "motor_interfaces/msg/motor_command.hpp"
#include "motor_interfaces/msg/motor_state.hpp"
#include "std_msgs/msg/bool.hpp"

namespace
{

constexpr float kSoftModeKd = 0.025f;
constexpr double kDefaultLoopRateHz = 200.0;
constexpr int kDefaultFeedbackTimeoutMs = 250;
constexpr int kDefaultFeedbackPollMs = 15;
constexpr int kOriginFeedbackPollMs = 50;
constexpr int kDefaultEnableSettleMs = 100;
constexpr int kDefaultAk80EnableSettleMs = 250;
constexpr int kDefaultStartupOriginPollMs = 100;
constexpr int kDefaultBusWarmupMs = 100;
constexpr int kDefaultStartupStaggerMs = 200;
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

// Launch may pass can_ids:=1 as integer; node expects a CSV string.
std::string csv_param_as_string(
  rclcpp::Node & node,
  const std::string & name,
  const std::string & default_value)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.dynamic_typing = true;
  node.declare_parameter(name, default_value, descriptor);
  const rclcpp::Parameter param = node.get_parameter(name);
  switch (param.get_type()) {
    case rclcpp::ParameterType::PARAMETER_STRING:
      return param.as_string();
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      return std::to_string(param.as_int());
    default:
      throw std::invalid_argument(
        "parameter '" + name + "' must be a comma-separated string or integer");
  }
}

}  // namespace

class CanGatewayNode : public rclcpp::Node
{
public:
  struct DriveChannel
  {
    std::string ns;
    int can_id{0};
    cm_interface::MotorMitProfile profile{cm_interface::kAk70_10};

    rclcpp::Publisher<motor_interfaces::msg::MotorState>::SharedPtr state_pub;
    rclcpp::Subscription<motor_interfaces::msg::MotorCommand>::SharedPtr cmd_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr soft_sub;

    motor_interfaces::msg::MotorCommand pending_command;
    bool has_pending_command{false};
    bool soft_mode{false};

    SteadyTime last_feedback_time{};
    bool has_feedback{false};
    bool comm_fault{false};
    float last_position_rad{0.0f};
    bool has_last_position{false};
    float soft_mode_on_position_rad{0.0f};
    bool has_soft_mode_on_position{false};
    bool pending_soft_origin_reset{false};
  };

  CanGatewayNode()
  : Node("can_gateway_node")
  {
    can_interface_ = declare_parameter<std::string>("can_interface", "can0");
    loop_rate_hz_ = declare_parameter<double>("loop_rate_hz", kDefaultLoopRateHz);
    feedback_timeout_ms_ = declare_parameter<int>(
      "feedback_timeout_ms", kDefaultFeedbackTimeoutMs);
    feedback_poll_ms_ = declare_parameter<int>("feedback_poll_ms", kDefaultFeedbackPollMs);
    startup_stagger_ms_ = declare_parameter<int>(
      "startup_stagger_ms", kDefaultStartupStaggerMs);
    enable_settle_ms_ = declare_parameter<int>("enable_settle_ms", kDefaultEnableSettleMs);
    ak80_enable_settle_ms_ = declare_parameter<int>(
      "ak80_enable_settle_ms", kDefaultAk80EnableSettleMs);
    startup_origin_poll_ms_ = declare_parameter<int>(
      "startup_origin_poll_ms", kDefaultStartupOriginPollMs);
    bus_warmup_ms_ = declare_parameter<int>("bus_warmup_ms", kDefaultBusWarmupMs);

    const auto namespaces = split_csv(csv_param_as_string(
      *this, "namespaces", "knee_motor,hip_motor,wheel_motor1,wheel_motor2"));
    const auto motor_models = split_csv(csv_param_as_string(
      *this, "motor_models", "ak80_64,ak70_10,ak10_9,ak10_9"));
    const auto can_ids_str = split_csv(csv_param_as_string(*this, "can_ids", "4,3,1,2"));

    if (namespaces.size() != motor_models.size() || namespaces.size() != can_ids_str.size()) {
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
    std::vector<int> seen_can_ids;
    seen_can_ids.reserve(namespaces.size());
    for (size_t i = 0; i < namespaces.size(); ++i) {
      DriveChannel ch;
      ch.ns = namespaces[i];
      try {
        ch.can_id = std::stoi(can_ids_str[i]);
      } catch (const std::exception &) {
        throw std::invalid_argument(
          "can_ids[" + std::to_string(i) + "] for namespace '" + ch.ns +
          "': invalid integer '" + can_ids_str[i] + "'");
      }
      if (ch.can_id < 0 || ch.can_id > 0x7FF) {
        throw std::invalid_argument(
          "can_ids[" + std::to_string(i) + "] for namespace '" + ch.ns +
          "': must be in [0, 2047]");
      }
      for (size_t j = 0; j < seen_can_ids.size(); ++j) {
        if (seen_can_ids[j] == ch.can_id) {
          throw std::invalid_argument(
            "Duplicate can_id " + std::to_string(ch.can_id) + " for namespaces '" +
            drives_[j].ns + "' and '" + ch.ns + "'");
        }
      }
      seen_can_ids.push_back(ch.can_id);
      ch.profile = cm_interface::get_motor_mit_profile(motor_models[i]);
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
        [this, can_id = ch.can_id](const motor_interfaces::msg::MotorCommand::SharedPtr msg) {
          on_motor_command(can_id, msg);
        });

      ch.soft_sub = create_subscription<std_msgs::msg::Bool>(
        prefix + "/soft_mode",
        10,
        [this, can_id = ch.can_id](const std_msgs::msg::Bool::SharedPtr msg) {
          on_soft_mode(can_id, msg);
        });

      RCLCPP_INFO(
        get_logger(), "Drive %s: %s can_id=%d", ch.ns.c_str(), ch.profile.name, ch.can_id);
    }

    RCLCPP_INFO(
      get_logger(),
      "%s | %zu drives | loop %.0f Hz | feedback_timeout %d ms | feedback_poll %d ms",
      can_interface_.c_str(), drives_.size(), loop_rate_hz_, feedback_timeout_ms_,
      feedback_poll_ms_);

    startup_all_drives();

    const auto period = std::chrono::duration<double>(1.0 / loop_rate_hz_);
    loop_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&CanGatewayNode::loop_timer_callback, this));
  }

  ~CanGatewayNode() override
  {
    for (const auto & ch : drives_) {
      send_disable(ch);
    }
    if (can_socket_ >= 0) {
      close(can_socket_);
      can_socket_ = -1;
    }
  }

private:
  DriveChannel * find_drive_by_can_id(int can_id)
  {
    for (auto & ch : drives_) {
      if (ch.can_id == can_id) {
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

    RCLCPP_INFO(get_logger(), "CAN gateway bound to %s (single socket, no per-drive filters).",
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

  void process_pending_rx()
  {
    if (can_socket_ < 0) {
      return;
    }

    struct pollfd pfd{};
    pfd.fd = can_socket_;
    pfd.events = POLLIN;

    while (poll(&pfd, 1, 0) > 0) {
      struct can_frame frame{};
      const ssize_t nbytes = read(can_socket_, &frame, sizeof(frame));
      if (nbytes != static_cast<ssize_t>(sizeof(frame))) {
        break;
      }
      route_feedback_frame(frame);
    }
  }

  void poll_bus_feedback(int poll_timeout_ms)
  {
    if (can_socket_ < 0) {
      return;
    }

    const auto deadline = SteadyClock::now() + std::chrono::milliseconds(poll_timeout_ms);
    while (rclcpp::ok() && SteadyClock::now() < deadline) {
      const auto remaining_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - SteadyClock::now()).count();
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
        return;
      }
      if (poll_result == 0) {
        break;
      }

      struct can_frame frame{};
      const ssize_t nbytes = read(can_socket_, &frame, sizeof(frame));
      if (nbytes != static_cast<ssize_t>(sizeof(frame))) {
        continue;
      }
      route_feedback_frame(frame);
    }
  }

  bool route_feedback_frame(const struct can_frame & frame)
  {
    for (auto & ch : drives_) {
      cm_interface::MitFeedback fb;
      if (!fb.unpack_reply(frame, ch.can_id, ch.profile)) {
        continue;
      }

      if (fb.error_code != 0) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "[%s] can_id=%d MIT fault: %s (code %d)",
          ch.ns.c_str(), ch.can_id, cm_interface::mit_error_string(fb.error_code), fb.error_code);
      }

      const bool first = !ch.has_feedback;
      ch.last_feedback_time = SteadyClock::now();
      ch.has_feedback = true;
      ch.comm_fault = false;
      ch.last_position_rad = fb.position_rad;
      ch.has_last_position = true;

      if (first) {
        RCLCPP_INFO(
          get_logger(), "[%s] can_id=%d MIT feedback active.", ch.ns.c_str(), ch.can_id);
      }

      publish_state(ch, fb);
      return true;
    }
    return false;
  }

  void publish_state(const DriveChannel & ch, const cm_interface::MitFeedback & fb)
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
    float p_delta, float v_des, float kp, float kd, float t_ff, bool force_apply)
  {
    struct can_frame frame{};
    cm_interface::pack_mit_command_frame(
      frame, ch.can_id, p_delta, v_des, kp, kd, t_ff, ch.profile, force_apply);
    if (!write_frame(frame)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "[%s] MIT TX failed on can_id=%d.", ch.ns.c_str(), ch.can_id);
      return false;
    }
    return true;
  }

  bool send_enable(const DriveChannel & ch, const char * phase = nullptr)
  {
    struct can_frame frame{};
    cm_interface::pack_control_frame(frame, ch.can_id, 0xFC);
    if (write_frame(frame)) {
      if (phase != nullptr && phase[0] != '\0') {
        RCLCPP_INFO(
          get_logger(), "[STARTUP] [%s] can_id=%d enable sent (%s).",
          ch.ns.c_str(), ch.can_id, phase);
      } else {
        RCLCPP_INFO(
          get_logger(), "[STARTUP] [%s] can_id=%d enable sent.",
          ch.ns.c_str(), ch.can_id);
      }
      return true;
    }
    RCLCPP_ERROR(get_logger(), "[STARTUP] [%s] can_id=%d enable failed.", ch.ns.c_str(), ch.can_id);
    return false;
  }

  void send_disable(const DriveChannel & ch)
  {
    struct can_frame frame{};
    cm_interface::pack_control_frame(frame, ch.can_id, 0xFD);
    write_frame(frame);
  }

  void process_pending_soft_origin_resets()
  {
    for (auto & ch : drives_) {
      if (!ch.pending_soft_origin_reset) {
        continue;
      }
      ch.pending_soft_origin_reset = false;
      send_set_origin(ch, startup_origin_poll_ms_);
      send_mit(ch, 0.0f, 0.0f, 0.0f, ch.profile.mit_kd, 0.0f, true);
      RCLCPP_INFO(
        get_logger(), "[%s] soft_mode off: origin reset at current position", ch.ns.c_str());
    }
  }

  bool send_set_origin(DriveChannel & ch, int feedback_poll_ms = kOriginFeedbackPollMs)
  {
    struct can_frame frame{};
    cm_interface::pack_control_frame(frame, ch.can_id, 0xFE);
    if (!write_frame(frame)) {
      RCLCPP_ERROR(get_logger(), "[STARTUP] [%s] set origin failed.", ch.ns.c_str());
      return false;
    }
    RCLCPP_INFO(get_logger(), "[STARTUP] [%s] set motor origin.", ch.ns.c_str());
    poll_bus_feedback(feedback_poll_ms);
    return true;
  }

  bool is_ak80_drive(const DriveChannel & ch) const
  {
    return std::strcmp(ch.profile.name, "AK80-64") == 0;
  }

  int enable_settle_ms_for(const DriveChannel & ch) const
  {
    return is_ak80_drive(ch) ? ak80_enable_settle_ms_ : enable_settle_ms_;
  }

  void startup_one_drive(DriveChannel & ch)
  {
    const int settle_ms = enable_settle_ms_for(ch);
    send_enable(ch);
    std::this_thread::sleep_for(std::chrono::milliseconds(settle_ms));
    send_set_origin(ch, startup_origin_poll_ms_);
    send_mit(ch, 0.0f, 0.0f, 0.0f, ch.profile.mit_kd, 0.0f, true);
    poll_bus_feedback(feedback_poll_ms_);
  }

  void startup_all_drives()
  {
    if (bus_warmup_ms_ > 0) {
      RCLCPP_INFO(
        get_logger(), "[STARTUP] Waiting %d ms for %s to settle before enable.",
        bus_warmup_ms_, can_interface_.c_str());
      std::this_thread::sleep_for(std::chrono::milliseconds(bus_warmup_ms_));
    }

    std::vector<size_t> order(drives_.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [this](size_t a, size_t b) {
      return drives_[a].can_id < drives_[b].can_id;
    });

    std::string order_log;
    for (size_t i = 0; i < order.size(); ++i) {
      if (i > 0) {
        order_log += " -> ";
      }
      const auto & ch = drives_[order[i]];
      order_log += ch.ns + "(can_id=" + std::to_string(ch.can_id) + ")";
    }
    RCLCPP_INFO(
      get_logger(),
      "[STARTUP] Enable order by can_id (knee/AK80-64 last): %s", order_log.c_str());

    for (size_t i = 0; i < order.size(); ++i) {
      if (i > 0 && startup_stagger_ms_ > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(startup_stagger_ms_));
      }
      startup_one_drive(drives_[order[i]]);
    }

    for (auto & ch : drives_) {
      if (!ch.has_feedback) {
        RCLCPP_WARN(
          get_logger(),
          "[STARTUP] [%s] can_id=%d no MIT feedback yet; retrying enable sequence.",
          ch.ns.c_str(), ch.can_id);
        startup_one_drive(ch);
      }
      RCLCPP_INFO(
        get_logger(), "[STARTUP] %s can_id=%d feedback=%s",
        ch.ns.c_str(), ch.can_id, ch.has_feedback ? "active" : "missing");
    }

    refresh_all_drive_feedback();
  }

  void ping_drive_for_feedback(DriveChannel & ch, int poll_timeout_ms)
  {
    send_mit(ch, 0.0f, 0.0f, 0.0f, ch.profile.mit_kd, 0.0f, true);
    poll_bus_feedback(poll_timeout_ms);
    process_pending_rx();
  }

  int startup_feedback_poll_ms() const
  {
    return std::max({feedback_timeout_ms_, startup_origin_poll_ms_, feedback_poll_ms_});
  }

  void refresh_all_drive_feedback()
  {
    // Staggered per-drive startup can age feedback on early drives beyond
    // feedback_timeout_ms before the first loop tick. Ping each drive in turn
    // (batch pings on a shared bus often drop replies for early-enabled drives),
    // then retry any drive that is still missing or stale.
    RCLCPP_INFO(
      get_logger(),
      "[STARTUP] Refreshing MIT feedback from all drives before service loop.");

    const int per_drive_poll_ms = startup_feedback_poll_ms();
    for (auto & ch : drives_) {
      ping_drive_for_feedback(ch, per_drive_poll_ms);
    }

    constexpr int kStartupRefreshRetries = 2;
    for (int attempt = 0; attempt < kStartupRefreshRetries; ++attempt) {
      bool need_retry = false;
      for (auto & ch : drives_) {
        if (!ch.has_feedback || !feedback_is_fresh(ch)) {
          need_retry = true;
          RCLCPP_WARN(
            get_logger(),
            "[STARTUP] [%s] can_id=%d %s; retrying MIT ping (%d/%d).",
            ch.ns.c_str(), ch.can_id,
            ch.has_feedback ? "feedback stale" : "no MIT feedback yet",
            attempt + 1, kStartupRefreshRetries);
          ping_drive_for_feedback(ch, per_drive_poll_ms);
        }
      }
      if (!need_retry) {
        break;
      }
    }

    bool all_initiated = !drives_.empty();
    for (const auto & ch : drives_) {
      if (!ch.has_feedback) {
        all_initiated = false;
        RCLCPP_WARN(
          get_logger(),
          "[STARTUP] [%s] can_id=%d still no MIT feedback after refresh poll.",
          ch.ns.c_str(), ch.can_id);
      } else if (!feedback_is_fresh(ch)) {
        all_initiated = false;
        RCLCPP_WARN(
          get_logger(),
          "[STARTUP] [%s] can_id=%d feedback not fresh after final %d ms poll.",
          ch.ns.c_str(), ch.can_id, per_drive_poll_ms);
      }
    }

    bool any_feedback = false;
    for (const auto & ch : drives_) {
      if (ch.has_feedback) {
        any_feedback = true;
        break;
      }
    }

    if (!any_feedback && !drives_.empty()) {
      RCLCPP_ERROR(
        get_logger(),
        "[STARTUP] Motor initilization failed, check power and CAN wiring.");
    } else if (all_initiated) {
      RCLCPP_INFO(get_logger(), "[STARTUP] All motors successfully initiated.");
    }

    comm_fault_checks_armed_ = true;
  }

  bool feedback_is_fresh(const DriveChannel & ch) const
  {
    if (!ch.has_feedback) {
      return false;
    }
    const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      SteadyClock::now() - ch.last_feedback_time).count();
    return age_ms <= feedback_timeout_ms_;
  }

  void mark_comm_fault(DriveChannel & ch)
  {
    if (!ch.has_feedback || ch.comm_fault) {
      return;
    }
    ch.comm_fault = true;
    RCLCPP_ERROR(
      get_logger(),
      "[%s] can_id=%d comm fault: no fresh MIT feedback for %d ms; zero hold",
      ch.ns.c_str(), ch.can_id, feedback_timeout_ms_);
  }

  void service_drive_tx(DriveChannel & ch)
  {
    if (ch.soft_mode) {
      send_mit(ch, 0.0f, 0.0f, 0.0f, kSoftModeKd, 0.0f, true);
      return;
    }

    if (ch.comm_fault) {
      send_mit(ch, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, true);
      return;
    }

    if (comm_fault_checks_armed_ && ch.has_feedback && !feedback_is_fresh(ch)) {
      mark_comm_fault(ch);
      send_mit(ch, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, true);
      return;
    }

    if (!ch.has_feedback) {
      send_mit(ch, 0.0f, 0.0f, 0.0f, ch.profile.mit_kd, 0.0f, true);
      return;
    }

    if (ch.has_pending_command) {
      const auto & cmd = ch.pending_command;
      send_mit(ch, cmd.position, cmd.velocity, cmd.kp, cmd.kd, cmd.torque, false);
      return;
    }

    send_mit(ch, 0.0f, 0.0f, 0.0f, ch.profile.mit_kd, 0.0f, true);
  }

  void loop_timer_callback()
  {
    if (can_socket_ < 0) {
      return;
    }

    process_pending_rx();
    process_pending_soft_origin_resets();
    // Poll before service_drive_tx so freshness reflects replies to recent MIT TX.
    poll_bus_feedback(feedback_poll_ms_);

    for (auto & ch : drives_) {
      service_drive_tx(ch);
    }
  }

  void on_motor_command(
    int can_id, const motor_interfaces::msg::MotorCommand::SharedPtr msg)
  {
    DriveChannel * ch = find_drive_by_can_id(can_id);
    if (ch == nullptr || ch->soft_mode) {
      return;
    }
    ch->pending_command = *msg;
    ch->has_pending_command = true;
  }

  void on_soft_mode(int can_id, const std_msgs::msg::Bool::SharedPtr msg)
  {
    DriveChannel * ch = find_drive_by_can_id(can_id);
    if (ch == nullptr || ch->soft_mode == msg->data) {
      return;
    }

    ch->soft_mode = msg->data;
    if (ch->soft_mode) {
      ch->soft_mode_on_position_rad = ch->last_position_rad;
      ch->has_soft_mode_on_position = ch->has_last_position;
      ch->has_pending_command = false;
      send_mit(*ch, 0.0f, 0.0f, 0.0f, kSoftModeKd, 0.0f, true);
    } else {
      ch->has_pending_command = false;
      ch->pending_soft_origin_reset = true;
      send_mit(*ch, 0.0f, 0.0f, 0.0f, ch->profile.mit_kd, 0.0f, true);
    }

    RCLCPP_INFO(
      get_logger(), "[%s] soft_mode=%s", ch->ns.c_str(), ch->soft_mode ? "true" : "false");
  }

  std::string can_interface_{"can0"};
  double loop_rate_hz_{kDefaultLoopRateHz};
  int feedback_timeout_ms_{kDefaultFeedbackTimeoutMs};
  int feedback_poll_ms_{kDefaultFeedbackPollMs};
  int startup_stagger_ms_{kDefaultStartupStaggerMs};
  int enable_settle_ms_{kDefaultEnableSettleMs};
  int ak80_enable_settle_ms_{kDefaultAk80EnableSettleMs};
  int startup_origin_poll_ms_{kDefaultStartupOriginPollMs};
  int bus_warmup_ms_{kDefaultBusWarmupMs};

  std::vector<DriveChannel> drives_;
  int can_socket_{-1};
  rclcpp::TimerBase::SharedPtr loop_timer_;
  bool comm_fault_checks_armed_{false};
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
