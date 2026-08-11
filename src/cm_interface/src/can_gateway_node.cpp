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
//   standby_retry_ms       — retry CAN open / drive connect when not ready (default 5000)
//   start_in_soft_mode     — all drives start in damping-only mode (default true)
//
// Publishes /boom_stack/ready (latched) when all drives have fresh MIT feedback.
// Full-stack reconnect: any fault resets and re-enables all drives in can_id order.
//
// Startup: enables drives sorted by can_id (AK80/knee last). If CAN or motors are
// not ready at launch, standby_retry_callback retries every standby_retry_ms.
// Soft-mode off triggers set-origin + Kd hold on that drive.
// loop_timer_callback: drain RX, poll for feedback, then service_drive_tx (MIT TX).
// Comm-fault uses last_feedback_time from the pre-service poll, not from TX this tick.
// refresh_all_drive_feedback() pings each drive sequentially (with retries) so
// early-enabled drives are not starved by batch MIT TX on a shared bus.

#include <algorithm>
#include <cmath>
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

constexpr float kSoftModeKd = 0.25f;
constexpr double kDefaultLoopRateHz = 200.0;
constexpr int kDefaultFeedbackTimeoutMs = 250;
constexpr int kDefaultFeedbackPollMs = 15;
constexpr int kOriginFeedbackPollMs = 50;
constexpr int kDefaultEnableSettleMs = 100;
constexpr int kDefaultAk80EnableSettleMs = 250;
constexpr int kDefaultStartupOriginPollMs = 100;
constexpr int kDefaultBusWarmupMs = 100;
constexpr int kDefaultStartupStaggerMs = 200;
constexpr int kDefaultStandbyRetryMs = 5000;
constexpr int kDefaultCanRxBufferBytes = 1 << 20;
// Wrapped motor position must be within this after set-origin before motor_state is published.
constexpr float kPostOriginPositionTolRad = 0.15f;

using SteadyClock = std::chrono::steady_clock;
using SteadyTime = SteadyClock::time_point;

/** Split a comma-separated parameter string and trim whitespace around each entry. */
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

/** Coerce a launch param that may arrive as int or string into a CSV string. */
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
  enum class ConnectState
  {
    Disconnected,
    Connecting,
    Connected,
    Fault,
  };

  enum class SoftOriginResetState
  {
    Idle,
    Active,
  };

  struct DriveChannel
  {
    std::string ns;
    int can_id{0};
    cm_interface::MotorMitProfile profile{cm_interface::kAk70_10};

    rclcpp::Publisher<motor_interfaces::msg::MotorState>::SharedPtr state_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr soft_mode_pub;
    rclcpp::Subscription<motor_interfaces::msg::MotorCommand>::SharedPtr cmd_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr soft_sub;

    motor_interfaces::msg::MotorCommand pending_command;
    bool has_pending_command{false};
    bool soft_mode{false};

    SteadyTime last_feedback_time{};
    bool has_feedback{false};
    bool comm_fault{false};
    ConnectState connect_state{ConnectState::Disconnected};
    float last_position_rad{0.0f};
    bool has_last_position{false};
    float soft_mode_on_position_rad{0.0f};
    bool has_soft_mode_on_position{false};
    SoftOriginResetState origin_reset_state{SoftOriginResetState::Idle};
    bool origin_set_origin_sent{false};
    SteadyTime origin_reset_deadline{};
    bool startup_enable_done{false};
    bool suppress_state_publish{false};
    bool has_last_feedback{false};
    cm_interface::MitFeedback last_feedback{};
  };

  /** Load drive lists, wire per-namespace I/O, and defer connect to standby retry. */
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
    standby_retry_ms_ = declare_parameter<int>("standby_retry_ms", kDefaultStandbyRetryMs);
    start_in_soft_mode_ = declare_parameter<bool>("start_in_soft_mode", true);

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
    if (standby_retry_ms_ <= 0) {
      throw std::invalid_argument("standby_retry_ms must be > 0");
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
      if (start_in_soft_mode_) {
        ch.soft_mode = true;
      }
      drives_.push_back(std::move(ch));
    }

    if (!open_can_socket()) {
      RCLCPP_WARN(
        get_logger(),
        "[STANDBY] CAN interface '%s' not available at launch; will retry every %d ms.",
        can_interface_.c_str(), standby_retry_ms_);
    }

    const auto cmd_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    for (auto & ch : drives_) {
      const std::string prefix = "/" + ch.ns;
      ch.state_pub = create_publisher<motor_interfaces::msg::MotorState>(
        prefix + "/motor_state", 10);
      ch.soft_mode_pub = create_publisher<std_msgs::msg::Bool>(
        prefix + "/soft_mode", 10);

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

    stack_ready_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/boom_stack/ready",
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    if (start_in_soft_mode_) {
      publish_all_soft_mode_state();
    }
    set_stack_ready(false);

    RCLCPP_INFO(
      get_logger(),
      "%s | %zu drives | loop %.0f Hz | feedback_timeout %d ms | feedback_poll %d ms | "
      "standby_retry %d ms | start_in_soft_mode %s",
      can_interface_.c_str(), drives_.size(), loop_rate_hz_, feedback_timeout_ms_,
      feedback_poll_ms_, standby_retry_ms_, start_in_soft_mode_ ? "true" : "false");

    const auto period = std::chrono::duration<double>(1.0 / loop_rate_hz_);
    loop_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&CanGatewayNode::loop_timer_callback, this));

    standby_timer_ = create_wall_timer(
      std::chrono::milliseconds(standby_retry_ms_),
      std::bind(&CanGatewayNode::standby_retry_callback, this));

    standby_initial_timer_ = create_wall_timer(
      std::chrono::nanoseconds(1),
      [this]() {
        if (standby_initial_timer_) {
          standby_initial_timer_->cancel();
          standby_initial_timer_.reset();
        }
        standby_retry_callback();
      });
  }

  /** Disable every drive and close the shared CAN socket. */
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
  /** Return the drive channel for can_id, or nullptr if unknown. */
  DriveChannel * find_drive_by_can_id(int can_id)
  {
    for (auto & ch : drives_) {
      if (ch.can_id == can_id) {
        return &ch;
      }
    }
    return nullptr;
  }

  /** Publish current soft_mode for one drive (for late subscribers after connect). */
  void publish_drive_soft_mode(const DriveChannel & ch)
  {
    if (!ch.soft_mode_pub) {
      return;
    }
    std_msgs::msg::Bool msg;
    msg.data = ch.soft_mode;
    ch.soft_mode_pub->publish(msg);
  }

  /** Publish soft_mode on every namespace (boot default or after late motor connect). */
  void publish_all_soft_mode_state()
  {
    for (const auto & ch : drives_) {
      publish_drive_soft_mode(ch);
    }
  }

  /** Publish /boom_stack/ready and log transitions. */
  void set_stack_ready(bool ready)
  {
    if (stack_ready_ == ready) {
      return;
    }
    stack_ready_ = ready;
    std_msgs::msg::Bool msg;
    msg.data = ready;
    stack_ready_pub_->publish(msg);
    RCLCPP_INFO(
      get_logger(), "[STACK] boom_stack ready=%s", ready ? "true" : "false");
  }

  /** Sync stack_ready with drive health; republish soft_mode when becoming ready. */
  void update_stack_ready_state()
  {
    const bool ready = all_drives_startup_ready();
    if (ready) {
      if (!comm_fault_checks_armed_) {
        comm_fault_checks_armed_ = true;
        RCLCPP_INFO(get_logger(), "[STARTUP] All drives recovered; comm fault checks armed.");
      }
      if (start_in_soft_mode_ && !republished_soft_mode_on_connect_) {
        publish_all_soft_mode_state();
        republished_soft_mode_on_connect_ = true;
        RCLCPP_INFO(get_logger(), "[STARTUP] Republished soft_mode for all drives.");
      }
    }
    set_stack_ready(ready);
  }

  /** Open/bind one raw SocketCAN socket with a large RX buffer; no per-drive filters. */
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

  /** Write one CAN frame; returns false if the socket is closed or write is short. */
  bool write_frame(const struct can_frame & frame)
  {
    if (can_socket_ < 0) {
      return false;
    }
    return write(can_socket_, &frame, sizeof(frame)) == static_cast<ssize_t>(sizeof(frame));
  }

  /** Non-blocking drain of the RX queue into route_feedback_frame. */
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

  /** Block up to poll_timeout_ms waiting for CAN frames and route any MIT replies. */
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

  /**
   * Match a CAN frame to a drive MIT reply, update freshness/fault state, and publish
   * motor_state unless suppress_state_publish is set (e.g. during origin reset).
   */
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
      ch.connect_state = ConnectState::Connected;
      ch.last_position_rad = fb.position_rad;
      ch.has_last_position = true;

      ch.last_feedback = fb;
      ch.has_last_feedback = true;

      if (first) {
        RCLCPP_INFO(
          get_logger(), "[%s] can_id=%d MIT feedback active.", ch.ns.c_str(), ch.can_id);
      }

      if (!ch.suppress_state_publish) {
        publish_state(ch, fb);
      }
      return true;
    }
    return false;
  }

  /** Publish MotorState from the latest MIT feedback for this drive. */
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

  /**
   * Pack and transmit one MIT impedance command. force_apply bypasses command clamping
   * used for holds / soft-mode / zero-hold paths.
   */
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

  /** Send MIT enable (0xFC). Optional phase string is included in the startup log line. */
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

  /** Send MIT disable (0xFD); best-effort, used on node shutdown. */
  void send_disable(const DriveChannel & ch)
  {
    struct can_frame frame{};
    cm_interface::pack_control_frame(frame, ch.can_id, 0xFD);
    write_frame(frame);
  }

  /**
   * Non-blocking soft_mode off origin reset: one step per drive per loop tick.
   */
  void step_soft_origin_resets()
  {
    const int max_wait_ms = std::max(
      startup_origin_poll_ms_ * 2,
      feedback_poll_ms_ * 10);

    for (auto & ch : drives_) {
      if (ch.origin_reset_state != SoftOriginResetState::Active) {
        continue;
      }

      if (!ch.origin_set_origin_sent) {
        ch.suppress_state_publish = true;
        struct can_frame frame{};
        cm_interface::pack_control_frame(frame, ch.can_id, 0xFE);
        if (write_frame(frame)) {
          RCLCPP_INFO(get_logger(), "[STARTUP] [%s] set motor origin.", ch.ns.c_str());
        } else {
          RCLCPP_ERROR(get_logger(), "[STARTUP] [%s] set origin failed.", ch.ns.c_str());
        }
        ch.origin_set_origin_sent = true;
        ch.origin_reset_deadline =
          SteadyClock::now() + std::chrono::milliseconds(max_wait_ms);
        send_mit(ch, 0.0f, 0.0f, 0.0f, ch.profile.mit_kd, 0.0f, true);
        continue;
      }

      send_startup_mit_hold(ch);

      const bool near_origin = ch.has_last_position &&
        std::fabs(ch.last_position_rad) <= kPostOriginPositionTolRad;
      const bool timed_out = SteadyClock::now() >= ch.origin_reset_deadline;

      if (!near_origin && !timed_out) {
        continue;
      }

      if (!near_origin) {
        RCLCPP_WARN(
          get_logger(),
          "[STARTUP] [%s] post-origin position not near zero (%.4f rad); "
          "downstream may see stale feedback",
          ch.ns.c_str(), ch.has_last_position ? ch.last_position_rad : 0.0f);
      } else {
        RCLCPP_INFO(
          get_logger(),
          "[STARTUP] [%s] post-origin position %.4f rad",
          ch.ns.c_str(), ch.last_position_rad);
      }

      ch.origin_reset_state = SoftOriginResetState::Idle;
      ch.origin_set_origin_sent = false;
      ch.suppress_state_publish = false;
      publish_cached_state(ch);
      RCLCPP_INFO(
        get_logger(), "[%s] soft_mode off: origin reset at current position", ch.ns.c_str());
    }
  }

  /** Send set-origin (0xFE) then briefly poll for the resulting MIT reply. */
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

  /** Longer post-enable settle for AK80-64 knees; default settle for other drives. */
  int enable_settle_ms_for(const DriveChannel & ch) const
  {
    return is_ak80_drive(ch) ? ak80_enable_settle_ms_ : enable_settle_ms_;
  }

  /** Publish last MIT feedback, or ping with a Kd hold if none is cached yet. */
  void publish_cached_state(DriveChannel & ch)
  {
    if (ch.has_last_feedback) {
      publish_state(ch, ch.last_feedback);
      return;
    }
    send_startup_mit_hold(ch);
    poll_bus_feedback(feedback_poll_ms_);
  }

  /**
   * After set-origin, keep sending Kd hold until wrapped position is near zero so
   * subscribers do not see a large pre-origin jump.
   */
  bool wait_for_post_origin_position(DriveChannel & ch)
  {
    const int max_wait_ms = std::max(
      startup_origin_poll_ms_ * 2,
      feedback_poll_ms_ * 10);
    const auto deadline = SteadyClock::now() + std::chrono::milliseconds(max_wait_ms);
    const int period_ms = service_period_ms();

    while (rclcpp::ok() && SteadyClock::now() < deadline) {
      send_startup_mit_hold(ch);
      poll_bus_feedback(period_ms);
      if (ch.has_last_position &&
        std::fabs(ch.last_position_rad) <= kPostOriginPositionTolRad)
      {
        RCLCPP_INFO(
          get_logger(),
          "[STARTUP] [%s] post-origin position %.4f rad",
          ch.ns.c_str(), ch.last_position_rad);
        return true;
      }
    }

    RCLCPP_WARN(
      get_logger(),
      "[STARTUP] [%s] post-origin position not near zero (%.4f rad); "
      "downstream may see stale feedback",
      ch.ns.c_str(), ch.has_last_position ? ch.last_position_rad : 0.0f);
    return false;
  }

  /** MIT hold during connect: soft Kd when in soft_mode, else profile Kd. */
  void send_connect_hold(DriveChannel & ch)
  {
    if (ch.soft_mode) {
      send_mit(ch, 0.0f, 0.0f, 0.0f, kSoftModeKd, 0.0f, true);
    } else {
      send_startup_mit_hold(ch);
    }
  }

  /** Poll until first MIT feedback or deadline (used when skipping set-origin). */
  bool wait_for_first_feedback(DriveChannel & ch, int max_wait_ms)
  {
    const auto deadline = SteadyClock::now() + std::chrono::milliseconds(max_wait_ms);
    const int period_ms = service_period_ms();

    while (rclcpp::ok() && SteadyClock::now() < deadline) {
      send_connect_hold(ch);
      poll_bus_feedback(period_ms);
      if (ch.has_feedback) {
        return true;
      }
    }
    return ch.has_feedback;
  }

  /**
   * Enable one drive and wait for feedback. Skips set-origin when soft_mode is true;
   * origin reset happens when soft_mode is toggled off.
   */
  void connect_one_drive(DriveChannel & ch)
  {
    const int settle_ms = enable_settle_ms_for(ch);
    ch.connect_state = ConnectState::Connecting;
    ch.suppress_state_publish = true;
    send_enable(ch);
    std::this_thread::sleep_for(std::chrono::milliseconds(settle_ms));
    process_pending_rx();

    if (ch.soft_mode) {
      send_connect_hold(ch);
      wait_for_first_feedback(ch, startup_feedback_poll_ms() * 2);
    } else {
      send_set_origin(ch, startup_origin_poll_ms_);
      send_startup_mit_hold(ch);
      wait_for_post_origin_position(ch);
    }

    ch.suppress_state_publish = false;
    publish_cached_state(ch);
    ch.startup_enable_done = true;
    ch.connect_state = ch.has_feedback ? ConnectState::Connected : ConnectState::Fault;
    publish_drive_soft_mode(ch);
  }

  /**
   * Bus warmup (once), then full-stack reconnect in ascending can_id order.
   * If any drive needs reconnect, all drives are reset and re-enabled together.
   */
  void run_startup_connect_pass(bool apply_bus_warmup)
  {
    if (can_socket_ < 0) {
      return;
    }

    if (apply_bus_warmup && bus_warmup_ms_ > 0 && !bus_warmup_done_) {
      RCLCPP_INFO(
        get_logger(), "[STARTUP] Waiting %d ms for %s to settle before enable.",
        bus_warmup_ms_, can_interface_.c_str());
      std::this_thread::sleep_for(std::chrono::milliseconds(bus_warmup_ms_));
      bus_warmup_done_ = true;
    }

    if (!any_drive_needs_connect()) {
      update_stack_ready_state();
      return;
    }

    set_stack_ready(false);
    republished_soft_mode_on_connect_ = false;
    RCLCPP_INFO(
      get_logger(),
      "[STANDBY] Full-stack reconnect (motors or CAN may not be ready yet)...");

    for (auto & ch : drives_) {
      reset_drive_for_reconnect(ch);
      if (start_in_soft_mode_) {
        ch.soft_mode = true;
      }
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
      "[STARTUP] Connect order by can_id (knee/AK80-64 last): %s", order_log.c_str());

    for (size_t i = 0; i < order.size(); ++i) {
      if (i > 0 && startup_stagger_ms_ > 0) {
        maintain_startup_drives(startup_stagger_ms_);
      }
      connect_one_drive(drives_[order[i]]);
    }

    for (auto & ch : drives_) {
      if (drive_needs_connect(ch)) {
        RCLCPP_WARN(
          get_logger(),
          "[STARTUP] [%s] can_id=%d no MIT feedback yet; retrying connect sequence.",
          ch.ns.c_str(), ch.can_id);
        reset_drive_for_reconnect(ch);
        if (start_in_soft_mode_) {
          ch.soft_mode = true;
        }
        connect_one_drive(ch);
      }
      RCLCPP_INFO(
        get_logger(), "[STARTUP] %s can_id=%d feedback=%s",
        ch.ns.c_str(), ch.can_id, ch.has_feedback ? "active" : "missing");
    }

    refresh_all_drive_feedback();
    update_stack_ready_state();
  }

  /** True if at least one drive needs enable/reconnect. */
  bool any_drive_needs_connect() const
  {
    for (const auto & ch : drives_) {
      if (drive_needs_connect(ch)) {
        return true;
      }
    }
    return false;
  }

  /** Nominal service-loop period in ms (ceil of 1000 / loop_rate_hz). */
  int service_period_ms() const
  {
    return std::max(1, static_cast<int>(std::ceil(1000.0 / loop_rate_hz_)));
  }

  /** Conservative RX wait used when sizing startup refresh deadlines. */
  int startup_feedback_poll_ms() const
  {
    return std::max({feedback_timeout_ms_, startup_origin_poll_ms_, feedback_poll_ms_});
  }

  /** MIT zero-position Kd hold used to keep a drive engaged and elicit feedback. */
  void send_startup_mit_hold(DriveChannel & ch)
  {
    send_mit(ch, 0.0f, 0.0f, 0.0f, ch.profile.mit_kd, 0.0f, true);
  }

  /**
   * During stagger gaps / refresh, keep already-enabled drives alive with periodic
   * MIT Kd holds and RX polls so they are not starved on the shared bus.
   */
  void maintain_startup_drives(int duration_ms)
  {
    if (duration_ms <= 0) {
      return;
    }

    const auto deadline = SteadyClock::now() + std::chrono::milliseconds(duration_ms);
    const int period_ms = service_period_ms();
    while (rclcpp::ok() && SteadyClock::now() < deadline) {
      process_pending_rx();
      for (auto & ch : drives_) {
        if (ch.startup_enable_done) {
          send_connect_hold(ch);
        }
      }
      poll_bus_feedback(period_ms);
    }
    process_pending_rx();
  }

  /** True if this drive has MIT feedback newer than feedback_timeout_ms. */
  bool feedback_is_fresh(const DriveChannel & ch) const
  {
    if (!ch.has_feedback) {
      return false;
    }
    const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      SteadyClock::now() - ch.last_feedback_time).count();
    return age_ms <= feedback_timeout_ms_;
  }

  /** True when every drive finished enable and currently has fresh MIT feedback. */
  bool all_drives_startup_ready() const
  {
    for (const auto & ch : drives_) {
      if (!ch.startup_enable_done || !ch.has_feedback || !feedback_is_fresh(ch)) {
        return false;
      }
    }
    return !drives_.empty();
  }

  /** True when a drive needs enable/reconnect (cold start, power cycle, or comm fault). */
  bool drive_needs_connect(const DriveChannel & ch) const
  {
    if (!ch.startup_enable_done) {
      return true;
    }
    if (!ch.has_feedback) {
      return true;
    }
    if (ch.comm_fault) {
      return true;
    }
    if (!feedback_is_fresh(ch)) {
      return true;
    }
    return false;
  }

  /** Clear stale connection state before a standby reconnect attempt. */
  void reset_drive_for_reconnect(DriveChannel & ch)
  {
    ch.has_feedback = false;
    ch.comm_fault = false;
    ch.connect_state = ConnectState::Disconnected;
    ch.startup_enable_done = false;
    ch.has_pending_command = false;
    ch.has_last_feedback = false;
    ch.has_last_position = false;
    ch.origin_reset_state = SoftOriginResetState::Idle;
    ch.origin_set_origin_sent = false;
  }

  /**
   * After staggered enable, loop MIT hold + RX until all drives report fresh feedback
   * (or retry enable). Arms comm-fault checks only when every drive is ready.
   */
  void refresh_all_drive_feedback()
  {
    RCLCPP_INFO(
      get_logger(),
      "[STARTUP] Refreshing MIT feedback from all drives before service loop.");

    const int period_ms = service_period_ms();
    const int max_refresh_ms = std::max(
      startup_feedback_poll_ms() * static_cast<int>(drives_.size()),
      feedback_timeout_ms_ * 2);
    const auto deadline = SteadyClock::now() + std::chrono::milliseconds(max_refresh_ms);

    while (rclcpp::ok() && SteadyClock::now() < deadline) {
      process_pending_rx();
      for (auto & ch : drives_) {
        if (ch.startup_enable_done) {
          send_connect_hold(ch);
        }
      }
      poll_bus_feedback(period_ms);
      if (all_drives_startup_ready()) {
        break;
      }
    }
    process_pending_rx();

    if (!all_drives_startup_ready()) {
      for (auto & ch : drives_) {
        if (!ch.has_feedback || !feedback_is_fresh(ch)) {
          RCLCPP_WARN(
            get_logger(),
            "[STARTUP] [%s] can_id=%d %s after refresh; retrying enable sequence.",
            ch.ns.c_str(), ch.can_id,
            ch.has_feedback ? "feedback still stale" : "no MIT feedback");
          reset_drive_for_reconnect(ch);
          if (start_in_soft_mode_) {
            ch.soft_mode = true;
          }
          connect_one_drive(ch);
        }
      }
      maintain_startup_drives(feedback_timeout_ms_ * 2);
    }

    bool all_initiated = all_drives_startup_ready();
    for (const auto & ch : drives_) {
      if (!ch.has_feedback) {
        all_initiated = false;
        RCLCPP_WARN(
          get_logger(),
          "[STARTUP] [%s] can_id=%d still no MIT feedback after refresh.",
          ch.ns.c_str(), ch.can_id);
      } else if (!feedback_is_fresh(ch)) {
        all_initiated = false;
        RCLCPP_WARN(
          get_logger(),
          "[STARTUP] [%s] can_id=%d feedback not fresh after refresh.",
          ch.ns.c_str(), ch.can_id);
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
      RCLCPP_WARN(
        get_logger(),
        "[STARTUP] No MIT feedback yet; standby will retry every %d ms.",
        standby_retry_ms_);
    } else if (all_initiated) {
      RCLCPP_INFO(get_logger(), "[STARTUP] All motors successfully initiated.");
    } else {
      RCLCPP_WARN(
        get_logger(),
        "[STARTUP] Not all drives have fresh feedback; comm fault checks deferred "
        "until the service loop recovers them.");
    }

    comm_fault_checks_armed_ = all_initiated;
  }

  /**
   * Periodic retry: open CAN if down, then connect any drives still missing feedback.
   * Keeps the node alive when motors are powered after the Pi boots.
   */
  void standby_retry_callback()
  {
    if (startup_in_progress_) {
      return;
    }
    startup_in_progress_ = true;

    if (can_socket_ < 0) {
      RCLCPP_INFO(
        get_logger(), "[STANDBY] Attempting CAN connect on %s...", can_interface_.c_str());
      if (!open_can_socket()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), standby_retry_ms_,
          "[STANDBY] CAN interface '%s' not available; retrying every %d ms.",
          can_interface_.c_str(), standby_retry_ms_);
        startup_in_progress_ = false;
        return;
      }
      RCLCPP_INFO(get_logger(), "[STANDBY] CAN connected on %s.", can_interface_.c_str());
    }

    if (all_drives_startup_ready()) {
      if (!comm_fault_checks_armed_) {
        comm_fault_checks_armed_ = true;
        RCLCPP_INFO(get_logger(), "[STANDBY] All drives ready; comm fault checks armed.");
      }
      update_stack_ready_state();
      startup_in_progress_ = false;
      return;
    }

    republished_soft_mode_on_connect_ = false;
    run_startup_connect_pass(!bus_warmup_done_);
    startup_in_progress_ = false;
  }

  /** Latch a one-shot comm fault once feedback has gone stale after first contact. */
  void mark_comm_fault(DriveChannel & ch)
  {
    if (!ch.has_feedback || ch.comm_fault) {
      return;
    }
    ch.comm_fault = true;
    ch.connect_state = ConnectState::Fault;
    republished_soft_mode_on_connect_ = false;
    set_stack_ready(false);
    RCLCPP_ERROR(
      get_logger(),
      "[%s] can_id=%d comm fault: no fresh MIT feedback for %d ms; zero hold",
      ch.ns.c_str(), ch.can_id, feedback_timeout_ms_);
  }

  /**
   * Per-tick TX priority: soft-mode damping, then zero-hold on comm fault / stale
   * feedback, then pending MotorCommand, else profile Kd hold.
   */
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

  /**
   * Service loop: drain RX, apply pending soft-origin resets, poll for feedback, then
   * TX each drive. Freshness used for comm fault is from this pre-TX poll.
   */
  void loop_timer_callback()
  {
    if (can_socket_ < 0) {
      return;
    }

    process_pending_rx();
    step_soft_origin_resets();
    poll_bus_feedback(feedback_poll_ms_);

    if (stack_ready_ && any_drive_needs_connect()) {
      set_stack_ready(false);
      republished_soft_mode_on_connect_ = false;
    } else if (!stack_ready_ && all_drives_startup_ready()) {
      update_stack_ready_state();
    }

    for (auto & ch : drives_) {
      service_drive_tx(ch);
    }
  }

  /** Cache the latest MotorCommand for later TX; ignored while soft_mode is active. */
  void on_motor_command(
    int can_id, const motor_interfaces::msg::MotorCommand::SharedPtr msg)
  {
    if (!stack_ready_) {
      return;
    }
    DriveChannel * ch = find_drive_by_can_id(can_id);
    if (ch == nullptr || ch->soft_mode) {
      return;
    }
    ch->pending_command = *msg;
    ch->has_pending_command = true;
  }

  /**
   * Soft on: clear pending cmds and apply soft Kd. Soft off: queue set-origin at the
   * current pose (handled in process_pending_soft_origin_resets).
   */
  void on_soft_mode(int can_id, const std_msgs::msg::Bool::SharedPtr msg)
  {
    DriveChannel * ch = find_drive_by_can_id(can_id);
    if (ch == nullptr || ch->soft_mode == msg->data) {
      return;
    }

    if (!msg->data && !stack_ready_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "[%s] Ignoring soft_mode=false while stack is not ready.",
        ch->ns.c_str());
      return;
    }

    ch->soft_mode = msg->data;
    if (ch->soft_mode) {
      ch->soft_mode_on_position_rad = ch->last_position_rad;
      ch->has_soft_mode_on_position = ch->has_last_position;
      ch->has_pending_command = false;
      ch->origin_reset_state = SoftOriginResetState::Idle;
      ch->origin_set_origin_sent = false;
      send_mit(*ch, 0.0f, 0.0f, 0.0f, kSoftModeKd, 0.0f, true);
    } else {
      ch->has_pending_command = false;
      ch->origin_reset_state = SoftOriginResetState::Active;
      ch->origin_set_origin_sent = false;
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
  int standby_retry_ms_{kDefaultStandbyRetryMs};
  bool start_in_soft_mode_{true};

  std::vector<DriveChannel> drives_;
  int can_socket_{-1};
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stack_ready_pub_;
  rclcpp::TimerBase::SharedPtr loop_timer_;
  rclcpp::TimerBase::SharedPtr standby_timer_;
  rclcpp::TimerBase::SharedPtr standby_initial_timer_;
  bool comm_fault_checks_armed_{false};
  bool startup_in_progress_{false};
  bool bus_warmup_done_{false};
  bool republished_soft_mode_on_connect_{false};
  bool stack_ready_{false};
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
