// can_gateway_node: one SocketCAN interface, multiple MIT drives.
// Replaces one motor_node_continuous per namespace for multi-motor stacks.
//
// Parameters (TWEAK via launch or ros2 param):
//   can_interface          — SocketCAN device (e.g. can0)
//   namespaces,motor_models,can_ids — comma-separated lists, same length
//   loop_rate_hz           — TX/RX service rate (default 200)
//   feedback_timeout_ms    — stale feedback → comm fault Kd hold until recovery
//   feedback_poll_ms       — blocking RX poll each loop iteration
//   startup_stagger_ms     — delay between per-drive enable sequences
//   enable_settle_ms       — post-enable wait (non-AK80 drives)
//   ak80_enable_settle_ms  — post-enable wait for AK80-64 knee
//   startup_origin_poll_ms — RX wait after set-origin (0xFE)
//   bus_warmup_ms          — delay after bind before first enable
//   alive_check_period_ms  — how often to test that every drive has fresh MIT feedback
//   reconnect_cooldown_ms  — min time between reconnect attempts (failed init / power off)
//
// Startup / reconnect: enable (0xFC) all drives by can_id, wait until connected,
// then run the same structure as the controller set-origin button (soft_mode
// true → wait → soft_mode false → set-origin+enable → despos=0 + hold_joint).
// motors_were_down_ stays true until that full sequence succeeds; feedback alone
// does not clear it or skip reconnect.
// Soft-mode off (manual or reconnect) triggers per-drive set-origin + enable.
// loop_timer_callback: drain RX, poll, maybe reconnect while motors_were_down_, MIT TX.
// Comm-fault uses last_feedback_time from the pre-service poll, not from TX this tick.

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
#include "std_msgs/msg/float32.hpp"

namespace
{

// Light damping while a drive is in soft_mode (Kp=0, Tff=0) so the joint can
// be moved by hand without going fully limp.
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
constexpr int kDefaultAliveCheckPeriodMs = 500;
constexpr int kDefaultReconnectCooldownMs = 2000;
// Matches joint_position_sequence set-origin button sleeps.
constexpr int kSetOriginButtonPhaseMs = 100;
// Kernel default SO_RCVBUF is too small for a shared bus at 200 Hz × N drives.
constexpr int kDefaultCanRxBufferBytes = 1 << 20;
// Wrapped motor position must be within this after set-origin before motor_state is published.
constexpr float kPostOriginPositionTolRad = 0.15f;

using SteadyClock = std::chrono::steady_clock;
using SteadyTime = SteadyClock::time_point;

/** Split a comma-separated parameter, trimming whitespace and skipping empty tokens. */
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

/**
 * Read a CSV launch parameter that may arrive as a string or a single integer.
 * Launch `can_ids:=1` is INTEGER; the node always consumes a CSV string.
 */
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

/**
 * Shared SocketCAN gateway for multiple MIT drives.
 *
 * One raw socket, no per-drive hardware filters. ROS topics are namespaced
 * per drive (`/<ns>/motor_command`, `/<ns>/motor_state`, `/<ns>/soft_mode`,
 * `/<ns>/hold_joint`, `/<ns>/joint_despos`, `/<ns>/origin_reset`).
 * Command callbacks only latch the latest message; TX happens on the wall timer
 * so MIT frames stay paced and origin-reset stays off the subscription thread.
 */
class CanGatewayNode : public rclcpp::Node
{
public:
  /** Per-drive ROS I/O and MIT runtime state on the shared CAN socket. */
  struct DriveChannel
  {
    std::string ns;
    int can_id{0};
    cm_interface::MotorMitProfile profile{cm_interface::kAk70_10};

    rclcpp::Publisher<motor_interfaces::msg::MotorState>::SharedPtr state_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr origin_reset_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr soft_mode_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr hold_joint_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr joint_despos_pub;
    rclcpp::Subscription<motor_interfaces::msg::MotorCommand>::SharedPtr cmd_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr soft_sub;

    motor_interfaces::msg::MotorCommand pending_command;
    bool has_pending_command{false};
    bool soft_mode{false};

    SteadyTime last_feedback_time{};
    bool has_feedback{false};
    // Sticky until a MIT reply arrives; TX becomes a Kd hold (not user commands).
    bool comm_fault{false};
    float last_position_rad{0.0f};
    bool has_last_position{false};
    // Snapshot at soft_mode entry; not consumed by TX (origin is set on falling edge).
    float soft_mode_on_position_rad{0.0f};
    bool has_soft_mode_on_position{false};
    // Falling edge of soft_mode is handled on the timer: set-origin is blocking.
    bool pending_soft_origin_reset{false};
    bool startup_enable_done{false};
    // Hide pre-origin wrap from motor_state so unwrapper/translator do not jump.
    bool suppress_state_publish{false};
    bool has_last_feedback{false};
    cm_interface::MitFeedback last_feedback{};
  };

  /**
   * Load drive lists, bind SocketCAN, run the enable/origin sequence, then
   * start the TX/RX wall timer. Throws if parameters are invalid or bind fails.
   */
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
    alive_check_period_ms_ = declare_parameter<int>(
      "alive_check_period_ms", kDefaultAliveCheckPeriodMs);
    reconnect_cooldown_ms_ = declare_parameter<int>(
      "reconnect_cooldown_ms", kDefaultReconnectCooldownMs);

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
    if (alive_check_period_ms_ < 0) {
      throw std::invalid_argument("alive_check_period_ms must be >= 0");
    }
    if (reconnect_cooldown_ms_ < 0) {
      throw std::invalid_argument("reconnect_cooldown_ms must be >= 0");
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
      if (ch.can_id < 0 || ch.can_id > 0x7FF) {  // 11-bit standard CAN ID
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

    // Depth 1: only the latest motor_command matters; MIT TX is timer-paced.
    // Lambdas capture can_id by value, not &ch — DriveChannel entries can move.
    const auto cmd_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    for (auto & ch : drives_) {
      const std::string prefix = "/" + ch.ns;
      ch.state_pub = create_publisher<motor_interfaces::msg::MotorState>(
        prefix + "/motor_state", 10);
      ch.origin_reset_pub = create_publisher<std_msgs::msg::Bool>(
        prefix + "/origin_reset", 10);
      ch.soft_mode_pub = create_publisher<std_msgs::msg::Bool>(
        prefix + "/soft_mode", 10);
      ch.hold_joint_pub = create_publisher<std_msgs::msg::Bool>(
        prefix + "/hold_joint", 10);
      ch.joint_despos_pub = create_publisher<std_msgs::msg::Float32>(
        prefix + "/joint_despos", 10);

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
      "%s | %zu drives | loop %.0f Hz | feedback_timeout %d ms | feedback_poll %d ms | "
      "alive_check %d ms | reconnect_cooldown %d ms",
      can_interface_.c_str(), drives_.size(), loop_rate_hz_, feedback_timeout_ms_,
      feedback_poll_ms_, alive_check_period_ms_, reconnect_cooldown_ms_);

    startup_all_drives();

    const auto period = std::chrono::duration<double>(1.0 / loop_rate_hz_);
    loop_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&CanGatewayNode::loop_timer_callback, this));
  }

  /** Best-effort disable all drives and close the socket. */
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
  /** Linear lookup is enough: typical stacks have 2–4 drives. */
  DriveChannel * find_drive_by_can_id(int can_id)
  {
    for (auto & ch : drives_) {
      if (ch.can_id == can_id) {
        return &ch;
      }
    }
    return nullptr;
  }

  /**
   * Bind one unfiltered RAW socket to can_interface_.
   * Frames are demuxed in software by unpack_reply (can_id + MIT payload).
   */
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

  /** Write one CAN frame; false if the socket is closed or write is short. */
  bool write_frame(const struct can_frame & frame)
  {
    if (can_socket_ < 0) {
      return false;
    }
    return write(can_socket_, &frame, sizeof(frame)) == static_cast<ssize_t>(sizeof(frame));
  }

  /** Non-blocking drain of frames already in the kernel RX queue. */
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

  /**
   * Block up to poll_timeout_ms waiting for MIT replies.
   * Used after TX so freshness is based on this poll, not on TX this tick.
   */
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
          continue;  // signal during poll; remaining deadline still applies
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
   * Demux one RX frame onto the first drive whose MIT reply unpacks.
   * Clears comm_fault and optionally publishes motor_state.
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

  /** Publish MIT feedback as motor_state on /<ns>/motor_state. */
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

  /** Arm unwrapper/translator to re-latch despos after a set-origin. */
  void publish_origin_reset(const DriveChannel & ch)
  {
    if (ch.origin_reset_pub == nullptr) {
      return;
    }
    std_msgs::msg::Bool msg;
    msg.data = true;
    ch.origin_reset_pub->publish(msg);
  }

  /**
   * Pack and TX one MIT command.
   * force_apply sets the position-apply bit even for a zero delta (hold / origin).
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

  /** MIT enable (0xFC). Optional phase string is logged for startup diagnostics. */
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

  /** MIT disable (0xFD). Best-effort; used from the destructor. */
  void send_disable(const DriveChannel & ch)
  {
    struct can_frame frame{};
    cm_interface::pack_control_frame(frame, ch.can_id, 0xFD);
    write_frame(frame);
  }

  /**
   * Handle soft_mode falling edges on the timer thread.
   * Suppress motor_state until post-origin position is near zero so downstream
   * nodes do not integrate the wrap as motion.
   */
  void process_pending_soft_origin_resets()
  {
    for (auto & ch : drives_) {
      if (!ch.pending_soft_origin_reset) {
        continue;
      }
      ch.pending_soft_origin_reset = false;
      ch.suppress_state_publish = true;
      process_pending_rx();
      send_set_origin(ch, startup_origin_poll_ms_);
      // 0xFE does not guarantee MIT run mode; re-enable even if feedback continues.
      send_enable(ch, "soft_mode off");
      send_mit(ch, 0.0f, 0.0f, 0.0f, ch.profile.mit_kd, 0.0f, true);
      wait_for_post_origin_position(ch);
      ch.suppress_state_publish = false;
      publish_cached_state(ch);
      RCLCPP_INFO(
        get_logger(), "[%s] soft_mode off: origin reset at current position", ch.ns.c_str());
    }
  }

  /** MIT set-origin (0xFE) then wait for a reply so last_position is current. */
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

  /** AK80-64 (knee) needs a longer post-enable settle than AK70/AK10. */
  bool is_ak80_drive(const DriveChannel & ch) const
  {
    return std::strcmp(ch.profile.name, "AK80-64") == 0;
  }

  /** Post-enable wait: AK80-64 uses ak80_enable_settle_ms_, others enable_settle_ms_. */
  int enable_settle_ms_for(const DriveChannel & ch) const
  {
    return is_ak80_drive(ch) ? ak80_enable_settle_ms_ : enable_settle_ms_;
  }

  /** Publish last MIT sample, or ping the drive once if none is cached yet. */
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
   * After 0xFE the encoder wrap is not instant. Hold every enabled drive with
   * profile Kd (siblings must not starve) until |position| is within tolerance.
   */
  bool wait_for_post_origin_position(DriveChannel & ch)
  {
    const int max_wait_ms = std::max(
      startup_origin_poll_ms_ * 2,
      feedback_poll_ms_ * 10);
    const auto deadline = SteadyClock::now() + std::chrono::milliseconds(max_wait_ms);
    const int period_ms = service_period_ms();

    while (rclcpp::ok() && SteadyClock::now() < deadline) {
      for (auto & other : drives_) {
        if (other.startup_enable_done) {
          send_startup_mit_hold(other);
        }
      }
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

  /** Enable → settle with MIT holds on all enabled drives. No set-origin. */
  void enable_one_drive(DriveChannel & ch, const char * phase = nullptr)
  {
    const int settle_ms = enable_settle_ms_for(ch);
    send_enable(ch, phase);
    // Mark done before settle so maintain_startup_drives includes this drive.
    ch.startup_enable_done = true;
    if (settle_ms > 0) {
      maintain_startup_drives(settle_ms);
    }
    process_pending_rx();
    send_startup_mit_hold(ch);
    poll_bus_feedback(feedback_poll_ms_);
  }

  /** Clear runtime state so a drive is re-enabled from scratch. */
  void reset_drive_for_reinit(DriveChannel & ch)
  {
    ch.startup_enable_done = false;
    ch.has_feedback = false;
    ch.comm_fault = false;
    ch.has_last_feedback = false;
    ch.has_pending_command = false;
  }

  /** Indices sorted by ascending can_id (AK80/knee last). */
  std::vector<size_t> drive_order_by_can_id() const
  {
    std::vector<size_t> order(drives_.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [this](size_t a, size_t b) {
      return drives_[a].can_id < drives_[b].can_id;
    });
    return order;
  }

  /** Comma-separated namespace list for reconnect logs. */
  std::string join_drive_names(const std::vector<size_t> & indices) const
  {
    if (indices.empty()) {
      return "-";
    }
    std::string out;
    for (size_t i = 0; i < indices.size(); ++i) {
      if (i > 0) {
        out += ", ";
      }
      out += drives_[indices[i]].ns;
    }
    return out;
  }

  /** Names of drives that do not currently have fresh MIT feedback. */
  std::vector<size_t> missing_drive_indices() const
  {
    std::vector<size_t> missing;
    for (size_t i = 0; i < drives_.size(); ++i) {
      if (!feedback_is_fresh(drives_[i])) {
        missing.push_back(i);
      }
    }
    return missing;
  }

  /** Enable every drive in can_id order; stagger with MIT holds on already-enabled. */
  void enable_all_drives_in_can_id_order(const char * tag)
  {
    const std::vector<size_t> order = drive_order_by_can_id();

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
      "%s Enable order by can_id (knee/AK80-64 last): %s", tag, order_log.c_str());

    for (size_t i = 0; i < order.size(); ++i) {
      if (i > 0 && startup_stagger_ms_ > 0) {
        maintain_startup_drives(startup_stagger_ms_);
      }
      enable_one_drive(drives_[order[i]]);
    }
  }

  /** Enable-only retry: always re-send 0xFC; feedback does not imply run mode. */
  void retry_drives_without_feedback(const char * tag)
  {
    for (auto & ch : drives_) {
      if (!ch.has_feedback) {
        RCLCPP_WARN(
          get_logger(),
          "%s [%s] can_id=%d no MIT feedback yet; retrying enable.",
          tag, ch.ns.c_str(), ch.can_id);
        enable_one_drive(ch, "retry");
      } else {
        // Still re-enable: a drive can reply while not accepting MIT position cmds.
        send_enable(ch, "feedback present; force enable");
        send_startup_mit_hold(ch);
      }
      RCLCPP_INFO(
        get_logger(), "%s %s can_id=%d feedback=%s",
        tag, ch.ns.c_str(), ch.can_id, ch.has_feedback ? "active" : "missing");
    }
    poll_bus_feedback(feedback_poll_ms_);
  }

  /** Publish Bool on a per-drive topic (soft_mode / hold_joint / origin_reset). */
  void publish_bool(
    const rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr & pub, bool value)
  {
    if (pub == nullptr) {
      return;
    }
    std_msgs::msg::Bool msg;
    msg.data = value;
    pub->publish(msg);
  }

  /**
   * Same structure as joint_position_sequence Back button: soft_mode true → wait →
   * soft_mode false (set-origin+enable) → wait → joint_despos=0 + hold_joint.
   * Updates gateway soft_mode state directly; publishes for unwrapper/translator/teleop.
   */
  void run_set_origin_button_sequence(const char * tag)
  {
    RCLCPP_INFO(
      get_logger(),
      "%s Running controller set-origin sequence (soft_mode pulse + despos/hold).",
      tag);

    for (auto & ch : drives_) {
      ch.soft_mode = true;
      ch.soft_mode_on_position_rad = ch.last_position_rad;
      ch.has_soft_mode_on_position = ch.has_last_position;
      ch.has_pending_command = false;
      send_mit(ch, 0.0f, 0.0f, 0.0f, kSoftModeKd, 0.0f, true);
      publish_bool(ch.soft_mode_pub, true);
    }
    maintain_soft_drives(kSetOriginButtonPhaseMs);

    for (auto & ch : drives_) {
      ch.soft_mode = false;
      ch.has_pending_command = false;
      ch.pending_soft_origin_reset = true;
      send_mit(ch, 0.0f, 0.0f, 0.0f, ch.profile.mit_kd, 0.0f, true);
      publish_bool(ch.soft_mode_pub, false);
    }
    process_pending_soft_origin_resets();
    maintain_startup_drives(kSetOriginButtonPhaseMs);

    for (auto & ch : drives_) {
      // Post-origin wrap ≈ 0; unwrapper resets total → joint curpos ≈ 0.
      if (ch.joint_despos_pub != nullptr) {
        std_msgs::msg::Float32 despos;
        despos.data = 0.0f;
        ch.joint_despos_pub->publish(despos);
      }
      publish_bool(ch.hold_joint_pub, true);
    }

    RCLCPP_INFO(
      get_logger(),
      "%s Set-origin button sequence complete (despos=0, hold_joint=true).", tag);
  }

  /** Soft Kd MIT on every enabled drive for duration_ms (set-origin button phase 1). */
  void maintain_soft_drives(int duration_ms)
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
          send_mit(ch, 0.0f, 0.0f, 0.0f, kSoftModeKd, 0.0f, true);
        }
      }
      poll_bus_feedback(period_ms);
    }
    process_pending_rx();
  }

  /** Keep MIT holds going until every drive has fresh feedback or timeout. */
  bool maintain_until_all_fresh(const char * tag)
  {
    const int max_ms = std::max(
      feedback_timeout_ms_ * 2,
      startup_feedback_poll_ms() * static_cast<int>(drives_.size()));
    const auto deadline = SteadyClock::now() + std::chrono::milliseconds(max_ms);
    const int period_ms = service_period_ms();

    while (rclcpp::ok() && SteadyClock::now() < deadline) {
      if (all_drives_startup_ready()) {
        return true;
      }
      maintain_startup_drives(period_ms);
    }

    if (!all_drives_startup_ready()) {
      RCLCPP_WARN(
        get_logger(),
        "%s Timed out waiting for fresh feedback on all drives after set-origin.",
        tag);
      return false;
    }
    return true;
  }

  /**
   * Enable all → wait for fresh feedback → controller set-origin button sequence.
   * motors_were_down_ clears only on full success. Feedback alone is not enough.
   */
  void startup_all_drives()
  {
    if (reconnect_in_progress_) {
      return;
    }
    reconnect_in_progress_ = true;
    comm_fault_checks_armed_ = false;
    motors_were_down_ = true;

    const std::vector<size_t> missing_before = missing_drive_indices();
    const bool is_reconnect = startup_succeeded_once_;
    const char * tag = is_reconnect ? "[RECONNECT]" : "[STARTUP]";

    if (is_reconnect) {
      RCLCPP_WARN(
        get_logger(),
        "[RECONNECT] Motors were down; running full reconnect. missing=[%s]",
        join_drive_names(missing_before).c_str());
    }

    for (auto & ch : drives_) {
      reset_drive_for_reinit(ch);
    }

    if (bus_warmup_ms_ > 0) {
      RCLCPP_INFO(
        get_logger(), "%s Waiting %d ms for %s to settle before enable.",
        tag, bus_warmup_ms_, can_interface_.c_str());
      std::this_thread::sleep_for(std::chrono::milliseconds(bus_warmup_ms_));
    }

    enable_all_drives_in_can_id_order(tag);
    retry_drives_without_feedback(tag);
    const bool all_connected = refresh_all_drive_feedback(tag);

    if (!all_connected) {
      RCLCPP_ERROR(
        get_logger(),
        "%s Not all motors connected; skipping set-origin. motors_were_down stays true.",
        tag);
      last_reconnect_time_ = SteadyClock::now();
      last_alive_check_time_ = last_reconnect_time_;
      reconnect_in_progress_ = false;
      return;
    }

    run_set_origin_button_sequence(tag);
    const bool all_ready = maintain_until_all_fresh(tag);

    if (all_ready) {
      motors_were_down_ = false;
      startup_succeeded_once_ = true;
      comm_fault_checks_armed_ = true;
      RCLCPP_INFO(
        get_logger(),
        "%s All motors successfully initiated; motors_were_down=false.", tag);
    } else {
      RCLCPP_WARN(
        get_logger(),
        "%s Post-origin feedback not fresh; motors_were_down stays true.",
        tag);
      comm_fault_checks_armed_ = false;
    }

    last_reconnect_time_ = SteadyClock::now();
    last_alive_check_time_ = last_reconnect_time_;
    reconnect_in_progress_ = false;
  }

  /** Wall-timer period in ms; used as the inner poll quantum during startup holds. */
  int service_period_ms() const
  {
    return std::max(1, static_cast<int>(std::ceil(1000.0 / loop_rate_hz_)));
  }

  /** Upper bound used to size the post-startup refresh window. */
  int startup_feedback_poll_ms() const
  {
    return std::max({feedback_timeout_ms_, startup_origin_poll_ms_, feedback_poll_ms_});
  }

  /** Zero-delta MIT with profile Kd; keeps the drive engaged without a ROS command. */
  void send_startup_mit_hold(DriveChannel & ch)
  {
    send_mit(ch, 0.0f, 0.0f, 0.0f, ch.profile.mit_kd, 0.0f, true);
  }

  // Drives need periodic MIT refresh to stay engaged; call during stagger gaps and refresh.
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
          send_startup_mit_hold(ch);
        }
      }
      poll_bus_feedback(period_ms);
    }
    process_pending_rx();
  }

  /** True if a MIT reply arrived within feedback_timeout_ms_. */
  bool feedback_is_fresh(const DriveChannel & ch) const
  {
    if (!ch.has_feedback) {
      return false;
    }
    const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      SteadyClock::now() - ch.last_feedback_time).count();
    return age_ms <= feedback_timeout_ms_;
  }

  /** Every drive has completed enable and has fresh MIT feedback. */
  bool all_drives_startup_ready() const
  {
    for (const auto & ch : drives_) {
      if (!ch.startup_enable_done || !ch.has_feedback || !feedback_is_fresh(ch)) {
        return false;
      }
    }
    return !drives_.empty();
  }

  /** Every drive has a MIT reply within feedback_timeout_ms_. */
  bool all_drives_alive() const
  {
    for (const auto & ch : drives_) {
      if (!feedback_is_fresh(ch)) {
        return false;
      }
    }
    return !drives_.empty();
  }

  /**
   * Periodic liveness check. Stale feedback sets motors_were_down_. While that
   * flag is true, always run full reconnect on cooldown — feedback alone is not
   * enough to call motors alive again.
   */
  void maybe_reconnect_lost_drives()
  {
    if (reconnect_in_progress_ || can_socket_ < 0) {
      return;
    }

    const auto now = SteadyClock::now();
    const auto since_check_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      now - last_alive_check_time_).count();
    if (since_check_ms < alive_check_period_ms_) {
      return;
    }
    last_alive_check_time_ = now;

    if (!all_drives_alive()) {
      if (!motors_were_down_) {
        RCLCPP_WARN(
          get_logger(),
          "[RECONNECT] Drive feedback lost; motors_were_down=true until full reconnect.");
      }
      motors_were_down_ = true;
    }

    if (!motors_were_down_) {
      return;
    }

    const auto since_reconnect_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      now - last_reconnect_time_).count();
    if (since_reconnect_ms < reconnect_cooldown_ms_) {
      return;
    }

    startup_all_drives();
  }

  /**
   * MIT hold + RX poll until every enabled drive has fresh feedback.
   * Retries are enable-only (no set-origin). Returns true if all connected.
   */
  bool refresh_all_drive_feedback(const char * tag)
  {
    RCLCPP_INFO(
      get_logger(),
      "%s Refreshing MIT feedback from all drives before set-origin.", tag);

    const int period_ms = service_period_ms();
    const int max_refresh_ms = std::max(
      startup_feedback_poll_ms() * static_cast<int>(drives_.size()),
      feedback_timeout_ms_ * 2);
    const auto deadline = SteadyClock::now() + std::chrono::milliseconds(max_refresh_ms);

    while (rclcpp::ok() && SteadyClock::now() < deadline) {
      process_pending_rx();
      for (auto & ch : drives_) {
        if (ch.startup_enable_done) {
          send_startup_mit_hold(ch);
        }
      }
      poll_bus_feedback(period_ms);
      if (all_drives_startup_ready()) {
        break;
      }
    }
    process_pending_rx();

    if (!all_drives_startup_ready()) {
      RCLCPP_WARN(
        get_logger(),
        "%s Not all drives fresh after refresh; re-enabling every drive.", tag);
      for (auto & ch : drives_) {
        enable_one_drive(ch, "refresh re-enable all");
      }
      maintain_startup_drives(feedback_timeout_ms_ * 2);
    }

    bool all_connected = all_drives_startup_ready();
    for (const auto & ch : drives_) {
      if (!ch.has_feedback) {
        all_connected = false;
        RCLCPP_WARN(
          get_logger(),
          "%s [%s] can_id=%d still no MIT feedback after refresh.",
          tag, ch.ns.c_str(), ch.can_id);
      } else if (!feedback_is_fresh(ch)) {
        all_connected = false;
        RCLCPP_WARN(
          get_logger(),
          "%s [%s] can_id=%d feedback not fresh after refresh.",
          tag, ch.ns.c_str(), ch.can_id);
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
        "%s Motor initilization failed, check power and CAN wiring.", tag);
    } else if (all_connected) {
      RCLCPP_INFO(get_logger(), "%s All motors connected.", tag);
    } else {
      RCLCPP_WARN(
        get_logger(),
        "%s Not all drives have fresh feedback after enable phase.", tag);
    }

    return all_connected;
  }

  /**
   * Latch a comm fault on first stale timeout. Cleared when a MIT reply arrives.
   * No-op if the drive never had feedback (startup still in progress).
   */
  void mark_comm_fault(DriveChannel & ch)
  {
    if (!ch.has_feedback || ch.comm_fault) {
      return;
    }
    ch.comm_fault = true;
    RCLCPP_ERROR(
      get_logger(),
      "[%s] can_id=%d comm fault: no fresh MIT feedback for %d ms; Kd hold until recovery",
      ch.ns.c_str(), ch.can_id, feedback_timeout_ms_);
  }

  /**
   * One MIT TX for this drive. Priority: soft_mode damping, comm-fault Kd hold
   * (recoverable; kp=kd=0 does not elicit replies), Kd hold until first feedback,
   * then the latched motor_command (or Kd hold).
   */
  void service_drive_tx(DriveChannel & ch)
  {
    if (ch.soft_mode) {
      send_mit(ch, 0.0f, 0.0f, 0.0f, kSoftModeKd, 0.0f, true);
      return;
    }

    if (ch.comm_fault) {
      send_startup_mit_hold(ch);
      return;
    }

    if (comm_fault_checks_armed_ && ch.has_feedback && !feedback_is_fresh(ch)) {
      mark_comm_fault(ch);
      send_startup_mit_hold(ch);
      return;
    }

    if (!ch.has_feedback) {
      send_startup_mit_hold(ch);
      return;
    }

    if (ch.has_pending_command) {
      const auto & cmd = ch.pending_command;
      send_mit(ch, cmd.position, cmd.velocity, cmd.kp, cmd.kd, cmd.torque, false);
      return;
    }

    send_startup_mit_hold(ch);
  }

  /**
   * Drain RX, run deferred origin resets, poll for replies, maybe reconnect, then TX.
   * Comm-fault freshness uses last_feedback_time from this poll, not from TX.
   */
  void loop_timer_callback()
  {
    if (can_socket_ < 0) {
      return;
    }

    process_pending_rx();
    process_pending_soft_origin_resets();
    // Poll before service_drive_tx so freshness reflects replies to recent MIT TX.
    poll_bus_feedback(feedback_poll_ms_);

    maybe_reconnect_lost_drives();

    if (!comm_fault_checks_armed_ && all_drives_startup_ready()) {
      comm_fault_checks_armed_ = true;
      RCLCPP_INFO(get_logger(), "[STARTUP] All drives recovered; comm fault checks armed.");
    }

    for (auto & ch : drives_) {
      service_drive_tx(ch);
    }
  }

  /** Latch the latest command. Ignored in soft_mode (joint is hand-moved). */
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

  /**
   * Rising edge: damp with kSoftModeKd and drop pending commands.
   * Falling edge: Kd hold now; set-origin is deferred to the timer thread.
   */
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
  int alive_check_period_ms_{kDefaultAliveCheckPeriodMs};
  int reconnect_cooldown_ms_{kDefaultReconnectCooldownMs};

  std::vector<DriveChannel> drives_;
  int can_socket_{-1};
  rclcpp::TimerBase::SharedPtr loop_timer_;
  // Stay false until every drive has fresh feedback so startup stagger is not
  // treated as a comm fault.
  bool comm_fault_checks_armed_{false};
  bool reconnect_in_progress_{false};
  // True until a full enable + set-origin-button sequence succeeds. Feedback
  // returning alone does not clear this or skip reconnect.
  bool motors_were_down_{true};
  bool startup_succeeded_once_{false};
  SteadyTime last_alive_check_time_{};
  SteadyTime last_reconnect_time_{};
};

/** Spin the gateway. Constructor failures (params, CAN bind) are fatal. */
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
