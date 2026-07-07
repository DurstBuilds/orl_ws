// joint_translator_node: maps joint targets to motor deltas via proportional
// control at loop_hz (default 200 Hz). Position error is a hybrid of dead
// reckoning (commanded total position) and feedback (joint_curpos):
//   e = (1-w)*e_commanded + w*e_feedback
// commanded_joint_position integrates published deltaP (quantized to the
// 15-bit CAN packing grid so it matches what the motor executes);
// joint_curpos is feedback.
// Hold (deltaP=0) when hold_joint is true (teleop release), goal/limit latched,
// or within motor_error_tolerance (hold only, no latch).
//
// Parameters (TWEAK):
//   motor_model            — ak70_10 | ak10_9 | ak80_64 (sets P + MIT defaults)
//   gear_ratio             — motor rad per joint rad (> 0)
//   loop_hz                — control loop rate (default 200)
//   motor_error_tolerance  — motor-space goal/hold tolerance (rad)
//   feedback_blend         — w in [0,1]: fraction of error from joint_curpos feedback
//   joint_angle_limit_deg  — 0 disables; else clamp ±limit in joint space
//   omega_max              — "auto" or max motor speed (rad/s) for delta cap
//   mit_kp, mit_kd         — override profile MIT gains on motor_command
//
// Subscribes (relative to namespace): motor_total_position, joint_despos,
// soft_mode, hold_joint. Publishes joint_curpos, commanded_joint_position,
// motor_command.
// Soft mode: stops publishing motor_command. Post-soft latch waits for unwrapper
// reset before tracking joint_despos again.

#include <cmath>
#include <mutex>
#include <stdexcept>
#include <string>

#include "cm_interface/mit_can_codec.hpp"
#include "cm_interface/motor_mit_profile.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "motor_interfaces/msg/motor_command.hpp"
#include "motor_interfaces/msg/motor_total_position.hpp"

namespace
{

constexpr float kDefaultLoopHz = 200.0f;
constexpr float kDefaultMotorErrorTolerance = 1e-3f;
constexpr float kDefaultFeedbackBlend = 0.3f;
constexpr float kMitKdMax = 5.0f;
constexpr float kJointAngleLimitEps = 1e-4f;
constexpr double kDefaultJointAngleLimitDeg = 0.0;

float clamp(float value, float low, float high)
{
  if (value < low) {
    return low;
  }
  if (value > high) {
    return high;
  }
  return value;
}

float clamp_magnitude(float value, float max_magnitude)
{
  return clamp(value, -max_magnitude, max_magnitude);
}

}  // namespace

class JointTranslatorNode : public rclcpp::Node
{
public:
  JointTranslatorNode() : Node("joint_translator_node")
  {
    const std::string motor_model = declare_parameter<std::string>(
      "motor_model", "ak70_10");
    gear_ratio_ = declare_parameter<double>("gear_ratio", 0.0);
    loop_hz_ = declare_parameter<double>("loop_hz", kDefaultLoopHz);
    motor_error_tolerance_ = declare_parameter<double>(
      "motor_error_tolerance", kDefaultMotorErrorTolerance);
    feedback_blend_ = static_cast<float>(declare_parameter<double>(
      "feedback_blend", kDefaultFeedbackBlend));
    if (feedback_blend_ < 0.0 || feedback_blend_ > 1.0) {
      throw std::invalid_argument("feedback_blend must be in [0, 1]");
    }
    const double joint_angle_limit_deg = declare_parameter<double>(
      "joint_angle_limit_deg", kDefaultJointAngleLimitDeg);
    if (joint_angle_limit_deg < 0.0) {
      throw std::invalid_argument("joint_angle_limit_deg must be >= 0 (0 disables clamp)");
    }
    joint_angle_limit_rad_ = static_cast<float>(joint_angle_limit_deg * M_PI / 180.0);

    try {
      profile_ = cm_interface::get_motor_mit_profile(motor_model);
    } catch (const std::exception & e) {
      RCLCPP_FATAL(get_logger(), "%s", e.what());
      throw;
    }

    mit_kp_ = declare_parameter<double>("mit_kp", static_cast<double>(profile_.mit_kp));
    mit_kd_ = declare_parameter<double>("mit_kd", static_cast<double>(profile_.mit_kd));
    const std::string omega_max_param = declare_parameter<std::string>("omega_max", "auto");
    pd_kp_ = profile_.pd_kp;
    omega_max_ = profile_.omega_max;

    if (gear_ratio_ <= 0.0) {
      throw std::invalid_argument("gear_ratio must be > 0");
    }
    if (loop_hz_ <= 0.0) {
      throw std::invalid_argument("loop_hz must be > 0");
    }
    if (omega_max_param != "auto") {
      omega_max_ = static_cast<float>(std::stod(omega_max_param));
    }
    if (omega_max_ <= 0.0f) {
      throw std::invalid_argument("omega_max must be > 0");
    }
    pdelta_max_ = omega_max_ / static_cast<float>(loop_hz_);

    param_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&JointTranslatorNode::on_set_parameters, this, std::placeholders::_1));

    motor_total_sub_ = create_subscription<motor_interfaces::msg::MotorTotalPosition>(
      "motor_total_position",
      10,
      std::bind(&JointTranslatorNode::motor_total_position_callback, this, std::placeholders::_1));

    joint_despos_sub_ = create_subscription<std_msgs::msg::Float32>(
      "joint_despos",
      10,
      std::bind(&JointTranslatorNode::joint_despos_callback, this, std::placeholders::_1));

    soft_mode_sub_ = create_subscription<std_msgs::msg::Bool>(
      "soft_mode",
      10,
      std::bind(&JointTranslatorNode::soft_mode_callback, this, std::placeholders::_1));

    hold_joint_sub_ = create_subscription<std_msgs::msg::Bool>(
      "hold_joint",
      10,
      std::bind(&JointTranslatorNode::hold_joint_callback, this, std::placeholders::_1));

    joint_curpos_pub_ = create_publisher<std_msgs::msg::Float32>("joint_curpos", 10);
    commanded_joint_position_pub_ = create_publisher<std_msgs::msg::Float32>(
      "commanded_joint_position", 10);
    motor_command_pub_ = create_publisher<motor_interfaces::msg::MotorCommand>("motor_command", 10);

    std_msgs::msg::Float32 initial_curpos;
    initial_curpos.data = 0.0f;
    joint_curpos_pub_->publish(initial_curpos);
    commanded_joint_position_pub_->publish(initial_curpos);

    const auto loop_period = std::chrono::duration<double>(1.0 / loop_hz_);
    loop_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(loop_period),
      std::bind(&JointTranslatorNode::loop_timer_callback, this));

    RCLCPP_INFO(
      get_logger(),
      "motor_model=%s gear_ratio=%.4f loop_hz=%.1f "
      "omega_max=%.3f pdelta_max=%.4f motor_error_tolerance=%.4f feedback_blend=%.3f "
      "joint_angle_limit_deg=%.1f pd_kp=%.4f mit_kp=%.2f mit_kd=%.3f",
      profile_.name, gear_ratio_, loop_hz_, omega_max_, pdelta_max_,
      motor_error_tolerance_, feedback_blend_, joint_angle_limit_deg, pd_kp_, mit_kp_, mit_kd_);
  }

private:
  struct ControlState
  {
    float total_position{0.0f};
    float commanded_total_position{0.0f};
    float joint_despos{0.0f};
    bool soft_mode{false};
    bool hold_joint{false};
    bool has_hold_joint{false};
    bool has_total{false};
    bool has_commanded{false};
    bool has_despos{false};
    bool at_goal_latched{false};
    bool awaiting_post_soft_latch{false};
  };

  ControlState read_state()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return ControlState{
      total_position_, commanded_total_position_, joint_despos_, soft_mode_,
      hold_joint_, has_hold_joint_, has_total_position_, has_commanded_total_position_,
      has_joint_despos_, at_goal_latched_, awaiting_post_soft_latch_};
  }

  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & param : parameters) {
      if (param.get_name() == "feedback_blend") {
        const double value = param.as_double();
        if (value < 0.0 || value > 1.0) {
          result.successful = false;
          result.reason = "feedback_blend must be in [0, 1]";
          return result;
        }
        feedback_blend_ = static_cast<float>(value);
        RCLCPP_INFO(get_logger(), "feedback_blend=%.3f", feedback_blend_);
      } else if (param.get_name() == "motor_error_tolerance") {
        const double value = param.as_double();
        if (value <= 0.0) {
          result.successful = false;
          result.reason = "motor_error_tolerance must be > 0";
          return result;
        }
        motor_error_tolerance_ = value;
        RCLCPP_INFO(get_logger(), "motor_error_tolerance=%.4f", motor_error_tolerance_);
      }
    }
    return result;
  }

  void motor_total_position_callback(
    const motor_interfaces::msg::MotorTotalPosition::SharedPtr msg)
  {
    const float joint_curpos = static_cast<float>(msg->total_position / gear_ratio_);

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      total_position_ = msg->total_position;
      has_total_position_ = true;
      if (awaiting_post_soft_latch_ && !soft_mode_) {
        joint_despos_ = clamp_joint_despos(
          total_position_ / static_cast<float>(gear_ratio_));
        has_joint_despos_ = true;
        at_goal_latched_ = true;
        awaiting_post_soft_latch_ = false;
        commanded_total_position_ = total_position_;
        publish_commanded_joint_position(commanded_total_position_);
        RCLCPP_INFO(
          get_logger(),
          "soft_mode off: latched joint_despos=%.4f at current position",
          joint_despos_);
      }
    }

    std_msgs::msg::Float32 out;
    out.data = joint_curpos;
    joint_curpos_pub_->publish(out);
  }

  bool joint_despos_changed_enough(float clamped_new, float clamped_prev) const
  {
    if (!has_joint_despos_) {
      return true;
    }
    const float gear_ratio = static_cast<float>(gear_ratio_);
    const float motor_tol = static_cast<float>(motor_error_tolerance_);
    const float joint_change_tol = motor_tol / gear_ratio;
    if (std::fabs(clamped_new - clamped_prev) > joint_change_tol) {
      return true;
    }
    const float new_desired_total = clamped_new * gear_ratio;
    const float prev_desired_total = clamped_prev * gear_ratio;
    return std::fabs(new_desired_total - prev_desired_total) > motor_tol;
  }

  bool despos_moved_away_from_limit(float clamped_new, float clamped_prev) const
  {
    if (!joint_angle_limit_enabled() || !at_goal_latched_) {
      return false;
    }
    const bool was_at_positive_limit =
      std::fabs(clamped_prev - joint_angle_limit_rad_) < kJointAngleLimitEps;
    const bool was_at_negative_limit =
      std::fabs(clamped_prev + joint_angle_limit_rad_) < kJointAngleLimitEps;
    if (was_at_positive_limit && clamped_new < clamped_prev - kJointAngleLimitEps) {
      return true;
    }
    if (was_at_negative_limit && clamped_new > clamped_prev + kJointAngleLimitEps) {
      return true;
    }
    return false;
  }

  void joint_despos_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const float clamped_prev = has_joint_despos_ ? joint_despos_ : msg->data;
    const float clamped_new = clamp_joint_despos(msg->data);
    if (joint_despos_changed_enough(clamped_new, clamped_prev) ||
      despos_moved_away_from_limit(clamped_new, clamped_prev))
    {
      at_goal_latched_ = false;
    }
    joint_despos_ = clamped_new;
    has_joint_despos_ = true;
  }

  void soft_mode_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (soft_mode_ != msg->data) {
      const bool was_soft_mode = soft_mode_;
      soft_mode_ = msg->data;
      if (was_soft_mode && !soft_mode_) {
        awaiting_post_soft_latch_ = true;
      }
      RCLCPP_INFO(get_logger(), "soft_mode=%s", soft_mode_ ? "true" : "false");
    }
  }

  void hold_joint_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    has_hold_joint_ = true;
    if (hold_joint_ != msg->data) {
      hold_joint_ = msg->data;
      if (!hold_joint_) {
        at_goal_latched_ = false;
      }
      RCLCPP_DEBUG(get_logger(), "hold_joint=%s", hold_joint_ ? "true" : "false");
    }
  }

  float clamp_joint_despos(float joint_despos) const
  {
    if (joint_angle_limit_rad_ <= 0.0f) {
      return joint_despos;
    }
    return clamp(joint_despos, -joint_angle_limit_rad_, joint_angle_limit_rad_);
  }

  void latch_goal_at_current_position(float joint_curpos)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    at_goal_latched_ = true;
    joint_despos_ = clamp_joint_despos(joint_curpos);
    has_joint_despos_ = true;
  }

  void latch_goal_at_joint_limit(float joint_limit_rad)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    at_goal_latched_ = true;
    joint_despos_ = joint_limit_rad;
    has_joint_despos_ = true;
    commanded_total_position_ =
      joint_limit_rad * static_cast<float>(gear_ratio_);
    publish_commanded_joint_position(commanded_total_position_);
  }

  bool joint_angle_limit_enabled() const
  {
    return joint_angle_limit_rad_ > 0.0f;
  }

  bool commanding_into_positive_limit(float motor_error, float motor_tol) const
  {
    return motor_error > motor_tol;
  }

  bool commanding_into_negative_limit(float motor_error, float motor_tol) const
  {
    return motor_error < -motor_tol;
  }

  bool at_joint_angle_limit_reactive(
    float joint_curpos,
    float motor_error,
    float motor_tol) const
  {
    if (!joint_angle_limit_enabled()) {
      return false;
    }
    if (joint_curpos >= joint_angle_limit_rad_ - kJointAngleLimitEps &&
      commanding_into_positive_limit(motor_error, motor_tol))
    {
      return true;
    }
    if (joint_curpos <= -joint_angle_limit_rad_ + kJointAngleLimitEps &&
      commanding_into_negative_limit(motor_error, motor_tol))
    {
      return true;
    }
    return false;
  }

  bool would_exceed_joint_angle_limit(
    float total_position,
    float motor_delta,
    float motor_error,
    float motor_tol) const
  {
    if (!joint_angle_limit_enabled()) {
      return false;
    }
    const float predicted_joint =
      (total_position + motor_delta) / static_cast<float>(gear_ratio_);
    if (predicted_joint > joint_angle_limit_rad_ + kJointAngleLimitEps) {
      return commanding_into_positive_limit(motor_error, motor_tol);
    }
    if (predicted_joint < -joint_angle_limit_rad_ - kJointAngleLimitEps) {
      return commanding_into_negative_limit(motor_error, motor_tol);
    }
    return false;
  }

  void clear_goal_latch_if_tracking_away_from_limit(
    float joint_curpos,
    float motor_error,
    bool hold_joint,
    float motor_tol)
  {
    if (!at_goal_latched_ || hold_joint || !joint_angle_limit_enabled()) {
      return;
    }
    if (joint_curpos >= joint_angle_limit_rad_ - kJointAngleLimitEps &&
      commanding_into_negative_limit(motor_error, motor_tol))
    {
      at_goal_latched_ = false;
      return;
    }
    if (joint_curpos <= -joint_angle_limit_rad_ + kJointAngleLimitEps &&
      commanding_into_positive_limit(motor_error, motor_tol))
    {
      at_goal_latched_ = false;
    }
  }

  float compute_hybrid_motor_error(
    float desired_total,
    float commanded_total,
    float feedback_total,
    float predicted_commanded_delta = 0.0f) const
  {
    const float w = feedback_blend_;
    const float commanded_error = desired_total - (commanded_total + predicted_commanded_delta);
    const float feedback_error = desired_total - feedback_total;
    return (1.0f - w) * commanded_error + w * feedback_error;
  }

  float compute_motor_delta(float total_position_error) const
  {
    return clamp_magnitude(pd_kp_ * total_position_error, pdelta_max_);
  }

  // The CAN gateway packs deltaP into a 15-bit integer over [p_min, p_max]
  // (see pack_position_continuous). Round-trip through the same codec so the
  // integrated commanded position matches the delta the motor actually
  // executes, not the pre-quantization float.
  float quantize_motor_delta(float motor_delta) const
  {
    if (std::fabs(motor_delta) < cm_interface::kCmdZeroEps) {
      return 0.0f;  // Gateway sends hold (apply bit clear); no motion.
    }
    const int p_int = cm_interface::float_to_uint(
      motor_delta, profile_.p_min, profile_.p_max, 15);
    return cm_interface::uint_to_float(p_int, profile_.p_min, profile_.p_max, 15);
  }

  void publish_motor_command(float motor_delta)
  {
    motor_interfaces::msg::MotorCommand cmd;
    cmd.position = motor_delta;
    cmd.velocity = 0.0f;
    cmd.torque = 0.0f;
    cmd.kp = static_cast<float>(mit_kp_);
    cmd.kd = clamp(static_cast<float>(mit_kd_), 0.0f, kMitKdMax);
    motor_command_pub_->publish(cmd);
  }

  void publish_commanded_joint_position(float commanded_total_motor_rad)
  {
    std_msgs::msg::Float32 out;
    out.data = commanded_total_motor_rad / static_cast<float>(gear_ratio_);
    commanded_joint_position_pub_->publish(out);
  }

  void integrate_commanded_delta(float motor_delta)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    commanded_total_position_ += motor_delta;
    publish_commanded_joint_position(commanded_total_position_);
  }

  void publish_motor_damping_hold()
  {
    motor_interfaces::msg::MotorCommand cmd;
    cmd.position = 0.0f;
    cmd.velocity = 0.0f;
    cmd.torque = 0.0f;
    cmd.kp = 0.0f;
    cmd.kd = clamp(static_cast<float>(mit_kd_), 0.0f, kMitKdMax);
    motor_command_pub_->publish(cmd);
  }

  void loop_timer_callback()
  {
    const auto state = read_state();

    if (state.soft_mode) {
      return;
    }

    if (state.awaiting_post_soft_latch) {
      publish_motor_damping_hold();
      return;
    }

    if (!state.has_total || !state.has_commanded || !state.has_despos) {
      if (!state.has_total) {
        warn_no_total_position();
      }
      publish_motor_command(0.0f);
      return;
    }

    const float gear_ratio = static_cast<float>(gear_ratio_);
    const float joint_curpos = state.total_position / gear_ratio;
    const float desired_total = state.joint_despos * gear_ratio;
    const float motor_error = compute_hybrid_motor_error(
      desired_total, state.commanded_total_position, state.total_position);
    const float motor_tol = static_cast<float>(motor_error_tolerance_);
    const bool hold_from_teleop = state.has_hold_joint && state.hold_joint;

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      clear_goal_latch_if_tracking_away_from_limit(
        joint_curpos, motor_error, hold_from_teleop, motor_tol);
    }

    const bool within_tolerance = std::fabs(motor_error) < motor_tol;

    if (hold_from_teleop || state.at_goal_latched || within_tolerance) {
      publish_motor_command(0.0f);
      return;
    }

    const float motor_delta = quantize_motor_delta(compute_motor_delta(motor_error));

    if (at_joint_angle_limit_reactive(joint_curpos, motor_error, motor_tol)) {
      const float limit_rad = commanding_into_positive_limit(motor_error, motor_tol) ?
        joint_angle_limit_rad_ : -joint_angle_limit_rad_;
      latch_goal_at_joint_limit(limit_rad);
      publish_motor_command(0.0f);
      return;
    }

    if (would_exceed_joint_angle_limit(
        state.commanded_total_position, motor_delta, motor_error, motor_tol))
    {
      const float limit_rad = commanding_into_positive_limit(motor_error, motor_tol) ?
        joint_angle_limit_rad_ : -joint_angle_limit_rad_;
      latch_goal_at_joint_limit(limit_rad);
      publish_motor_command(0.0f);
      return;
    }

    publish_motor_command(motor_delta);
    integrate_commanded_delta(motor_delta);
  }

  void warn_no_total_position()
  {
    const auto now = this->now();
    if (!warned_no_total_ ||
      (now - last_warn_time_).seconds() >= 5.0)
    {
      RCLCPP_WARN(get_logger(), "No motor_total_position yet.");
      warned_no_total_ = true;
      last_warn_time_ = now;
    }
  }

  rclcpp::Subscription<motor_interfaces::msg::MotorTotalPosition>::SharedPtr motor_total_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr joint_despos_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr soft_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hold_joint_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr joint_curpos_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr commanded_joint_position_pub_;
  rclcpp::Publisher<motor_interfaces::msg::MotorCommand>::SharedPtr motor_command_pub_;
  rclcpp::TimerBase::SharedPtr loop_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  std::mutex state_mutex_;
  float total_position_{0.0f};
  float commanded_total_position_{0.0f};
  bool has_commanded_total_position_{true};
  float joint_despos_{0.0f};
  bool soft_mode_{false};
  bool hold_joint_{false};
  bool has_hold_joint_{false};
  bool has_total_position_{false};
  bool has_joint_despos_{true};
  bool at_goal_latched_{false};
  bool awaiting_post_soft_latch_{false};

  double gear_ratio_{0.0};
  double loop_hz_{kDefaultLoopHz};
  double motor_error_tolerance_{kDefaultMotorErrorTolerance};
  float feedback_blend_{kDefaultFeedbackBlend};
  cm_interface::MotorMitProfile profile_{cm_interface::kAk70_10};
  float pd_kp_{0.0f};
  float omega_max_{0.0f};
  float pdelta_max_{0.0f};
  float joint_angle_limit_rad_{0.0f};
  double mit_kp_{0.0};
  double mit_kd_{0.0};

  bool warned_no_total_{false};
  rclcpp::Time last_warn_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<JointTranslatorNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("joint_translator_node"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
