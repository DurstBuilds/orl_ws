// joint_translator_node: maps motor total position to joint space and commands
// motor deltas via PD control at loop_hz (default 200 Hz). Every tick either
// sends a PD delta toward joint_despos or a hold (deltaP=0) if idle/at goal.

#include <cmath>
#include <mutex>
#include <stdexcept>
#include <string>

#include "cm_interface/motor_mit_profile.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "motor_interfaces/msg/motor_command.hpp"
#include "motor_interfaces/msg/motor_state.hpp"
#include "motor_interfaces/msg/motor_total_position.hpp"

namespace
{

constexpr float kDefaultLoopHz = 200.0f;
constexpr float kDefaultJointErrorTolerance = 1e-3f;
constexpr float kMitKdMax = 5.0f;
constexpr float kVelocityRampFraction = 0.01f;

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
    joint_error_tolerance_ = declare_parameter<double>(
      "joint_error_tolerance", kDefaultJointErrorTolerance);

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
    pd_kd_ = profile_.pd_kd;
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
    velocity_ramp_step_ = kVelocityRampFraction * omega_max_;

    motor_total_sub_ = create_subscription<motor_interfaces::msg::MotorTotalPosition>(
      "motor_total_position",
      10,
      std::bind(&JointTranslatorNode::motor_total_position_callback, this, std::placeholders::_1));

    motor_state_sub_ = create_subscription<motor_interfaces::msg::MotorState>(
      "motor_state",
      10,
      std::bind(&JointTranslatorNode::motor_state_callback, this, std::placeholders::_1));

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
    motor_command_pub_ = create_publisher<motor_interfaces::msg::MotorCommand>("motor_command", 10);

    const auto loop_period = std::chrono::duration<double>(1.0 / loop_hz_);
    loop_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(loop_period),
      std::bind(&JointTranslatorNode::loop_timer_callback, this));

    RCLCPP_INFO(
      get_logger(),
      "motor_model=%s gear_ratio=%.4f loop_hz=%.1f "
      "omega_max=%.3f pdelta_max=%.4f velocity_ramp_step=%.4f joint_error_tolerance=%.4f "
      "pd_kp=%.4f pd_kd=%.4f mit_kp=%.2f mit_kd=%.3f",
      profile_.name, gear_ratio_, loop_hz_, omega_max_, pdelta_max_, velocity_ramp_step_,
      joint_error_tolerance_, pd_kp_, pd_kd_, mit_kp_, mit_kd_);
  }

private:
  struct ControlState
  {
    float total_position{0.0f};
    float motor_velocity{0.0f};
    float joint_despos{0.0f};
    bool soft_mode{false};
    bool hold_joint{false};
    bool has_hold_joint{false};
    bool has_total{false};
    bool has_velocity{false};
    bool has_despos{false};
    bool at_goal_latched{false};
  };

  ControlState read_state()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return ControlState{
      total_position_, motor_velocity_, joint_despos_, soft_mode_, hold_joint_,
      has_hold_joint_, has_total_position_, has_motor_velocity_, has_joint_despos_,
      at_goal_latched_};
  }

  void motor_total_position_callback(
    const motor_interfaces::msg::MotorTotalPosition::SharedPtr msg)
  {
    const float joint_curpos = static_cast<float>(msg->total_position / gear_ratio_);

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      total_position_ = msg->total_position;
      has_total_position_ = true;
    }

    std_msgs::msg::Float32 out;
    out.data = joint_curpos;
    joint_curpos_pub_->publish(out);
  }

  void motor_state_callback(const motor_interfaces::msg::MotorState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    motor_velocity_ = msg->velocity;
    has_motor_velocity_ = true;
  }

  void joint_despos_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!has_joint_despos_ ||
      std::fabs(msg->data - joint_despos_) > static_cast<float>(joint_error_tolerance_))
    {
      at_goal_latched_ = false;
    }
    joint_despos_ = msg->data;
    has_joint_despos_ = true;
  }

  void soft_mode_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (soft_mode_ != msg->data) {
      const bool was_soft_mode = soft_mode_;
      soft_mode_ = msg->data;
      if (was_soft_mode && !soft_mode_ && has_total_position_) {
        // On soft-mode exit, pin desired joint position to current so we do not
        // chase a stale pre-soft target.
        joint_despos_ = total_position_ / static_cast<float>(gear_ratio_);
        has_joint_despos_ = true;
        at_goal_latched_ = true;
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
      RCLCPP_INFO(get_logger(), "hold_joint=%s", hold_joint_ ? "true" : "false");
    }
  }

  float compute_motor_delta(float total_position_error, float motor_velocity) const
  {
    const float raw = pd_kp_ * total_position_error - pd_kd_ * motor_velocity;
    return clamp_magnitude(raw, pdelta_max_);
  }

  float ramp_command_velocity(float target_velocity)
  {
    const float delta_v = target_velocity - ramped_command_velocity_;
    const float step = clamp(delta_v, -velocity_ramp_step_, velocity_ramp_step_);
    ramped_command_velocity_ += step;
    return ramped_command_velocity_;
  }

  void reset_velocity_ramp()
  {
    ramped_command_velocity_ = 0.0f;
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

  // 200 Hz: PD command if active, hold (deltaP=0) otherwise.
  void loop_timer_callback()
  {
    const auto state = read_state();

    if (state.soft_mode) {
      reset_velocity_ramp();
      return;
    }

    if (state.has_hold_joint && state.hold_joint) {
      reset_velocity_ramp();
      publish_motor_command(0.0f);
      return;
    }

    if (!state.has_total || !state.has_despos || state.at_goal_latched) {
      if (!state.has_total) {
        warn_no_total_position();
      }
      reset_velocity_ramp();
      publish_motor_command(0.0f);
      return;
    }

    const float joint_curpos = state.total_position / static_cast<float>(gear_ratio_);
    const float joint_error = state.joint_despos - joint_curpos;
    if (std::fabs(joint_error) < static_cast<float>(joint_error_tolerance_)) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      at_goal_latched_ = true;
      reset_velocity_ramp();
      publish_motor_command(0.0f);
      return;
    }

    const float desired_total = state.joint_despos * static_cast<float>(gear_ratio_);
    const float total_position_error = desired_total - state.total_position;
    const float motor_velocity = state.has_velocity ? state.motor_velocity : 0.0f;

    const float raw_delta = compute_motor_delta(total_position_error, motor_velocity);
    const float loop_hz = static_cast<float>(loop_hz_);
    const float target_command_velocity = raw_delta * loop_hz;
    const float ramped_velocity = ramp_command_velocity(target_command_velocity);
    const float ramped_delta = clamp_magnitude(ramped_velocity / loop_hz, pdelta_max_);
    publish_motor_command(ramped_delta);
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
  rclcpp::Subscription<motor_interfaces::msg::MotorState>::SharedPtr motor_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr joint_despos_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr soft_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hold_joint_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr joint_curpos_pub_;
  rclcpp::Publisher<motor_interfaces::msg::MotorCommand>::SharedPtr motor_command_pub_;
  rclcpp::TimerBase::SharedPtr loop_timer_;

  std::mutex state_mutex_;
  float total_position_{0.0f};
  float motor_velocity_{0.0f};
  float joint_despos_{0.0f};
  bool soft_mode_{false};
  bool hold_joint_{false};
  bool has_hold_joint_{false};
  bool has_total_position_{false};
  bool has_motor_velocity_{false};
  bool has_joint_despos_{false};
  bool at_goal_latched_{false};

  double gear_ratio_{0.0};
  double loop_hz_{kDefaultLoopHz};
  double joint_error_tolerance_{kDefaultJointErrorTolerance};
  cm_interface::MotorMitProfile profile_{cm_interface::kAk70_10};
  float pd_kp_{0.0f};
  float pd_kd_{0.0f};
  float omega_max_{0.0f};
  float pdelta_max_{0.0f};
  float velocity_ramp_step_{0.0f};
  float ramped_command_velocity_{0.0f};
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
