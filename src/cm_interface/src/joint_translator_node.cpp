// joint_translator_node: maps motor total position to joint space and commands
// motor deltas via PD control until joint_despos is reached.
// Hold (blank) motor commands at feedback_hz; deltaP commands at command_hz.

#include <cmath>
#include <mutex>
#include <stdexcept>
#include <string>

#include "cm_interface/motor_mit_profile.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "motor_interfaces/msg/motor_command.hpp"
#include "motor_interfaces/msg/motor_state.hpp"
#include "motor_interfaces/msg/motor_total_position.hpp"

namespace
{

constexpr float kDefaultFeedbackHz = 200.0f;
constexpr float kDefaultCommandHz = 50.0f;
constexpr float kDefaultJointErrorTolerance = 1e-3f;
constexpr float kDefaultMitKp = 4.0f;
constexpr float kDefaultMitKd = 0.02f;
constexpr float kMitKdMax = 5.0f;

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
    feedback_hz_ = declare_parameter<double>("feedback_hz", kDefaultFeedbackHz);
    command_hz_ = declare_parameter<double>("command_hz", kDefaultCommandHz);
    joint_error_tolerance_ = declare_parameter<double>(
      "joint_error_tolerance", kDefaultJointErrorTolerance);
    mit_kp_ = declare_parameter<double>("mit_kp", kDefaultMitKp);
    mit_kd_ = declare_parameter<double>("mit_kd", kDefaultMitKd);

    try {
      profile_ = cm_interface::get_motor_mit_profile(motor_model);
    } catch (const std::exception & e) {
      RCLCPP_FATAL(get_logger(), "%s", e.what());
      throw;
    }
    pd_kp_ = profile_.pd_kp;
    pd_kd_ = profile_.pd_kd;
    pdelta_max_ = profile_.pdelta_max;

    if (gear_ratio_ <= 0.0) {
      throw std::invalid_argument("gear_ratio must be > 0");
    }
    if (feedback_hz_ <= 0.0) {
      throw std::invalid_argument("feedback_hz must be > 0");
    }
    if (command_hz_ <= 0.0) {
      throw std::invalid_argument("command_hz must be > 0");
    }

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

    joint_curpos_pub_ = create_publisher<std_msgs::msg::Float32>("joint_curpos", 10);
    motor_command_pub_ = create_publisher<motor_interfaces::msg::MotorCommand>("motor_command", 10);

    const auto feedback_period = std::chrono::duration<double>(1.0 / feedback_hz_);
    feedback_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(feedback_period),
      std::bind(&JointTranslatorNode::feedback_timer_callback, this));

    const auto command_period = std::chrono::duration<double>(1.0 / command_hz_);
    command_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(command_period),
      std::bind(&JointTranslatorNode::command_timer_callback, this));

    RCLCPP_INFO(
      get_logger(),
      "motor_model=%s gear_ratio=%.4f feedback_hz=%.1f command_hz=%.1f "
      "pdelta_max=%.4f joint_error_tolerance=%.4f pd_kp=%.4f pd_kd=%.4f "
      "mit_kp=%.2f mit_kd=%.3f",
      profile_.name, gear_ratio_, feedback_hz_, command_hz_, pdelta_max_,
      joint_error_tolerance_, pd_kp_, pd_kd_, mit_kp_, mit_kd_);
  }

private:
  struct ControlState
  {
    float total_position{0.0f};
    float motor_velocity{0.0f};
    float joint_despos{0.0f};
    bool has_total{false};
    bool has_velocity{false};
    bool has_despos{false};
    bool at_goal_latched{false};
  };

  ControlState read_state()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return ControlState{
      total_position_, motor_velocity_, joint_despos_,
      has_total_position_, has_motor_velocity_, has_joint_despos_,
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

  float compute_motor_delta(float total_position_error, float motor_velocity) const
  {
    const float raw = pd_kp_ * total_position_error - pd_kd_ * motor_velocity;
    return clamp_magnitude(raw, pdelta_max_);
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

  // 200 Hz: blank commands (hold) to keep motor feedback streaming.
  void feedback_timer_callback()
  {
    publish_motor_command(0.0f);
  }

  // 50 Hz: PD deltaP commands toward joint_despos.
  void command_timer_callback()
  {
    const auto state = read_state();

    if (!state.has_total) {
      warn_no_total_position();
      return;
    }

    if (!state.has_despos) {
      return;
    }

    if (state.at_goal_latched) {
      return;
    }

    const float joint_curpos = state.total_position / static_cast<float>(gear_ratio_);
    const float joint_error = state.joint_despos - joint_curpos;
    if (std::fabs(joint_error) < static_cast<float>(joint_error_tolerance_)) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      at_goal_latched_ = true;
      return;
    }

    const float desired_total = state.joint_despos * static_cast<float>(gear_ratio_);
    const float total_position_error = desired_total - state.total_position;
    const float motor_velocity = state.has_velocity ? state.motor_velocity : 0.0f;

    publish_motor_command(compute_motor_delta(total_position_error, motor_velocity));
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
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr joint_curpos_pub_;
  rclcpp::Publisher<motor_interfaces::msg::MotorCommand>::SharedPtr motor_command_pub_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;
  rclcpp::TimerBase::SharedPtr command_timer_;

  std::mutex state_mutex_;
  float total_position_{0.0f};
  float motor_velocity_{0.0f};
  float joint_despos_{0.0f};
  bool has_total_position_{false};
  bool has_motor_velocity_{false};
  bool has_joint_despos_{false};
  bool at_goal_latched_{false};

  double gear_ratio_{0.0};
  double feedback_hz_{kDefaultFeedbackHz};
  double command_hz_{kDefaultCommandHz};
  double joint_error_tolerance_{kDefaultJointErrorTolerance};
  cm_interface::MotorMitProfile profile_{cm_interface::kAk70_10};
  float pd_kp_{0.0f};
  float pd_kd_{0.0f};
  float pdelta_max_{0.0f};
  double mit_kp_{kDefaultMitKp};
  double mit_kd_{kDefaultMitKd};

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
