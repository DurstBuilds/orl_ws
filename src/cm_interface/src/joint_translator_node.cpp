// joint_translator_node: maps motor total position to joint space and commands
// motor deltas using "settled then jump" control. When the motor has reached
// its last target, the exact delta to the new joint_despos is sent in one shot.
// Blank (hold) commands stream at feedback_hz to keep motor feedback alive.

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
constexpr float kDefaultJointErrorTolerance = 1e-1f;
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

    motor_tolerance_ = static_cast<float>(joint_error_tolerance_ * gear_ratio_);

    if (gear_ratio_ <= 0.0) {
      throw std::invalid_argument("gear_ratio must be > 0");
    }
    if (feedback_hz_ <= 0.0) {
      throw std::invalid_argument("feedback_hz must be > 0");
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

    RCLCPP_INFO(
      get_logger(),
      "motor_model=%s gear_ratio=%.4f feedback_hz=%.1f "
      "joint_error_tolerance=%.4f motor_tolerance=%.4f "
      "mit_kp=%.2f mit_kd=%.3f",
      profile_.name, gear_ratio_, feedback_hz_,
      joint_error_tolerance_, motor_tolerance_, mit_kp_, mit_kd_);
  }

private:
  void motor_total_position_callback(
    const motor_interfaces::msg::MotorTotalPosition::SharedPtr msg)
  {
    const float joint_curpos = static_cast<float>(msg->total_position / gear_ratio_);

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      total_position_ = msg->total_position;

      if (!has_total_position_) {
        last_motor_target_ = total_position_;
        has_total_position_ = true;
      }
    }

    std_msgs::msg::Float32 out;
    out.data = joint_curpos;
    joint_curpos_pub_->publish(out);

    try_send_command();
  }

  void motor_state_callback(const motor_interfaces::msg::MotorState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    motor_velocity_ = msg->velocity;
  }

  void joint_despos_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      joint_despos_ = msg->data;
      has_pending_despos_ = true;
    }

    try_send_command();
  }

  // Settled-then-jump: if motor is at last_motor_target_, send the full delta
  // to the new joint_despos in one shot.
  void try_send_command()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (!has_total_position_ || !has_pending_despos_) {
      return;
    }

    const bool settled =
      std::fabs(total_position_ - last_motor_target_) < motor_tolerance_;
    if (!settled) {
      return;
    }

    const float new_motor_target =
      joint_despos_ * static_cast<float>(gear_ratio_);
    const float motor_delta = new_motor_target - total_position_;

    if (std::fabs(motor_delta) < motor_tolerance_) {
      has_pending_despos_ = false;
      return;
    }

    publish_motor_command(motor_delta);
    last_motor_target_ = new_motor_target;
    has_pending_despos_ = false;
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

  rclcpp::Subscription<motor_interfaces::msg::MotorTotalPosition>::SharedPtr motor_total_sub_;
  rclcpp::Subscription<motor_interfaces::msg::MotorState>::SharedPtr motor_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr joint_despos_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr joint_curpos_pub_;
  rclcpp::Publisher<motor_interfaces::msg::MotorCommand>::SharedPtr motor_command_pub_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;

  std::mutex state_mutex_;
  float total_position_{0.0f};
  float motor_velocity_{0.0f};
  float joint_despos_{0.0f};
  float last_motor_target_{0.0f};
  bool has_total_position_{false};
  bool has_pending_despos_{false};

  double gear_ratio_{0.0};
  double feedback_hz_{kDefaultFeedbackHz};
  double joint_error_tolerance_{kDefaultJointErrorTolerance};
  float motor_tolerance_{0.0f};
  cm_interface::MotorMitProfile profile_{cm_interface::kAk70_10};
  double mit_kp_{kDefaultMitKp};
  double mit_kd_{kDefaultMitKd};
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
