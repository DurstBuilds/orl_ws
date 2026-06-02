// motor_unwrapper_node: accumulates unwrapped position from motor_state feedback.
// Wrapped feedback is in [-pi, pi]; deltas are unwrapped and summed across rotations.

#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "motor_interfaces/msg/motor_state.hpp"
#include "motor_interfaces/msg/motor_total_position.hpp"
#include "std_msgs/msg/bool.hpp"

namespace
{

constexpr float kPositionEps = 1e-6f;
// Single-step unwrapped delta above this is treated as set-origin / telemetry reset.
constexpr float kOriginJumpThreshold = 2.0f;

// Shortest signed delta in (-pi, pi], handles pi <-> -pi boundary crossings.
float unwrap_delta(float prev_wrapped, float new_wrapped)
{
  const float diff = new_wrapped - prev_wrapped;
  return std::atan2(std::sin(diff), std::cos(diff));
}

}  // namespace

class MotorUnwrapperNode : public rclcpp::Node
{
public:
  MotorUnwrapperNode() : Node("motor_unwrapper_node")
  {
    subscription_ = create_subscription<motor_interfaces::msg::MotorState>(
      "motor_state",
      10,
      std::bind(&MotorUnwrapperNode::motor_state_callback, this, std::placeholders::_1));

    soft_mode_sub_ = create_subscription<std_msgs::msg::Bool>(
      "soft_mode",
      10,
      std::bind(&MotorUnwrapperNode::soft_mode_callback, this, std::placeholders::_1));

    publisher_ = create_publisher<motor_interfaces::msg::MotorTotalPosition>(
      "motor_total_position", 10);

    RCLCPP_INFO(
      get_logger(),
      "Subscribed to motor_state and soft_mode; publishing motor_total_position "
      "(unwrap range [-pi, pi]).");
  }

private:
  void soft_mode_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (soft_mode_ && !msg->data) {
      reset_total_on_next_feedback_ = true;
    }
    soft_mode_ = msg->data;
  }

  void motor_state_callback(const motor_interfaces::msg::MotorState::SharedPtr msg)
  {
    const float new_wrapped = msg->position;

    if (reset_total_on_next_feedback_) {
      reset_total_on_next_feedback_ = false;
      total_position_ = new_wrapped;
      last_wrapped_ = new_wrapped;
      has_last_ = true;
      RCLCPP_INFO(
        get_logger(),
        "soft_mode off: reset total position to wrapped %.4f rad",
        new_wrapped);
      publish_total(new_wrapped);
      return;
    }

    if (has_last_ && std::fabs(new_wrapped - last_wrapped_) < kPositionEps) {
      return;
    }

    if (!has_last_) {
      total_position_ = new_wrapped;
      last_wrapped_ = new_wrapped;
      has_last_ = true;
    } else {
      const float delta = unwrap_delta(last_wrapped_, new_wrapped);

      // Do not use raw (new - prev) for jump detection: crossing pi->-pi gives
      // |raw| ~ 2*pi even though unwrapped delta is small.
      if (std::fabs(delta) > kOriginJumpThreshold) {
        RCLCPP_WARN(
          get_logger(),
          "Large unwrapped step (%.4f rad); resetting total to %.4f",
          delta, new_wrapped);
        total_position_ = new_wrapped;
      } else {
        total_position_ += delta;
      }
      last_wrapped_ = new_wrapped;
    }

    publish_total(new_wrapped);
  }

  void publish_total(float wrapped_position)
  {
    motor_interfaces::msg::MotorTotalPosition out;
    out.wrapped_position = wrapped_position;
    out.total_position = total_position_;
    publisher_->publish(out);
  }

  rclcpp::Subscription<motor_interfaces::msg::MotorState>::SharedPtr subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr soft_mode_sub_;
  rclcpp::Publisher<motor_interfaces::msg::MotorTotalPosition>::SharedPtr publisher_;

  bool has_last_{false};
  bool soft_mode_{false};
  bool reset_total_on_next_feedback_{false};
  float last_wrapped_{0.0f};
  float total_position_{0.0f};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorUnwrapperNode>());
  rclcpp::shutdown();
  return 0;
}
