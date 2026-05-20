// motor_unwrapper_node: accumulates unwrapped position from motor_state feedback.
// Wrapped feedback is in [-pi, pi]; deltas are unwrapped and summed across rotations.

#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "motor_interfaces/msg/motor_state.hpp"
#include "motor_interfaces/msg/motor_total_position.hpp"

namespace
{

constexpr float kWrapPeriod = 2.0f * static_cast<float>(M_PI);
constexpr float kPositionEps = 1e-6f;

float unwrap_delta(float prev_wrapped, float new_wrapped)
{
  float delta = new_wrapped - prev_wrapped;
  while (delta > static_cast<float>(M_PI)) {
    delta -= kWrapPeriod;
  }
  while (delta < -static_cast<float>(M_PI)) {
    delta += kWrapPeriod;
  }
  return delta;
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

    publisher_ = create_publisher<motor_interfaces::msg::MotorTotalPosition>(
      "motor_total_position", 10);

    RCLCPP_INFO(
      get_logger(),
      "Subscribed to motor_state; publishing motor_total_position "
      "(unwrap range [-pi, pi]).");
  }

private:
  void motor_state_callback(const motor_interfaces::msg::MotorState::SharedPtr msg)
  {
    const float new_wrapped = msg->position;

    if (has_last_ && std::fabs(new_wrapped - last_wrapped_) < kPositionEps) {
      return;
    }

    if (!has_last_) {
      total_position_ = new_wrapped;
      last_wrapped_ = new_wrapped;
      has_last_ = true;
    } else {
      const float delta = unwrap_delta(last_wrapped_, new_wrapped);
      const float raw_diff = new_wrapped - last_wrapped_;

      if (std::fabs(delta) > static_cast<float>(M_PI) ||
        std::fabs(raw_diff) > static_cast<float>(M_PI))
      {
        RCLCPP_WARN(
          get_logger(),
          "Large position jump (raw=%.4f, unwrapped_delta=%.4f); resetting total to %.4f",
          raw_diff, delta, new_wrapped);
        total_position_ = new_wrapped;
      } else {
        total_position_ += delta;
      }
      last_wrapped_ = new_wrapped;
    }

    motor_interfaces::msg::MotorTotalPosition out;
    out.wrapped_position = new_wrapped;
    out.total_position = total_position_;
    publisher_->publish(out);
  }

  rclcpp::Subscription<motor_interfaces::msg::MotorState>::SharedPtr subscription_;
  rclcpp::Publisher<motor_interfaces::msg::MotorTotalPosition>::SharedPtr publisher_;

  bool has_last_{false};
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
