#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "motor_interfaces/msg/motor_command.hpp"

class TerminalMotorCommandPublisher : public rclcpp::Node
{
public:
  TerminalMotorCommandPublisher() : Node("terminal_motor_command")
  {
    publisher_ = this->create_publisher<motor_interfaces::msg::MotorCommand>("motor_command", 10);
  }

  void publish_command(float position, float velocity, float kp, float kd, float torque)
  {
    motor_interfaces::msg::MotorCommand msg;
    msg.position = position;
    msg.velocity = velocity;
    msg.kp = kp;
    msg.kd = kd;
    msg.torque = torque;
    publisher_->publish(msg);

    RCLCPP_INFO(this->get_logger(),
      "Published: P=%.3f V=%.3f KP=%.3f KD=%.3f T=%.3f",
      position, velocity, kp, kd, torque);
  }

private:
  rclcpp::Publisher<motor_interfaces::msg::MotorCommand>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TerminalMotorCommandPublisher>();

  std::string input;
  while (rclcpp::ok()) {
    float p, v, kp, kd, t;

    std::cout << "Enter position velocity kp kd torque (q to quit): ";
    std::getline(std::cin, input);
    if (input == "q" || input == "Q") {
      break;
    }

    std::stringstream ss(input);
    if (ss >> p >> v >> kp >> kd >> t) {
      node->publish_command(p, v, kp, kd, t);
    } else {
      RCLCPP_ERROR(node->get_logger(), "Invalid input. Expected 5 numbers.");
    }
  }

  rclcpp::shutdown();
  return 0;
}