#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

class TerminalPositionPublisher : public rclcpp::Node
{
public:
  TerminalPositionPublisher() : Node("terminal_position_publisher")
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float32>("/motor_command", 10);
  }

  void publish_position(float position)
  {
    std_msgs::msg::Float32 msg;
    msg.data = position;
    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published position: %.3f", position);
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TerminalPositionPublisher>();

  std::string input;
  while (rclcpp::ok()) {
    std::cout << "Enter motor position (q to quit): ";
    std::getline(std::cin, input);

    if (!rclcpp::ok() || input == "q" || input == "Q") {
      break;
    }

    try {
      float position = std::stof(input);
      node->publish_position(position);
    } catch (const std::exception &) {
      RCLCPP_ERROR(node->get_logger(), "Invalid number.");
    }
  }

  rclcpp::shutdown();
  return 0;
}