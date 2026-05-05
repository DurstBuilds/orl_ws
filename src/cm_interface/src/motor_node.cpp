#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <string>

#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "rclcpp/rclcpp.hpp"
#include "motor_interfaces/msg/motor_command.hpp"

class MotorNode : public rclcpp::Node
{
public:
  MotorNode() : Node("motor_node")
  {
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create CAN socket.");
      return;
    }

    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, "can0", IFNAMSIZ - 1);

    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
      RCLCPP_ERROR(this->get_logger(), "can0 not found.");
      close(can_socket_);
      can_socket_ = -1;
      return;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(can_socket_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Bind failed.");
      close(can_socket_);
      can_socket_ = -1;
      return;
    }

    subscription_ = this->create_subscription<motor_interfaces::msg::MotorCommand>(
      "motor_command",
      10,
      std::bind(&MotorNode::motor_command_callback, this, std::placeholders::_1));

    enable_motor();
  }

  ~MotorNode() override
  {
    disable_motor();
    if (can_socket_ >= 0) {
      close(can_socket_);
    }
  }

private:
  int float_to_uint(float x, float x_min, float x_max, int bits)
  {
    if (x < x_min) x = x_min;
    if (x > x_max) x = x_max;
    float span = x_max - x_min;
    return static_cast<int>((x - x_min) * (((1 << bits) - 1) / span));
  }

  void enable_motor()
  {
    if (can_socket_ < 0) return;

    struct can_frame frame;
    frame.can_id = 0x00;
    frame.can_dlc = 8;
    frame.data[0] = 0xFF; frame.data[1] = 0xFF; frame.data[2] = 0xFF; frame.data[3] = 0xFF;
    frame.data[4] = 0xFF; frame.data[5] = 0xFF; frame.data[6] = 0xFF; frame.data[7] = 0xFC;

    write(can_socket_, &frame, sizeof(frame));
  }

  void disable_motor()
  {
    if (can_socket_ < 0) return;

    struct can_frame frame;
    frame.can_id = 0x00;
    frame.can_dlc = 8;
    frame.data[0] = 0xFF; frame.data[1] = 0xFF; frame.data[2] = 0xFF; frame.data[3] = 0xFF;
    frame.data[4] = 0xFF; frame.data[5] = 0xFF; frame.data[6] = 0xFF; frame.data[7] = 0xFD;

    write(can_socket_, &frame, sizeof(frame));
  }

  void send_mit_command(float p_des, float v_des, float kp, float kd, float t_ff)
  {
    float P_MIN = -12.5f, P_MAX = 12.5f;
    float V_MIN = -50.0f, V_MAX = 50.0f;
    float T_MIN = -25.0f, T_MAX = 25.0f;
    float Kp_MIN = 0.0f, Kp_MAX = 500.0f;
    float Kd_MIN = 0.0f, Kd_MAX = 5.0f;

    int p_int  = float_to_uint(p_des, P_MIN, P_MAX, 16);
    int v_int  = float_to_uint(v_des, V_MIN, V_MAX, 12);
    int kp_int = float_to_uint(kp, Kp_MIN, Kp_MAX, 12);
    int kd_int = float_to_uint(kd, Kd_MIN, Kd_MAX, 12);
    int t_int  = float_to_uint(t_ff, T_MIN, T_MAX, 12);

    struct can_frame frame;
    frame.can_id = 0x00;
    frame.can_dlc = 8;

    frame.data[0] = p_int >> 8;
    frame.data[1] = p_int & 0xFF;
    frame.data[2] = v_int >> 4;
    frame.data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
    frame.data[4] = kp_int & 0xFF;
    frame.data[5] = kd_int >> 4;
    frame.data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
    frame.data[7] = t_int & 0xFF;

    if (write(can_socket_, &frame, sizeof(frame)) == sizeof(frame)) {
      RCLCPP_INFO(this->get_logger(), "Sent MIT cmd: P=%.3f V=%.3f KP=%.3f KD=%.3f T=%.3f",
                  p_des, v_des, kp, kd, t_ff);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to send MIT command.");
    }
  }

  bool changed(const motor_interfaces::msg::MotorCommand & msg)
  {
    if (!has_last_) return true;
    return msg.position != last_msg_.position ||
           msg.velocity != last_msg_.velocity ||
           msg.kp != last_msg_.kp ||
           msg.kd != last_msg_.kd ||
           msg.torque != last_msg_.torque;
  }

  void motor_command_callback(const motor_interfaces::msg::MotorCommand::SharedPtr msg)
  {
    if (changed(*msg)) {
      last_msg_ = *msg;
      has_last_ = true;
      send_mit_command(msg->position, msg->velocity, msg->kp, msg->kd, msg->torque);
    }
  }

  rclcpp::Subscription<motor_interfaces::msg::MotorCommand>::SharedPtr subscription_;
  int can_socket_{-1};
  motor_interfaces::msg::MotorCommand last_msg_{};
  bool has_last_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorNode>());
  rclcpp::shutdown();
  return 0;
}