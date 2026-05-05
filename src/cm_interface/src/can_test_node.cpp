#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"

class CanTestNode : public rclcpp::Node {
public:
  CanTestNode() : Node("can_test_node") {
    
    // 1. Create Socket
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create socket.");
      return;
    }

    // 2. Find can0
    struct ifreq ifr;
    std::strcpy(ifr.ifr_name, "can0");
    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
      RCLCPP_ERROR(this->get_logger(), "can0 not found.");
      return;
    }

    // 3. Bind to Hardware
    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Bind failed.");
    } else {
      RCLCPP_INFO(this->get_logger(), "SUCCESS: Bound to can0. Attempting to Enable Motor...");

      // === [NEW] ENABLE MOTOR COMMAND ===
      struct can_frame frame;
      frame.can_id = 0x01; // Default ID for AK70-10. Verify on your motor label!
      frame.can_dlc = 8;
      // Payload to enter MIT Mode (8 bytes)
      frame.data[0] = 0xFF; frame.data[1] = 0xFF; frame.data[2] = 0xFF; frame.data[3] = 0xFF;
      frame.data[4] = 0xFF; frame.data[5] = 0xFF; frame.data[6] = 0xFF; frame.data[7] = 0xFC;

      if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
          RCLCPP_ERROR(this->get_logger(), "Failed to send Enable command!");
      } else {
          RCLCPP_INFO(this->get_logger(), "ENABLE command sent. Motor should be energized.");
      }
    }
  }

  ~CanTestNode() {
    if (can_socket_ >= 0) {
      // === [NEW] DISABLE MOTOR COMMAND ===
      // Sending 0xFD exits MIT mode and relaxes the motor
      struct can_frame frame;
      frame.can_id = 0x01;
      frame.can_dlc = 8;
      frame.data[0] = 0xFF; frame.data[1] = 0xFF; frame.data[2] = 0xFF; frame.data[3] = 0xFF;
      frame.data[4] = 0xFF; frame.data[5] = 0xFF; frame.data[6] = 0xFF; frame.data[7] = 0xFD;

      write(can_socket_, &frame, sizeof(struct can_frame));
      RCLCPP_INFO(this->get_logger(), "DISABLE command sent. Motor relaxed.");

      close(can_socket_);
      RCLCPP_INFO(this->get_logger(), "CAN Socket closed safely.");
    }
  }

private:
  int can_socket_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanTestNode>());
  rclcpp::shutdown();
  return 0;
}