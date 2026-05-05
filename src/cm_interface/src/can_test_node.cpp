#include <iostream>            // Standard library for basic console input/output
#include <cstring>             // Library for string functions like strcpy
#include <sys/socket.h>        // Core Linux library for socket communication
#include <sys/ioctl.h>         // Library for device-specific I/O control operations
#include <net/if.h>            // Library for network interface structures (like can0)
#include <linux/can.h>         // Linux kernel headers for CAN protocol definitions
#include <linux/can/raw.h>     // Linux kernel headers for raw CAN socket access
#include <unistd.h>            // Library for standard symbolic constants (like close())
#include "rclcpp/rclcpp.hpp"    // Core ROS2 C++ library to make this a ROS node

// Define a class that inherits from rclcpp::Node to get all ROS2 features
class CanTestNode : public rclcpp::Node {
public:
  // Constructor: This runs once when the node is launched
  CanTestNode() : Node("can_test_node") {
    
    // Step 1: Create a Raw CAN Socket
    // PF_CAN: Protocol Family CAN
    // SOCK_RAW: We want the raw data frames, no fancy filtering yet
    // CAN_RAW: Use the standard raw CAN protocol
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    // If the socket returns a negative number, the OS failed to open the driver
    if (can_socket_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create socket. Is SocketCAN installed?");
      return; // Stop initialization if we can't even open the socket
    }

    // Step 2: Locate the 'can0' interface on the Pi 5
    struct ifreq ifr;                     // Create an 'interface request' structure
    std::strcpy(ifr.ifr_name, "can0");    // Tell the structure we are looking for "can0"
    
    // ioctl (Input/Output Control) asks the OS for the 'Index Number' of can0
    // The hardware doesn't understand "can0", it only understands index numbers like '1' or '2'
    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
      RCLCPP_ERROR(this->get_logger(), "can0 not found. Did you run 'sudo ip link set can0 up'?");
      return;
    }

    // Step 3: Bind the socket to the hardware interface
    struct sockaddr_can addr;             // Create a CAN-specific address structure
    addr.can_family = AF_CAN;             // Set the address family to CAN
    addr.can_ifindex = ifr.ifr_ifindex;   // Provide the index number we just found

    // bind() "locks" our C++ code onto the physical pins of the CAN hat
    if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Bind failed. Is another program using can0?");
    } else {
      // If we reach this line, the Pi 5 and your code are officially shaking hands
      RCLCPP_INFO(this->get_logger(), "SUCCESS: C++ Node is now bound to can0!");
    }
  }

  // Destructor: This runs when the node is killed (Ctrl+C)
  ~CanTestNode() {
    // Always close your sockets to prevent "Memory Leaks" or locking the hardware
    if (can_socket_ >= 0) {
      close(can_socket_);
      RCLCPP_INFO(this->get_logger(), "CAN Socket closed safely.");
    }
  }

private:
  int can_socket_; // This integer stores the "ID" of our connection to the CAN bus
};

// The main function where the program actually starts execution
int main(int argc, char ** argv) {
  // Initialize the ROS2 communication system
  rclcpp::init(argc, argv);
  
  // Create the node and keep it running in a loop
  rclcpp::spin(std::make_shared<CanTestNode>());
  
  // Shut down ROS2 cleanly when the loop is broken
  rclcpp::shutdown();
  
  return 0; // Return 0 to indicate the program finished successfully
}