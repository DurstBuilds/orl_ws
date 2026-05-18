#include <cerrno>
#include <cmath>
#include <cstring>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "rclcpp/rclcpp.hpp"

// Converts a float to an unsigned int, given range and number of bits [cite: 153]
int float_to_uint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    
    // Clamp the limits so we don't overflow the motor's memory [cite: 153]
    if(x < x_min) x = x_min;
    else if(x > x_max) x = x_max;
    
    return (int) ((x - x_min) * ((float)((1 << bits) - 1) / span)); // [cite: 153, 161]
}

class CanTestNode : public rclcpp::Node {
public:
  CanTestNode() : Node("can_test_node"), can_socket_(-1) {
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create socket: %s", std::strerror(errno));
      return;
    }

    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, "can0", IFNAMSIZ - 1);
    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
      RCLCPP_ERROR(this->get_logger(), "can0 not found: %s", std::strerror(errno));
      close(can_socket_);
      can_socket_ = -1;
      return;
    }

    if (!is_can0_up()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "can0 exists but is DOWN. Bring it up, e.g.: "
        "sudo ip link set can0 down && "
        "sudo ip link set can0 type can bitrate 1000000 && "
        "sudo ip link set can0 up");
      close(can_socket_);
      can_socket_ = -1;
      return;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(can_socket_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Bind failed: %s", std::strerror(errno));
      close(can_socket_);
      can_socket_ = -1;
      return;
    }

    RCLCPP_INFO(this->get_logger(), "SUCCESS: Bound to can0. Attempting to Enable Motor...");

    struct can_frame frame{};
    frame.can_id = 0x00;  // Default ID for AK70-10. Verify on your motor label!
    frame.can_dlc = 8;
    frame.data[0] = 0xFF; frame.data[1] = 0xFF; frame.data[2] = 0xFF; frame.data[3] = 0xFF;
    frame.data[4] = 0xFF; frame.data[5] = 0xFF; frame.data[6] = 0xFF; frame.data[7] = 0xFC;

    if (!write_can_frame(frame, "Enable")) {
      log_can_troubleshooting();
      return;
    }
    RCLCPP_INFO(this->get_logger(), "ENABLE command sent. Motor should be energized.");

    send_mit_command(0.0f, 0.0f, 1.5f, 0.1f, 0.0f);
  }

  ~CanTestNode() {
    if (can_socket_ >= 0) {
      // === [NEW] DISABLE MOTOR COMMAND ===
      // Sending 0xFD exits MIT mode and relaxes the motor
      struct can_frame frame;
      frame.can_id = 0x00;
      frame.can_dlc = 8;
      frame.data[0] = 0xFF; frame.data[1] = 0xFF; frame.data[2] = 0xFF; frame.data[3] = 0xFF;
      frame.data[4] = 0xFF; frame.data[5] = 0xFF; frame.data[6] = 0xFF; frame.data[7] = 0xFD;

      write(can_socket_, &frame, sizeof(struct can_frame));
      RCLCPP_INFO(this->get_logger(), "DISABLE command sent. Motor relaxed.");

      close(can_socket_);
      RCLCPP_INFO(this->get_logger(), "CAN Socket closed safely.");
    }
  }

  void send_mit_command(float p_des, float v_des, float kp, float kd, float t_ff) {
    // 1. Hardware-Specific Limits for the AK70-10 
    float P_MIN = -12.5f;   float P_MAX = 12.5f;   // Radians 
    float V_MIN = -50.0f;   float V_MAX = 50.0f;   // Rad/s 
    float T_MIN = -25.0f;   float T_MAX = 25.0f;   // N.m 
    float Kp_MIN = 0.0f;    float Kp_MAX = 500.0f; // Stiffness [cite: 153]
    float Kd_MIN = 0.0f;    float Kd_MAX = 5.0f;   // Damping [cite: 153]

    // 2. Compress the floats into integers [cite: 153]
    // Position gets 16 bits of resolution; everything else gets 12 bits
    int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16); //[cite: 153]
    int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12); //[cite: 154]
    int kp_int = float_to_uint(kp, Kp_MIN, Kp_MAX, 12); //[cite: 154]
    int kd_int = float_to_uint(kd, Kd_MIN, Kd_MAX, 12); //[cite: 154]
    int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12); //[cite: 154]

    // 3. Bit-shifting magic to pack the 8-byte CAN frame [cite: 155]
    struct can_frame frame;
    frame.can_id = 0x00; // Your Motor's CAN ID
    frame.can_dlc = 8;
    
    frame.data[0] = p_int >> 8;                                 // Position high 8 bits [cite: 155]
    frame.data[1] = p_int & 0xFF;                               // Position low 8 bits [cite: 155]
    frame.data[2] = v_int >> 4;                                 // Speed high 8 bits [cite: 155]
    frame.data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);       // Speed low 4 bits, KP high 4 bits [cite: 155]
    frame.data[4] = kp_int & 0xFF;                              // KP low 8 bits [cite: 155]
    frame.data[5] = kd_int >> 4;                                // Kd high 8 bits [cite: 156]
    frame.data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);       // Kd low 4 bits, torque high 4 bits [cite: 156]
    frame.data[7] = t_int & 0xFF;                               // Torque low 8 bits [cite: 156]

    if (write_can_frame(frame, "MIT")) {
      RCLCPP_INFO(this->get_logger(), "Sent MIT Cmd | P: %.2f | V: %.2f", p_des, v_des);
    }
  }

private:
  bool is_can0_up()
  {
    int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0) {
      return false;
    }

    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, "can0", IFNAMSIZ - 1);

    const bool up = ioctl(fd, SIOCGIFFLAGS, &ifr) == 0 &&
                    (ifr.ifr_flags & IFF_UP) != 0;
    close(fd);
    return up;
  }

  bool write_can_frame(const struct can_frame & frame, const char * label)
  {
    if (can_socket_ < 0) {
      return false;
    }

    const ssize_t nbytes = write(can_socket_, &frame, sizeof(frame));
    if (nbytes == static_cast<ssize_t>(sizeof(frame))) {
      return true;
    }

    RCLCPP_ERROR(
      this->get_logger(), "Failed to send %s command (wrote %zd/%zu bytes): %s",
      label, nbytes, sizeof(frame), std::strerror(errno));
    return false;
  }

  void log_can_troubleshooting()
  {
    if (errno == ENETDOWN) {
      RCLCPP_ERROR(
        this->get_logger(),
        "can0 is not running (ENETDOWN). Run: "
        "sudo ip link set can0 type can bitrate 1000000 && sudo ip link set can0 up");
    } else if (errno == ENOBUFS || errno == EAGAIN) {
      RCLCPP_ERROR(
        this->get_logger(),
        "TX queue full / no bus ACK (%s). Check: motor powered, CAN_H/CAN_L wired, "
        "120 ohm termination, bitrate matches motor (usually 1 Mbps), and "
        "'ip -details -statistics link show can0' for TX errors / bus-off.",
        std::strerror(errno));
    }
  }

  int can_socket_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanTestNode>());
  rclcpp::shutdown();
  return 0;
}