#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cerrno>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>

#include <fcntl.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "rclcpp/rclcpp.hpp"

namespace
{

constexpr float kPMin = -12.5f;
constexpr float kPMax = 12.5f;
constexpr float kVMin = -50.0f;
constexpr float kVMax = 50.0f;
constexpr float kTMin = -25.0f;
constexpr float kTMax = 25.0f;
constexpr float kKtActual = 0.122f;  // AK70-10 Nm/A (CubeMars / TMotor tables)

const char * error_string(int code)
{
  switch (code) {
    case 0: return "No Error";
    case 1: return "Over temperature";
    case 2: return "Over current";
    case 3: return "Over voltage";
    case 4: return "Under voltage";
    case 5: return "Encoder fault";
    case 6: return "Phase current unbalance";
    default: return "Unknown error";
  }
}

float uint_to_float(int x, float x_min, float x_max, int bits)
{
  const float span = x_max - x_min;
  return static_cast<float>(x) * span / static_cast<float>((1 << bits) - 1) + x_min;
}

int float_to_uint(float x, float x_min, float x_max, int bits)
{
  if (x < x_min) {
    x = x_min;
  } else if (x > x_max) {
    x = x_max;
  }
  const float span = x_max - x_min;
  return static_cast<int>((x - x_min) * static_cast<float>((1 << bits) - 1) / span);
}

struct MitFeedback
{
  int motor_id{0};
  uint32_t can_id{0};
  float position_rad{0.0f};
  float velocity_rad_s{0.0f};
  float torque_nm{0.0f};
  float current_a{0.0f};
  int temperature_c{0};
  int error_code{0};
  uint8_t raw[8]{};

  bool decode(const struct can_frame & frame, int expected_motor_id)
  {
    if (frame.can_dlc < 6) {
      return false;
    }

    for (int i = 0; i < 8; ++i) {
      raw[i] = i < frame.can_dlc ? frame.data[i] : 0;
    }

    motor_id = frame.data[0];
    can_id = frame.can_id & CAN_SFF_MASK;

    if (motor_id != expected_motor_id && static_cast<int>(can_id) != expected_motor_id) {
      return false;
    }

    const int position_uint = (frame.data[1] << 8) | frame.data[2];
    const int velocity_uint = (frame.data[3] << 4) | (frame.data[4] >> 4);
    const int torque_uint = ((frame.data[4] & 0x0F) << 8) | frame.data[5];

    position_rad = uint_to_float(position_uint, kPMin, kPMax, 16);
    velocity_rad_s = uint_to_float(velocity_uint, kVMin, kVMax, 12);
    torque_nm = uint_to_float(torque_uint, kTMin, kTMax, 12);
    current_a = torque_nm / kKtActual;

    if (frame.can_dlc >= 8) {
      temperature_c = static_cast<int>(frame.data[6]);
      error_code = static_cast<int>(frame.data[7]);
    } else {
      temperature_c = 0;
      error_code = 0;
    }

    return true;
  }
};

std::string format_raw(const uint8_t * data)
{
  std::ostringstream oss;
  oss << std::hex << std::uppercase << std::setfill('0');
  for (int i = 0; i < 8; ++i) {
    if (i > 0) {
      oss << ' ';
    }
    oss << std::setw(2) << static_cast<int>(data[i]);
  }
  return oss.str();
}

}  // namespace

class MotorFeedbackNode : public rclcpp::Node
{
public:
  MotorFeedbackNode()
  : Node("motor_feedback_node"),
    can_socket_(-1),
    running_(false)
  {
    can_interface_ = declare_parameter<std::string>("can_interface", "can0");
    motor_id_ = declare_parameter<int>("motor_id", 0);
    poll_rate_hz_ = declare_parameter<double>("poll_rate_hz", 50.0);
    auto_enable_ = declare_parameter<bool>("auto_enable", true);
    print_raw_ = declare_parameter<bool>("print_raw", true);

    if (!open_can_socket()) {
      return;
    }

    running_ = true;
    rx_thread_ = std::thread(&MotorFeedbackNode::rx_loop, this);

    if (auto_enable_) {
      send_enable();
    }

    const auto period = std::chrono::duration<double>(1.0 / std::max(poll_rate_hz_, 1.0));
    poll_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&MotorFeedbackNode::poll_motor, this));

    RCLCPP_INFO(
      get_logger(),
      "Listening on %s for AK70-10 motor ID %d at %.1f Hz (send MIT commands to receive feedback)",
      can_interface_.c_str(), motor_id_, poll_rate_hz_);
  }

  ~MotorFeedbackNode() override
  {
    running_ = false;
    if (can_socket_ >= 0) {
      send_disable();
      shutdown(can_socket_, SHUT_RDWR);
    }
    if (rx_thread_.joinable()) {
      rx_thread_.join();
    }
    if (can_socket_ >= 0) {
      close(can_socket_);
    }
  }

private:
  bool open_can_socket()
  {
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to create CAN socket: %s", std::strerror(errno));
      return false;
    }

    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);

    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
      RCLCPP_ERROR(get_logger(), "%s not found: %s", can_interface_.c_str(), std::strerror(errno));
      close(can_socket_);
      can_socket_ = -1;
      return false;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(can_socket_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
      RCLCPP_ERROR(get_logger(), "Bind to %s failed: %s", can_interface_.c_str(), std::strerror(errno));
      close(can_socket_);
      can_socket_ = -1;
      return false;
    }

    const int flags = fcntl(can_socket_, F_GETFL, 0);
    if (flags >= 0) {
      fcntl(can_socket_, F_SETFL, flags | O_NONBLOCK);
    }

    return true;
  }

  bool write_frame(const struct can_frame & frame, const char * label)
  {
    const ssize_t nbytes = write(can_socket_, &frame, sizeof(frame));
    if (nbytes == static_cast<ssize_t>(sizeof(frame))) {
      return true;
    }

    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Failed to send %s frame: %s", label, std::strerror(errno));
    return false;
  }

  void send_enable()
  {
    struct can_frame frame{};
    frame.can_id = static_cast<canid_t>(motor_id_);
    frame.can_dlc = 8;
    frame.data[0] = 0xFF;
    frame.data[1] = 0xFF;
    frame.data[2] = 0xFF;
    frame.data[3] = 0xFF;
    frame.data[4] = 0xFF;
    frame.data[5] = 0xFF;
    frame.data[6] = 0xFF;
    frame.data[7] = 0xFC;

    if (write_frame(frame, "enable")) {
      RCLCPP_INFO(get_logger(), "Sent MIT enable (0xFC) to motor %d", motor_id_);
    }
  }

  void send_disable()
  {
    struct can_frame frame{};
    frame.can_id = static_cast<canid_t>(motor_id_);
    frame.can_dlc = 8;
    frame.data[0] = 0xFF;
    frame.data[1] = 0xFF;
    frame.data[2] = 0xFF;
    frame.data[3] = 0xFF;
    frame.data[4] = 0xFF;
    frame.data[5] = 0xFF;
    frame.data[6] = 0xFF;
    frame.data[7] = 0xFD;
    write_frame(frame, "disable");
  }

  void poll_motor()
  {
    // MIT mode is request/response: send a zero-torque hold command to trigger feedback.
    struct can_frame frame{};
    frame.can_id = static_cast<canid_t>(motor_id_);
    frame.can_dlc = 8;

    const int p_int = float_to_uint(0.0f, kPMin, kPMax, 16);
    const int v_int = float_to_uint(0.0f, kVMin, kVMax, 12);
    const int kp_int = float_to_uint(0.0f, 0.0f, 500.0f, 12);
    const int kd_int = float_to_uint(0.5f, 0.0f, 5.0f, 12);
    const int t_int = float_to_uint(0.0f, kTMin, kTMax, 12);

    frame.data[0] = static_cast<uint8_t>(p_int >> 8);
    frame.data[1] = static_cast<uint8_t>(p_int & 0xFF);
    frame.data[2] = static_cast<uint8_t>(v_int >> 4);
    frame.data[3] = static_cast<uint8_t>(((v_int & 0xF) << 4) | (kp_int >> 8));
    frame.data[4] = static_cast<uint8_t>(kp_int & 0xFF);
    frame.data[5] = static_cast<uint8_t>(kd_int >> 4);
    frame.data[6] = static_cast<uint8_t>(((kd_int & 0xF) << 4) | (t_int >> 8));
    frame.data[7] = static_cast<uint8_t>(t_int & 0xFF);

    write_frame(frame, "MIT poll");
  }

  void rx_loop()
  {
    while (running_ && rclcpp::ok()) {
      struct pollfd pfd;
      pfd.fd = can_socket_;
      pfd.events = POLLIN;

      const int poll_result = poll(&pfd, 1, 100);
      if (poll_result < 0) {
        if (errno == EINTR) {
          continue;
        }
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000, "poll() failed: %s", std::strerror(errno));
        continue;
      }
      if (poll_result == 0) {
        continue;
      }

      struct can_frame frame{};
      const ssize_t nbytes = read(can_socket_, &frame, sizeof(frame));
      if (nbytes < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          continue;
        }
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000, "read() failed: %s", std::strerror(errno));
        continue;
      }
      if (nbytes != static_cast<ssize_t>(sizeof(frame))) {
        continue;
      }

      MitFeedback fb;
      if (!fb.decode(frame, motor_id_)) {
        continue;
      }

      print_feedback(fb);
    }
  }

  void print_feedback(const MitFeedback & fb)
  {
    const double position_deg = fb.position_rad * 180.0 / M_PI;

    std::ostringstream out;
    out << std::fixed << std::setprecision(4);
    out << "\n========== AK70-10 MIT Feedback ==========\n";
    out << "CAN ID:        0x" << std::hex << std::uppercase << fb.can_id << std::dec << '\n';
    out << "Motor ID:      " << fb.motor_id << '\n';
    out << "Position:      " << fb.position_rad << " rad  (" << position_deg << " deg)\n";
    out << "Velocity:      " << fb.velocity_rad_s << " rad/s\n";
    out << "Torque:        " << fb.torque_nm << " Nm\n";
    out << "Current (est): " << fb.current_a << " A\n";
    out << "Temperature:   " << fb.temperature_c << " C\n";
    out << "Error code:    " << fb.error_code << " (" << error_string(fb.error_code) << ")\n";
    if (print_raw_) {
      out << "Raw bytes:     " << format_raw(fb.raw) << '\n';
    }
    out << "==========================================";

    RCLCPP_INFO(get_logger(), "%s", out.str().c_str());
  }

  std::string can_interface_;
  int motor_id_{0};
  double poll_rate_hz_{50.0};
  bool auto_enable_{true};
  bool print_raw_{true};

  int can_socket_;
  std::atomic<bool> running_;
  std::thread rx_thread_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorFeedbackNode>());
  rclcpp::shutdown();
  return 0;
}
