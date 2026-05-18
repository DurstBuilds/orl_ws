#include <atomic>
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

// AK70-10 MIT parameters (manual section 5.3, table p.63)
constexpr float kPMin = -12.5f;
constexpr float kPMax = 12.5f;
constexpr float kVMin = -50.0f;
constexpr float kVMax = 50.0f;
constexpr float kTMax = 25.0f;

const char * mit_error_string(int code)
{
  switch (code) {
    case 0: return "No fault";
    case 1: return "Motor over-temperature";
    case 2: return "Over-current";
    case 3: return "Over-voltage";
    case 4: return "Under-voltage";
    case 5: return "Encoder fault";
    case 6: return "MOSFET over-temperature";
    case 7: return "Motor stall";
    default: return "Unknown fault";
  }
}

// Manual p.67: uint_to_float
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
  const float span = x_max - x_min;
  return static_cast<float>(x_int) * span / static_cast<float>((1 << bits) - 1) + x_min;
}

// Manual p.45: motor_receive (servo mode timed upload, frame 0x29)
struct Ak70ServoFeedback
{
  uint32_t can_id{0};
  float position_deg{0.0f};
  float speed_erpm{0.0f};
  float current_a{0.0f};
  int temperature_c{0};
  int error_code{0};
  uint8_t raw[8]{};

  bool motor_receive(const struct can_frame & frame, int expected_motor_id)
  {
    if (!(frame.can_id & CAN_EFF_FLAG) || frame.can_dlc < 7) {
      return false;
    }

    const uint32_t ext_id = frame.can_id & CAN_EFF_MASK;
    const uint8_t function_id = static_cast<uint8_t>((ext_id >> 8) & 0xFF);
    const uint8_t node_id = static_cast<uint8_t>(ext_id & 0xFF);

  // Frame 0x29 = servo mode real-time feedback (manual p.44)
    if (function_id != 0x29 || static_cast<int>(node_id) != expected_motor_id) {
      return false;
    }

    for (int i = 0; i < 8; ++i) {
      raw[i] = i < frame.can_dlc ? frame.data[i] : 0;
    }

    can_id = ext_id;
    const int16_t pos_int = static_cast<int16_t>((frame.data[0] << 8) | frame.data[1]);
    const int16_t spd_int = static_cast<int16_t>((frame.data[2] << 8) | frame.data[3]);
    const int16_t cur_int = static_cast<int16_t>((frame.data[4] << 8) | frame.data[5]);

    position_deg = static_cast<float>(pos_int) * 0.1f;
    speed_erpm = static_cast<float>(spd_int) * 10.0f;
    current_a = static_cast<float>(cur_int) * 0.01f;
    temperature_c = static_cast<int8_t>(frame.data[6]);
    error_code = frame.can_dlc >= 8 ? static_cast<int>(frame.data[7]) : 0;

    return true;
  }
};

// Manual p.66-67: unpack_reply (MIT mode)
struct Ak70MitFeedback
{
  int drive_id{0};
  uint32_t can_id{0};
  float position_rad{0.0f};
  float velocity_rad_s{0.0f};
  float torque_nm{0.0f};
  float temperature_c{0.0f};
  int error_code{0};
  uint8_t raw[8]{};

  bool unpack_reply(const struct can_frame & frame, int expected_drive_id)
  {
    if (frame.can_dlc < 7) {
      return false;
    }

    for (int i = 0; i < 8; ++i) {
      raw[i] = i < frame.can_dlc ? frame.data[i] : 0;
    }

    can_id = frame.can_id & CAN_SFF_MASK;
    drive_id = frame.data[0];

    // MIT feedback CAN ID = 0x00 + drive ID (manual p.61)
    if (static_cast<int>(can_id) != expected_drive_id ||
        drive_id != expected_drive_id)
    {
      return false;
    }

    const int p_int = (frame.data[1] << 8) | frame.data[2];
    const int v_int = (frame.data[3] << 4) | (frame.data[4] >> 4);
    const int i_int = ((frame.data[4] & 0x0F) << 8) | frame.data[5];
    const int t_int = frame.data[6];

    position_rad = uint_to_float(p_int, kPMin, kPMax, 16);
    velocity_rad_s = uint_to_float(v_int, kVMin, kVMax, 12);
    torque_nm = uint_to_float(i_int, -kTMax, kTMax, 12);
    temperature_c = static_cast<float>(t_int) - 40.0f;  // manual: range -40~215 C
    error_code = frame.can_dlc >= 8 ? static_cast<int>(frame.data[7]) : 0;

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
    motor_id_ = declare_parameter<int>("motor_id", 1);  // manual default drive ID is 1
    print_raw_ = declare_parameter<bool>("print_raw", true);

    if (!open_can_socket()) {
      return;
    }

    running_ = true;
    rx_thread_ = std::thread(&MotorFeedbackNode::rx_loop, this);

    RCLCPP_INFO(
      get_logger(),
      "Receive-only on %s (motor/drive ID %d). "
      "MIT CAN ID 0x%02X, servo periodic extended ID 0x%04X.",
      can_interface_.c_str(), motor_id_, motor_id_, (0x29 << 8) | motor_id_);
  }

  ~MotorFeedbackNode() override
  {
    running_ = false;
    if (can_socket_ >= 0) {
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

      Ak70MitFeedback mit_fb;
      if (mit_fb.unpack_reply(frame, motor_id_)) {
        print_mit_feedback(mit_fb);
        continue;
      }

      Ak70ServoFeedback servo_fb;
      if (servo_fb.motor_receive(frame, motor_id_)) {
        print_servo_feedback(servo_fb);
      }
    }
  }

  void print_mit_feedback(const Ak70MitFeedback & fb)
  {
    const double position_deg = fb.position_rad * 180.0 / M_PI;

    std::ostringstream out;
    out << std::fixed << std::setprecision(4);
    out << "\n========== AK70-10 MIT Feedback ==========\n";
    out << "CAN ID:        0x" << std::hex << std::uppercase << fb.can_id << std::dec << '\n';
    out << "Drive ID:      " << fb.drive_id << '\n';
    out << "Position:      " << fb.position_rad << " rad  (" << position_deg << " deg)\n";
    out << "Velocity:      " << fb.velocity_rad_s << " rad/s\n";
    out << "Torque:        " << fb.torque_nm << " Nm\n";
    out << "Temperature:   " << fb.temperature_c << " C\n";
    out << "Error code:    " << fb.error_code << " (" << mit_error_string(fb.error_code) << ")\n";
    if (print_raw_) {
      out << "Raw bytes:     " << format_raw(fb.raw) << '\n';
    }
    out << "==========================================";

    RCLCPP_INFO(get_logger(), "%s", out.str().c_str());
  }

  void print_servo_feedback(const Ak70ServoFeedback & fb)
  {
    std::ostringstream out;
    out << std::fixed << std::setprecision(4);
    out << "\n========== AK70-10 Servo Feedback (0x29) ==========\n";
    out << "CAN ID:        0x" << std::hex << std::uppercase << fb.can_id << std::dec << '\n';
    out << "Position:      " << fb.position_deg << " deg\n";
    out << "Speed:         " << fb.speed_erpm << " ERPM\n";
    out << "Current:       " << fb.current_a << " A\n";
    out << "Temperature:   " << fb.temperature_c << " C\n";
    out << "Error code:    " << fb.error_code << " (" << mit_error_string(fb.error_code) << ")\n";
    if (print_raw_) {
      out << "Raw bytes:     " << format_raw(fb.raw) << '\n';
    }
    out << "===================================================";

    RCLCPP_INFO(get_logger(), "%s", out.str().c_str());
  }

  std::string can_interface_;
  int motor_id_{1};
  bool print_raw_{true};

  int can_socket_;
  std::atomic<bool> running_;
  std::thread rx_thread_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorFeedbackNode>());
  rclcpp::shutdown();
  return 0;
}
