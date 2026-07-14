// encoder_node: Poll Vishay RAIK060 multi-turn encoder over Linux spidev.
//
// At startup (optional), sends SPI command 0x24 (Apply "Zero position" offset) via an
// advanced bidirectional frame, then polls with simple frames.
//
// Parameters:
//   spi_device                      — spidev path (default /dev/spidev1.2, SPI1 CE2 / GPIO16)
//   spi_speed_hz                    — SPI clock (100 kHz .. 3 MHz)
//   poll_rate_hz                    — publish rate
//   frame_id                        — EncoderState header frame_id
//   cs_delay_us                     — tFCD delay from /CS fall to first SCLK (>= 5 us)
//   apply_zero_position_on_startup  — send 0x24 once at start (default true)
//   log_raw_frames                  — log RX bytes on CRC failure

#include <array>
#include <chrono>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/spi/spidev.h>

#include "encoder_interface/raik060_spi_codec.hpp"
#include "encoder_interface/msg/encoder_state.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

class SpiDevice
{
public:
  SpiDevice(const std::string & device, uint32_t speed_hz, uint16_t cs_delay_us)
  : speed_hz_(speed_hz), cs_delay_us_(cs_delay_us)
  {
    fd_ = open(device.c_str(), O_RDWR);
    if (fd_ < 0) {
      throw std::runtime_error(
        "Failed to open SPI device '" + device + "': " + std::strerror(errno));
    }

    const uint8_t mode = encoder_interface::kSpiMode;
    const uint8_t bits = 8;
    if (ioctl(fd_, SPI_IOC_WR_MODE, &mode) < 0 ||
      ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
      ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz) < 0)
    {
      close_fd();
      throw std::runtime_error(
        "Failed to configure SPI device '" + device + "': " + std::strerror(errno));
    }
  }

  ~SpiDevice()
  {
    close_fd();
  }

  SpiDevice(const SpiDevice &) = delete;
  SpiDevice & operator=(const SpiDevice &) = delete;

  bool read_simple_frame_mt(std::array<uint8_t, encoder_interface::kSimpleFrameClockBytes> & rx)
  {
    std::array<uint8_t, encoder_interface::kSimpleFrameClockBytes> tx{};
    return transfer(tx.data(), rx.data(), encoder_interface::kSimpleFrameClockBytes);
  }

  bool send_command(
    uint8_t command,
    std::array<uint8_t, encoder_interface::kAdvancedFrameClockBytes> & rx)
  {
    const auto tx = encoder_interface::pack_spi_command_frame(command);
    return transfer(tx.data(), rx.data(), encoder_interface::kAdvancedFrameClockBytes);
  }

private:
  bool transfer(const uint8_t * tx, uint8_t * rx, uint32_t len)
  {
    if (fd_ < 0) {
      return false;
    }

    // Assert /CS, wait tFCD (>= 5 us) with no SCLK edges, then clock the frame.
    spi_ioc_transfer delay_xfer{};
    delay_xfer.delay_usecs = cs_delay_us_;
    delay_xfer.cs_change = 0;

    spi_ioc_transfer data_xfer{};
    data_xfer.tx_buf = reinterpret_cast<uint64_t>(tx);
    data_xfer.rx_buf = reinterpret_cast<uint64_t>(rx);
    data_xfer.len = len;
    data_xfer.speed_hz = speed_hz_;
    data_xfer.cs_change = 0;

    std::array<spi_ioc_transfer, 2> transfers{delay_xfer, data_xfer};
    if (ioctl(fd_, SPI_IOC_MESSAGE(static_cast<int>(transfers.size())), transfers.data()) < 0) {
      return false;
    }
    return true;
  }

  void close_fd()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  int fd_{-1};
  uint32_t speed_hz_{1000000};
  uint16_t cs_delay_us_{10};
};

class EncoderNode : public rclcpp::Node
{
public:
  EncoderNode()
  : Node("encoder_node")
  {
    spi_device_ = declare_parameter<std::string>("spi_device", "/dev/spidev1.2");
    spi_speed_hz_ = declare_parameter<int>("spi_speed_hz", 500000);
    poll_rate_hz_ = declare_parameter<double>("poll_rate_hz", 100.0);
    frame_id_ = declare_parameter<std::string>("frame_id", "encoder_link");
    cs_delay_us_ = declare_parameter<int>("cs_delay_us", 20);
    apply_zero_position_on_startup_ =
      declare_parameter<bool>("apply_zero_position_on_startup", true);
    log_raw_frames_ = declare_parameter<bool>("log_raw_frames", false);

    if (spi_speed_hz_ < 100000 || spi_speed_hz_ > 3000000) {
      RCLCPP_WARN(
        get_logger(),
        "spi_speed_hz=%d outside datasheet range [100000, 3000000]",
        spi_speed_hz_);
    }
    if (poll_rate_hz_ <= 0.0) {
      throw std::runtime_error("poll_rate_hz must be positive");
    }

    spi_ = std::make_unique<SpiDevice>(
      spi_device_, static_cast<uint32_t>(spi_speed_hz_),
      static_cast<uint16_t>(cs_delay_us_));

    if (apply_zero_position_on_startup_) {
      apply_zero_position_offset();
    }

    publisher_ = create_publisher<encoder_interface::msg::EncoderState>("encoder/state", 10);

    const auto period = std::chrono::duration<double>(1.0 / poll_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&EncoderNode::poll_once, this));

    RCLCPP_INFO(
      get_logger(),
      "RAIK060 encoder node started: device=%s speed=%d Hz poll=%.1f Hz zero_on_startup=%s",
      spi_device_.c_str(), spi_speed_hz_, poll_rate_hz_,
      apply_zero_position_on_startup_ ? "true" : "false");
  }

private:
  void apply_zero_position_offset()
  {
    using encoder_interface::kCmdApplyZeroPosition;

    std::array<uint8_t, encoder_interface::kAdvancedFrameClockBytes> rx{};
    if (!spi_->send_command(kCmdApplyZeroPosition, rx)) {
      throw std::runtime_error(
        std::string("SPI 0x24 Apply Zero position failed on ") + spi_device_ + ": " +
        std::strerror(errno));
    }

    const auto ack = encoder_interface::parse_advanced_command_ack(rx);
    if (!encoder_interface::spi_command_ack_ok(ack, kCmdApplyZeroPosition)) {
      throw std::runtime_error(
        "SPI 0x24 Apply Zero position ack invalid "
        "(expected RW|cmd=0xA4 data=0x00 with CRC B valid; got RW|cmd=0x" +
        to_hex2(ack.rw_command) + " data=0x" + to_hex2(ack.data) +
        " crc_b_valid=" + (ack.crc_b_valid ? "true" : "false") + ")");
    }

    // Allow NVM write / position update to settle before poll loop.
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    RCLCPP_INFO(
      get_logger(),
      "Applied encoder Zero position offset (SPI cmd 0x24) at current shaft angle");
  }

  static std::string to_hex2(uint8_t value)
  {
    static constexpr char kHex[] = "0123456789abcdef";
    return std::string{kHex[(value >> 4) & 0x0F], kHex[value & 0x0F]};
  }

  void poll_once()
  {
    std::array<uint8_t, encoder_interface::kSimpleFrameClockBytes> rx{};
    if (!spi_->read_simple_frame_mt(rx)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "SPI read failed on %s: %s", spi_device_.c_str(), std::strerror(errno));
      publish_invalid();
      return;
    }

    const auto frame = encoder_interface::parse_simple_frame_mt(rx);

    encoder_interface::msg::EncoderState msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.position_counts = frame.position_counts;
    msg.turn_count = frame.turn_count;
    msg.angle_rad = encoder_interface::counts_to_angle_rad(frame.position_counts);
    msg.total_angle_rad = encoder_interface::total_angle_rad(frame.turn_count, frame.position_counts);
    msg.error_active = !frame.error_ok;
    msg.warning_active = !frame.warning_ok;
    msg.crc_valid = frame.crc_valid;

    if (!frame.crc_valid) {
      if (log_raw_frames_) {
        RCLCPP_WARN(
          get_logger(),
          "Encoder CRC invalid (turn=%d pos=%u) raw=[%02x %02x %02x %02x %02x %02x]",
          frame.turn_count, frame.position_counts,
          rx[0], rx[1], rx[2], rx[3], rx[4], rx[5]);
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Encoder CRC invalid (turn=%d pos=%u); try log_raw_frames:=true, "
          "cs_delay_us:=50, or spi_speed_hz:=250000",
          frame.turn_count, frame.position_counts);
      }
    }

    publisher_->publish(msg);
  }

  void publish_invalid()
  {
    encoder_interface::msg::EncoderState msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.crc_valid = false;
    publisher_->publish(msg);
  }

  std::string spi_device_;
  int spi_speed_hz_{1000000};
  double poll_rate_hz_{100.0};
  std::string frame_id_;
  int cs_delay_us_{20};
  bool apply_zero_position_on_startup_{true};
  bool log_raw_frames_{false};

  std::unique_ptr<SpiDevice> spi_;
  rclcpp::Publisher<encoder_interface::msg::EncoderState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<EncoderNode>());
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(rclcpp::get_logger("encoder_node"), "Fatal error: %s", ex.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
