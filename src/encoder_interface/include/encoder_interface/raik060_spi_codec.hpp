// raik060_spi_codec.hpp — Vishay RAIK060 multi-turn SPI simple frame parser.
//
// Datasheet: RAIK060 (Document 32602), SPI Mode 1, 44-bit simple frame (MT).

#ifndef ENCODER_INTERFACE__RAIK060_SPI_CODEC_HPP_
#define ENCODER_INTERFACE__RAIK060_SPI_CODEC_HPP_

#include <array>
#include <cmath>
#include <cstdint>

namespace encoder_interface
{

constexpr uint32_t kPositionResolution = 262144;  // 18 bits
constexpr int kSimpleFrameBits = 44;
constexpr int kSimpleFrameClockBytes = 6;         // 48 clocks; first 44 bits used
constexpr uint8_t kSpiMode = 0x01;                // SPI_MODE_1 (CPOL=0, CPHA=1)
constexpr uint8_t kCrcPoly = 0x97;

struct SimpleFrameMt
{
  int16_t turn_count{0};
  uint32_t position_counts{0};
  bool warning_ok{true};   // raw W bit (1 = OK)
  bool error_ok{true};     // raw E bit (1 = OK)
  bool crc_valid{false};
};

inline bool get_bit_msb_first(const std::array<uint8_t, kSimpleFrameClockBytes> & data, int bit_index)
{
  const int byte_index = bit_index / 8;
  const int bit_in_byte = 7 - (bit_index % 8);
  return (data[static_cast<size_t>(byte_index)] >> bit_in_byte) & 0x01U;
}

inline uint8_t compute_crc8_msb_first(uint64_t data, int num_bits)
{
  uint8_t crc = 0;
  for (int bit = num_bits - 1; bit >= 0; --bit) {
    const uint8_t msb = static_cast<uint8_t>((crc >> 7) & 0x01U);
    crc = static_cast<uint8_t>(crc << 1);
    const uint8_t data_bit = static_cast<uint8_t>((data >> bit) & 0x01U);
    if (msb ^ data_bit) {
      crc ^= kCrcPoly;
    }
  }
  return crc;
}

inline SimpleFrameMt parse_simple_frame_mt(const std::array<uint8_t, kSimpleFrameClockBytes> & rx)
{
  SimpleFrameMt frame;

  uint16_t counter_raw = 0;
  for (int bit = 0; bit < 16; ++bit) {
    counter_raw = static_cast<uint16_t>((counter_raw << 1) | (get_bit_msb_first(rx, bit) ? 1U : 0U));
  }
  frame.turn_count = static_cast<int16_t>(counter_raw);

  uint32_t position_raw = 0;
  for (int bit = 16; bit < 34; ++bit) {
    position_raw = (position_raw << 1) | (get_bit_msb_first(rx, bit) ? 1U : 0U);
  }
  frame.position_counts = position_raw & 0x03FFFFU;

  frame.warning_ok = get_bit_msb_first(rx, 34);
  frame.error_ok = get_bit_msb_first(rx, 35);

  uint8_t crc_received = 0;
  for (int bit = 36; bit < 44; ++bit) {
    crc_received = static_cast<uint8_t>((crc_received << 1) | (get_bit_msb_first(rx, bit) ? 1U : 0U));
  }

  const uint64_t crc_payload =
    (static_cast<uint64_t>(counter_raw) << 20) |
    (static_cast<uint64_t>(frame.position_counts) << 2) |
    (static_cast<uint64_t>(frame.warning_ok ? 1U : 0U) << 1) |
    static_cast<uint64_t>(frame.error_ok ? 1U : 0U);

  const uint8_t crc_computed = compute_crc8_msb_first(crc_payload, 36);
  frame.crc_valid = crc_received == static_cast<uint8_t>(~crc_computed);

  return frame;
}

inline double counts_to_angle_rad(uint32_t position_counts)
{
  return static_cast<double>(position_counts) * (2.0 * M_PI / static_cast<double>(kPositionResolution));
}

inline double total_angle_rad(int16_t turn_count, uint32_t position_counts)
{
  return static_cast<double>(turn_count) * 2.0 * M_PI + counts_to_angle_rad(position_counts);
}

}  // namespace encoder_interface

#endif  // ENCODER_INTERFACE__RAIK060_SPI_CODEC_HPP_
