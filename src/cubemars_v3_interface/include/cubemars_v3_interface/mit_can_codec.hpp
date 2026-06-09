#ifndef CUBEMARS_V3_INTERFACE__MIT_CAN_CODEC_HPP_
#define CUBEMARS_V3_INTERFACE__MIT_CAN_CODEC_HPP_

// V3 firmware MIT codec (AK 3.0 manual section 4.2).
// Extended CAN ID: (control_mode << 8) | drive_id, MIT control mode = 8.

#include <cstdint>

#include <linux/can.h>

#include "cubemars_v3_interface/motor_mit_profile.hpp"

namespace cubemars_v3_interface
{

inline constexpr uint32_t kMitControlModeId = 8;

inline const char * mit_error_string(int code)
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

inline int float_to_uint(float x, float x_min, float x_max, int bits)
{
  if (x < x_min) {
    x = x_min;
  }
  if (x > x_max) {
    x = x_max;
  }
  const float span = x_max - x_min;
  return static_cast<int>((x - x_min) * static_cast<float>((1 << bits) - 1) / span);
}

inline int16_t read_int16_be(uint8_t hi, uint8_t lo)
{
  return static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | static_cast<uint16_t>(lo));
}

inline uint32_t make_mit_arbitration_id(int drive_id)
{
  return (kMitControlModeId << 8) | static_cast<uint32_t>(drive_id & 0xFF);
}

inline canid_t make_extended_can_id(int drive_id)
{
  return static_cast<canid_t>(make_mit_arbitration_id(drive_id) | CAN_EFF_FLAG);
}

inline int drive_id_from_frame(const struct can_frame & frame)
{
  const canid_t raw = frame.can_id;
  if ((raw & CAN_EFF_FLAG) == 0) {
    return -1;
  }
  return static_cast<int>(raw & CAN_EFF_MASK) & 0xFF;
}

inline bool feedback_frame_matches_drive(const struct can_frame & frame, int expected_drive_id)
{
  if (frame.can_dlc < 7) {
    return false;
  }
  if ((frame.can_id & CAN_EFF_FLAG) == 0) {
    return false;
  }

  const uint32_t arb_id = static_cast<uint32_t>(frame.can_id & CAN_EFF_MASK);
  const uint32_t expected_arb = make_mit_arbitration_id(expected_drive_id);
  return arb_id == expected_arb;
}

struct MitFeedback
{
  int drive_id{0};
  uint32_t can_id{0};
  float position_rad{0.0f};
  float velocity_rad_s{0.0f};
  float torque_nm{0.0f};
  float temperature_c{0.0f};
  int error_code{0};

  bool unpack_reply(const struct can_frame & frame, int expected_drive_id)
  {
    if (!feedback_frame_matches_drive(frame, expected_drive_id)) {
      return false;
    }

    can_id = static_cast<uint32_t>(frame.can_id & CAN_EFF_MASK);
    drive_id = drive_id_from_frame(frame);

    position_rad = static_cast<float>(read_int16_be(frame.data[0], frame.data[1])) * 0.1f;
    velocity_rad_s = static_cast<float>(read_int16_be(frame.data[2], frame.data[3])) * 10.0f;
    torque_nm = static_cast<float>(read_int16_be(frame.data[4], frame.data[5])) * 0.01f;
    temperature_c = static_cast<float>(frame.data[6]);
    error_code = frame.can_dlc >= 8 ? static_cast<int>(frame.data[7]) : 0;

    return true;
  }
};

inline void pack_mit_command_frame(
  struct can_frame & frame,
  int drive_id,
  float p_des, float v_des, float kp, float kd, float t_ff,
  const MotorMitProfile & profile)
{
  const int p_int = float_to_uint(p_des, profile.p_min, profile.p_max, 16);
  const int v_int = float_to_uint(v_des, profile.v_min, profile.v_max, 12);
  const int kp_int = float_to_uint(kp, profile.kp_min, profile.kp_max, 12);
  const int kd_int = float_to_uint(kd, profile.kd_min, profile.kd_max, 12);
  const int t_int = float_to_uint(t_ff, profile.t_min, profile.t_max, 12);

  frame.can_id = make_extended_can_id(drive_id);
  frame.can_dlc = 8;
  frame.data[0] = static_cast<uint8_t>(kp_int >> 4);
  frame.data[1] = static_cast<uint8_t>(((kp_int & 0xF) << 4) | (kd_int >> 8));
  frame.data[2] = static_cast<uint8_t>(kd_int & 0xFF);
  frame.data[3] = static_cast<uint8_t>(p_int >> 8);
  frame.data[4] = static_cast<uint8_t>(p_int & 0xFF);
  frame.data[5] = static_cast<uint8_t>(v_int >> 4);
  frame.data[6] = static_cast<uint8_t>(((v_int & 0xF) << 4) | (t_int >> 8));
  frame.data[7] = static_cast<uint8_t>(t_int & 0xFF);
}

}  // namespace cubemars_v3_interface

#endif  // CUBEMARS_V3_INTERFACE__MIT_CAN_CODEC_HPP_
