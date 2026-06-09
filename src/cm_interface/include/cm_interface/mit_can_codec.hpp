#ifndef CM_INTERFACE__MIT_CAN_CODEC_HPP_
#define CM_INTERFACE__MIT_CAN_CODEC_HPP_

#include <cmath>
#include <cstdint>
#include <cstring>

#include <linux/can.h>

#include "cm_interface/motor_mit_profile.hpp"

namespace cm_interface
{

inline constexpr int kDefaultMasterCanId = 0;
inline constexpr float kCmdZeroEps = 1e-6f;
inline constexpr int kPositionApplyBit = 0x8000;

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

inline float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
  const float span = x_max - x_min;
  return static_cast<float>(x_int) * span / static_cast<float>((1 << bits) - 1) + x_min;
}

inline int arbitration_id_from_frame(const struct can_frame & frame)
{
  const canid_t raw = frame.can_id;
  if (raw & CAN_EFF_FLAG) {
    return static_cast<int>(raw & CAN_EFF_MASK);
  }
  return static_cast<int>(raw & CAN_SFF_MASK);
}

inline int motor_id_from_feedback_data(uint8_t data0, int expected_drive_id)
{
  const int id_low_nibble = static_cast<int>(data0 & 0x0F);
  if (id_low_nibble == expected_drive_id) {
    return id_low_nibble;
  }
  return static_cast<int>(data0);
}

inline bool feedback_frame_matches_drive(const struct can_frame & frame, int expected_drive_id)
{
  if (frame.can_dlc < 7) {
    return false;
  }

  const int arb_id = arbitration_id_from_frame(frame);
  const int motor_id = motor_id_from_feedback_data(frame.data[0], expected_drive_id);
  if (motor_id != expected_drive_id) {
    return false;
  }

  return arb_id == expected_drive_id || arb_id == kDefaultMasterCanId;
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

  bool unpack_reply(
    const struct can_frame & frame,
    int expected_drive_id,
    const MotorMitProfile & profile)
  {
    if (!feedback_frame_matches_drive(frame, expected_drive_id)) {
      return false;
    }

    can_id = static_cast<uint32_t>(arbitration_id_from_frame(frame));
    drive_id = motor_id_from_feedback_data(frame.data[0], expected_drive_id);

    const int p_int = (frame.data[1] << 8) | frame.data[2];
    const int v_int = (frame.data[3] << 4) | (frame.data[4] >> 4);
    const int i_int = ((frame.data[4] & 0x0F) << 8) | frame.data[5];
    const int t_int = frame.data[6];

    position_rad = uint_to_float(p_int, profile.p_min, profile.p_max, 16);
    velocity_rad_s = uint_to_float(v_int, profile.v_min, profile.v_max, 12);
    torque_nm = uint_to_float(i_int, profile.t_min, profile.t_max, 12);
    temperature_c = static_cast<float>(t_int) - 40.0f;
    error_code = frame.can_dlc >= 8 ? static_cast<int>(frame.data[7]) : 0;

    return true;
  }
};

inline int pack_position_continuous(
  float p_delta, float v_des, float t_ff,
  const MotorMitProfile & profile, bool force_apply = false)
{
  const bool hold = std::fabs(p_delta) < kCmdZeroEps &&
    std::fabs(v_des) < kCmdZeroEps &&
    std::fabs(t_ff) < kCmdZeroEps;

  int p_int = float_to_uint(p_delta, profile.p_min, profile.p_max, 15) & 0x7FFF;
  if (force_apply || !hold) {
    p_int |= kPositionApplyBit;
  }
  return p_int;
}

inline void pack_mit_command_frame(
  struct can_frame & frame,
  int can_id,
  float p_delta, float v_des, float kp, float kd, float t_ff,
  const MotorMitProfile & profile, bool force_apply = false)
{
  const int p_int = pack_position_continuous(p_delta, v_des, t_ff, profile, force_apply);
  const int v_int = float_to_uint(v_des, profile.v_min, profile.v_max, 12);
  const int kp_int = float_to_uint(kp, profile.kp_min, profile.kp_max, 12);
  const int kd_int = float_to_uint(kd, profile.kd_min, profile.kd_max, 12);
  const int t_int = float_to_uint(t_ff, profile.t_min, profile.t_max, 12);

  frame.can_id = static_cast<canid_t>(can_id);
  frame.can_dlc = 8;
  frame.data[0] = static_cast<uint8_t>(p_int >> 8);
  frame.data[1] = p_int & 0xFF;
  frame.data[2] = v_int >> 4;
  frame.data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
  frame.data[4] = kp_int & 0xFF;
  frame.data[5] = kd_int >> 4;
  frame.data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
  frame.data[7] = t_int & 0xFF;
}

inline void pack_control_frame(struct can_frame & frame, int can_id, uint8_t cmd_byte)
{
  frame.can_id = static_cast<canid_t>(can_id);
  frame.can_dlc = 8;
  std::memset(frame.data, 0xFF, 7);
  frame.data[7] = cmd_byte;
}

}  // namespace cm_interface

#endif  // CM_INTERFACE__MIT_CAN_CODEC_HPP_
