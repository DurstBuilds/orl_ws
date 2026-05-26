#ifndef CM_INTERFACE__MOTOR_MIT_PROFILE_HPP_
#define CM_INTERFACE__MOTOR_MIT_PROFILE_HPP_

#include <cmath>
#include <stdexcept>
#include <string>

namespace cm_interface
{

struct MotorMitProfile
{
  const char * name;
  float p_min;
  float p_max;
  float v_min;
  float v_max;
  float t_min;
  float t_max;
  float kp_min;
  float kp_max;
  float kd_min;
  float kd_max;
  float pd_kp;       // joint_translator PD position gain (per tick at loop_hz)
  float pd_kd;       // joint_translator PD velocity damping (per tick at loop_hz)
  float pdelta_max;  // max |deltaP| per tick (rad); tuned for 200 Hz loop
  float mit_kp;      // default MIT Kp for motor_command messages
  float mit_kd;      // default MIT Kd for motor_command messages
};

inline constexpr MotorMitProfile kAk70_10{
  "AK70-10",
  -static_cast<float>(M_PI),
  static_cast<float>(M_PI),
  -50.0f, 50.0f,
  -25.0f, 25.0f,
  0.0f, 500.0f,
  0.0f, 5.0f,
  0.1f, 0.005f,
  0.05f,
  4.0f, 0.02f,
};

inline constexpr MotorMitProfile kAk10_9{
  "AK10-9",
  -static_cast<float>(M_PI),
  static_cast<float>(M_PI),
  -50.0f, 50.0f,
  -65.0f, 65.0f,
  0.0f, 500.0f,
  0.0f, 5.0f,
  0.025f, 0.002f,
  0.05f,
  10.0f, 0.1f,
};

inline constexpr MotorMitProfile kAk80_64{
  "AK80-64",
  -static_cast<float>(M_PI),
  static_cast<float>(M_PI),
  -8.0f, 8.0f,
  -144.0f, 144.0f,
  0.0f, 500.0f,
  0.0f, 5.0f,
  0.015f, 0.00001f,
  0.02f,
  50.0f, 0.25f,
};

inline const MotorMitProfile & get_motor_mit_profile(const std::string & motor_model)
{
  if (motor_model == "ak70_10") {
    return kAk70_10;
  }
  if (motor_model == "ak10_9") {
    return kAk10_9;
  }
  if (motor_model == "ak80_64") {
    return kAk80_64;
  }

  throw std::invalid_argument(
    "Unknown motor_model '" + motor_model +
    "'. Use ak70_10, ak10_9, or ak80_64.");
}

}  // namespace cm_interface

#endif  // CM_INTERFACE__MOTOR_MIT_PROFILE_HPP_
