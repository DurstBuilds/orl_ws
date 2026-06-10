#ifndef CM_INTERFACE__MOTOR_MIT_PROFILE_HPP_
#define CM_INTERFACE__MOTOR_MIT_PROFILE_HPP_

// Motor-specific scaling and control defaults for MIT CAN and joint_translator_node.
// TWEAK: add new motors here and in get_motor_mit_profile(); rebuild required.
//
// Fields:
//   name          — human-readable label (logged by nodes)
//   p_min..kd_max — CAN MIT pack/unpack ranges
//   pd_kp, pd_kd   — joint_translator outer-loop gains (per tick at loop_hz)
//   omega_max     — max motor velocity command (rad/s); caps per-tick delta
//   mit_kp, mit_kd — default MIT stiffness on motor_command messages

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
  float omega_max;   // max motor speed command magnitude (rad/s)
  float mit_kp;      // default MIT Kp for motor_command messages
  float mit_kd;      // default MIT Kd for motor_command messages
};

inline constexpr MotorMitProfile kAk70_10{
  "AK70-10",
  -static_cast<float>(M_PI),  // p_min
  static_cast<float>(M_PI),  // p_max
  -50.0f, 50.0f,   // v_min, v_max
  -25.0f, 25.0f,   // t_min, t_max
  0.0f, 500.0f,    // kp_min, kp_max
  0.0f, 5.0f,      // kd_min, kd_max
  0.7f, 0.005f,    // pd_kp, pd_kd
  20.0f,           // omega_max (rad/s)
  10.0f, 0.05f,    // mit_kp, mit_kd (Nm/rad)
};

inline constexpr MotorMitProfile kAk10_9{
  "AK10-9",
  -static_cast<float>(M_PI),  // p_min
  static_cast<float>(M_PI),  // p_max
  -50.0f, 50.0f,  // v_min, v_max
  -65.0f, 65.0f,  // t_min, t_max
  0.0f, 500.0f,  // kp_min, kp_max
  0.0f, 5.0f,    // kd_min, kd_max
  0.1f, 0.0f,    // pd_kp, pd_kd
  5.0f,         // omega_max (rad/s)
  30.0f, 0.1f,   // mit_kp, mit_kd (Nm/rad)
};

inline constexpr MotorMitProfile kAk80_64{
  "AK80-64",
  -static_cast<float>(M_PI), // p_min
  static_cast<float>(M_PI),  // p_max
  -8.0f, 8.0f,      // v_min, v_max
  -144.0f, 144.0f,  // t_min, t_max
  0.0f, 500.0f,     // kp_min, kp_max
  0.0f, 5.0f,       // kd_min, kd_max
  0.030f, 0.0005f,  // pd_kp, pd_kd
  4.0f,             // omega_max (rad/s)
  75.0f, 0.25f,     // mit_kp, mit_kd (Nm/rad)
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
