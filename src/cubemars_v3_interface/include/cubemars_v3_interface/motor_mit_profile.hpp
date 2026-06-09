#ifndef CUBEMARS_V3_INTERFACE__MOTOR_MIT_PROFILE_HPP_
#define CUBEMARS_V3_INTERFACE__MOTOR_MIT_PROFILE_HPP_

// Motor-specific scaling and default MIT gains for V3 firmware.
// TWEAK: add new motors here and in get_motor_mit_profile(); rebuild required.

#include <stdexcept>
#include <string>

namespace cubemars_v3_interface
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
  float mit_kp;
  float mit_kd;
};

inline constexpr MotorMitProfile kAk60_6{
  "AK60-6",
  -12.56f, 12.56f,
  -60.0f, 60.0f,
  -12.0f, 12.0f,
  0.0f, 500.0f,
  0.0f, 5.0f,
  2.0f, 1.0f,
};

inline const MotorMitProfile & get_motor_mit_profile(const std::string & motor_model)
{
  if (motor_model == "ak60_6") {
    return kAk60_6;
  }

  throw std::invalid_argument(
    "Unknown motor_model '" + motor_model + "'. Use ak60_6.");
}

}  // namespace cubemars_v3_interface

#endif  // CUBEMARS_V3_INTERFACE__MOTOR_MIT_PROFILE_HPP_
