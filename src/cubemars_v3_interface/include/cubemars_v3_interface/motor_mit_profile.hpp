#ifndef CUBEMARS_V3_INTERFACE__MOTOR_MIT_PROFILE_HPP_
#define CUBEMARS_V3_INTERFACE__MOTOR_MIT_PROFILE_HPP_

// Motor-specific scaling and default MIT gains for V3 firmware.
// Limits must match the drive's advertised MIT ranges; mismatch silently
// saturates commanded values in float_to_uint() and produces wrong torque.
// TWEAK: add new motors here and in get_motor_mit_profile(); rebuild required.

#include <stdexcept>
#include <string>

namespace cubemars_v3_interface
{

/// Quantization bounds and suggested default gains for one motor family.
///
/// p/v/t/kp/kd min/max define the float↔uint mapping used on the wire.
/// mit_kp / mit_kd are conveniences for higher-level controllers; the
/// gateway itself forwards whatever gains arrive in MotorCommand.
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

/// AK60-6 ranges from the CubeMars AK 3.0 MIT tables (±4π rad position).
inline constexpr MotorMitProfile kAk60_6{
  "AK60-6",
  -12.56f, 12.56f,
  -60.0f, 60.0f,
  -12.0f, 12.0f,
  0.0f, 500.0f,
  0.0f, 5.0f,
  2.0f, 1.0f,
};

/// Resolve a launch/param motor_model string to its MIT profile.
/// Unknown names fail fast at node construction rather than on first TX.
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
