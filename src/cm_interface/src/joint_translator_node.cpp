// joint_translator_node: maps motor total position to joint space and commands
// motor deltas until joint_despos is reached.
//
// When |motor_error| < coarse threshold (default 0.25*pi): 100 Hz hold (0 delta)
// keeps feedback flowing. After lockin_settle_hold_count holds, 5 Hz lock-in sends
// the exact motor error to snap to despos.

#include <atomic>
#include <cmath>
#include <mutex>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "motor_interfaces/msg/motor_command.hpp"
#include "motor_interfaces/msg/motor_total_position.hpp"

namespace
{

constexpr float kDefaultControlHz = 100.0f;
constexpr float kDefaultLockinHz = 5.0f;
constexpr float kDefaultMotorStepMax = 0.1f;
constexpr float kDefaultJointErrorTolerance = 1e-3f;
constexpr float kDefaultMotorErrorCoarseThreshold =
  0.25f * static_cast<float>(M_PI);
constexpr float kDefaultKp = 2.0f;
constexpr float kDefaultKd = 0.02f;
constexpr float kLockinKdScale = 2.0f;
constexpr float kKdMax = 5.0f;
constexpr int kDefaultLockinSettleHoldCount = 10;
constexpr float kMaxMotorDelta = static_cast<float>(M_PI);

float clamp(float value, float low, float high)
{
  if (value < low) {
    return low;
  }
  if (value > high) {
    return high;
  }
  return value;
}

float sign(float value)
{
  if (value > 0.0f) {
    return 1.0f;
  }
  if (value < 0.0f) {
    return -1.0f;
  }
  return 0.0f;
}

}  // namespace

class JointTranslatorNode : public rclcpp::Node
{
public:
  JointTranslatorNode() : Node("joint_translator_node")
  {
    gear_ratio_ = declare_parameter<double>("gear_ratio", 0.0);
    control_hz_ = declare_parameter<double>("control_hz", kDefaultControlHz);
    lockin_hz_ = declare_parameter<double>("lockin_hz", kDefaultLockinHz);
    motor_step_max_ = declare_parameter<double>("motor_step_max", kDefaultMotorStepMax);
    joint_error_tolerance_ = declare_parameter<double>(
      "joint_error_tolerance", kDefaultJointErrorTolerance);
    motor_error_coarse_threshold_ = declare_parameter<double>(
      "motor_error_coarse_threshold", kDefaultMotorErrorCoarseThreshold);
    lockin_settle_hold_count_ = declare_parameter<int>(
      "lockin_settle_hold_count", kDefaultLockinSettleHoldCount);
    kp_ = declare_parameter<double>("kp", kDefaultKp);
    kd_ = declare_parameter<double>("kd", kDefaultKd);

    if (gear_ratio_ <= 0.0) {
      throw std::invalid_argument("gear_ratio must be > 0");
    }
    if (control_hz_ <= 0.0) {
      throw std::invalid_argument("control_hz must be > 0");
    }
    if (lockin_hz_ <= 0.0) {
      throw std::invalid_argument("lockin_hz must be > 0");
    }
    if (lockin_settle_hold_count_ < 0) {
      throw std::invalid_argument("lockin_settle_hold_count must be >= 0");
    }

    motor_total_sub_ = create_subscription<motor_interfaces::msg::MotorTotalPosition>(
      "motor_total_position",
      10,
      std::bind(&JointTranslatorNode::motor_total_position_callback, this, std::placeholders::_1));

    joint_despos_sub_ = create_subscription<std_msgs::msg::Float32>(
      "joint_despos",
      10,
      std::bind(&JointTranslatorNode::joint_despos_callback, this, std::placeholders::_1));

    joint_curpos_pub_ = create_publisher<std_msgs::msg::Float32>("joint_curpos", 10);
    motor_command_pub_ = create_publisher<motor_interfaces::msg::MotorCommand>("motor_command", 10);

    const auto control_period = std::chrono::duration<double>(1.0 / control_hz_);
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(control_period),
      std::bind(&JointTranslatorNode::control_timer_callback, this));

    const auto lockin_period = std::chrono::duration<double>(1.0 / lockin_hz_);
    lockin_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(lockin_period),
      std::bind(&JointTranslatorNode::lockin_timer_callback, this));

    RCLCPP_INFO(
      get_logger(),
      "gear_ratio=%.4f control_hz=%.1f lockin_hz=%.1f motor_step_max=%.4f "
      "joint_error_tolerance=%.4f motor_error_coarse_threshold=%.4f "
      "lockin_settle_hold_count=%d kp=%.2f kd=%.3f",
      gear_ratio_, control_hz_, lockin_hz_, motor_step_max_, joint_error_tolerance_,
      motor_error_coarse_threshold_, lockin_settle_hold_count_, kp_, kd_);
  }

private:
  struct ControlState
  {
    float total_position{0.0f};
    float joint_despos{0.0f};
    bool has_total{false};
    bool has_despos{false};
  };

  ControlState read_state()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return ControlState{
      total_position_, joint_despos_, has_total_position_, has_joint_despos_};
  }

  void motor_total_position_callback(
    const motor_interfaces::msg::MotorTotalPosition::SharedPtr msg)
  {
    const float joint_curpos = static_cast<float>(msg->total_position / gear_ratio_);

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      total_position_ = msg->total_position;
      has_total_position_ = true;
    }

    std_msgs::msg::Float32 out;
    out.data = joint_curpos;
    joint_curpos_pub_->publish(out);
  }

  void joint_despos_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    joint_despos_ = msg->data;
    has_joint_despos_ = true;
    settle_hold_count_.store(0);
  }

  void reset_settle_hold_count()
  {
    settle_hold_count_.store(0);
  }

  float compute_approach_motor_delta(float joint_error) const
  {
    const float motor_step = std::min(
      static_cast<float>(motor_step_max_),
      std::fabs(joint_error) * static_cast<float>(gear_ratio_));
    const float motor_delta = sign(joint_error) * motor_step;
    return clamp(motor_delta, -kMaxMotorDelta, kMaxMotorDelta);
  }

  float compute_snap_motor_delta(float joint_error) const
  {
    const float motor_error = joint_error * static_cast<float>(gear_ratio_);
    return clamp(motor_error, -kMaxMotorDelta, kMaxMotorDelta);
  }

  void publish_motor_command(float motor_delta, float kd_scale = 1.0f)
  {
    motor_interfaces::msg::MotorCommand cmd;
    cmd.position = motor_delta;
    cmd.velocity = 0.0f;
    cmd.torque = 0.0f;
    cmd.kp = static_cast<float>(kp_);
    cmd.kd = clamp(
      static_cast<float>(kd_) * kd_scale, 0.0f, kKdMax);
    motor_command_pub_->publish(cmd);
  }

  // 100 Hz: approach with scaled deltas, or hold (0) once within coarse threshold.
  void control_timer_callback()
  {
    const auto state = read_state();

    if (!state.has_total) {
      publish_motor_command(0.0f);
      warn_no_total_position();
      return;
    }

    if (!state.has_despos) {
      publish_motor_command(0.0f);
      return;
    }

    const float joint_curpos = state.total_position / static_cast<float>(gear_ratio_);
    const float joint_error = state.joint_despos - joint_curpos;
    const float motor_error = joint_error * static_cast<float>(gear_ratio_);
    const float tol = static_cast<float>(joint_error_tolerance_);
    const float coarse = static_cast<float>(motor_error_coarse_threshold_);

    float motor_delta = 0.0f;

    if (std::fabs(joint_error) >= tol) {
      if (std::fabs(motor_error) >= coarse) {
        motor_delta = compute_approach_motor_delta(joint_error);
      } else if (motor_delta == 0.0f &&
        settle_hold_count_.load() < lockin_settle_hold_count_)
      {
        settle_hold_count_.fetch_add(1);
      }
    } else {
      reset_settle_hold_count();
    }

    if (motor_delta != 0.0f) {
      reset_settle_hold_count();
    }

    publish_motor_command(motor_delta);
  }

  // 5 Hz: exact snap deltas when within coarse threshold but not yet at goal.
  void lockin_timer_callback()
  {
    const auto state = read_state();

    if (!state.has_total || !state.has_despos) {
      return;
    }

    const float joint_curpos = state.total_position / static_cast<float>(gear_ratio_);
    const float joint_error = state.joint_despos - joint_curpos;
    const float motor_error = joint_error * static_cast<float>(gear_ratio_);
    const float tol = static_cast<float>(joint_error_tolerance_);
    const float coarse = static_cast<float>(motor_error_coarse_threshold_);

    if (std::fabs(joint_error) < tol) {
      return;
    }
    if (std::fabs(motor_error) >= coarse) {
      return;
    }
    if (settle_hold_count_.load() < lockin_settle_hold_count_) {
      return;
    }

    publish_motor_command(compute_snap_motor_delta(joint_error), kLockinKdScale);
  }

  void warn_no_total_position()
  {
    const auto now = this->now();
    if (!warned_no_total_ ||
      (now - last_warn_time_).seconds() >= 5.0)
    {
      RCLCPP_WARN(get_logger(), "No motor_total_position yet; publishing hold.");
      warned_no_total_ = true;
      last_warn_time_ = now;
    }
  }

  rclcpp::Subscription<motor_interfaces::msg::MotorTotalPosition>::SharedPtr motor_total_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr joint_despos_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr joint_curpos_pub_;
  rclcpp::Publisher<motor_interfaces::msg::MotorCommand>::SharedPtr motor_command_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr lockin_timer_;

  std::mutex state_mutex_;
  float total_position_{0.0f};
  float joint_despos_{0.0f};
  bool has_total_position_{false};
  bool has_joint_despos_{false};

  double gear_ratio_{0.0};
  double control_hz_{kDefaultControlHz};
  double lockin_hz_{kDefaultLockinHz};
  double motor_step_max_{kDefaultMotorStepMax};
  double joint_error_tolerance_{kDefaultJointErrorTolerance};
  double motor_error_coarse_threshold_{kDefaultMotorErrorCoarseThreshold};
  int lockin_settle_hold_count_{kDefaultLockinSettleHoldCount};
  double kp_{kDefaultKp};
  double kd_{kDefaultKd};

  std::atomic<int> settle_hold_count_{0};

  bool warned_no_total_{false};
  rclcpp::Time last_warn_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<JointTranslatorNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("joint_translator_node"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
