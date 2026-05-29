#ifndef DM_MOTOR__TYPES_HPP_
#define DM_MOTOR__TYPES_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace dm_motor
{

enum class MotorMode : uint32_t
{
  kUnknown = 0,
  kMit = 1,
  kPositionVelocity = 2,
  kVelocity = 3,
  kPositionTorque = 4,
};

enum class MotorStatus : uint8_t
{
  kDisabled = 0x0,
  kEnabled = 0x1,
  kOutputEncoderCalibrationError = 0x3,
  kSensorOutputError = 0x4,
  kMotorEncoderCalibrationError = 0x5,
  kOverVoltage = 0x8,
  kUnderVoltage = 0x9,
  kOverCurrent = 0xA,
  kMosOverTemperature = 0xB,
  kRotorOverTemperature = 0xC,
  kCommunicationLost = 0xD,
  kOverload = 0xE,
  kUnknown = 0xFF,
};

struct CanFrame
{
  uint32_t arbitration_id {0U};
  std::array<uint8_t, 64> data {};
  size_t size {0U};
  bool is_extended_id {false};
  bool is_fd {true};
  bool bitrate_switch {true};
};

struct MitLimits
{
  float position_min {-12.5F};
  float position_max {12.5F};
  float velocity_min {-12.0F};
  float velocity_max {12.0F};
  float torque_min {-40.0F};
  float torque_max {40.0F};
  float kp_min {0.0F};
  float kp_max {500.0F};
  float kd_min {0.0F};
  float kd_max {5.0F};
};

inline MitLimits make_model_mit_limits(const std::string & model)
{
  if (model == "4310") {
    return MitLimits {
      -12.5F, 12.5F,
      -30.0F, 30.0F,
      -10.0F, 10.0F,
      0.0F, 500.0F,
      0.0F, 5.0F
    };
  }

  if (model == "4340" || model == "4340P") {
    return MitLimits {
      -12.5F, 12.5F,
      -10.0F, 10.0F,
      -28.0F, 28.0F,
      0.0F, 500.0F,
      0.0F, 5.0F
    };
  }

  if (model == "8009P") {
    return MitLimits {
      -12.5F, 12.5F,
      -25.0F, 25.0F,
      -30.0F, 30.0F,
      0.0F, 500.0F,
      0.0F, 5.0F
    };
  }

  if (model == "10010") {
    return MitLimits {
      -12.5F, 12.5F,
      -12.0F, 12.0F,
      -120.0F, 120.0F,
      0.0F, 500.0F,
      0.0F, 5.0F
    };
  }

  return MitLimits {};
}

struct MotorConfig
{
  std::string name;
  std::string interface_name;
  uint32_t can_id {0x101U};
  uint32_t master_id {0x000U};
  bool use_can_fd {true};
  bool bitrate_switch {true};
  std::string model {"DM-J4340-2EC"};
  std::string notes;
  MitLimits limits {};
  bool has_position_limit {false};
  float upper_position_limit {0.0F};
  float lower_position_limit {0.0F};
  float max_output_torque {0.0F};
  float max_output_speed {0.0F};
  float default_kp {0.0F};
  float default_kd {0.0F};
};

inline std::pair<float, float> make_model_default_pd_gains(const std::string & model)
{
  if (model == "4310") {
    return {10.0F, 1.0F};
  }

  if (model == "4340" || model == "4340P") {
    return {10.0F, 1.0F};
  }

  if (model == "8009P") {
    return {27.0F, 3.7F};
  }

  if (model == "10010") {
    return {27.0F, 3.7F};
  }

  return {0.0F, 0.0F};
}

struct MitCommand
{
  float position {0.0F};
  float velocity {0.0F};
  float kp {0.0F};
  float kd {0.0F};
  float torque {0.0F};
};

struct NamedMitCommand
{
  std::string name;
  MitCommand command {};
};

struct NamedVelocityCommand
{
  std::string name;
  float velocity {0.0F};
  float kd {1.0F};
  float torque {0.0F};
};

struct MotorState
{
  std::string name;
  std::string interface_name;
  uint32_t can_id {0U};
  uint32_t feedback_arbitration_id {0U};
  uint8_t feedback_id_low_nibble {0U};
  MotorStatus status {MotorStatus::kUnknown};
  float position {0.0F};
  float velocity {0.0F};
  float torque {0.0F};
  float mos_temperature_c {0.0F};
  float rotor_temperature_c {0.0F};
  bool valid {false};
  std::chrono::steady_clock::time_point stamp {};
};

struct MotorCommandResult
{
  std::string name;
  bool success {false};
  std::string message;
  std::optional<MotorState> state;
};

struct RegisterReply
{
  uint32_t arbitration_id {0U};
  uint16_t request_can_id {0U};
  uint8_t opcode {0U};
  uint8_t register_id {0U};
  std::array<uint8_t, 4> raw_bytes {};
  bool valid {false};
};

inline uint8_t feedback_nibble(const uint32_t can_id)
{
  return static_cast<uint8_t>(can_id & 0x0FU);
}

inline uint8_t can_id_low_byte(const uint32_t can_id)
{
  return static_cast<uint8_t>(can_id & 0xFFU);
}

inline bool is_fault_status(const MotorStatus status)
{
  switch (status) {
    case MotorStatus::kDisabled:
    case MotorStatus::kEnabled:
      return false;
    default:
      return true;
  }
}

inline std::string to_string(const MotorStatus status)
{
  switch (status) {
    case MotorStatus::kDisabled:
      return "disabled";
    case MotorStatus::kEnabled:
      return "enabled";
    case MotorStatus::kOutputEncoderCalibrationError:
      return "output_encoder_calibration_error";
    case MotorStatus::kSensorOutputError:
      return "sensor_output_error";
    case MotorStatus::kMotorEncoderCalibrationError:
      return "motor_encoder_calibration_error";
    case MotorStatus::kOverVoltage:
      return "over_voltage";
    case MotorStatus::kUnderVoltage:
      return "under_voltage";
    case MotorStatus::kOverCurrent:
      return "over_current";
    case MotorStatus::kMosOverTemperature:
      return "mos_over_temperature";
    case MotorStatus::kRotorOverTemperature:
      return "rotor_over_temperature";
    case MotorStatus::kCommunicationLost:
      return "communication_lost";
    case MotorStatus::kOverload:
      return "overload";
    case MotorStatus::kUnknown:
    default:
      return "unknown";
  }
}

inline std::string to_string(const MotorMode mode)
{
  switch (mode) {
    case MotorMode::kMit:
      return "mit";
    case MotorMode::kPositionVelocity:
      return "position_velocity";
    case MotorMode::kVelocity:
      return "velocity";
    case MotorMode::kPositionTorque:
      return "position_torque";
    case MotorMode::kUnknown:
    default:
      return "unknown";
  }
}

}  // namespace dm_motor

#endif  // DM_MOTOR__TYPES_HPP_
