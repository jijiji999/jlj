#include "dm_motor/damiao_protocol.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace dm_motor::protocol
{

namespace
{

constexpr std::array<uint8_t, 8> kEnablePayload {
  0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFCU
};

constexpr std::array<uint8_t, 8> kDisablePayload {
  0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFDU
};

constexpr std::array<uint8_t, 8> kSaveZeroPayload {
  0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFEU
};

constexpr std::array<uint8_t, 8> kClearErrorPayload {
  0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFBU
};

template<typename T>
T clamp_value(const T value, const T low, const T high)
{
  return std::min(std::max(value, low), high);
}

CanFrame build_common_control_frame(
  const MotorConfig & config,
  const std::array<uint8_t, 8> & payload)
{
  CanFrame frame;
  frame.arbitration_id = config.can_id;
  frame.size = payload.size();
  frame.is_fd = config.use_can_fd;
  frame.bitrate_switch = config.bitrate_switch;
  std::copy(payload.begin(), payload.end(), frame.data.begin());
  return frame;
}

std::array<uint8_t, 4> float_to_le_bytes(const float value)
{
  std::array<uint8_t, 4> bytes {};
  static_assert(sizeof(float) == 4U, "float must be 4 bytes");
  std::memcpy(bytes.data(), &value, sizeof(float));
  return bytes;
}

std::array<uint8_t, 4> u32_to_le_bytes(const uint32_t value)
{
  return {
    static_cast<uint8_t>(value & 0xFFU),
    static_cast<uint8_t>((value >> 8U) & 0xFFU),
    static_cast<uint8_t>((value >> 16U) & 0xFFU),
    static_cast<uint8_t>((value >> 24U) & 0xFFU)
  };
}

}  // namespace

uint32_t float_to_uint(const float value, const float min, const float max, const uint8_t bits)
{
  if (bits == 0U || bits > 31U) {
    throw std::invalid_argument("bits out of range");
  }

  if (!(std::isfinite(value) && std::isfinite(min) && std::isfinite(max)) || min >= max) {
    throw std::invalid_argument("invalid float_to_uint range");
  }

  const auto clamped = clamp_value(value, min, max);
  const float span = max - min;
  const float offset = clamped - min;
  const uint32_t max_int = (1U << bits) - 1U;
  const float scaled = std::round((offset * static_cast<float>(max_int)) / span);
  return static_cast<uint32_t>(clamp_value(scaled, 0.0F, static_cast<float>(max_int)));
}

float uint_to_float(const uint32_t value, const float min, const float max, const uint8_t bits)
{
  if (bits == 0U || bits > 31U) {
    throw std::invalid_argument("bits out of range");
  }

  if (!(std::isfinite(min) && std::isfinite(max)) || min >= max) {
    throw std::invalid_argument("invalid uint_to_float range");
  }

  const uint32_t max_int = (1U << bits) - 1U;
  const auto clipped = std::min(value, max_int);
  const float span = max - min;
  return (static_cast<float>(clipped) * span / static_cast<float>(max_int)) + min;
}

CanFrame build_enable_frame(const MotorConfig & config)
{
  return build_common_control_frame(config, kEnablePayload);
}

CanFrame build_disable_frame(const MotorConfig & config)
{
  return build_common_control_frame(config, kDisablePayload);
}

CanFrame build_save_zero_frame(const MotorConfig & config)
{
  return build_common_control_frame(config, kSaveZeroPayload);
}

CanFrame build_clear_error_frame(const MotorConfig & config)
{
  return build_common_control_frame(config, kClearErrorPayload);
}

CanFrame build_mit_frame(const MotorConfig & config, const MitCommand & command)
{
  const auto & limits = config.limits;
  const auto p_int = float_to_uint(command.position, limits.position_min, limits.position_max, 16U);
  const auto v_int = float_to_uint(command.velocity, limits.velocity_min, limits.velocity_max, 12U);
  const auto kp_int = float_to_uint(command.kp, limits.kp_min, limits.kp_max, 12U);
  const auto kd_int = float_to_uint(command.kd, limits.kd_min, limits.kd_max, 12U);
  const auto t_int = float_to_uint(command.torque, limits.torque_min, limits.torque_max, 12U);

  CanFrame frame;
  frame.arbitration_id = config.can_id;
  frame.size = 8U;
  frame.is_fd = config.use_can_fd;
  frame.bitrate_switch = config.bitrate_switch;
  frame.data[0] = static_cast<uint8_t>((p_int >> 8U) & 0xFFU);
  frame.data[1] = static_cast<uint8_t>(p_int & 0xFFU);
  frame.data[2] = static_cast<uint8_t>((v_int >> 4U) & 0xFFU);
  frame.data[3] = static_cast<uint8_t>(((v_int & 0x0FU) << 4U) | ((kp_int >> 8U) & 0x0FU));
  frame.data[4] = static_cast<uint8_t>(kp_int & 0xFFU);
  frame.data[5] = static_cast<uint8_t>((kd_int >> 4U) & 0xFFU);
  frame.data[6] = static_cast<uint8_t>(((kd_int & 0x0FU) << 4U) | ((t_int >> 8U) & 0x0FU));
  frame.data[7] = static_cast<uint8_t>(t_int & 0xFFU);
  return frame;
}

CanFrame build_register_read_frame(const MotorConfig & config, const uint8_t register_id)
{
  CanFrame frame;
  frame.arbitration_id = kRegisterAccessCanId;
  frame.size = 4U;
  frame.is_fd = config.use_can_fd;
  frame.bitrate_switch = config.bitrate_switch;
  frame.data[0] = can_id_low_byte(config.can_id);
  frame.data[1] = static_cast<uint8_t>((config.can_id >> 8U) & 0xFFU);
  frame.data[2] = kRegisterReadOpcode;
  frame.data[3] = register_id;
  return frame;
}

CanFrame build_register_write_u32_frame(
  const MotorConfig & config,
  const uint8_t register_id,
  const uint32_t value)
{
  CanFrame frame;
  frame.arbitration_id = kRegisterAccessCanId;
  frame.size = 8U;
  frame.is_fd = config.use_can_fd;
  frame.bitrate_switch = config.bitrate_switch;
  frame.data[0] = can_id_low_byte(config.can_id);
  frame.data[1] = static_cast<uint8_t>((config.can_id >> 8U) & 0xFFU);
  frame.data[2] = kRegisterWriteOpcode;
  frame.data[3] = register_id;

  const auto payload = u32_to_le_bytes(value);
  std::copy(payload.begin(), payload.end(), frame.data.begin() + 4);
  return frame;
}

CanFrame build_register_write_float_frame(
  const MotorConfig & config,
  const uint8_t register_id,
  const float value)
{
  CanFrame frame;
  frame.arbitration_id = kRegisterAccessCanId;
  frame.size = 8U;
  frame.is_fd = config.use_can_fd;
  frame.bitrate_switch = config.bitrate_switch;
  frame.data[0] = can_id_low_byte(config.can_id);
  frame.data[1] = static_cast<uint8_t>((config.can_id >> 8U) & 0xFFU);
  frame.data[2] = kRegisterWriteOpcode;
  frame.data[3] = register_id;

  const auto payload = float_to_le_bytes(value);
  std::copy(payload.begin(), payload.end(), frame.data.begin() + 4);
  return frame;
}

CanFrame build_store_parameters_frame(const MotorConfig & config)
{
  CanFrame frame;
  frame.arbitration_id = kRegisterAccessCanId;
  frame.size = 4U;
  frame.is_fd = config.use_can_fd;
  frame.bitrate_switch = config.bitrate_switch;
  frame.data[0] = can_id_low_byte(config.can_id);
  frame.data[1] = static_cast<uint8_t>((config.can_id >> 8U) & 0xFFU);
  frame.data[2] = kStoreParametersOpcode;
  frame.data[3] = kStoreParametersSubcode;
  return frame;
}

std::optional<MotorState> decode_feedback_frame(const CanFrame & frame, const MotorConfig & config)
{
  if (frame.arbitration_id != config.master_id || frame.size < 7U) {
    return std::nullopt;
  }

  const uint8_t id_and_status = frame.data[0];
  const uint8_t feedback_id = static_cast<uint8_t>(id_and_status & 0x0FU);
  const uint8_t status_nibble = static_cast<uint8_t>((id_and_status >> 4U) & 0x0FU);
  if (feedback_id != feedback_nibble(config.can_id)) {
    return std::nullopt;
  }

  const uint16_t pos_raw = static_cast<uint16_t>((frame.data[1] << 8U) | frame.data[2]);
  const uint16_t vel_raw = static_cast<uint16_t>((frame.data[3] << 4U) | (frame.data[4] >> 4U));
  const uint16_t torque_raw = static_cast<uint16_t>(((frame.data[4] & 0x0FU) << 8U) | frame.data[5]);

  MotorState state;
  state.name = config.name;
  state.interface_name = config.interface_name;
  state.can_id = config.can_id;
  state.feedback_arbitration_id = frame.arbitration_id;
  state.feedback_id_low_nibble = feedback_id;
  state.status = decode_status_nibble(status_nibble);
  state.position = uint_to_float(
    pos_raw,
    config.limits.position_min,
    config.limits.position_max,
    16U);
  state.velocity = uint_to_float(
    vel_raw,
    config.limits.velocity_min,
    config.limits.velocity_max,
    12U);
  state.torque = uint_to_float(
    torque_raw,
    config.limits.torque_min,
    config.limits.torque_max,
    12U);
  state.mos_temperature_c = static_cast<float>(frame.data[6]);
  state.rotor_temperature_c = frame.size >= 8U ? static_cast<float>(frame.data[7]) : 0.0F;
  state.valid = true;
  state.stamp = std::chrono::steady_clock::now();
  return state;
}

std::optional<RegisterReply> decode_register_reply(
  const CanFrame & frame,
  const MotorConfig & config,
  const uint8_t expected_opcode,
  const uint8_t expected_register_id)
{
  if (frame.arbitration_id != config.master_id || frame.size < 8U) {
    return std::nullopt;
  }

  const uint16_t reply_can_id =
    static_cast<uint16_t>(frame.data[0]) |
    static_cast<uint16_t>(frame.data[1] << 8U);
  if (reply_can_id != static_cast<uint16_t>(config.can_id & 0xFFFFU)) {
    return std::nullopt;
  }

  if (frame.data[2] != expected_opcode || frame.data[3] != expected_register_id) {
    return std::nullopt;
  }

  RegisterReply reply;
  reply.arbitration_id = frame.arbitration_id;
  reply.request_can_id = reply_can_id;
  reply.opcode = frame.data[2];
  reply.register_id = frame.data[3];
  reply.raw_bytes = {frame.data[4], frame.data[5], frame.data[6], frame.data[7]};
  reply.valid = true;
  return reply;
}

bool is_store_parameters_reply(const CanFrame & frame, const MotorConfig & config)
{
  if (frame.arbitration_id != config.master_id || frame.size < 4U) {
    return false;
  }

  const uint16_t reply_can_id =
    static_cast<uint16_t>(frame.data[0]) |
    static_cast<uint16_t>(frame.data[1] << 8U);
  if (reply_can_id != static_cast<uint16_t>(config.can_id & 0xFFFFU)) {
    return false;
  }

  return frame.data[2] == kStoreParametersOpcode && frame.data[3] == kStoreParametersSubcode;
}

float register_reply_to_float(const RegisterReply & reply)
{
  float value = std::numeric_limits<float>::quiet_NaN();
  std::memcpy(&value, reply.raw_bytes.data(), sizeof(float));
  return value;
}

uint32_t register_reply_to_u32(const RegisterReply & reply)
{
  return
    static_cast<uint32_t>(reply.raw_bytes[0]) |
    (static_cast<uint32_t>(reply.raw_bytes[1]) << 8U) |
    (static_cast<uint32_t>(reply.raw_bytes[2]) << 16U) |
    (static_cast<uint32_t>(reply.raw_bytes[3]) << 24U);
}

MotorStatus decode_status_nibble(const uint8_t status_nibble)
{
  switch (status_nibble) {
    case 0x0:
      return MotorStatus::kDisabled;
    case 0x1:
      return MotorStatus::kEnabled;
    case 0x3:
      return MotorStatus::kOutputEncoderCalibrationError;
    case 0x4:
      return MotorStatus::kSensorOutputError;
    case 0x5:
      return MotorStatus::kMotorEncoderCalibrationError;
    case 0x8:
      return MotorStatus::kOverVoltage;
    case 0x9:
      return MotorStatus::kUnderVoltage;
    case 0xA:
      return MotorStatus::kOverCurrent;
    case 0xB:
      return MotorStatus::kMosOverTemperature;
    case 0xC:
      return MotorStatus::kRotorOverTemperature;
    case 0xD:
      return MotorStatus::kCommunicationLost;
    case 0xE:
      return MotorStatus::kOverload;
    default:
      return MotorStatus::kUnknown;
  }
}

}  // namespace dm_motor::protocol
