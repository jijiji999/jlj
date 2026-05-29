#ifndef DM_MOTOR__DAMIAO_PROTOCOL_HPP_
#define DM_MOTOR__DAMIAO_PROTOCOL_HPP_

#include <array>
#include <cstdint>
#include <optional>

#include "dm_motor/types.hpp"

namespace dm_motor::registers
{

constexpr uint8_t kControlMode = 0x0A;
constexpr uint8_t kPMax = 0x15;
constexpr uint8_t kVMax = 0x16;
constexpr uint8_t kTMax = 0x17;
constexpr uint8_t kCanBaudRate = 0x23;
constexpr uint8_t kMotorPosition = 0x50;
constexpr uint8_t kOutputPosition = 0x51;

}  // namespace dm_motor::registers

namespace dm_motor::protocol
{

constexpr uint32_t kRegisterAccessCanId = 0x7FFU;
constexpr uint8_t kRegisterReadOpcode = 0x33U;
constexpr uint8_t kRegisterWriteOpcode = 0x55U;
constexpr uint8_t kStoreParametersOpcode = 0xAAU;
constexpr uint8_t kStoreParametersSubcode = 0x01U;

uint32_t float_to_uint(float value, float min, float max, uint8_t bits);
float uint_to_float(uint32_t value, float min, float max, uint8_t bits);

CanFrame build_enable_frame(const MotorConfig & config);
CanFrame build_disable_frame(const MotorConfig & config);
CanFrame build_save_zero_frame(const MotorConfig & config);
CanFrame build_clear_error_frame(const MotorConfig & config);
CanFrame build_mit_frame(const MotorConfig & config, const MitCommand & command);

CanFrame build_register_read_frame(const MotorConfig & config, uint8_t register_id);
CanFrame build_register_write_u32_frame(
  const MotorConfig & config,
  uint8_t register_id,
  uint32_t value);
CanFrame build_register_write_float_frame(
  const MotorConfig & config,
  uint8_t register_id,
  float value);
CanFrame build_store_parameters_frame(const MotorConfig & config);

std::optional<MotorState> decode_feedback_frame(
  const CanFrame & frame,
  const MotorConfig & config);

std::optional<RegisterReply> decode_register_reply(
  const CanFrame & frame,
  const MotorConfig & config,
  uint8_t expected_opcode,
  uint8_t expected_register_id);

bool is_store_parameters_reply(const CanFrame & frame, const MotorConfig & config);
float register_reply_to_float(const RegisterReply & reply);
uint32_t register_reply_to_u32(const RegisterReply & reply);
MotorStatus decode_status_nibble(uint8_t status_nibble);

}  // namespace dm_motor::protocol

#endif  // DM_MOTOR__DAMIAO_PROTOCOL_HPP_
