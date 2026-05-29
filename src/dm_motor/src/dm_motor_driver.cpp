#include "dm_motor/dm_motor_driver.hpp"

#include <sstream>
#include <stdexcept>

namespace dm_motor
{

namespace
{

float clamp_command_value(const float value, const float min_value, const float max_value)
{
  return std::max(min_value, std::min(value, max_value));
}

MitCommand apply_motor_safety_limits(const MotorConfig & config, MitCommand command)
{
  if (config.has_position_limit) {
    command.position = clamp_command_value(
      command.position,
      config.lower_position_limit,
      config.upper_position_limit);
  } else {
    command.position = clamp_command_value(
      command.position,
      config.limits.position_min,
      config.limits.position_max);
  }

  if (config.max_output_speed > 0.0F) {
    command.velocity = clamp_command_value(
      command.velocity,
      -config.max_output_speed,
      config.max_output_speed);
  } else {
    command.velocity = clamp_command_value(
      command.velocity,
      config.limits.velocity_min,
      config.limits.velocity_max);
  }

  command.kp = clamp_command_value(command.kp, config.limits.kp_min, config.limits.kp_max);
  command.kd = clamp_command_value(command.kd, config.limits.kd_min, config.limits.kd_max);

  if (config.max_output_torque > 0.0F) {
    command.torque = clamp_command_value(
      command.torque,
      -config.max_output_torque,
      config.max_output_torque);
  } else {
    command.torque = clamp_command_value(
      command.torque,
      config.limits.torque_min,
      config.limits.torque_max);
  }

  return command;
}

}  // namespace

DmMotorDriver::DmMotorDriver(MotorConfig config, std::shared_ptr<SocketCanTransport> transport)
: config_(std::move(config)), transport_(std::move(transport))
{
  if (!transport_) {
    throw std::invalid_argument("DmMotorDriver requires a valid transport");
  }
}

const MotorConfig & DmMotorDriver::config() const
{
  return config_;
}

std::shared_ptr<SocketCanTransport> DmMotorDriver::transport() const
{
  return transport_;
}

const std::optional<MotorState> & DmMotorDriver::last_state() const
{
  return last_state_;
}

CanFrame DmMotorDriver::make_enable_frame() const
{
  return protocol::build_enable_frame(config_);
}

CanFrame DmMotorDriver::make_disable_frame() const
{
  return protocol::build_disable_frame(config_);
}

CanFrame DmMotorDriver::make_save_zero_frame() const
{
  return protocol::build_save_zero_frame(config_);
}

CanFrame DmMotorDriver::make_clear_error_frame() const
{
  return protocol::build_clear_error_frame(config_);
}

CanFrame DmMotorDriver::make_mit_frame(const MitCommand & command) const
{
  return protocol::build_mit_frame(config_, command);
}

MotorCommandResult DmMotorDriver::enable(const std::chrono::milliseconds timeout)
{
  return send_control_and_wait(make_enable_frame(), "enable", MotorStatus::kEnabled, timeout);
}

MotorCommandResult DmMotorDriver::disable(const std::chrono::milliseconds timeout)
{
  return send_control_and_wait(make_disable_frame(), "disable", MotorStatus::kDisabled, timeout);
}

MotorCommandResult DmMotorDriver::clear_error(const std::chrono::milliseconds timeout)
{
  return send_control_and_wait(
    make_clear_error_frame(),
    "clear_error",
    MotorStatus::kDisabled,
    timeout);
}

MotorCommandResult DmMotorDriver::save_zero_position(const std::chrono::milliseconds timeout)
{
  return send_control_and_wait(
    make_save_zero_frame(),
    "save_zero_position",
    MotorStatus::kEnabled,
    timeout);
}

MotorCommandResult DmMotorDriver::send_mit_command(
  const MitCommand & command,
  const std::chrono::milliseconds timeout)
{
  const auto clamped_command = apply_motor_safety_limits(config_, command);
  return send_control_and_wait(
    make_mit_frame(clamped_command),
    "send_mit_command",
    MotorStatus::kEnabled,
    timeout);
}

MotorCommandResult DmMotorDriver::rotate_velocity(
  const float velocity,
  const float kd,
  const float torque_ff,
  const std::chrono::milliseconds timeout)
{
  MitCommand command;
  command.position = 0.0F;
  command.velocity = velocity;
  command.kp = 0.0F;
  command.kd = kd;
  command.torque = torque_ff;
  return send_mit_command(command, timeout);
}

std::optional<float> DmMotorDriver::read_register_float(
  const uint8_t register_id,
  const std::chrono::milliseconds timeout)
{
  transport_->send(protocol::build_register_read_frame(config_, register_id));
  const auto reply = wait_for_register_reply(protocol::kRegisterReadOpcode, register_id, timeout);
  if (!reply.has_value()) {
    return std::nullopt;
  }
  return protocol::register_reply_to_float(*reply);
}

std::optional<uint32_t> DmMotorDriver::read_register_u32(
  const uint8_t register_id,
  const std::chrono::milliseconds timeout)
{
  transport_->send(protocol::build_register_read_frame(config_, register_id));
  const auto reply = wait_for_register_reply(protocol::kRegisterReadOpcode, register_id, timeout);
  if (!reply.has_value()) {
    return std::nullopt;
  }
  return protocol::register_reply_to_u32(*reply);
}

bool DmMotorDriver::write_register_float(
  const uint8_t register_id,
  const float value,
  const std::chrono::milliseconds timeout)
{
  transport_->send(protocol::build_register_write_float_frame(config_, register_id, value));
  return wait_for_register_reply(protocol::kRegisterWriteOpcode, register_id, timeout).has_value();
}

bool DmMotorDriver::write_register_u32(
  const uint8_t register_id,
  const uint32_t value,
  const std::chrono::milliseconds timeout)
{
  transport_->send(protocol::build_register_write_u32_frame(config_, register_id, value));
  return wait_for_register_reply(protocol::kRegisterWriteOpcode, register_id, timeout).has_value();
}

bool DmMotorDriver::store_parameters(const std::chrono::milliseconds timeout)
{
  transport_->send(protocol::build_store_parameters_frame(config_));
  const auto frames = transport_->receive_available(timeout, 32U);
  for (const auto & frame : frames) {
    if (protocol::is_store_parameters_reply(frame, config_)) {
      return true;
    }

    if (const auto state = protocol::decode_feedback_frame(frame, config_)) {
      remember_state(*state);
    }
  }
  return false;
}

bool DmMotorDriver::switch_mode(const MotorMode mode, const std::chrono::milliseconds timeout)
{
  return write_register_u32(
    registers::kControlMode,
    static_cast<uint32_t>(mode),
    timeout);
}

std::optional<MotorState> DmMotorDriver::wait_for_feedback(const std::chrono::milliseconds timeout)
{
  const auto frames = transport_->receive_available(timeout, 64U);
  for (const auto & frame : frames) {
    if (const auto state = protocol::decode_feedback_frame(frame, config_)) {
      remember_state(*state);
      return state;
    }
  }

  return std::nullopt;
}

void DmMotorDriver::remember_state(const MotorState & state)
{
  last_state_ = state;
}

std::optional<RegisterReply> DmMotorDriver::wait_for_register_reply(
  const uint8_t expected_opcode,
  const uint8_t expected_register_id,
  const std::chrono::milliseconds timeout)
{
  const auto frames = transport_->receive_available(timeout, 64U);
  for (const auto & frame : frames) {
    if (const auto reply =
          protocol::decode_register_reply(frame, config_, expected_opcode, expected_register_id))
    {
      return reply;
    }

    if (const auto state = protocol::decode_feedback_frame(frame, config_)) {
      remember_state(*state);
    }
  }

  return std::nullopt;
}

MotorCommandResult DmMotorDriver::build_feedback_result(
  const std::string & action,
  const std::optional<MotorState> & state,
  const MotorStatus success_status) const
{
  MotorCommandResult result;
  result.name = config_.name;
  result.state = state;

  if (!state.has_value()) {
    result.success = false;
    result.message = action + ": no feedback received";
    return result;
  }

  if (state->status == success_status) {
    result.success = true;
    result.message = action + ": success";
    return result;
  }

  if (success_status == MotorStatus::kDisabled && state->status == MotorStatus::kEnabled) {
    result.success = false;
    result.message = action + ": motor still enabled";
    return result;
  }

  if (is_fault_status(state->status)) {
    result.success = false;
    result.message = action + ": motor fault " + to_string(state->status);
    return result;
  }

  std::ostringstream oss;
  oss << action << ": received state " << to_string(state->status);
  result.success = false;
  result.message = oss.str();
  return result;
}

MotorCommandResult DmMotorDriver::send_control_and_wait(
  const CanFrame & frame,
  const std::string & action,
  const MotorStatus success_status,
  const std::chrono::milliseconds timeout)
{
  transport_->send(frame);
  return build_feedback_result(action, wait_for_feedback(timeout), success_status);
}

}  // namespace dm_motor
