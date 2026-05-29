#ifndef DM_MOTOR__DM_MOTOR_DRIVER_HPP_
#define DM_MOTOR__DM_MOTOR_DRIVER_HPP_

#include <chrono>
#include <memory>
#include <optional>

#include "dm_motor/damiao_protocol.hpp"
#include "dm_motor/socketcan_transport.hpp"
#include "dm_motor/types.hpp"

namespace dm_motor
{

class DmMotorDriver
{
public:
  DmMotorDriver(MotorConfig config, std::shared_ptr<SocketCanTransport> transport);

  const MotorConfig & config() const;
  std::shared_ptr<SocketCanTransport> transport() const;
  const std::optional<MotorState> & last_state() const;

  CanFrame make_enable_frame() const;
  CanFrame make_disable_frame() const;
  CanFrame make_save_zero_frame() const;
  CanFrame make_clear_error_frame() const;
  CanFrame make_mit_frame(const MitCommand & command) const;

  MotorCommandResult enable(std::chrono::milliseconds timeout = std::chrono::milliseconds(20));
  MotorCommandResult disable(std::chrono::milliseconds timeout = std::chrono::milliseconds(20));
  MotorCommandResult clear_error(std::chrono::milliseconds timeout = std::chrono::milliseconds(20));
  MotorCommandResult save_zero_position(
    std::chrono::milliseconds timeout = std::chrono::milliseconds(20));
  MotorCommandResult send_mit_command(
    const MitCommand & command,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(20));
  MotorCommandResult rotate_velocity(
    float velocity,
    float kd = 1.0F,
    float torque_ff = 0.0F,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(20));

  std::optional<float> read_register_float(
    uint8_t register_id,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(20));
  std::optional<uint32_t> read_register_u32(
    uint8_t register_id,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(20));
  bool write_register_float(
    uint8_t register_id,
    float value,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(20));
  bool write_register_u32(
    uint8_t register_id,
    uint32_t value,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(20));
  bool store_parameters(std::chrono::milliseconds timeout = std::chrono::milliseconds(30));
  bool switch_mode(
    MotorMode mode,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(20));

  std::optional<MotorState> wait_for_feedback(std::chrono::milliseconds timeout);
  void remember_state(const MotorState & state);

private:
  std::optional<RegisterReply> wait_for_register_reply(
    uint8_t expected_opcode,
    uint8_t expected_register_id,
    std::chrono::milliseconds timeout);
  MotorCommandResult build_feedback_result(
    const std::string & action,
    const std::optional<MotorState> & state,
    MotorStatus success_status) const;
  MotorCommandResult send_control_and_wait(
    const CanFrame & frame,
    const std::string & action,
    MotorStatus success_status,
    std::chrono::milliseconds timeout);

  MotorConfig config_;
  std::shared_ptr<SocketCanTransport> transport_;
  std::optional<MotorState> last_state_;
};

}  // namespace dm_motor

#endif  // DM_MOTOR__DM_MOTOR_DRIVER_HPP_
