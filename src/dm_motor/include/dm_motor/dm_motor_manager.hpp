#ifndef DM_MOTOR__DM_MOTOR_MANAGER_HPP_
#define DM_MOTOR__DM_MOTOR_MANAGER_HPP_

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "dm_motor/dm_motor_driver.hpp"

namespace dm_motor
{

class DmMotorManager
{
public:
  explicit DmMotorManager(const std::string & config_path);

  const std::string & config_path() const;
  const std::vector<std::string> & motor_names() const;
  std::vector<MotorConfig> motor_configs() const;
  std::optional<MotorConfig> motor_config(const std::string & name) const;

  MotorCommandResult enable_motor(
    const std::string & name,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(20));
  MotorCommandResult disable_motor(
    const std::string & name,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(20));
  MotorCommandResult clear_motor_error(
    const std::string & name,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(20));
  MotorCommandResult save_motor_zero(
    const std::string & name,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(20));
  MotorCommandResult command_motor_mit(
    const std::string & name,
    const MitCommand & command,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(20));
  MotorCommandResult rotate_motor_velocity(
    const std::string & name,
    float velocity,
    float kd = 1.0F,
    float torque_ff = 0.0F,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(20));

  std::vector<MotorCommandResult> enable_motors(
    const std::vector<std::string> & names,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(20));
  std::vector<MotorCommandResult> disable_motors(
    const std::vector<std::string> & names,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(20));
  std::vector<MotorCommandResult> command_group_mit(
    const std::vector<NamedMitCommand> & commands,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(20));
  std::vector<MotorCommandResult> rotate_motors_velocity(
    const std::vector<NamedVelocityCommand> & commands,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(20));

  std::vector<MotorCommandResult> enable_all(
    std::chrono::milliseconds timeout = std::chrono::milliseconds(20));
  std::vector<MotorCommandResult> disable_all(
    std::chrono::milliseconds timeout = std::chrono::milliseconds(20));

  std::vector<MotorState> snapshot_states() const;
  bool poll_once(std::chrono::milliseconds timeout = std::chrono::milliseconds(0));

private:
  struct InterfaceOptions
  {
    bool use_can_fd {true};
    bool bitrate_switch {true};
  };

  struct FeedbackMatch
  {
    std::string name;
    MotorState state;
  };

  void load_config(const std::string & path);
  void validate_config() const;
  void open_transports();

  DmMotorDriver * find_driver(const std::string & name);
  const DmMotorDriver * find_driver(const std::string & name) const;
  std::optional<FeedbackMatch> handle_feedback_frame(
    const std::string & interface_name,
    const CanFrame & frame);
  std::string route_key(const std::string & interface_name, uint8_t nibble) const;
  std::vector<MotorCommandResult> send_batch_and_collect(
    const std::vector<std::pair<DmMotorDriver *, CanFrame>> & batch,
    const std::string & action,
    std::optional<MotorStatus> expected_status,
    std::chrono::milliseconds timeout);
  MotorCommandResult build_result_from_driver(
    const DmMotorDriver & driver,
    const std::string & action,
    std::optional<MotorStatus> expected_status) const;

  std::string config_path_;
  std::vector<std::string> motor_order_;
  std::vector<MotorConfig> configs_;
  std::unordered_map<std::string, InterfaceOptions> interface_options_;
  std::unordered_map<std::string, std::shared_ptr<SocketCanTransport>> transports_;
  std::unordered_map<std::string, std::unique_ptr<DmMotorDriver>> drivers_;
  std::unordered_map<std::string, DmMotorDriver *> feedback_routes_;
};

}  // namespace dm_motor

#endif  // DM_MOTOR__DM_MOTOR_MANAGER_HPP_
