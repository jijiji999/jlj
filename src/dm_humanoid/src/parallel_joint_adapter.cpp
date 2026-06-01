#include "dm_humanoid/parallel_joint_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <unordered_map>
#include <utility>

namespace dm_humanoid
{

namespace
{

dm_motor::MitCommand merge_parallel_commands(
  const dm_motor::MitCommand & pitch_command,
  const dm_motor::MitCommand & roll_command)
{
  dm_motor::MitCommand merged;
  merged.kp = std::max(pitch_command.kp, roll_command.kp);
  merged.kd = std::max(pitch_command.kd, roll_command.kd);
  merged.torque = 0.0F;
  return merged;
}

}  // namespace

bool ParallelJointAdapter::add_mechanism(
  const ParallelMechanismSpec & spec,
  std::string & error_message)
{
  if (spec.pitch_joint_name.empty() || spec.roll_joint_name.empty()) {
    error_message = "Parallel mechanism is missing pitch_joint or roll_joint";
    return false;
  }
  if (spec.motor_1_name.empty() || spec.motor_2_name.empty()) {
    error_message = "Parallel mechanism is missing motor_1 or motor_2";
    return false;
  }
  if (spec.pitch_joint_name == spec.roll_joint_name) {
    error_message = "Pitch and roll joint names must be different";
    return false;
  }
  if (spec.motor_1_name == spec.motor_2_name) {
    error_message = "motor_1 and motor_2 must be different";
    return false;
  }
  if (joint_to_group_index_.count(spec.pitch_joint_name) > 0U ||
    joint_to_group_index_.count(spec.roll_joint_name) > 0U)
  {
    error_message = "Parallel joint name is already registered";
    return false;
  }
  if (motor_to_group_index_.count(spec.motor_1_name) > 0U ||
    motor_to_group_index_.count(spec.motor_2_name) > 0U)
  {
    error_message = "Parallel motor name is already registered";
    return false;
  }

  MechanismGroup group;
  group.spec = spec;
  if (!group.kinematics.load_from_yaml(spec.config_path, error_message)) {
    return false;
  }

  groups_.push_back(std::move(group));
  const size_t index = groups_.size() - 1U;
  joint_to_group_index_[spec.pitch_joint_name] = index;
  joint_to_group_index_[spec.roll_joint_name] = index;
  motor_to_group_index_[spec.motor_1_name] = index;
  motor_to_group_index_[spec.motor_2_name] = index;
  error_message.clear();
  return true;
}

bool ParallelJointAdapter::empty() const
{
  return groups_.empty();
}

bool ParallelJointAdapter::is_parallel_joint(const std::string & name) const
{
  return joint_to_group_index_.count(name) > 0U;
}

bool ParallelJointAdapter::is_parallel_motor(const std::string & name) const
{
  return motor_to_group_index_.count(name) > 0U;
}

std::optional<std::pair<float, float>> ParallelJointAdapter::joint_limits(const std::string & name) const
{
  const auto * group = find_group_by_joint(name);
  if (group == nullptr) {
    return std::nullopt;
  }

  if (name == group->spec.pitch_joint_name) {
    return std::make_pair(
      static_cast<float>(group->kinematics.pitch_limits().min),
      static_cast<float>(group->kinematics.pitch_limits().max));
  }

  return std::make_pair(
    static_cast<float>(group->kinematics.roll_limits().min),
    static_cast<float>(group->kinematics.roll_limits().max));
}

bool ParallelJointAdapter::rebuild_joint_states(
  const std::vector<dm_motor::MotorState> & motor_states,
  const std::unordered_map<std::string, LogicalJointState> & previous_joint_states,
  std::unordered_map<std::string, LogicalJointState> & logical_joint_states,
  std::vector<std::string> & error_messages) const
{
  logical_joint_states.clear();
  error_messages.clear();

  std::unordered_map<std::string, const dm_motor::MotorState *> motor_lookup;
  motor_lookup.reserve(motor_states.size());
  for (const auto & state : motor_states) {
    motor_lookup[state.name] = &state;
    if (!is_parallel_motor(state.name)) {
      logical_joint_states[state.name] = LogicalJointState {
        state.name,
        state.position,
        state.velocity,
        state.torque,
        state.valid
      };
    }
  }

  for (const auto & group : groups_) {
    const auto motor_1_it = motor_lookup.find(group.spec.motor_1_name);
    const auto motor_2_it = motor_lookup.find(group.spec.motor_2_name);
    if (motor_1_it == motor_lookup.end() || motor_2_it == motor_lookup.end()) {
      error_messages.push_back(
        "Missing raw motor feedback for parallel mechanism: " + group.spec.name);
      continue;
    }

    double pitch_seed = 0.0;
    double roll_seed = 0.0;
    const auto pitch_previous_it = previous_joint_states.find(group.spec.pitch_joint_name);
    if (pitch_previous_it != previous_joint_states.end()) {
      pitch_seed = pitch_previous_it->second.position;
    }
    const auto roll_previous_it = previous_joint_states.find(group.spec.roll_joint_name);
    if (roll_previous_it != previous_joint_states.end()) {
      roll_seed = roll_previous_it->second.position;
    }

    double pitch = 0.0;
    double roll = 0.0;
    std::string solve_error;
    if (!group.kinematics.forward(
        motor_1_it->second->position,
        motor_2_it->second->position,
        pitch,
        roll,
        solve_error,
        pitch_seed,
        roll_seed))
    {
      error_messages.push_back(
        "Forward solve failed for " + group.spec.name + ": " + solve_error);
      continue;
    }

    double pitch_velocity = 0.0;
    double roll_velocity = 0.0;
    double pitch_effort = 0.0;
    double roll_effort = 0.0;
    std::string dynamics_error;
    if (!group.kinematics.motor_to_joint_dynamics(
        pitch,
        roll,
        motor_1_it->second->position,
        motor_2_it->second->position,
        motor_1_it->second->velocity,
        motor_2_it->second->velocity,
        motor_1_it->second->torque,
        motor_2_it->second->torque,
        pitch_velocity,
        roll_velocity,
        pitch_effort,
        roll_effort,
        dynamics_error))
    {
      error_messages.push_back(
        "Velocity/effort mapping failed for " + group.spec.name + ": " + dynamics_error);
    }

    logical_joint_states[group.spec.pitch_joint_name] = LogicalJointState {
      group.spec.pitch_joint_name,
      static_cast<float>(pitch),
      static_cast<float>(pitch_velocity),
      static_cast<float>(pitch_effort),
      true
    };
    logical_joint_states[group.spec.roll_joint_name] = LogicalJointState {
      group.spec.roll_joint_name,
      static_cast<float>(roll),
      static_cast<float>(roll_velocity),
      static_cast<float>(roll_effort),
      true
    };
  }

  return error_messages.empty();
}

bool ParallelJointAdapter::translate_joint_commands(
  const std::vector<LogicalJointCommand> & logical_joint_commands,
  std::vector<dm_motor::NamedMitCommand> & motor_commands,
  std::vector<std::string> & error_messages) const
{
  motor_commands.clear();
  error_messages.clear();

  std::unordered_map<std::string, const dm_motor::MitCommand *> logical_lookup;
  logical_lookup.reserve(logical_joint_commands.size());
  for (const auto & item : logical_joint_commands) {
    logical_lookup[item.name] = &item.command;
    if (!is_parallel_joint(item.name)) {
      motor_commands.push_back(dm_motor::NamedMitCommand {item.name, item.command});
    }
  }

  for (const auto & group : groups_) {
    const auto pitch_it = logical_lookup.find(group.spec.pitch_joint_name);
    const auto roll_it = logical_lookup.find(group.spec.roll_joint_name);
    if (pitch_it == logical_lookup.end() || roll_it == logical_lookup.end()) {
      error_messages.push_back(
        "Parallel mechanism command is incomplete for " + group.spec.name);
      continue;
    }

    double motor_1 = 0.0;
    double motor_2 = 0.0;
    std::string solve_error;
    if (!group.kinematics.inverse(
        pitch_it->second->position,
        roll_it->second->position,
        motor_1,
        motor_2,
        solve_error))
    {
      error_messages.push_back(
        "Inverse solve failed for " + group.spec.name + ": " + solve_error);
      continue;
    }

    double motor_1_velocity = 0.0;
    double motor_2_velocity = 0.0;
    if (std::abs(pitch_it->second->velocity) > 1.0e-6 ||
      std::abs(roll_it->second->velocity) > 1.0e-6)
    {
      std::string velocity_error;
      if (!group.kinematics.joint_to_motor_velocities(
          pitch_it->second->position,
          roll_it->second->position,
          motor_1,
          motor_2,
          pitch_it->second->velocity,
          roll_it->second->velocity,
          motor_1_velocity,
          motor_2_velocity,
          velocity_error))
      {
        error_messages.push_back(
          "Velocity mapping failed for " + group.spec.name + ": " + velocity_error);
      }
    }

    if (std::abs(pitch_it->second->torque) > 1.0e-6F ||
      std::abs(roll_it->second->torque) > 1.0e-6F)
    {
      error_messages.push_back(
        "Logical torque feedforward for parallel mechanism " + group.spec.name +
        " is not mapped yet and will be ignored");
    }

    const auto merged_command = merge_parallel_commands(*pitch_it->second, *roll_it->second);

    dm_motor::MitCommand motor_1_command = merged_command;
    motor_1_command.position = static_cast<float>(motor_1);
    motor_1_command.velocity = static_cast<float>(motor_1_velocity);
    dm_motor::MitCommand motor_2_command = merged_command;
    motor_2_command.position = static_cast<float>(motor_2);
    motor_2_command.velocity = static_cast<float>(motor_2_velocity);

    motor_commands.push_back(dm_motor::NamedMitCommand {
      group.spec.motor_1_name,
      motor_1_command
    });
    motor_commands.push_back(dm_motor::NamedMitCommand {
      group.spec.motor_2_name,
      motor_2_command
    });
  }

  return error_messages.empty();
}

const ParallelJointAdapter::MechanismGroup * ParallelJointAdapter::find_group_by_joint(
  const std::string & name) const
{
  const auto it = joint_to_group_index_.find(name);
  if (it == joint_to_group_index_.end()) {
    return nullptr;
  }
  return &groups_[it->second];
}

}  // namespace dm_humanoid
