#ifndef DM_HUMANOID__PARALLEL_JOINT_ADAPTER_HPP_
#define DM_HUMANOID__PARALLEL_JOINT_ADAPTER_HPP_

#include "dm_humanoid/parallel_kinematics.hpp"
#include "dm_motor/types.hpp"

#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace dm_humanoid
{

struct LogicalJointState
{
  std::string name;
  float position {0.0F};
  float velocity {0.0F};
  float effort {0.0F};
  bool valid {false};
};

struct LogicalJointCommand
{
  std::string name;
  dm_motor::MitCommand command {};
};

struct ParallelMechanismSpec
{
  std::string name;
  std::string pitch_joint_name;
  std::string roll_joint_name;
  std::string motor_1_name;
  std::string motor_2_name;
  std::string config_path;
};

class ParallelJointAdapter
{
public:
  bool add_mechanism(const ParallelMechanismSpec & spec, std::string & error_message);

  bool empty() const;
  bool is_parallel_joint(const std::string & name) const;
  bool is_parallel_motor(const std::string & name) const;
  std::optional<std::pair<float, float>> joint_limits(const std::string & name) const;

  bool rebuild_joint_states(
    const std::vector<dm_motor::MotorState> & motor_states,
    const std::unordered_map<std::string, LogicalJointState> & previous_joint_states,
    std::unordered_map<std::string, LogicalJointState> & logical_joint_states,
    std::vector<std::string> & error_messages) const;

  bool translate_joint_commands(
    const std::vector<LogicalJointCommand> & logical_joint_commands,
    std::vector<dm_motor::NamedMitCommand> & motor_commands,
    std::vector<std::string> & error_messages) const;

private:
  struct MechanismGroup
  {
    ParallelMechanismSpec spec;
    ParallelKinematics kinematics;
  };

  const MechanismGroup * find_group_by_joint(const std::string & name) const;

  std::vector<MechanismGroup> groups_;
  std::unordered_map<std::string, size_t> joint_to_group_index_;
  std::unordered_map<std::string, size_t> motor_to_group_index_;
};

}  // namespace dm_humanoid

#endif  // DM_HUMANOID__PARALLEL_JOINT_ADAPTER_HPP_
