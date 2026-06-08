#ifndef DM_HUMANOID__PARALLEL_KINEMATICS_HPP_
#define DM_HUMANOID__PARALLEL_KINEMATICS_HPP_

#include <Eigen/Dense>

#include <string>
#include <utility>
#include <vector>

namespace dm_humanoid
{

enum class ParallelMechanismType
{
  kAnkle,
  kWaist,
};

struct ParallelAxisLimits
{
  double min {0.0};
  double max {0.0};
};

class ParallelKinematics
{
public:
  bool load_from_yaml(const std::string & yaml_file, std::string & error_message);

  ParallelMechanismType type() const;
  const ParallelAxisLimits & pitch_limits() const;
  const ParallelAxisLimits & roll_limits() const;

  bool inverse(
    double pitch,
    double roll,
    double & motor_1,
    double & motor_2,
    std::string & error_message) const;

  bool forward(
    double motor_1,
    double motor_2,
    double & pitch,
    double & roll,
    std::string & error_message,
    double pitch_seed = 0.0,
    double roll_seed = 0.0) const;

  bool joint_to_motor_velocities(
    double pitch,
    double roll,
    double motor_1,
    double motor_2,
    double pitch_velocity,
    double roll_velocity,
    double & motor_1_velocity,
    double & motor_2_velocity,
    std::string & error_message) const;

  bool motor_to_joint_dynamics(
    double pitch,
    double roll,
    double motor_1,
    double motor_2,
    double motor_1_velocity,
    double motor_2_velocity,
    double motor_1_effort,
    double motor_2_effort,
    double & pitch_velocity,
    double & roll_velocity,
    double & pitch_effort,
    double & roll_effort,
    std::string & error_message) const;

private:
  struct MechanismParams
  {
    ParallelMechanismType type {ParallelMechanismType::kAnkle};
    Eigen::Vector3d h0 {Eigen::Vector3d::Zero()};
    Eigen::Vector3d ra01 {Eigen::Vector3d::Zero()};
    Eigen::Vector3d ra02 {Eigen::Vector3d::Zero()};
    Eigen::Vector3d rb01 {Eigen::Vector3d::Zero()};
    Eigen::Vector3d rb02 {Eigen::Vector3d::Zero()};
    Eigen::Vector3d rc01 {Eigen::Vector3d::Zero()};
    Eigen::Vector3d rc02 {Eigen::Vector3d::Zero()};
    Eigen::Vector3d lb1_local {Eigen::Vector3d::Zero()};
    Eigen::Vector3d lb2_local {Eigen::Vector3d::Zero()};
    double lrod1 {0.0};
    double lrod2 {0.0};
    double m01 {0.0};
    double m02 {0.0};
    double motor_dir1 {1.0};
    double motor_dir2 {1.0};
    ParallelAxisLimits pitch_limits {};
    ParallelAxisLimits roll_limits {};
  };

  Eigen::Matrix3d make_rotation_y(double pitch) const;
  Eigen::Matrix3d make_rotation_x(double roll) const;
  Eigen::Matrix3d make_joint_rotation(double pitch, double roll) const;
  Eigen::Vector3d compute_rb(
    const Eigen::Vector3d & ra,
    const Eigen::Vector3d & lb_local,
    double theta) const;
  bool solve_theta(
    const Eigen::Vector3d & rc,
    const Eigen::Vector3d & ra,
    const Eigen::Vector3d & lb_local,
    double rod_length,
    double zero_offset,
    double & theta,
    int & error_state) const;
  bool compute_jacobian(
    double pitch,
    double roll,
    double motor_1,
    double motor_2,
    Eigen::Matrix2d & jacobian,
    int & error_state) const;
  bool is_loaded() const;

  MechanismParams params_ {};
  Eigen::Vector3d rotation_axis_ {Eigen::Vector3d::UnitX()};
  bool loaded_ {false};
};

}  // namespace dm_humanoid

#endif  // DM_HUMANOID__PARALLEL_KINEMATICS_HPP_
