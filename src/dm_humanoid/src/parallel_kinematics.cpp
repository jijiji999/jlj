#include "dm_humanoid/parallel_kinematics.hpp"

#include "yaml-cpp/yaml.h"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>

namespace dm_humanoid
{

namespace
{

constexpr double kSmallValue = 1.0e-10;
constexpr double kReachabilityTolerance = 1.0e-9;
constexpr double kForwardTolerance = 1.0e-6;
constexpr int kForwardMaxIterations = 100;
constexpr int kForwardBacktrackIterations = 8;
constexpr double kSingularDampingThreshold = 1.0e-2;
constexpr double kSingularDamping = 1.0e-2;

Eigen::Vector3d read_vector3(const YAML::Node & node, const char * key)
{
  const auto values = node[key];
  if (!values || !values.IsSequence() || values.size() != 3U) {
    throw std::runtime_error(std::string("Expected 3-vector for key: ") + key);
  }

  return Eigen::Vector3d(
    values[0].as<double>(),
    values[1].as<double>(),
    values[2].as<double>());
}

ParallelAxisLimits read_axis_limits(const YAML::Node & node, const char * key)
{
  const auto values = node[key];
  if (!values || !values.IsSequence() || values.size() != 2U) {
    throw std::runtime_error(std::string("Expected 2-vector for key: ") + key);
  }

  return ParallelAxisLimits {
    values[0].as<double>(),
    values[1].as<double>()
  };
}

double read_motor_direction(const YAML::Node & node, const size_t index)
{
  if (!node || !node.IsSequence() || node.size() != 2U) {
    throw std::runtime_error("Expected 2-vector for key: motor_direction");
  }

  return node[index].as<double>() >= 0.0 ? 1.0 : -1.0;
}

double normalize_angle(const double angle)
{
  double normalized = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (normalized < 0.0) {
    normalized += 2.0 * M_PI;
  }
  return normalized - M_PI;
}

}  // namespace

bool ParallelKinematics::load_from_yaml(
  const std::string & yaml_file,
  std::string & error_message)
{
  try {
    const YAML::Node config = YAML::LoadFile(yaml_file);
    params_ = MechanismParams {};

    const auto type_name = config["joint_type"].as<std::string>();
    if (type_name == "ankle") {
      params_.type = ParallelMechanismType::kAnkle;
      rotation_axis_ = Eigen::Vector3d::UnitX();
    } else if (type_name == "waist") {
      params_.type = ParallelMechanismType::kWaist;
      rotation_axis_ = Eigen::Vector3d::UnitY();
    } else {
      throw std::runtime_error("Unsupported joint_type: " + type_name);
    }

    params_.h0 = read_vector3(config, "h0");
    params_.ra01 = read_vector3(config, "ra01");
    params_.ra02 = read_vector3(config, "ra02");
    params_.rb01 = read_vector3(config, "rb01");
    params_.rb02 = read_vector3(config, "rb02");
    params_.rc01 = read_vector3(config, "rc01");
    params_.rc02 = read_vector3(config, "rc02");

    params_.lb1_local = params_.rb01 - params_.ra01;
    params_.lb2_local = params_.rb02 - params_.ra02;
    params_.lrod1 = (params_.rb01 - params_.rc01).norm();
    params_.lrod2 = (params_.rb02 - params_.rc02).norm();
    params_.m01 = config["m01"].as<double>();
    params_.m02 = config["m02"].as<double>();
    if (config["motor_direction"]) {
      params_.motor_dir1 = read_motor_direction(config["motor_direction"], 0U);
      params_.motor_dir2 = read_motor_direction(config["motor_direction"], 1U);
    }
    params_.pitch_limits = read_axis_limits(config, "pitch_range");
    params_.roll_limits = read_axis_limits(config, "roll_range");

    loaded_ = true;
    error_message.clear();
    return true;
  } catch (const std::exception & exception) {
    loaded_ = false;
    error_message = exception.what();
    return false;
  }
}

ParallelMechanismType ParallelKinematics::type() const
{
  return params_.type;
}

const ParallelAxisLimits & ParallelKinematics::pitch_limits() const
{
  return params_.pitch_limits;
}

const ParallelAxisLimits & ParallelKinematics::roll_limits() const
{
  return params_.roll_limits;
}

bool ParallelKinematics::inverse(
  const double pitch,
  const double roll,
  double & motor_1,
  double & motor_2,
  std::string & error_message) const
{
  if (!is_loaded()) {
    error_message = "Parallel kinematics not loaded";
    return false;
  }

  const Eigen::Vector3d rc1 = make_joint_rotation(pitch, roll) * params_.rc01;
  const Eigen::Vector3d rc2 = make_joint_rotation(pitch, roll) * params_.rc02;

  int error_state = 0;
  if (!solve_theta(
      rc1,
      params_.ra01,
      params_.lb1_local,
      params_.lrod1,
      params_.m01,
      motor_1,
      error_state))
  {
    motor_1 = NAN;
    motor_2 = NAN;
    error_message = "Failed to solve motor_1 angle, error_state=" + std::to_string(error_state);
    return false;
  }

  if (!solve_theta(
      rc2,
      params_.ra02,
      params_.lb2_local,
      params_.lrod2,
      params_.m02,
      motor_2,
      error_state))
  {
    motor_1 = NAN;
    motor_2 = NAN;
    error_message = "Failed to solve motor_2 angle, error_state=" + std::to_string(error_state);
    return false;
  }

  motor_1 = normalize_angle(params_.motor_dir1 * motor_1);
  motor_2 = normalize_angle(params_.motor_dir2 * motor_2);

  error_message.clear();
  return true;
}

bool ParallelKinematics::forward(
  const double motor_1,
  const double motor_2,
  double & pitch,
  double & roll,
  std::string & error_message,
  const double pitch_seed,
  const double roll_seed) const
{
  if (!is_loaded()) {
    error_message = "Parallel kinematics not loaded";
    return false;
  }

  pitch = pitch_seed;
  roll = roll_seed;

  if (pitch < params_.pitch_limits.min || pitch > params_.pitch_limits.max) {
    pitch = 0.5 * (params_.pitch_limits.min + params_.pitch_limits.max);
  }
  if (roll < params_.roll_limits.min || roll > params_.roll_limits.max) {
    roll = 0.5 * (params_.roll_limits.min + params_.roll_limits.max);
  }

  for (int iteration = 0; iteration < kForwardMaxIterations; ++iteration) {
    pitch = std::clamp(pitch, params_.pitch_limits.min, params_.pitch_limits.max);
    roll = std::clamp(roll, params_.roll_limits.min, params_.roll_limits.max);

    double solved_motor_1 = 0.0;
    double solved_motor_2 = 0.0;
    std::string inverse_error;
    if (!inverse(pitch, roll, solved_motor_1, solved_motor_2, inverse_error)) {
      error_message = "Forward solve failed during inverse step: " + inverse_error;
      return false;
    }

    if (std::isnan(solved_motor_1) || std::isnan(solved_motor_2)) {
      error_message = "Forward solve produced NaN motor angles";
      return false;
    }

    const Eigen::Vector2d motor_error(
      solved_motor_1 - motor_1,
      solved_motor_2 - motor_2);
    if (motor_error.norm() < kForwardTolerance) {
      error_message.clear();
      return true;
    }

    Eigen::Matrix2d jacobian;
    int error_state = 0;
    if (!compute_jacobian(pitch, roll, solved_motor_1, solved_motor_2, jacobian, error_state)) {
      error_message = "Forward solve failed during Jacobian step, error_state=" +
        std::to_string(error_state);
      return false;
    }

    Eigen::Vector2d correction;
    if (std::abs(jacobian.determinant()) < kSingularDampingThreshold) {
      Eigen::Matrix2d damped_normal = jacobian.transpose() * jacobian;
      damped_normal(0, 0) += kSingularDamping;
      damped_normal(1, 1) += kSingularDamping;
      correction = damped_normal.inverse() * jacobian.transpose() * motor_error;
    } else {
      correction = jacobian.inverse() * motor_error;
    }

    double step = 1.0;
    bool step_accepted = false;
    for (int backtrack = 0; backtrack < kForwardBacktrackIterations; ++backtrack) {
      const double trial_pitch = std::clamp(
        pitch - step * correction[0],
        params_.pitch_limits.min,
        params_.pitch_limits.max);
      const double trial_roll = std::clamp(
        roll - step * correction[1],
        params_.roll_limits.min,
        params_.roll_limits.max);

      double trial_motor_1 = 0.0;
      double trial_motor_2 = 0.0;
      std::string trial_error;
      if (!inverse(trial_pitch, trial_roll, trial_motor_1, trial_motor_2, trial_error)) {
        step *= 0.5;
        continue;
      }

      const Eigen::Vector2d trial_motor_error(
        trial_motor_1 - motor_1,
        trial_motor_2 - motor_2);
      if (trial_motor_error.norm() <= motor_error.norm() * 1.01) {
        step_accepted = true;
        break;
      }
      step *= 0.5;
    }

    if (!step_accepted) {
      error_message = "Forward solve line search failed";
      return false;
    }

    pitch -= step * correction[0];
    roll -= step * correction[1];
  }

  error_message = "Forward solve exceeded max iterations";
  return false;
}

bool ParallelKinematics::joint_to_motor_velocities(
  const double pitch,
  const double roll,
  const double motor_1,
  const double motor_2,
  const double pitch_velocity,
  const double roll_velocity,
  double & motor_1_velocity,
  double & motor_2_velocity,
  std::string & error_message) const
{
  if (!is_loaded()) {
    error_message = "Parallel kinematics not loaded";
    return false;
  }

  Eigen::Matrix2d jacobian;
  int error_state = 0;
  if (!compute_jacobian(pitch, roll, motor_1, motor_2, jacobian, error_state)) {
    error_message = "Velocity mapping Jacobian failed, error_state=" + std::to_string(error_state);
    return false;
  }

  const Eigen::Vector2d joint_velocity_pitch_roll(pitch_velocity, roll_velocity);
  const Eigen::Vector2d motor_velocity = jacobian * joint_velocity_pitch_roll;
  motor_1_velocity = motor_velocity[0];
  motor_2_velocity = motor_velocity[1];
  error_message.clear();
  return true;
}

bool ParallelKinematics::motor_to_joint_dynamics(
  const double pitch,
  const double roll,
  const double motor_1,
  const double motor_2,
  const double motor_1_velocity,
  const double motor_2_velocity,
  const double motor_1_effort,
  const double motor_2_effort,
  double & pitch_velocity,
  double & roll_velocity,
  double & pitch_effort,
  double & roll_effort,
  std::string & error_message) const
{
  if (!is_loaded()) {
    error_message = "Parallel kinematics not loaded";
    return false;
  }

  Eigen::Matrix2d jacobian;
  int error_state = 0;
  if (!compute_jacobian(pitch, roll, motor_1, motor_2, jacobian, error_state)) {
    error_message = "Dynamics mapping Jacobian failed, error_state=" + std::to_string(error_state);
    return false;
  }

  const Eigen::Vector2d motor_velocity(motor_1_velocity, motor_2_velocity);
  const Eigen::Vector2d joint_velocity_pitch_roll = jacobian.inverse() * motor_velocity;

  const Eigen::Vector2d motor_effort(motor_1_effort, motor_2_effort);
  const Eigen::Vector2d joint_effort_pitch_roll = jacobian.transpose() * motor_effort;

  pitch_velocity = joint_velocity_pitch_roll[0];
  roll_velocity = joint_velocity_pitch_roll[1];
  pitch_effort = joint_effort_pitch_roll[0];
  roll_effort = joint_effort_pitch_roll[1];
  error_message.clear();
  return true;
}

Eigen::Matrix3d ParallelKinematics::make_rotation_y(const double pitch) const
{
  Eigen::Matrix3d rotation_y;
  rotation_y <<
    std::cos(pitch), 0.0, std::sin(pitch),
    0.0, 1.0, 0.0,
    -std::sin(pitch), 0.0, std::cos(pitch);
  return rotation_y;
}

Eigen::Matrix3d ParallelKinematics::make_rotation_x(const double roll) const
{
  Eigen::Matrix3d rotation_x;
  rotation_x <<
    1.0, 0.0, 0.0,
    0.0, std::cos(roll), std::sin(roll),
    0.0, -std::sin(roll), std::cos(roll);
  return rotation_x;
}

Eigen::Matrix3d ParallelKinematics::make_joint_rotation(const double pitch, const double roll) const
{
  return make_rotation_y(pitch) * make_rotation_x(roll);
}

Eigen::Vector3d ParallelKinematics::compute_rb(
  const Eigen::Vector3d & ra,
  const Eigen::Vector3d & lb_local,
  const double theta) const
{
  const Eigen::Vector3d lb_rotated =
    lb_local * std::cos(theta) +
    rotation_axis_.cross(lb_local) * std::sin(theta) +
    rotation_axis_ * rotation_axis_.dot(lb_local) * (1.0 - std::cos(theta));
  return ra + lb_rotated;
}

bool ParallelKinematics::solve_theta(
  const Eigen::Vector3d & rc,
  const Eigen::Vector3d & ra,
  const Eigen::Vector3d & lb_local,
  const double rod_length,
  const double zero_offset,
  double & theta,
  int & error_state) const
{
  error_state = 0;

  const Eigen::Vector3d delta = rc - ra;
  const double delta_norm_sq = delta.squaredNorm();
  const double lb_length_sq = lb_local.squaredNorm();
  const double rotation_axis_dot_lb = rotation_axis_.dot(lb_local);
  const double coefficient_a = delta.dot(lb_local);
  const double coefficient_b = delta.dot(rotation_axis_.cross(lb_local));
  const double coefficient_d = delta.dot(rotation_axis_) * rotation_axis_dot_lb;
  const double coefficient_a_effective = coefficient_a - coefficient_d;
  const double coefficient_b_effective = coefficient_b;
  const double coefficient_c_effective =
    (delta_norm_sq + lb_length_sq - rod_length * rod_length) / 2.0 - coefficient_d;

  const double radius = std::sqrt(
    coefficient_a_effective * coefficient_a_effective +
    coefficient_b_effective * coefficient_b_effective);
  if (radius < kSmallValue) {
    theta = NAN;
    error_state = 1;
    return false;
  }

  double ratio = coefficient_c_effective / radius;
  if (ratio > 1.0 + kReachabilityTolerance || ratio < -1.0 - kReachabilityTolerance) {
    theta = NAN;
    error_state = 2;
    return false;
  }
  ratio = std::clamp(ratio, -1.0, 1.0);

  const double alpha = std::atan2(coefficient_a_effective, coefficient_b_effective);
  const double phi_asin = std::asin(ratio);
  const double phi1 = phi_asin - alpha;
  const double phi2 = M_PI - phi_asin - alpha;
  const double phi = coefficient_b_effective >= 0.0 ? phi1 : phi2;

  theta = phi - zero_offset;
  return true;
}

bool ParallelKinematics::compute_jacobian(
  const double pitch,
  const double roll,
  const double motor_1,
  const double motor_2,
  Eigen::Matrix2d & jacobian,
  int & error_state) const
{
  const double motor_1_mechanism = params_.motor_dir1 * motor_1;
  const double motor_2_mechanism = params_.motor_dir2 * motor_2;

  const Eigen::Vector3d rb1 = compute_rb(
    params_.ra01,
    params_.lb1_local,
    motor_1_mechanism + params_.m01);
  const Eigen::Vector3d rb2 = compute_rb(
    params_.ra02,
    params_.lb2_local,
    motor_2_mechanism + params_.m02);
  const Eigen::Vector3d rc1 = make_joint_rotation(pitch, roll) * params_.rc01;
  const Eigen::Vector3d rc2 = make_joint_rotation(pitch, roll) * params_.rc02;

  const Eigen::Vector3d rbar1 = rb1 - params_.ra01;
  const Eigen::Vector3d rbar2 = rb2 - params_.ra02;
  const Eigen::Vector3d rrod1 = rc1 - rb1;
  const Eigen::Vector3d rrod2 = rc2 - rb2;

  Eigen::Matrix2d motor_space_matrix;
  motor_space_matrix <<
    rotation_axis_.dot(rbar1.cross(rrod1)), 0.0,
    0.0, rotation_axis_.dot(rbar2.cross(rrod2));

  if (std::abs(motor_space_matrix.determinant()) < kSmallValue) {
    error_state = 1;
    return false;
  }

  const double sin_pitch = std::sin(pitch);
  const double cos_pitch = std::cos(pitch);
  const double sin_roll = std::sin(roll);
  const double cos_roll = std::cos(roll);

  Eigen::Matrix3d rotation_y_pitch_derivative;
  rotation_y_pitch_derivative <<
    -sin_pitch, 0.0, cos_pitch,
    0.0, 0.0, 0.0,
    -cos_pitch, 0.0, -sin_pitch;

  Eigen::Matrix3d rotation_x_roll_derivative;
  rotation_x_roll_derivative <<
    0.0, 0.0, 0.0,
    0.0, -sin_roll, cos_roll,
    0.0, -cos_roll, -sin_roll;

  const Eigen::Matrix3d rotation_y = make_rotation_y(pitch);
  const Eigen::Matrix3d rotation_x = make_rotation_x(roll);

  const Eigen::Vector3d rc1_pitch_derivative =
    rotation_y_pitch_derivative * rotation_x * params_.rc01;
  const Eigen::Vector3d rc1_roll_derivative =
    rotation_y * rotation_x_roll_derivative * params_.rc01;
  const Eigen::Vector3d rc2_pitch_derivative =
    rotation_y_pitch_derivative * rotation_x * params_.rc02;
  const Eigen::Vector3d rc2_roll_derivative =
    rotation_y * rotation_x_roll_derivative * params_.rc02;

  Eigen::Matrix2d joint_space_matrix;
  joint_space_matrix <<
    rrod1.dot(rc1_pitch_derivative), rrod1.dot(rc1_roll_derivative),
    rrod2.dot(rc2_pitch_derivative), rrod2.dot(rc2_roll_derivative);

  jacobian = motor_space_matrix.inverse() * joint_space_matrix;
  jacobian(0, 0) *= params_.motor_dir1;
  jacobian(0, 1) *= params_.motor_dir1;
  jacobian(1, 0) *= params_.motor_dir2;
  jacobian(1, 1) *= params_.motor_dir2;

  error_state = 0;
  return true;
}

bool ParallelKinematics::is_loaded() const
{
  return loaded_;
}

}  // namespace dm_humanoid
