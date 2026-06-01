#include "dm_test/dm_trajectory_test_node.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "yaml-cpp/yaml.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace dm_test
{

namespace
{

template<typename T>
T read_optional_value(const YAML::Node & node, const char * key, const T & default_value)
{
  return node[key] ? node[key].as<T>() : default_value;
}

std::string read_required_string(const YAML::Node & node, const char * key)
{
  if (!node[key]) {
    throw std::runtime_error(std::string("Missing required config key: ") + key);
  }
  return node[key].as<std::string>();
}

constexpr double kTimeEpsilon = 1e-9;

}  // namespace

DmTrajectoryTestNode::DmTrajectoryTestNode()
: Node("dm_trajectory_test_node")
{
  this->declare_parameter<std::string>("config_path", "");
  this->declare_parameter<std::string>("trajectory_name", "left_arm_small_range");
  this->declare_parameter<std::string>("trajectory_csv_path", "");
  this->declare_parameter<std::string>("time_column_name", "time");
  this->declare_parameter<double>("command_rate_hz", 100.0);
  this->declare_parameter<double>("playback_speed", 1.0);
  this->declare_parameter<double>("csv_row_rate_hz", 0.0);
  this->declare_parameter<bool>("enable_pre_positioning", true);
  this->declare_parameter<double>("pre_position_duration_sec", 5.0);
  this->declare_parameter<int>("command_timeout_ms", 20);
  this->declare_parameter<double>("start_delay_sec", 1.0);
  this->declare_parameter<bool>("auto_enable_on_start", true);
  this->declare_parameter<bool>("auto_disable_on_shutdown", false);

  trajectory_name_ = this->get_parameter("trajectory_name").as_string();
  trajectory_csv_path_ = this->get_parameter("trajectory_csv_path").as_string();
  time_column_name_ = this->get_parameter("time_column_name").as_string();
  command_rate_hz_ = this->get_parameter("command_rate_hz").as_double();
  playback_speed_ = this->get_parameter("playback_speed").as_double();
  csv_row_rate_hz_ = this->get_parameter("csv_row_rate_hz").as_double();
  enable_pre_positioning_ = this->get_parameter("enable_pre_positioning").as_bool();
  pre_position_duration_sec_ = this->get_parameter("pre_position_duration_sec").as_double();
  command_timeout_ms_ = this->get_parameter("command_timeout_ms").as_int();
  start_delay_sec_ = this->get_parameter("start_delay_sec").as_double();
  auto_enable_on_start_ = this->get_parameter("auto_enable_on_start").as_bool();
  auto_disable_on_shutdown_ = this->get_parameter("auto_disable_on_shutdown").as_bool();

  if (command_rate_hz_ <= 0.0) {
    throw std::runtime_error("command_rate_hz must be > 0");
  }
  if (playback_speed_ <= 0.0) {
    throw std::runtime_error("playback_speed must be > 0");
  }
  if (csv_row_rate_hz_ < 0.0) {
    throw std::runtime_error("csv_row_rate_hz must be >= 0");
  }
  if (pre_position_duration_sec_ < 0.0) {
    throw std::runtime_error("pre_position_duration_sec must be >= 0");
  }
  if (command_timeout_ms_ < 0) {
    throw std::runtime_error("command_timeout_ms must be >= 0");
  }
  if (start_delay_sec_ < 0.0) {
    throw std::runtime_error("start_delay_sec must be >= 0");
  }

  test_config_path_ = resolve_test_config_path(this->get_parameter("config_path").as_string());
  load_test_config();
  manager_ = std::make_unique<dm_motor::DmMotorManager>(motor_config_path_);
  validate_active_group_against_motor_config();

  reference_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "dm_test/reference_joint_states", 20);
  actual_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "dm_test/actual_joint_states", 20);
  error_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "dm_test/error_joint_states", 20);

  start_test_session();

  const auto period = std::chrono::duration<double>(1.0 / command_rate_hz_);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    std::bind(&DmTrajectoryTestNode::timer_callback, this));

  RCLCPP_INFO(
    this->get_logger(),
    "dm_trajectory_test_node ready. test_config=%s motor_config=%s trajectory=%s group=%s joints=%zu rate=%.2fHz",
    test_config_path_.c_str(),
    motor_config_path_.c_str(),
    active_trajectory_.name.c_str(),
    active_group_.name.c_str(),
    active_group_.joints.size(),
    command_rate_hz_);
}

DmTrajectoryTestNode::~DmTrajectoryTestNode()
{
  if (auto_disable_on_shutdown_) {
    stop_test_session();
  }
}

std::string DmTrajectoryTestNode::resolve_test_config_path(const std::string & configured_path) const
{
  if (!configured_path.empty()) {
    return configured_path;
  }

  const auto share_dir = ament_index_cpp::get_package_share_directory("dm_test");
  return share_dir + "/config/dm_test_trajectories.yaml";
}

std::string DmTrajectoryTestNode::resolve_path_relative_to_test_config(const std::string & path) const
{
  if (path.empty()) {
    return path;
  }

  constexpr const char * package_prefix = "package://";
  if (path.rfind(package_prefix, 0) == 0U) {
    const std::string package_uri = path.substr(std::char_traits<char>::length(package_prefix));
    const auto separator_index = package_uri.find('/');
    if (separator_index == std::string::npos) {
      throw std::runtime_error("package URI must include a package and relative path: " + path);
    }

    const auto package_name = package_uri.substr(0, separator_index);
    const auto relative_path = package_uri.substr(separator_index + 1U);
    const auto share_dir = ament_index_cpp::get_package_share_directory(package_name);
    return (std::filesystem::path(share_dir) / relative_path).lexically_normal().string();
  }

  const std::filesystem::path file_path(path);
  if (file_path.is_absolute()) {
    return file_path.string();
  }

  return (std::filesystem::path(test_config_path_).parent_path() / file_path).lexically_normal().string();
}

void DmTrajectoryTestNode::load_test_config()
{
  const YAML::Node root = YAML::LoadFile(test_config_path_);
  motor_config_path_ = resolve_path_relative_to_test_config(
    read_required_string(root, "motor_config_path"));

  const YAML::Node groups = root["groups"];
  if (!groups || !groups.IsMap()) {
    throw std::runtime_error("groups must be a YAML map");
  }

  const YAML::Node trajectory_node =
    root["trajectories"] ? root["trajectories"][trajectory_name_] : YAML::Node();
  if (!trajectory_node) {
    throw std::runtime_error("trajectory not found in config: " + trajectory_name_);
  }

  const std::string group_name = read_required_string(trajectory_node, "group");
  const YAML::Node group_node = groups[group_name];
  if (!group_node) {
    throw std::runtime_error(
      "group referenced by trajectory is not defined: " + group_name);
  }

  active_group_.name = group_name;
  active_group_.joints.clear();
  active_group_.joint_gains.clear();
  active_group_.csv_column_names.clear();
  active_group_.default_kp = read_optional_value<double>(group_node, "default_kp", 25.0);
  active_group_.default_kd = read_optional_value<double>(group_node, "default_kd", 1.0);

  const YAML::Node joints_node = group_node["joints"];
  if (!joints_node || !joints_node.IsSequence() || joints_node.size() == 0U) {
    throw std::runtime_error("group must define a non-empty joints sequence: " + group_name);
  }

  active_group_.joints.reserve(joints_node.size());
  for (const auto & joint_node : joints_node) {
    active_group_.joints.push_back(joint_node.as<std::string>());
  }

  if (const YAML::Node joint_gains_node = group_node["joint_gains"]) {
    if (!joint_gains_node.IsMap()) {
      throw std::runtime_error("joint_gains must be a map in group: " + group_name);
    }

    for (const auto & item : joint_gains_node) {
      const auto joint_name = item.first.as<std::string>();
      const auto gain_node = item.second;
      active_group_.joint_gains[joint_name] = JointGain {
        read_optional_value<double>(gain_node, "kp", active_group_.default_kp),
        read_optional_value<double>(gain_node, "kd", active_group_.default_kd)
      };
    }
  }

  if (const YAML::Node csv_columns_node = group_node["csv_columns"]) {
    if (!csv_columns_node.IsMap()) {
      throw std::runtime_error("csv_columns must be a map in group: " + group_name);
    }
    for (const auto & item : csv_columns_node) {
      active_group_.csv_column_names[item.first.as<std::string>()] = item.second.as<std::string>();
    }
  }

  std::unordered_map<std::string, size_t> joint_index_by_name;
  for (size_t index = 0; index < active_group_.joints.size(); ++index) {
    joint_index_by_name[active_group_.joints[index]] = index;
  }

  active_trajectory_.name = trajectory_name_;
  active_trajectory_.group_name = group_name;
  active_trajectory_.description = read_optional_value<std::string>(
    trajectory_node, "description", std::string());
  active_trajectory_.loop = read_optional_value<bool>(trajectory_node, "loop", false);
  active_trajectory_.use_absolute_positions = false;
  active_trajectory_.points.clear();

  const auto csv_path_from_config = read_optional_value<std::string>(
    trajectory_node, "csv_path", std::string());
  const auto resolved_csv_path = !trajectory_csv_path_.empty() ?
    resolve_path_relative_to_test_config(trajectory_csv_path_) :
    resolve_path_relative_to_test_config(csv_path_from_config);
  if (!resolved_csv_path.empty()) {
    load_csv_trajectory(resolved_csv_path);
    return;
  }

  const YAML::Node points_node = trajectory_node["points"];
  if (!points_node || !points_node.IsSequence() || points_node.size() == 0U) {
    throw std::runtime_error("trajectory points must be a non-empty sequence when csv_path is not used");
  }

  std::vector<RawTrajectoryPoint> raw_points;
  raw_points.reserve(points_node.size());
  for (const auto & point_node : points_node) {
    RawTrajectoryPoint raw_point;
    raw_point.time_from_start = point_node["time_from_start"].as<double>();
    if (raw_point.time_from_start < 0.0) {
      throw std::runtime_error("trajectory point time_from_start must be >= 0");
    }

    if (const YAML::Node offsets_node = point_node["offsets"]) {
      if (!offsets_node.IsMap()) {
        throw std::runtime_error("trajectory point offsets must be a map");
      }

      for (const auto & item : offsets_node) {
        const auto joint_name = item.first.as<std::string>();
        if (joint_index_by_name.find(joint_name) == joint_index_by_name.end()) {
          throw std::runtime_error(
            "trajectory offset references joint outside its group: " + joint_name);
        }
        raw_point.offsets[joint_name] = item.second.as<double>();
      }
    }

    raw_points.push_back(std::move(raw_point));
  }

  std::sort(
    raw_points.begin(), raw_points.end(),
    [](const RawTrajectoryPoint & lhs, const RawTrajectoryPoint & rhs) {
      return lhs.time_from_start < rhs.time_from_start;
    });

  if (raw_points.front().time_from_start > kTimeEpsilon) {
    raw_points.insert(raw_points.begin(), RawTrajectoryPoint {});
  }

  std::vector<double> current_offsets(active_group_.joints.size(), 0.0);
  active_trajectory_.points.clear();
  active_trajectory_.points.reserve(raw_points.size());

  for (size_t index = 0; index < raw_points.size(); ++index) {
    const auto & raw_point = raw_points[index];
    if (index > 0U) {
      const double previous_time = active_trajectory_.points.back().time_from_start;
      if (raw_point.time_from_start <= previous_time + kTimeEpsilon) {
        throw std::runtime_error("trajectory point times must be strictly increasing");
      }
    }

    for (const auto & [joint_name, offset] : raw_point.offsets) {
      current_offsets[joint_index_by_name.at(joint_name)] = offset;
    }

    active_trajectory_.points.push_back(TrajectoryPoint {
      raw_point.time_from_start,
      current_offsets
    });
  }

  active_trajectory_.duration =
    active_trajectory_.points.empty() ? 0.0 : active_trajectory_.points.back().time_from_start;
}

void DmTrajectoryTestNode::load_csv_trajectory(const std::string & csv_path)
{
  std::ifstream input(csv_path);
  if (!input.is_open()) {
    throw std::runtime_error("failed to open trajectory csv: " + csv_path);
  }

  std::string header_line;
  if (!std::getline(input, header_line)) {
    throw std::runtime_error("trajectory csv is empty: " + csv_path);
  }

  const auto headers = split_csv_line(header_line);
  if (headers.empty()) {
    throw std::runtime_error("trajectory csv header is empty: " + csv_path);
  }

  const auto time_index = find_time_column_index(headers);
  if (!time_index.has_value() && csv_row_rate_hz_ <= 0.0) {
    throw std::runtime_error(
      "trajectory csv is missing the configured time column and csv_row_rate_hz is not set: " +
      time_column_name_);
  }
  const auto joint_column_indices = resolve_csv_joint_column_indices(headers);

  std::vector<TrajectoryPoint> csv_points;
  std::string line;
  size_t line_number = 1U;
  size_t data_row_index = 0U;
  while (std::getline(input, line)) {
    ++line_number;
    if (trim_copy(line).empty()) {
      continue;
    }

    const auto cells = split_csv_line(line);

    TrajectoryPoint point;
    if (time_index.has_value()) {
      if (cells.size() <= *time_index) {
        throw std::runtime_error(
          "trajectory csv line is missing the time column at line " + std::to_string(line_number));
      }

      const auto time_cell = trim_copy(cells[*time_index]);
      if (time_cell.empty()) {
        continue;
      }
      point.time_from_start = std::stod(time_cell);
    } else {
      point.time_from_start = static_cast<double>(data_row_index) / csv_row_rate_hz_;
    }

    if (point.time_from_start < 0.0) {
      throw std::runtime_error("trajectory csv time must be >= 0 at line " + std::to_string(line_number));
    }

    point.offsets.reserve(active_group_.joints.size());
    for (size_t joint_index = 0; joint_index < active_group_.joints.size(); ++joint_index) {
      const auto column_index = joint_column_indices[joint_index];
      if (cells.size() <= column_index) {
        throw std::runtime_error(
          "trajectory csv line is missing joint column " +
          active_group_.joints[joint_index] + " at line " + std::to_string(line_number));
      }

      const auto value_cell = trim_copy(cells[column_index]);
      if (value_cell.empty()) {
        throw std::runtime_error(
          "trajectory csv joint value is empty for " + active_group_.joints[joint_index] +
          " at line " + std::to_string(line_number));
      }
      point.offsets.push_back(std::stod(value_cell));
    }

    csv_points.push_back(std::move(point));
    ++data_row_index;
  }

  if (csv_points.empty()) {
    throw std::runtime_error("trajectory csv contains no valid data rows: " + csv_path);
  }

  std::sort(
    csv_points.begin(), csv_points.end(),
    [](const TrajectoryPoint & lhs, const TrajectoryPoint & rhs) {
      return lhs.time_from_start < rhs.time_from_start;
    });

  for (size_t index = 1; index < csv_points.size(); ++index) {
    if (csv_points[index].time_from_start <= csv_points[index - 1U].time_from_start + kTimeEpsilon) {
      throw std::runtime_error("trajectory csv times must be strictly increasing");
    }
  }

  active_trajectory_.name = trajectory_name_;
  active_trajectory_.description =
    "CSV trajectory loaded from " + csv_path;
  active_trajectory_.use_absolute_positions = true;
  active_trajectory_.points = std::move(csv_points);
  active_trajectory_.duration = active_trajectory_.points.back().time_from_start;
}

void DmTrajectoryTestNode::validate_active_group_against_motor_config()
{
  motor_configs_by_name_.clear();
  for (const auto & joint_name : active_group_.joints) {
    const auto config = manager_->motor_config(joint_name);
    if (!config.has_value()) {
      throw std::runtime_error("joint in active group is missing from motor config: " + joint_name);
    }
    motor_configs_by_name_[joint_name] = *config;
  }
}

void DmTrajectoryTestNode::start_test_session()
{
  if (auto_enable_on_start_) {
    const auto results = manager_->enable_motors(
      active_group_.joints,
      std::chrono::milliseconds(command_timeout_ms_));
    log_command_results(results);
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      "auto_enable_on_start is false, assuming target joints are already enabled");
  }

  warmup_start_time_ = std::chrono::steady_clock::now();
  phase_ = ExecutionPhase::kWaitingForState;

  std::ostringstream oss;
  oss << "trajectory=" << active_trajectory_.name
      << " loop=" << (active_trajectory_.loop ? "true" : "false")
      << " playback_speed=" << playback_speed_
      << " enable_pre_positioning=" << (enable_pre_positioning_ ? "true" : "false")
      << " pre_position_duration_sec=" << pre_position_duration_sec_
      << " duration=" << active_trajectory_.duration
      << " joints=";
  for (size_t index = 0; index < active_group_.joints.size(); ++index) {
    if (index > 0U) {
      oss << ",";
    }
    oss << active_group_.joints[index];
  }
  RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
}

void DmTrajectoryTestNode::stop_test_session()
{
  if (!manager_) {
    return;
  }

  const auto results = manager_->disable_motors(
    active_group_.joints,
    std::chrono::milliseconds(command_timeout_ms_));
  log_command_results(results);
}

void DmTrajectoryTestNode::timer_callback()
{
  update_latest_states();

  if (phase_ == ExecutionPhase::kWaitingForState) {
    const auto elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - warmup_start_time_).count();
    if (elapsed < start_delay_sec_) {
      return;
    }

    initial_positions_ready_ = capture_initial_positions();
    if (!initial_positions_ready_) {
      std::ostringstream missing;
      bool first = true;
      for (const auto & joint_name : active_group_.joints) {
        if (latest_states_by_name_.find(joint_name) != latest_states_by_name_.end()) {
          continue;
        }
        if (!first) {
          missing << ",";
        }
        missing << joint_name;
        first = false;
      }

      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Waiting for initial joint states for trajectory %s, missing: %s",
        active_trajectory_.name.c_str(),
        missing.str().c_str());
      return;
    }

    std::ostringstream initial_pose;
    for (size_t index = 0; index < active_group_.joints.size(); ++index) {
      const auto & joint_name = active_group_.joints[index];
      if (index > 0U) {
        initial_pose << ", ";
      }
      initial_pose << joint_name << "=" << initial_positions_by_name_.at(joint_name);
    }
    RCLCPP_INFO(
      this->get_logger(),
      "Initial joint positions captured. %s",
      initial_pose.str().c_str());

    if (enable_pre_positioning_ && pre_position_duration_sec_ > kTimeEpsilon) {
      pre_position_start_time_ = std::chrono::steady_clock::now();
      phase_ = ExecutionPhase::kPrePositioning;
      RCLCPP_INFO(
        this->get_logger(),
        "Starting slow pre-positioning phase for %.2f seconds before trajectory playback.",
        pre_position_duration_sec_);
    } else {
      trajectory_start_time_ = std::chrono::steady_clock::now();
      phase_ = ExecutionPhase::kTracking;
      RCLCPP_INFO(this->get_logger(), "Pre-positioning skipped, starting trajectory playback.");
    }
  }

  SampledTrajectory sampled;
  if (phase_ == ExecutionPhase::kPrePositioning) {
    const double pre_position_elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - pre_position_start_time_).count();
    const double progress_ratio =
      pre_position_duration_sec_ <= kTimeEpsilon ?
      1.0 :
      std::clamp(pre_position_elapsed / pre_position_duration_sec_, 0.0, 1.0);

    sampled = build_pre_position_sample(progress_ratio);

    if (progress_ratio >= 1.0 - kTimeEpsilon) {
      trajectory_start_time_ = std::chrono::steady_clock::now();
      phase_ = ExecutionPhase::kTracking;
      RCLCPP_INFO(
        this->get_logger(),
        "Pre-positioning completed, entering trajectory tracking.");
    }
  } else {
    double sample_time = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - trajectory_start_time_).count() * playback_speed_;

    if (active_trajectory_.duration <= kTimeEpsilon) {
      sample_time = 0.0;
    } else if (active_trajectory_.loop) {
      sample_time = std::fmod(sample_time, active_trajectory_.duration);
    } else if (sample_time >= active_trajectory_.duration) {
      sample_time = active_trajectory_.duration;
      if (!completion_logged_) {
        RCLCPP_INFO(
          this->get_logger(),
          "Trajectory %s reached its final point and will keep holding the final pose.",
          active_trajectory_.name.c_str());
        completion_logged_ = true;
      }
    }

    sampled = sample_trajectory(sample_time);
  }
  std::vector<double> target_positions;
  std::vector<double> target_velocities;
  auto commands = build_commands(sampled, target_positions, target_velocities);

  const auto results = manager_->command_group_mit(
    commands,
    std::chrono::milliseconds(command_timeout_ms_));
  update_latest_states_from_results(results);
  publish_tracking_messages(target_positions, target_velocities);
  log_command_results(results);
}

bool DmTrajectoryTestNode::capture_initial_positions()
{
  initial_positions_by_name_.clear();

  for (const auto & joint_name : active_group_.joints) {
    const auto state_it = latest_states_by_name_.find(joint_name);
    if (state_it == latest_states_by_name_.end()) {
      return false;
    }
    initial_positions_by_name_[joint_name] = state_it->second.position;
  }

  return true;
}

DmTrajectoryTestNode::SampledTrajectory DmTrajectoryTestNode::build_pre_position_sample(
  const double progress_ratio) const
{
  SampledTrajectory sample;
  sample.offsets.assign(active_group_.joints.size(), 0.0);
  sample.velocities.assign(active_group_.joints.size(), 0.0);

  if (active_trajectory_.points.empty()) {
    return sample;
  }

  const auto & trajectory_start = active_trajectory_.points.front().offsets;
  const double clamped_ratio = std::clamp(progress_ratio, 0.0, 1.0);
  const double blend_duration = std::max(pre_position_duration_sec_, kTimeEpsilon);

  for (size_t joint_index = 0; joint_index < active_group_.joints.size(); ++joint_index) {
    const auto & joint_name = active_group_.joints[joint_index];
    const double current_position = initial_positions_by_name_.at(joint_name);
    const double target_position = active_trajectory_.use_absolute_positions ?
      trajectory_start[joint_index] :
      initial_positions_by_name_.at(joint_name) + trajectory_start[joint_index];

    sample.offsets[joint_index] = active_trajectory_.use_absolute_positions ?
      current_position + clamped_ratio * (target_position - current_position) :
      clamped_ratio * trajectory_start[joint_index];
    sample.velocities[joint_index] =
      clamped_ratio >= 1.0 - kTimeEpsilon ?
      0.0 :
      (target_position - current_position) / blend_duration;
  }

  return sample;
}

DmTrajectoryTestNode::SampledTrajectory DmTrajectoryTestNode::sample_trajectory(
  const double time_from_start) const
{
  SampledTrajectory sample;
  sample.offsets.assign(active_group_.joints.size(), 0.0);
  sample.velocities.assign(active_group_.joints.size(), 0.0);

  if (active_trajectory_.points.empty()) {
    return sample;
  }

  if (active_trajectory_.points.size() == 1U || time_from_start <= 0.0) {
    sample.offsets = active_trajectory_.points.front().offsets;
    return sample;
  }

  if (time_from_start >= active_trajectory_.duration) {
    sample.offsets = active_trajectory_.points.back().offsets;
    return sample;
  }

  for (size_t index = 1; index < active_trajectory_.points.size(); ++index) {
    const auto & previous_point = active_trajectory_.points[index - 1U];
    const auto & next_point = active_trajectory_.points[index];
    if (time_from_start > next_point.time_from_start) {
      continue;
    }

    const double segment_duration = next_point.time_from_start - previous_point.time_from_start;
    const double ratio =
      segment_duration <= kTimeEpsilon ?
      0.0 :
      (time_from_start - previous_point.time_from_start) / segment_duration;

    for (size_t joint_index = 0; joint_index < active_group_.joints.size(); ++joint_index) {
      const double start_offset = previous_point.offsets[joint_index];
      const double end_offset = next_point.offsets[joint_index];
      sample.offsets[joint_index] = start_offset + ratio * (end_offset - start_offset);
      sample.velocities[joint_index] =
        segment_duration <= kTimeEpsilon ? 0.0 : (end_offset - start_offset) / segment_duration;
    }
    return sample;
  }

  sample.offsets = active_trajectory_.points.back().offsets;
  return sample;
}

std::vector<dm_motor::NamedMitCommand> DmTrajectoryTestNode::build_commands(
  const SampledTrajectory & sample,
  std::vector<double> & target_positions,
  std::vector<double> & target_velocities)
{
  std::vector<dm_motor::NamedMitCommand> commands;
  commands.reserve(active_group_.joints.size());
  target_positions.clear();
  target_positions.reserve(active_group_.joints.size());
  target_velocities.clear();
  target_velocities.reserve(active_group_.joints.size());

  for (size_t joint_index = 0; joint_index < active_group_.joints.size(); ++joint_index) {
    const auto & joint_name = active_group_.joints[joint_index];
    const auto & motor_config = motor_configs_by_name_.at(joint_name);
    const auto unclamped_position = active_trajectory_.use_absolute_positions ?
      sample.offsets[joint_index] :
      initial_positions_by_name_.at(joint_name) + sample.offsets[joint_index];
    const double position = std::clamp(
      unclamped_position,
      static_cast<double>(motor_config.limits.position_min),
      static_cast<double>(motor_config.limits.position_max));
    const double velocity = std::clamp(
      sample.velocities[joint_index],
      static_cast<double>(motor_config.limits.velocity_min),
      static_cast<double>(motor_config.limits.velocity_max));

    if (std::abs(position - unclamped_position) > 1e-6) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Joint %s target was clamped from %.4f to %.4f",
        joint_name.c_str(),
        unclamped_position,
        position);
    }

    const auto gain = gain_for_joint(joint_name);
    dm_motor::MitCommand command;
    command.position = static_cast<float>(position);
    command.velocity = static_cast<float>(velocity);
    command.kp = static_cast<float>(gain.kp);
    command.kd = static_cast<float>(gain.kd);
    command.torque = 0.0F;

    commands.push_back(dm_motor::NamedMitCommand {joint_name, command});
    target_positions.push_back(position);
    target_velocities.push_back(velocity);
  }

  return commands;
}

void DmTrajectoryTestNode::update_latest_states()
{
  (void)manager_->poll_once(std::chrono::milliseconds(0));
  for (const auto & state : manager_->snapshot_states()) {
    latest_states_by_name_[state.name] = state;
  }
}

void DmTrajectoryTestNode::update_latest_states_from_results(
  const std::vector<dm_motor::MotorCommandResult> & results)
{
  for (const auto & result : results) {
    if (result.state.has_value()) {
      latest_states_by_name_[result.name] = *result.state;
    }
  }
}

void DmTrajectoryTestNode::publish_tracking_messages(
  const std::vector<double> & target_positions,
  const std::vector<double> & target_velocities)
{
  sensor_msgs::msg::JointState reference_msg;
  sensor_msgs::msg::JointState actual_msg;
  sensor_msgs::msg::JointState error_msg;

  const auto stamp = this->now();
  reference_msg.header.stamp = stamp;
  actual_msg.header.stamp = stamp;
  error_msg.header.stamp = stamp;

  reference_msg.name = active_group_.joints;
  actual_msg.name = active_group_.joints;
  error_msg.name = active_group_.joints;

  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (size_t index = 0; index < active_group_.joints.size(); ++index) {
    const auto & joint_name = active_group_.joints[index];
    reference_msg.position.push_back(target_positions[index]);
    reference_msg.velocity.push_back(target_velocities[index]);
    reference_msg.effort.push_back(0.0);

    const auto state_it = latest_states_by_name_.find(joint_name);
    if (state_it == latest_states_by_name_.end()) {
      actual_msg.position.push_back(nan);
      actual_msg.velocity.push_back(nan);
      actual_msg.effort.push_back(nan);
      error_msg.position.push_back(nan);
      error_msg.velocity.push_back(nan);
      error_msg.effort.push_back(nan);
      continue;
    }

    const auto & state = state_it->second;
    actual_msg.position.push_back(state.position);
    actual_msg.velocity.push_back(state.velocity);
    actual_msg.effort.push_back(state.torque);

    error_msg.position.push_back(state.position - target_positions[index]);
    error_msg.velocity.push_back(state.velocity - target_velocities[index]);
    error_msg.effort.push_back(state.torque);
  }

  reference_pub_->publish(reference_msg);
  actual_pub_->publish(actual_msg);
  error_pub_->publish(error_msg);
}

void DmTrajectoryTestNode::log_command_results(
  const std::vector<dm_motor::MotorCommandResult> & results)
{
  size_t failure_count = 0U;
  std::ostringstream failures;
  for (const auto & result : results) {
    if (result.success) {
      continue;
    }

    ++failure_count;
    if (failure_count > 1U) {
      failures << " | ";
    }
    failures << result.name << ": " << result.message;
  }

  if (failure_count == 0U) {
    return;
  }

  RCLCPP_WARN_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000,
    "Trajectory command reported %zu/%zu failures: %s",
    failure_count,
    results.size(),
    failures.str().c_str());
}

DmTrajectoryTestNode::JointGain DmTrajectoryTestNode::gain_for_joint(
  const std::string & joint_name) const
{
  const auto it = active_group_.joint_gains.find(joint_name);
  if (it != active_group_.joint_gains.end()) {
    return it->second;
  }

  return JointGain {active_group_.default_kp, active_group_.default_kd};
}

std::vector<std::string> DmTrajectoryTestNode::split_csv_line(const std::string & line) const
{
  std::vector<std::string> cells;
  std::string current;
  bool in_quotes = false;

  for (size_t index = 0; index < line.size(); ++index) {
    const char ch = line[index];
    if (ch == '"') {
      if (in_quotes && index + 1U < line.size() && line[index + 1U] == '"') {
        current.push_back('"');
        ++index;
      } else {
        in_quotes = !in_quotes;
      }
      continue;
    }

    if (ch == ',' && !in_quotes) {
      cells.push_back(current);
      current.clear();
      continue;
    }

    current.push_back(ch);
  }

  cells.push_back(current);
  return cells;
}

std::optional<size_t> DmTrajectoryTestNode::find_time_column_index(
  const std::vector<std::string> & headers) const
{
  const auto configured_name = to_lower_copy(trim_copy(time_column_name_));
  for (size_t index = 0; index < headers.size(); ++index) {
    if (to_lower_copy(trim_copy(headers[index])) == configured_name) {
      return index;
    }
  }
  return std::nullopt;
}

std::vector<size_t> DmTrajectoryTestNode::resolve_csv_joint_column_indices(
  const std::vector<std::string> & headers) const
{
  std::vector<size_t> indices;
  indices.reserve(active_group_.joints.size());

  for (const auto & joint_name : active_group_.joints) {
    const auto mapped_name_it = active_group_.csv_column_names.find(joint_name);
    const auto expected_header_name = mapped_name_it != active_group_.csv_column_names.end() ?
      mapped_name_it->second :
      joint_name;
    const auto normalized_joint_name = to_lower_copy(expected_header_name);
    bool found = false;
    for (size_t header_index = 0; header_index < headers.size(); ++header_index) {
      if (to_lower_copy(trim_copy(headers[header_index])) == normalized_joint_name) {
        indices.push_back(header_index);
        found = true;
        break;
      }
    }

    if (!found) {
      throw std::runtime_error(
        "trajectory csv is missing a joint column named: " + expected_header_name);
    }
  }

  return indices;
}

std::string DmTrajectoryTestNode::trim_copy(const std::string & value)
{
  size_t begin = 0U;
  while (begin < value.size() && std::isspace(static_cast<unsigned char>(value[begin])) != 0) {
    ++begin;
  }

  size_t end = value.size();
  while (end > begin && std::isspace(static_cast<unsigned char>(value[end - 1U])) != 0) {
    --end;
  }

  return value.substr(begin, end - begin);
}

std::string DmTrajectoryTestNode::to_lower_copy(const std::string & value)
{
  std::string output = value;
  std::transform(
    output.begin(), output.end(), output.begin(),
    [](const unsigned char ch) {
      return static_cast<char>(std::tolower(ch));
    });
  return output;
}

}  // namespace dm_test
