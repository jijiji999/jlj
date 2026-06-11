#include "dm_motor/dm_motor_manager.hpp"
#include "dm_humanoid/parallel_joint_adapter.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "yaml-cpp/yaml.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <functional>
#include <memory>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#ifndef DM_MOTOR_HAS_ONNXRUNTIME
#define DM_MOTOR_HAS_ONNXRUNTIME 0
#endif

#if DM_MOTOR_HAS_ONNXRUNTIME
#include <onnxruntime_cxx_api.h>
#endif

namespace dm_motor
{

using dm_humanoid::LogicalJointCommand;
using dm_humanoid::LogicalJointState;
using dm_humanoid::ParallelJointAdapter;
using dm_humanoid::ParallelMechanismSpec;

namespace
{

using namespace std::chrono_literals;

constexpr size_t kCommandDimension = 3U;
constexpr size_t kImuAngularVelocityDimension = 3U;
constexpr size_t kProjectedGravityDimension = 3U;

template<typename T>
T read_optional_value(const YAML::Node & node, const char * key, const T & default_value)
{
  return node[key] ? node[key].as<T>() : default_value;
}

std::string to_lower_copy(const std::string & value)
{
  std::string lowered = value;
  std::transform(
    lowered.begin(), lowered.end(), lowered.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return lowered;
}

template<typename T>
T clamp_value(const T value, const T min_value, const T max_value)
{
  return std::max(min_value, std::min(value, max_value));
}

enum class RobotMode
{
  kFixed,
  kPassive,
  kLoco,
  kDance,
};

std::string mode_to_string(const RobotMode mode)
{
  switch (mode) {
    case RobotMode::kFixed:
      return "fixed";
    case RobotMode::kPassive:
      return "passive";
    case RobotMode::kLoco:
      return "loco";
    case RobotMode::kDance:
      return "dance";
    default:
      return "unknown";
  }
}

std::optional<RobotMode> parse_mode(const std::string & name)
{
  const auto lowered = to_lower_copy(name);
  if (lowered == "fixed") {
    return RobotMode::kFixed;
  }
  if (lowered == "passive") {
    return RobotMode::kPassive;
  }
  if (lowered == "loco") {
    return RobotMode::kLoco;
  }
  if (lowered == "dance") {
    return RobotMode::kDance;
  }
  return std::nullopt;
}

struct ModeGains
{
  float kp {0.0F};
  float kd {0.0F};
};

struct JoyMapping
{
  int mode_confirm_button {0};
  int fixed_modifier_button {4};
  int passive_modifier_axis {2};
  int loco_modifier_button {5};
  int dance_modifier_axis {5};
  float passive_modifier_axis_sign {1.0F};
  float dance_modifier_axis_sign {1.0F};
  float trigger_activation_threshold {0.5F};
  int linear_x_axis {1};
  int linear_y_axis {0};
  int yaw_axis {3};
  float linear_x_scale {0.6F};
  float linear_y_scale {0.25F};
  float yaw_scale {1.0F};
  float linear_x_sign {-1.0F};
  float linear_y_sign {1.0F};
  float yaw_sign {1.0F};
};

struct ObservationScales
{
  float angular_velocity {1.0F};
  float gravity {1.0F};
  float command {1.0F};
  float joint_position {1.0F};
  float joint_velocity {1.0F};
  float previous_action {1.0F};
};

struct PolicyConfig
{
  std::string model_path;
  std::string input_name;
  std::string output_name;
  size_t expected_observation_dim {0U};
  size_t expected_action_dim {0U};
  float action_clip {1.0F};
  ObservationScales observation_scales {};
};

struct JointControlConfig
{
  std::string name;
  std::optional<std::string> gain_profile_name;
  std::optional<float> configured_initial_position;
  std::optional<float> configured_fixed_position;
  float initial_position {0.0F};
  bool initial_position_ready {false};
  ModeGains fixed_gains {};
  ModeGains passive_gains {};
  ModeGains loco_gains {};
  ModeGains dance_gains {};
  float action_scale {0.25F};
  float action_offset {0.0F};
};

struct GainProfile
{
  ModeGains fixed {};
  ModeGains passive {};
  ModeGains loco {};
  ModeGains dance {};
};

struct ParallelMechanismConfig
{
  std::string name;
  std::string pitch_joint_name;
  std::string roll_joint_name;
  std::string motor_1_name;
  std::string motor_2_name;
  std::string config_path;
};

float joy_axis_or_zero(const sensor_msgs::msg::Joy & message, const int index)
{
  if (index < 0 || static_cast<size_t>(index) >= message.axes.size()) {
    return 0.0F;
  }
  return message.axes[static_cast<size_t>(index)];
}

bool joy_button_pressed(const sensor_msgs::msg::Joy & message, const int index)
{
  if (index < 0 || static_cast<size_t>(index) >= message.buttons.size()) {
    return false;
  }
  return message.buttons[static_cast<size_t>(index)] != 0;
}

bool joy_axis_activated(
  const sensor_msgs::msg::Joy & message,
  const int index,
  const float direction_sign,
  const float activation_threshold)
{
  const float axis_value = joy_axis_or_zero(message, index);
  return direction_sign * axis_value >= activation_threshold;
}

std::array<float, 3> compute_projected_gravity(const sensor_msgs::msg::Imu & imu)
{
  tf2::Quaternion orientation(
    imu.orientation.x,
    imu.orientation.y,
    imu.orientation.z,
    imu.orientation.w);

  if (orientation.length2() < 1.0e-8) {
    return {0.0F, 0.0F, -1.0F};
  }

  orientation.normalize();
  const tf2::Matrix3x3 world_from_body(orientation);
  const tf2::Vector3 gravity_world(0.0, 0.0, -1.0);
  const tf2::Vector3 gravity_body = world_from_body.transpose() * gravity_world;

  return {
    static_cast<float>(gravity_body.x()),
    static_cast<float>(gravity_body.y()),
    static_cast<float>(gravity_body.z())
  };
}

class PolicyBackend
{
public:
  virtual ~PolicyBackend() = default;

  virtual bool run(
    const std::vector<float> & observation,
    std::vector<float> & action,
    std::string & error_message) = 0;

  virtual std::string description() const = 0;
  virtual bool is_fallback() const = 0;
};

class ZeroPolicyBackend final : public PolicyBackend
{
public:
  ZeroPolicyBackend(const size_t action_dimension, std::string reason)
  : action_dimension_(action_dimension), reason_(std::move(reason))
  {
  }

  bool run(
    const std::vector<float> &,
    std::vector<float> & action,
    std::string & error_message) override
  {
    action.assign(action_dimension_, 0.0F);
    error_message = reason_;
    return true;
  }

  std::string description() const override
  {
    return reason_;
  }

  bool is_fallback() const override
  {
    return true;
  }

private:
  size_t action_dimension_ {0U};
  std::string reason_;
};

#if DM_MOTOR_HAS_ONNXRUNTIME
class OnnxPolicyBackend final : public PolicyBackend
{
public:
  explicit OnnxPolicyBackend(const PolicyConfig & config)
  : config_(config),
    env_(ORT_LOGGING_LEVEL_WARNING, "dm_humanoid_control"),
    session_options_(),
    session_(nullptr)
  {
    session_options_.SetIntraOpNumThreads(1);
    session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
    session_ = Ort::Session(env_, config_.model_path.c_str(), session_options_);

    Ort::AllocatorWithDefaultOptions allocator;
    const auto input_name = session_.GetInputNameAllocated(0, allocator);
    const auto output_name = session_.GetOutputNameAllocated(0, allocator);

    input_name_ = config_.input_name.empty() ? input_name.get() : config_.input_name;
    output_name_ = config_.output_name.empty() ? output_name.get() : config_.output_name;
  }

  bool run(
    const std::vector<float> & observation,
    std::vector<float> & action,
    std::string & error_message) override
  {
    try {
      const std::array<int64_t, 2> input_shape {
        1,
        static_cast<int64_t>(observation.size())
      };

      Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(
        OrtArenaAllocator,
        OrtMemTypeDefault);
      Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info,
        const_cast<float *>(observation.data()),
        observation.size(),
        input_shape.data(),
        input_shape.size());

      const std::array<const char *, 1> input_names {input_name_.c_str()};
      const std::array<const char *, 1> output_names {output_name_.c_str()};
      auto output_tensors = session_.Run(
        Ort::RunOptions {nullptr},
        input_names.data(),
        &input_tensor,
        1U,
        output_names.data(),
        1U);

      if (output_tensors.empty() || !output_tensors.front().IsTensor()) {
        error_message = "ONNX policy returned no tensor output";
        return false;
      }

      auto & output_tensor = output_tensors.front();
      const auto shape_info = output_tensor.GetTensorTypeAndShapeInfo();
      const size_t element_count = shape_info.GetElementCount();
      float * output_data = output_tensor.GetTensorMutableData<float>();

      action.assign(output_data, output_data + element_count);
      error_message.clear();
      return true;
    } catch (const Ort::Exception & exception) {
      error_message = exception.what();
      return false;
    }
  }

  std::string description() const override
  {
    return "onnxruntime";
  }

  bool is_fallback() const override
  {
    return false;
  }

private:
  PolicyConfig config_;
  Ort::Env env_;
  Ort::SessionOptions session_options_;
  Ort::Session session_;
  std::string input_name_;
  std::string output_name_;
};
#endif

}  // namespace

class DmHumanoidControlNode : public rclcpp::Node
{
public:
  DmHumanoidControlNode()
  : Node("dm_humanoid_control_node")
  {
    this->declare_parameter<std::string>("config_path", "");
    this->declare_parameter<std::string>("motor_config_path", "");
    this->declare_parameter<std::string>("joy_topic", "");
    this->declare_parameter<std::string>("startup_mode", "");
    this->declare_parameter<std::string>("policy_model_path", "");
    this->declare_parameter<std::string>("mode_command_topic", "");
    this->declare_parameter<double>("control_hz", 0.0);
    this->declare_parameter<double>("policy_hz", 0.0);
    this->declare_parameter<int>("command_timeout_ms", 0);
    this->declare_parameter<bool>("fixed_passive_test_only", false);

    controller_config_path_ = resolve_controller_config_path(
      this->get_parameter("config_path").as_string());
    const YAML::Node root = YAML::LoadFile(controller_config_path_);
    motor_config_path_ = resolve_motor_config_path(root);
    manager_ = std::make_unique<DmMotorManager>(motor_config_path_);
    load_controller_config(root);
    apply_parameter_overrides();
    create_policy_backend();

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_, 20,
      std::bind(&DmHumanoidControlNode::joy_callback, this, std::placeholders::_1));
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, 20,
      std::bind(&DmHumanoidControlNode::imu_callback, this, std::placeholders::_1));
    mode_command_sub_ = this->create_subscription<std_msgs::msg::String>(
      mode_command_topic_, 20,
      std::bind(&DmHumanoidControlNode::mode_command_callback, this, std::placeholders::_1));

    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 20);
    raw_joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "raw_motor_states", 20);
    mode_pub_ = this->create_publisher<std_msgs::msg::String>(mode_topic_, 10);

    if (publish_policy_debug_topics_) {
      observation_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "humanoid_control/policy_observation", 10);
      action_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "humanoid_control/policy_action", 10);
    }

    if (auto_enable_on_start_) {
      const auto results = manager_->enable_all(std::chrono::milliseconds(command_timeout_ms_));
      log_results("enable_all", results, true);
      const auto initial_states = manager_->snapshot_states();
      update_motor_state_cache(initial_states);
      rebuild_logical_joint_state_cache(initial_states);
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "auto_enable_on_start is false, control node will send commands without a startup enable step");
    }

    maybe_capture_initial_positions();
    switch_mode(startup_mode_);

    const auto period = std::chrono::duration<double>(1.0 / control_hz_);
    control_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&DmHumanoidControlNode::control_loop, this));

    RCLCPP_INFO(
      this->get_logger(),
      "dm_humanoid_control_node ready. controller_config=%s motor_config=%s joints=%zu mode=%s control_hz=%.2f joy=%s imu=%s policy=%s",
      controller_config_path_.c_str(),
      motor_config_path_.c_str(),
      joints_.size(),
      mode_to_string(mode_).c_str(),
      control_hz_,
      joy_topic_.c_str(),
      imu_topic_.c_str(),
      policy_backend_->description().c_str());
    if (fixed_passive_test_only_) {
      RCLCPP_INFO(
        this->get_logger(),
        "fixed_passive_test_only is enabled. mode_command_topic=%s",
        mode_command_topic_.c_str());
    }
  }

  ~DmHumanoidControlNode() override
  {
    if (auto_disable_on_shutdown_ && manager_) {
      const auto results = manager_->disable_all(std::chrono::milliseconds(command_timeout_ms_));
      log_results("disable_all", results, true);
    }
  }

private:
  std::string resolve_controller_config_path(const std::string & configured_path) const
  {
    if (!configured_path.empty()) {
      return configured_path;
    }

    const auto share_dir = ament_index_cpp::get_package_share_directory("dm_humanoid");
    return share_dir + "/config/dm_humanoid_29.yaml";
  }

  std::string resolve_default_motor_config_path() const
  {
    const auto share_dir = ament_index_cpp::get_package_share_directory("dm_motor");
    return share_dir + "/config/dm_motor_29.yaml";
  }

  std::string resolve_motor_config_path(const YAML::Node & root) const
  {
    const auto parameter_override = this->get_parameter("motor_config_path").as_string();
    if (!parameter_override.empty()) {
      return resolve_auxiliary_path(parameter_override);
    }

    const YAML::Node controller = root["controller"];
    const auto configured_path = read_optional_value<std::string>(
      controller, "motor_config_path", std::string());
    if (!configured_path.empty()) {
      return resolve_auxiliary_path(configured_path);
    }

    return resolve_default_motor_config_path();
  }

  std::string resolve_auxiliary_path(const std::string & maybe_relative_path) const
  {
    if (maybe_relative_path.empty()) {
      return maybe_relative_path;
    }

    const std::filesystem::path path(maybe_relative_path);
    if (path.is_absolute()) {
      return maybe_relative_path;
    }

    return (std::filesystem::path(controller_config_path_).parent_path() / path)
      .lexically_normal().string();
  }

  void load_parallel_mechanisms(const YAML::Node & root)
  {
    parallel_mechanism_configs_.clear();
    parallel_adapter_ = ParallelJointAdapter {};

    const YAML::Node mechanisms = root["parallel_mechanisms"];
    if (!mechanisms || !mechanisms.IsSequence()) {
      return;
    }

    for (const auto & mechanism_node : mechanisms) {
      ParallelMechanismConfig config;
      config.name = read_optional_value<std::string>(mechanism_node, "name", std::string());
      config.pitch_joint_name = read_optional_value<std::string>(
        mechanism_node, "pitch_joint", std::string());
      config.roll_joint_name = read_optional_value<std::string>(
        mechanism_node, "roll_joint", std::string());
      config.motor_1_name = read_optional_value<std::string>(
        mechanism_node, "motor_1", std::string());
      config.motor_2_name = read_optional_value<std::string>(
        mechanism_node, "motor_2", std::string());
      config.config_path = resolve_auxiliary_path(
        read_optional_value<std::string>(mechanism_node, "config_path", std::string()));

      std::string error_message;
      if (!parallel_adapter_.add_mechanism(
          ParallelMechanismSpec {
            config.name,
            config.pitch_joint_name,
            config.roll_joint_name,
            config.motor_1_name,
            config.motor_2_name,
            config.config_path},
          error_message))
      {
        throw std::runtime_error(
                "Failed to load parallel mechanism " + config.name + ": " + error_message);
      }

      parallel_mechanism_configs_.push_back(config);
    }
  }

  void load_controller_config(const YAML::Node & root)
  {
    const YAML::Node controller = root["controller"];
    load_parallel_mechanisms(root);

    joy_topic_ = read_optional_value<std::string>(controller, "joy_topic", "joy");
    imu_topic_ = read_optional_value<std::string>(controller, "imu_topic", "imu/data");
    mode_topic_ = read_optional_value<std::string>(
      controller, "mode_topic", "humanoid_control/mode");
    mode_command_topic_ = read_optional_value<std::string>(
      controller, "mode_command_topic", "humanoid_control/mode_command");
    control_hz_ = read_optional_value<double>(controller, "control_hz", 100.0);
    policy_hz_ = read_optional_value<double>(controller, "policy_hz", 50.0);
    command_timeout_ms_ = read_optional_value<int>(controller, "command_timeout_ms", 5);
    rx_poll_timeout_ms_ = read_optional_value<int>(controller, "rx_poll_timeout_ms", 0);
    auto_enable_on_start_ = read_optional_value<bool>(controller, "auto_enable_on_start", true);
    auto_disable_on_shutdown_ = read_optional_value<bool>(
      controller, "auto_disable_on_shutdown", true);
    publish_policy_debug_topics_ = read_optional_value<bool>(
      controller, "publish_policy_debug_topics", true);
    fixed_passive_test_only_ = read_optional_value<bool>(
      controller, "fixed_passive_test_only", false);

    const auto startup_mode_name = read_optional_value<std::string>(
      controller, "startup_mode", "passive");
    startup_mode_ = parse_mode(startup_mode_name).value_or(RobotMode::kPassive);

    const YAML::Node joystick = controller["joystick"];
    joy_mapping_.mode_confirm_button = read_optional_value<int>(
      joystick, "mode_confirm_button", 0);
    joy_mapping_.fixed_modifier_button = read_optional_value<int>(
      joystick, "fixed_modifier_button", 4);
    joy_mapping_.passive_modifier_axis = read_optional_value<int>(
      joystick, "passive_modifier_axis", 2);
    joy_mapping_.loco_modifier_button = read_optional_value<int>(
      joystick, "loco_modifier_button", 5);
    joy_mapping_.dance_modifier_axis = read_optional_value<int>(
      joystick, "dance_modifier_axis", 5);
    joy_mapping_.passive_modifier_axis_sign = read_optional_value<float>(
      joystick, "passive_modifier_axis_sign", 1.0F);
    joy_mapping_.dance_modifier_axis_sign = read_optional_value<float>(
      joystick, "dance_modifier_axis_sign", 1.0F);
    joy_mapping_.trigger_activation_threshold = read_optional_value<float>(
      joystick, "trigger_activation_threshold", 0.5F);
    joy_mapping_.linear_x_axis = read_optional_value<int>(joystick, "linear_x_axis", 1);
    joy_mapping_.linear_y_axis = read_optional_value<int>(joystick, "linear_y_axis", 0);
    joy_mapping_.yaw_axis = read_optional_value<int>(joystick, "yaw_axis", 3);
    joy_mapping_.linear_x_scale = read_optional_value<float>(joystick, "linear_x_scale", 0.6F);
    joy_mapping_.linear_y_scale = read_optional_value<float>(joystick, "linear_y_scale", 0.25F);
    joy_mapping_.yaw_scale = read_optional_value<float>(joystick, "yaw_scale", 1.0F);
    joy_mapping_.linear_x_sign = read_optional_value<float>(joystick, "linear_x_sign", -1.0F);
    joy_mapping_.linear_y_sign = read_optional_value<float>(joystick, "linear_y_sign", 1.0F);
    joy_mapping_.yaw_sign = read_optional_value<float>(joystick, "yaw_sign", 1.0F);

    const YAML::Node fixed_mode = controller["fixed_mode"];
    const YAML::Node passive_mode = controller["passive_mode"];
    const YAML::Node loco_mode = controller["loco_mode"];
    const YAML::Node dance_mode = controller["dance_mode"];

    const ModeGains fixed_default {
      read_optional_value<float>(fixed_mode, "kp", 80.0F),
      read_optional_value<float>(fixed_mode, "kd", 2.0F)
    };
    const ModeGains passive_default {
      read_optional_value<float>(passive_mode, "kp", 0.0F),
      read_optional_value<float>(passive_mode, "kd", 0.2F)
    };
    const ModeGains loco_default {
      read_optional_value<float>(loco_mode, "kp", 40.0F),
      read_optional_value<float>(loco_mode, "kd", 1.0F)
    };
    const ModeGains dance_default {
      read_optional_value<float>(dance_mode, "kp", 50.0F),
      read_optional_value<float>(dance_mode, "kd", 1.0F)
    };
    fixed_mode_transition_duration_sec_ = read_optional_value<float>(
      fixed_mode, "transition_duration_sec", 1.0F);
    fixed_mode_default_gains_ = fixed_default;
    passive_mode_default_gains_ = passive_default;

    gain_profiles_.clear();
    const YAML::Node gain_profiles = controller["gain_profiles"];
    if (gain_profiles && gain_profiles.IsMap()) {
      for (const auto & item : gain_profiles) {
        const auto profile_name = item.first.as<std::string>();
        const auto profile_node = item.second;
        GainProfile profile;
        profile.fixed.kp = read_optional_value<float>(profile_node, "fixed_kp", fixed_default.kp);
        profile.fixed.kd = read_optional_value<float>(profile_node, "fixed_kd", fixed_default.kd);
        profile.passive.kp = read_optional_value<float>(
          profile_node, "passive_kp", passive_default.kp);
        profile.passive.kd = read_optional_value<float>(
          profile_node, "passive_kd", passive_default.kd);
        profile.loco.kp = read_optional_value<float>(profile_node, "loco_kp", loco_default.kp);
        profile.loco.kd = read_optional_value<float>(profile_node, "loco_kd", loco_default.kd);
        profile.dance.kp = read_optional_value<float>(
          profile_node, "dance_kp", dance_default.kp);
        profile.dance.kd = read_optional_value<float>(
          profile_node, "dance_kd", dance_default.kd);
        gain_profiles_[profile_name] = profile;
      }
    }

    const YAML::Node policy = controller["policy"];
    policy_config_.model_path = resolve_auxiliary_path(
      read_optional_value<std::string>(policy, "model_path", std::string()));
    policy_config_.input_name = read_optional_value<std::string>(policy, "input_name", std::string());
    policy_config_.output_name = read_optional_value<std::string>(policy, "output_name", std::string());
    policy_config_.action_clip = read_optional_value<float>(loco_mode, "action_clip", 1.0F);
    policy_config_.observation_scales.angular_velocity = read_optional_value<float>(
      policy, "angular_velocity_scale", 1.0F);
    policy_config_.observation_scales.gravity = read_optional_value<float>(
      policy, "gravity_scale", 1.0F);
    policy_config_.observation_scales.command = read_optional_value<float>(
      policy, "command_scale", 1.0F);
    policy_config_.observation_scales.joint_position = read_optional_value<float>(
      policy, "joint_position_scale", 1.0F);
    policy_config_.observation_scales.joint_velocity = read_optional_value<float>(
      policy, "joint_velocity_scale", 1.0F);
    policy_config_.observation_scales.previous_action = read_optional_value<float>(
      policy, "previous_action_scale", 1.0F);

    std::vector<std::string> joint_order;
    if (controller["policy_joint_order"] && controller["policy_joint_order"].IsSequence()) {
      for (const auto & joint_name : controller["policy_joint_order"]) {
        joint_order.push_back(joint_name.as<std::string>());
      }
    } else {
      joint_order = default_joint_order();
      RCLCPP_WARN(
        this->get_logger(),
        "controller.policy_joint_order is not set, falling back to logical joint order");
    }

    validate_joint_order(joint_order);

    std::unordered_map<std::string, YAML::Node> motor_nodes;
    const YAML::Node joint_nodes = root["joints"] ? root["joints"] : root["motors"];
    if (joint_nodes && joint_nodes.IsSequence()) {
      for (const auto & motor_node : joint_nodes) {
        if (motor_node["name"]) {
          motor_nodes.emplace(
            motor_node["name"].as<std::string>(),
            YAML::Clone(motor_node));
        }
      }
    }

    joints_.clear();
    joints_.reserve(joint_order.size());
    joint_lookup_.clear();
    for (const auto & name : joint_order) {
      JointControlConfig config;
      config.name = name;
      config.fixed_gains = fixed_default;
      config.passive_gains = passive_default;
      config.loco_gains = loco_default;
      config.dance_gains = dance_default;
      config.action_scale = read_optional_value<float>(loco_mode, "action_scale", 0.25F);
      config.action_offset = read_optional_value<float>(loco_mode, "action_offset", 0.0F);

      const auto motor_it = motor_nodes.find(name);
      if (motor_it != motor_nodes.end()) {
        const auto & motor_node = motor_it->second;
        const YAML::Node control_node = motor_node["control"] ? motor_node["control"] : motor_node;

        if (control_node["gain_profile"]) {
          config.gain_profile_name = control_node["gain_profile"].as<std::string>();
          const auto profile_it = gain_profiles_.find(*config.gain_profile_name);
          if (profile_it == gain_profiles_.end()) {
            throw std::runtime_error(
                    "Unknown gain_profile for joint " + name + ": " + *config.gain_profile_name);
          }
          config.fixed_gains = profile_it->second.fixed;
          config.passive_gains = profile_it->second.passive;
          config.loco_gains = profile_it->second.loco;
          config.dance_gains = profile_it->second.dance;
        }

        if (control_node["initial_position"]) {
          config.configured_initial_position = control_node["initial_position"].as<float>();
          config.initial_position = *config.configured_initial_position;
          config.initial_position_ready = true;
        }
        if (control_node["fixed_position"]) {
          config.configured_fixed_position = control_node["fixed_position"].as<float>();
        }
        config.fixed_gains.kp = read_optional_value<float>(
          control_node, "fixed_kp", config.fixed_gains.kp);
        config.fixed_gains.kd = read_optional_value<float>(
          control_node, "fixed_kd", config.fixed_gains.kd);
        config.passive_gains.kp = read_optional_value<float>(
          control_node, "passive_kp", config.passive_gains.kp);
        config.passive_gains.kd = read_optional_value<float>(
          control_node, "passive_kd", config.passive_gains.kd);
        config.loco_gains.kp = read_optional_value<float>(
          control_node, "loco_kp", config.loco_gains.kp);
        config.loco_gains.kd = read_optional_value<float>(
          control_node, "loco_kd", config.loco_gains.kd);
        config.dance_gains.kp = read_optional_value<float>(
          control_node, "dance_kp", config.dance_gains.kp);
        config.dance_gains.kd = read_optional_value<float>(
          control_node, "dance_kd", config.dance_gains.kd);
        config.action_scale = read_optional_value<float>(
          control_node, "action_scale", config.action_scale);
        config.action_offset = read_optional_value<float>(
          control_node, "action_offset", config.action_offset);
      }

      joint_lookup_[config.name] = joints_.size();
      joints_.push_back(config);
    }

    policy_config_.expected_action_dim = read_optional_value<size_t>(
      policy, "expected_action_dim", joints_.size());
    const size_t derived_observation_dim =
      kImuAngularVelocityDimension + kProjectedGravityDimension + kCommandDimension +
      joints_.size() * 3U;
    policy_config_.expected_observation_dim = read_optional_value<size_t>(
      policy, "expected_observation_dim", derived_observation_dim);

    if (policy_config_.expected_action_dim != joints_.size()) {
      throw std::runtime_error(
              "policy.expected_action_dim must match controller joint count");
    }
    if (policy_config_.expected_observation_dim != derived_observation_dim) {
      throw std::runtime_error(
              "policy.expected_observation_dim does not match the hard-coded observation layout");
    }

    previous_action_.assign(policy_config_.expected_action_dim, 0.0F);
    latest_action_.assign(policy_config_.expected_action_dim, 0.0F);
  }

  std::vector<std::string> default_joint_order() const
  {
    std::vector<std::string> order;
    const auto motor_names = manager_->motor_names();
    order.reserve(motor_names.size());
    for (const auto & name : motor_names) {
      if (!parallel_adapter_.is_parallel_motor(name)) {
        order.push_back(name);
      }
    }

    for (const auto & mechanism : parallel_mechanism_configs_) {
      order.push_back(mechanism.pitch_joint_name);
      order.push_back(mechanism.roll_joint_name);
    }
    return order;
  }

  void validate_joint_order(const std::vector<std::string> & joint_order) const
  {
    const auto configured_names = default_joint_order();
    if (joint_order.size() != configured_names.size()) {
      throw std::runtime_error(
              "controller.policy_joint_order must contain every logical joint exactly once");
    }

    std::unordered_map<std::string, size_t> configured_lookup;
    configured_lookup.reserve(configured_names.size());
    for (const auto & name : configured_names) {
      configured_lookup[name] = 1U;
    }

    std::unordered_map<std::string, size_t> seen;
    seen.reserve(joint_order.size());
    for (const auto & name : joint_order) {
      if (configured_lookup.find(name) == configured_lookup.end()) {
        throw std::runtime_error("Unknown joint name in controller.policy_joint_order: " + name);
      }
      seen[name] += 1U;
      if (seen[name] > 1U) {
        throw std::runtime_error("Duplicate joint in controller.policy_joint_order: " + name);
      }
    }
  }

  void apply_parameter_overrides()
  {
    const auto startup_override = this->get_parameter("startup_mode").as_string();
    if (!startup_override.empty()) {
      const auto parsed_mode = parse_mode(startup_override);
      if (!parsed_mode.has_value()) {
        throw std::runtime_error("Invalid startup_mode parameter: " + startup_override);
      }
      startup_mode_ = *parsed_mode;
    }

    const auto model_override = this->get_parameter("policy_model_path").as_string();
    if (!model_override.empty()) {
      policy_config_.model_path = resolve_auxiliary_path(model_override);
    }

    const auto joy_topic_override = this->get_parameter("joy_topic").as_string();
    if (!joy_topic_override.empty()) {
      joy_topic_ = joy_topic_override;
    }

    const auto mode_command_topic_override = this->get_parameter("mode_command_topic").as_string();
    if (!mode_command_topic_override.empty()) {
      mode_command_topic_ = mode_command_topic_override;
    }

    const auto control_hz_override = this->get_parameter("control_hz").as_double();
    if (control_hz_override > 0.0) {
      control_hz_ = control_hz_override;
    }

    const auto policy_hz_override = this->get_parameter("policy_hz").as_double();
    if (policy_hz_override > 0.0) {
      policy_hz_ = policy_hz_override;
    }

    const auto command_timeout_override = this->get_parameter("command_timeout_ms").as_int();
    if (command_timeout_override > 0) {
      command_timeout_ms_ = command_timeout_override;
    }

    if (this->get_parameter("fixed_passive_test_only").as_bool()) {
      fixed_passive_test_only_ = true;
    }

    sanitize_startup_mode_for_test_only();

    if (control_hz_ <= 0.0) {
      throw std::runtime_error("control_hz must be > 0");
    }
    if (policy_hz_ <= 0.0) {
      throw std::runtime_error("policy_hz must be > 0");
    }
  }

  void sanitize_startup_mode_for_test_only()
  {
    if (!fixed_passive_test_only_) {
      return;
    }

    if (startup_mode_ == RobotMode::kFixed || startup_mode_ == RobotMode::kPassive) {
      return;
    }

    RCLCPP_WARN(
      this->get_logger(),
      "fixed_passive_test_only is enabled, overriding startup_mode=%s to passive",
      mode_to_string(startup_mode_).c_str());
    startup_mode_ = RobotMode::kPassive;
  }

  void create_policy_backend()
  {
    if (policy_config_.model_path.empty()) {
      policy_backend_ = std::make_unique<ZeroPolicyBackend>(
        policy_config_.expected_action_dim,
        "policy.model_path is empty, loco mode will output zero action");
      return;
    }

    if (!std::filesystem::exists(policy_config_.model_path)) {
      policy_backend_ = std::make_unique<ZeroPolicyBackend>(
        policy_config_.expected_action_dim,
        "policy model not found: " + policy_config_.model_path);
      return;
    }

#if DM_MOTOR_HAS_ONNXRUNTIME
    policy_backend_ = std::make_unique<OnnxPolicyBackend>(policy_config_);
#else
    policy_backend_ = std::make_unique<ZeroPolicyBackend>(
      policy_config_.expected_action_dim,
      "ONNX Runtime was not found at build time, loco mode will output zero action");
#endif
  }

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr message)
  {
    command_[0] = joy_mapping_.linear_x_sign * joy_mapping_.linear_x_scale *
      joy_axis_or_zero(*message, joy_mapping_.linear_x_axis);
    command_[1] = joy_mapping_.linear_y_sign * joy_mapping_.linear_y_scale *
      joy_axis_or_zero(*message, joy_mapping_.linear_y_axis);
    command_[2] = joy_mapping_.yaw_sign * joy_mapping_.yaw_scale *
      joy_axis_or_zero(*message, joy_mapping_.yaw_axis);

    handle_mode_combos(*message);
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr message)
  {
    latest_imu_ = *message;
    has_imu_ = true;
  }

  void mode_command_callback(const std_msgs::msg::String::SharedPtr message)
  {
    const auto requested_mode = parse_mode(message->data);
    if (!requested_mode.has_value()) {
      RCLCPP_WARN(
        this->get_logger(),
        "Ignoring unknown mode command: %s",
        message->data.c_str());
      return;
    }

    (void)request_mode(*requested_mode, "mode_command_topic");
  }

  void handle_mode_combos(const sensor_msgs::msg::Joy & message)
  {
    const bool confirm_pressed = joy_button_pressed(message, joy_mapping_.mode_confirm_button);
    const bool fixed_combo_active = confirm_pressed &&
      joy_button_pressed(message, joy_mapping_.fixed_modifier_button);
    const bool passive_combo_active = confirm_pressed &&
      joy_axis_activated(
        message,
        joy_mapping_.passive_modifier_axis,
        joy_mapping_.passive_modifier_axis_sign,
        joy_mapping_.trigger_activation_threshold);
    const bool loco_combo_active = confirm_pressed &&
      joy_button_pressed(message, joy_mapping_.loco_modifier_button);
    const bool dance_combo_active = confirm_pressed &&
      joy_axis_activated(
        message,
        joy_mapping_.dance_modifier_axis,
        joy_mapping_.dance_modifier_axis_sign,
        joy_mapping_.trigger_activation_threshold);

    if (fixed_combo_active && !previous_fixed_combo_active_) {
      (void)request_mode(RobotMode::kFixed, "joystick_combo");
    }
    if (passive_combo_active && !previous_passive_combo_active_) {
      (void)request_mode(RobotMode::kPassive, "joystick_combo");
    }
    if (loco_combo_active && !previous_loco_combo_active_) {
      (void)request_mode(RobotMode::kLoco, "joystick_combo");
    }
    if (dance_combo_active && !previous_dance_combo_active_) {
      (void)request_mode(RobotMode::kDance, "joystick_combo");
    }

    previous_fixed_combo_active_ = fixed_combo_active;
    previous_passive_combo_active_ = passive_combo_active;
    previous_loco_combo_active_ = loco_combo_active;
    previous_dance_combo_active_ = dance_combo_active;
  }

  bool request_mode(const RobotMode requested_mode, const std::string & source)
  {
    if (fixed_passive_test_only_ &&
      requested_mode != RobotMode::kFixed &&
      requested_mode != RobotMode::kPassive)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "Ignoring %s request for mode %s because fixed_passive_test_only is enabled",
        source.c_str(),
        mode_to_string(requested_mode).c_str());
      return false;
    }

    switch_mode(requested_mode);
    return true;
  }

  void switch_mode(const RobotMode new_mode)
  {
    if (mode_ == new_mode && published_initial_mode_) {
      return;
    }

    mode_ = new_mode;
    if (mode_ == RobotMode::kFixed) {
      start_fixed_mode_transition();
    } else {
      fixed_mode_transition_active_ = false;
      fixed_mode_transition_start_positions_.clear();
    }
    if (mode_ == RobotMode::kLoco) {
      std::fill(previous_action_.begin(), previous_action_.end(), 0.0F);
      std::fill(latest_action_.begin(), latest_action_.end(), 0.0F);
      latest_action_valid_ = false;
      last_policy_run_time_ = std::chrono::steady_clock::time_point {};
    }

    publish_mode();
    published_initial_mode_ = true;

    RCLCPP_INFO(
      this->get_logger(),
      "Switched humanoid mode to %s",
      mode_to_string(mode_).c_str());
  }

  void start_fixed_mode_transition()
  {
    fixed_mode_transition_start_positions_.clear();
    for (const auto & [name, state] : motor_state_cache_) {
      if (state.valid) {
        fixed_mode_transition_start_positions_[name] = state.position;
      }
    }

    if (fixed_mode_transition_duration_sec_ <= 1.0e-4F ||
      fixed_mode_transition_start_positions_.empty())
    {
      fixed_mode_transition_active_ = false;
      return;
    }

    fixed_mode_transition_active_ = true;
    fixed_mode_transition_start_time_ = std::chrono::steady_clock::now();
  }

  void publish_mode()
  {
    std_msgs::msg::String message;
    message.data = mode_to_string(mode_);
    mode_pub_->publish(message);
  }

  void control_loop()
  {
    (void)manager_->poll_once(std::chrono::milliseconds(rx_poll_timeout_ms_));
    const auto states = manager_->snapshot_states();
    update_motor_state_cache(states);
    maybe_capture_motor_initial_positions();

    const bool use_parallel_solver = mode_ == RobotMode::kLoco || mode_ == RobotMode::kDance;
    if (use_parallel_solver) {
      rebuild_logical_joint_state_cache(states);
      maybe_capture_initial_positions();
    } else {
      rebuild_simple_joint_state_cache(states);
    }

    publish_joint_states();
    publish_raw_motor_states(states);
    publish_mode();

    if (!use_parallel_solver) {
      if (!have_all_motor_states()) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 3000,
          "Waiting for feedback from all %zu raw motors before sending fixed/passive commands",
          manager_->motor_names().size());
        return;
      }

      auto commands = build_direct_motor_commands(mode_);
      if (commands.empty()) {
        return;
      }

      const auto results = manager_->command_group_mit(
        commands,
        std::chrono::milliseconds(command_timeout_ms_));
      log_results("control_direct", results, false);
      return;
    }

    if (!have_all_joint_states()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "Waiting for feedback from all %zu logical joints before sending loco/dance commands",
        joints_.size());
      return;
    }

    std::vector<LogicalJointCommand> logical_commands;
    switch (mode_) {
      case RobotMode::kDance:
        logical_commands = build_dance_commands();
        break;
      case RobotMode::kLoco:
        logical_commands = build_loco_commands();
        break;
      default:
        logical_commands = build_passive_commands();
        break;
    }

    if (logical_commands.empty()) {
      return;
    }

    std::vector<NamedMitCommand> commands;
    std::vector<std::string> translation_errors;
    const bool translated = parallel_adapter_.translate_joint_commands(
      logical_commands,
      commands,
      translation_errors);
    for (const auto & error : translation_errors) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "%s", error.c_str());
    }
    if (!translated || commands.empty()) {
      return;
    }

    const auto results = manager_->command_group_mit(
      commands,
      std::chrono::milliseconds(command_timeout_ms_));
    log_results("control", results, false);
  }

  void update_motor_state_cache(const std::vector<MotorState> & states)
  {
    for (const auto & state : states) {
      motor_state_cache_[state.name] = state;
    }
  }

  void rebuild_logical_joint_state_cache(const std::vector<MotorState> & states)
  {
    const auto previous_joint_states = logical_joint_state_cache_;
    std::vector<std::string> errors;
    logical_joint_state_cache_.clear();
    (void)parallel_adapter_.rebuild_joint_states(
      states,
      previous_joint_states,
      logical_joint_state_cache_,
      errors);
    for (const auto & error : errors) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "%s", error.c_str());
    }
  }

  void rebuild_simple_joint_state_cache(const std::vector<MotorState> & states)
  {
    logical_joint_state_cache_.clear();
    for (const auto & state : states) {
      if (parallel_adapter_.is_parallel_motor(state.name)) {
        continue;
      }
      logical_joint_state_cache_[state.name] = LogicalJointState {
        state.name,
        state.position,
        state.velocity,
        state.torque,
        state.valid
      };
    }
  }

  void maybe_capture_motor_initial_positions()
  {
    bool captured_any = false;
    for (const auto & [name, state] : motor_state_cache_) {
      if (!state.valid || motor_initial_position_cache_.count(name) > 0U) {
        continue;
      }
      motor_initial_position_cache_[name] = state.position;
      captured_any = true;
    }

    if (captured_any && all_motor_initial_positions_ready() && !logged_motor_initial_positions_ready_) {
      logged_motor_initial_positions_ready_ = true;
      RCLCPP_INFO(
        this->get_logger(),
        "Captured initial raw motor positions for all %zu motors",
        manager_->motor_names().size());
    }
  }

  void maybe_capture_initial_positions()
  {
    bool captured_any = false;
    for (auto & joint : joints_) {
      if (joint.initial_position_ready) {
        continue;
      }

      const auto state_it = logical_joint_state_cache_.find(joint.name);
      if (state_it == logical_joint_state_cache_.end() || !state_it->second.valid) {
        continue;
      }

      joint.initial_position = state_it->second.position;
      joint.initial_position_ready = true;
      captured_any = true;
    }

    if (captured_any && all_initial_positions_ready() && !logged_initial_positions_ready_) {
      logged_initial_positions_ready_ = true;
      RCLCPP_INFO(
        this->get_logger(),
        "Captured initial joint positions for all %zu joints",
        joints_.size());
    }
  }

  bool all_initial_positions_ready() const
  {
    return std::all_of(
      joints_.begin(), joints_.end(),
      [](const JointControlConfig & joint) {return joint.initial_position_ready;});
  }

  bool have_all_joint_states() const
  {
    return std::all_of(
      joints_.begin(), joints_.end(),
      [this](const JointControlConfig & joint) {
        const auto it = logical_joint_state_cache_.find(joint.name);
        return it != logical_joint_state_cache_.end() && it->second.valid;
      });
  }

  bool have_all_motor_states() const
  {
    const auto & motor_names = manager_->motor_names();
    return std::all_of(
      motor_names.begin(), motor_names.end(),
      [this](const std::string & name) {
        const auto it = motor_state_cache_.find(name);
        return it != motor_state_cache_.end() && it->second.valid;
      });
  }

  bool all_motor_initial_positions_ready() const
  {
    const auto & motor_names = manager_->motor_names();
    return std::all_of(
      motor_names.begin(), motor_names.end(),
      [this](const std::string & name) {
        return motor_initial_position_cache_.count(name) > 0U;
      });
  }

  const LogicalJointState * find_state(const std::string & name) const
  {
    const auto state_it = logical_joint_state_cache_.find(name);
    return state_it == logical_joint_state_cache_.end() ? nullptr : &state_it->second;
  }

  const JointControlConfig * find_joint_config(const std::string & name) const
  {
    const auto joint_it = joint_lookup_.find(name);
    if (joint_it == joint_lookup_.end()) {
      return nullptr;
    }
    return &joints_[joint_it->second];
  }

  std::vector<NamedMitCommand> build_direct_motor_commands(const RobotMode target_mode)
  {
    std::vector<NamedMitCommand> commands;
    const auto & motor_names = manager_->motor_names();
    commands.reserve(motor_names.size());

    if (target_mode == RobotMode::kFixed && !all_motor_initial_positions_ready()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "fixed mode is waiting for raw motor initial positions");
      return {};
    }

    float fixed_transition_progress = 1.0F;
    if (target_mode == RobotMode::kFixed && fixed_mode_transition_active_) {
      const auto elapsed = std::chrono::duration<float>(
        std::chrono::steady_clock::now() - fixed_mode_transition_start_time_).count();
      fixed_transition_progress = clamp_value(
        elapsed / std::max(fixed_mode_transition_duration_sec_, 1.0e-4F),
        0.0F,
        1.0F);
    }

    for (const auto & motor_name : motor_names) {
      const auto state_it = motor_state_cache_.find(motor_name);
      if (state_it == motor_state_cache_.end() || !state_it->second.valid) {
        return {};
      }

      const auto motor_config = manager_->motor_config(motor_name);
      if (!motor_config.has_value()) {
        return {};
      }

      const bool is_parallel_motor = parallel_adapter_.is_parallel_motor(motor_name);
      const auto * joint_config = find_joint_config(motor_name);
      const ModeGains * gains = target_mode == RobotMode::kFixed ?
        &fixed_mode_default_gains_ : &passive_mode_default_gains_;
      float target_position = state_it->second.position;

      if (joint_config != nullptr && !is_parallel_motor) {
        gains = target_mode == RobotMode::kFixed ?
          &joint_config->fixed_gains : &joint_config->passive_gains;
        if (target_mode == RobotMode::kFixed) {
          if (joint_config->configured_fixed_position.has_value()) {
            target_position = *joint_config->configured_fixed_position;
          } else if (joint_config->initial_position_ready) {
            target_position = joint_config->initial_position;
          }
        }
      } else if (target_mode == RobotMode::kFixed) {
        const auto initial_it = motor_initial_position_cache_.find(motor_name);
        if (initial_it == motor_initial_position_cache_.end()) {
          return {};
        }
        target_position = initial_it->second;
      }

      if (target_mode == RobotMode::kFixed && fixed_mode_transition_active_) {
        const auto start_it = fixed_mode_transition_start_positions_.find(motor_name);
        const float start_position = start_it != fixed_mode_transition_start_positions_.end() ?
          start_it->second : state_it->second.position;
        target_position = start_position +
          fixed_transition_progress * (target_position - start_position);
      }

      MitCommand command;
      command.position = clamp_value(
        target_position,
        motor_config->limits.position_min,
        motor_config->limits.position_max);
      command.velocity = 0.0F;
      command.kp = gains->kp;
      command.kd = gains->kd;
      command.torque = 0.0F;
      commands.push_back(NamedMitCommand {motor_name, command});
    }

    if (target_mode == RobotMode::kFixed && fixed_mode_transition_active_ &&
      fixed_transition_progress >= 0.999F)
    {
      fixed_mode_transition_active_ = false;
      fixed_mode_transition_start_positions_.clear();
    }

    return commands;
  }

  std::vector<LogicalJointCommand> build_fixed_commands()
  {
    if (!all_initial_positions_ready()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "fixed mode is waiting for initial joint positions");
      return {};
    }
    return build_pose_commands(RobotMode::kFixed, std::nullopt);
  }

  std::vector<LogicalJointCommand> build_passive_commands()
  {
    return build_pose_commands(RobotMode::kPassive, std::nullopt);
  }

  std::vector<LogicalJointCommand> build_dance_commands()
  {
    if (!warned_dance_placeholder_) {
      warned_dance_placeholder_ = true;
      RCLCPP_WARN(
        this->get_logger(),
        "dance mode is reserved and currently falls back to a pose-hold behavior");
    }

    if (!all_initial_positions_ready()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "dance mode is waiting for initial joint positions");
      return {};
    }
    return build_pose_commands(RobotMode::kDance, std::nullopt);
  }

  std::vector<LogicalJointCommand> build_loco_commands()
  {
    if (!has_imu_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "loco mode is waiting for imu/data");
      return build_passive_commands();
    }

    if (!all_initial_positions_ready()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "loco mode is waiting for initial joint positions");
      return build_passive_commands();
    }

    const auto now = std::chrono::steady_clock::now();
    const bool should_run_policy =
      !latest_action_valid_ ||
      std::chrono::duration<double>(now - last_policy_run_time_).count() >= (1.0 / policy_hz_);

    if (should_run_policy) {
      std::vector<float> observation;
      observation.reserve(policy_config_.expected_observation_dim);
      if (!build_policy_observation(observation)) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 3000,
          "Unable to build loco observation, falling back to passive behavior");
        return build_passive_commands();
      }

      std::vector<float> action;
      std::string run_message;
      if (!policy_backend_->run(observation, action, run_message)) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 3000,
          "Policy inference failed: %s",
          run_message.c_str());
        return build_fixed_commands();
      }

      if (policy_backend_->is_fallback()) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "loco mode is using fallback policy backend: %s",
          run_message.c_str());
      }

      if (action.size() != joints_.size()) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 3000,
          "Policy output dimension mismatch, expected %zu but got %zu",
          joints_.size(),
          action.size());
        return build_fixed_commands();
      }

      latest_action_ = action;
      latest_action_valid_ = true;
      last_policy_run_time_ = now;
      publish_policy_debug(observation, latest_action_);
    }

    std::vector<LogicalJointCommand> commands;
    commands.reserve(joints_.size());
    for (size_t index = 0; index < joints_.size(); ++index) {
      const auto & joint = joints_[index];
      const auto * state = find_state(joint.name);
      if (state == nullptr) {
        return {};
      }

      const float clipped_action = clamp_value(
        latest_action_[index],
        -policy_config_.action_clip,
        policy_config_.action_clip);
      float position_min = -std::numeric_limits<float>::infinity();
      float position_max = std::numeric_limits<float>::infinity();
      if (const auto logical_limits = parallel_adapter_.joint_limits(joint.name); logical_limits.has_value()) {
        position_min = logical_limits->first;
        position_max = logical_limits->second;
      } else {
        auto motor_config = manager_->motor_config(joint.name);
        if (!motor_config.has_value()) {
          return {};
        }
        position_min = motor_config->limits.position_min;
        position_max = motor_config->limits.position_max;
      }

      MitCommand command;
      command.position = clamp_value(
        joint.initial_position + joint.action_offset + joint.action_scale * clipped_action,
        position_min,
        position_max);
      command.velocity = 0.0F;
      command.kp = joint.loco_gains.kp;
      command.kd = joint.loco_gains.kd;
      command.torque = 0.0F;

      commands.push_back(LogicalJointCommand {joint.name, command});
    }

    previous_action_ = latest_action_;
    return commands;
  }

  std::vector<LogicalJointCommand> build_pose_commands(
    const RobotMode target_mode,
    const std::optional<std::vector<float>> & explicit_positions)
  {
    std::vector<LogicalJointCommand> commands;
    commands.reserve(joints_.size());

    for (size_t index = 0; index < joints_.size(); ++index) {
      const auto & joint = joints_[index];
      const auto * state = find_state(joint.name);
      if (state == nullptr) {
        return {};
      }

      const ModeGains * gains = &joint.fixed_gains;
      float target_position = joint.initial_position;
      switch (target_mode) {
        case RobotMode::kFixed:
          gains = &joint.fixed_gains;
          target_position = joint.configured_fixed_position.value_or(joint.initial_position);
          break;
        case RobotMode::kPassive:
          gains = &joint.passive_gains;
          target_position = state->position;
          break;
        case RobotMode::kDance:
          gains = &joint.dance_gains;
          target_position = joint.initial_position;
          break;
        default:
          break;
      }

      if (explicit_positions.has_value() && index < explicit_positions->size()) {
        target_position = explicit_positions->at(index);
      }

      float position_min = -std::numeric_limits<float>::infinity();
      float position_max = std::numeric_limits<float>::infinity();
      if (const auto logical_limits = parallel_adapter_.joint_limits(joint.name); logical_limits.has_value()) {
        position_min = logical_limits->first;
        position_max = logical_limits->second;
      } else {
        auto motor_config = manager_->motor_config(joint.name);
        if (!motor_config.has_value()) {
          return {};
        }
        position_min = motor_config->limits.position_min;
        position_max = motor_config->limits.position_max;
      }

      MitCommand command;
      command.position = clamp_value(
        target_position,
        position_min,
        position_max);
      command.velocity = 0.0F;
      command.kp = gains->kp;
      command.kd = gains->kd;
      command.torque = 0.0F;

      commands.push_back(LogicalJointCommand {joint.name, command});
    }

    return commands;
  }

  bool build_policy_observation(std::vector<float> & observation) const
  {
    if (!has_imu_ || !have_all_joint_states() || !all_initial_positions_ready()) {
      return false;
    }

    observation.clear();

    observation.push_back(
      static_cast<float>(latest_imu_.angular_velocity.x) *
      policy_config_.observation_scales.angular_velocity);
    observation.push_back(
      static_cast<float>(latest_imu_.angular_velocity.y) *
      policy_config_.observation_scales.angular_velocity);
    observation.push_back(
      static_cast<float>(latest_imu_.angular_velocity.z) *
      policy_config_.observation_scales.angular_velocity);

    const auto gravity = compute_projected_gravity(latest_imu_);
    observation.push_back(gravity[0] * policy_config_.observation_scales.gravity);
    observation.push_back(gravity[1] * policy_config_.observation_scales.gravity);
    observation.push_back(gravity[2] * policy_config_.observation_scales.gravity);

    observation.push_back(command_[0] * policy_config_.observation_scales.command);
    observation.push_back(command_[1] * policy_config_.observation_scales.command);
    observation.push_back(command_[2] * policy_config_.observation_scales.command);

    for (const auto & joint : joints_) {
      const auto * state = find_state(joint.name);
      if (state == nullptr) {
        return false;
      }
      observation.push_back(
        (state->position - joint.initial_position) *
        policy_config_.observation_scales.joint_position);
    }

    for (const auto & joint : joints_) {
      const auto * state = find_state(joint.name);
      if (state == nullptr) {
        return false;
      }
      observation.push_back(
        state->velocity * policy_config_.observation_scales.joint_velocity);
    }

    for (const auto action : previous_action_) {
      observation.push_back(action * policy_config_.observation_scales.previous_action);
    }

    return observation.size() == policy_config_.expected_observation_dim;
  }

  void publish_policy_debug(
    const std::vector<float> & observation,
    const std::vector<float> & action) const
  {
    if (!publish_policy_debug_topics_ || !observation_pub_ || !action_pub_) {
      return;
    }

    std_msgs::msg::Float32MultiArray observation_message;
    observation_message.data = observation;
    observation_pub_->publish(observation_message);

    std_msgs::msg::Float32MultiArray action_message;
    action_message.data = action;
    action_pub_->publish(action_message);
  }

  void publish_joint_states()
  {
    sensor_msgs::msg::JointState message;
    message.header.stamp = this->now();
    message.name.reserve(joints_.size());
    message.position.reserve(joints_.size());
    message.velocity.reserve(joints_.size());
    message.effort.reserve(joints_.size());

    for (const auto & joint : joints_) {
      const auto state_it = logical_joint_state_cache_.find(joint.name);
      if (state_it == logical_joint_state_cache_.end() || !state_it->second.valid) {
        continue;
      }

      message.name.push_back(state_it->second.name);
      message.position.push_back(state_it->second.position);
      message.velocity.push_back(state_it->second.velocity);
      message.effort.push_back(state_it->second.effort);
    }

    joint_state_pub_->publish(message);
  }

  void publish_raw_motor_states(const std::vector<MotorState> & states)
  {
    if (!raw_joint_state_pub_) {
      return;
    }

    sensor_msgs::msg::JointState message;
    message.header.stamp = this->now();
    message.name.reserve(states.size());
    message.position.reserve(states.size());
    message.velocity.reserve(states.size());
    message.effort.reserve(states.size());

    for (const auto & state : states) {
      message.name.push_back(state.name);
      message.position.push_back(state.position);
      message.velocity.push_back(state.velocity);
      message.effort.push_back(state.torque);
    }

    raw_joint_state_pub_->publish(message);
  }

  void log_results(
    const std::string & title,
    const std::vector<MotorCommandResult> & results,
    const bool log_successes)
  {
    size_t success_count = 0U;
    for (const auto & result : results) {
      if (result.success) {
        ++success_count;
        if (log_successes) {
          RCLCPP_INFO(
            this->get_logger(),
            "[%s] %s: %s",
            title.c_str(),
            result.name.c_str(),
            result.message.c_str());
        }
      } else {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 3000,
          "[%s] %s: %s",
          title.c_str(),
          result.name.c_str(),
          result.message.c_str());
      }
    }

    if (log_successes || success_count != results.size()) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "[%s] summary: %zu/%zu success",
        title.c_str(),
        success_count,
        results.size());
    }
  }

  std::unique_ptr<DmMotorManager> manager_;
  std::vector<JointControlConfig> joints_;
  std::unordered_map<std::string, size_t> joint_lookup_;
  std::unordered_map<std::string, MotorState> motor_state_cache_;
  std::unordered_map<std::string, float> motor_initial_position_cache_;
  std::unordered_map<std::string, GainProfile> gain_profiles_;
  std::unordered_map<std::string, LogicalJointState> logical_joint_state_cache_;
  std::vector<ParallelMechanismConfig> parallel_mechanism_configs_;
  ParallelJointAdapter parallel_adapter_ {};
  sensor_msgs::msg::Imu latest_imu_;
  bool has_imu_ {false};
  bool logged_initial_positions_ready_ {false};
  bool logged_motor_initial_positions_ready_ {false};
  bool warned_dance_placeholder_ {false};
  bool published_initial_mode_ {false};
  bool fixed_mode_transition_active_ {false};

  std::string controller_config_path_;
  std::string motor_config_path_;
  std::string joy_topic_ {"joy"};
  std::string imu_topic_ {"imu/data"};
  std::string mode_topic_ {"humanoid_control/mode"};
  std::string mode_command_topic_ {"humanoid_control/mode_command"};
  double control_hz_ {100.0};
  double policy_hz_ {50.0};
  int command_timeout_ms_ {5};
  int rx_poll_timeout_ms_ {0};
  bool auto_enable_on_start_ {true};
  bool auto_disable_on_shutdown_ {true};
  bool publish_policy_debug_topics_ {true};
  bool fixed_passive_test_only_ {false};
  RobotMode startup_mode_ {RobotMode::kPassive};
  RobotMode mode_ {RobotMode::kPassive};
  float fixed_mode_transition_duration_sec_ {1.0F};
  ModeGains fixed_mode_default_gains_ {};
  ModeGains passive_mode_default_gains_ {};
  JoyMapping joy_mapping_ {};
  PolicyConfig policy_config_ {};
  std::array<float, 3> command_ {0.0F, 0.0F, 0.0F};
  std::vector<float> previous_action_;
  std::vector<float> latest_action_;
  bool latest_action_valid_ {false};
  std::unordered_map<std::string, float> fixed_mode_transition_start_positions_;
  std::chrono::steady_clock::time_point fixed_mode_transition_start_time_ {};
  std::chrono::steady_clock::time_point last_policy_run_time_ {};
  bool previous_fixed_combo_active_ {false};
  bool previous_passive_combo_active_ {false};
  bool previous_loco_combo_active_ {false};
  bool previous_dance_combo_active_ {false};
  std::unique_ptr<PolicyBackend> policy_backend_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_command_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr raw_joint_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr observation_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr action_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};
}  // namespace dm_motor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dm_motor::DmHumanoidControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
