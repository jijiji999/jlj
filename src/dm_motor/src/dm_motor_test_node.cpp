#include "dm_motor/dm_motor_test_node.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include <algorithm>
#include <chrono>
#include <stdexcept>

namespace dm_motor
{

namespace
{

constexpr const char * kAllMotorsKeyword = "all";
constexpr double kAutoModelPdValue = -1.0;

}  // namespace

DmMotorTestNode::DmMotorTestNode()
: Node("dm_motor_test_node")
{
  this->declare_parameter<std::string>("config_path", "");
  this->declare_parameter<std::string>("target_motor", kAllMotorsKeyword);
  this->declare_parameter<bool>("auto_enable_on_start", true);
  this->declare_parameter<bool>("auto_disable_on_shutdown", true);
  this->declare_parameter<double>("command_rate_hz", 50.0);
  this->declare_parameter<int>("command_timeout_ms", 20);
  this->declare_parameter<double>("mit_position", 0.0);
  this->declare_parameter<double>("mit_velocity", 0.0);
  this->declare_parameter<double>("mit_kp", kAutoModelPdValue);
  this->declare_parameter<double>("mit_kd", kAutoModelPdValue);
  this->declare_parameter<double>("mit_torque", 0.0);

  const auto config_path =
    resolve_config_path(this->get_parameter("config_path").as_string());
  manager_ = std::make_unique<DmMotorManager>(config_path);

  target_motor_ = this->get_parameter("target_motor").as_string();
  target_motors_ = resolve_target_motors(target_motor_);
  auto_enable_on_start_ = this->get_parameter("auto_enable_on_start").as_bool();
  auto_disable_on_shutdown_ = this->get_parameter("auto_disable_on_shutdown").as_bool();
  command_rate_hz_ = this->get_parameter("command_rate_hz").as_double();
  command_timeout_ms_ = this->get_parameter("command_timeout_ms").as_int();

  if (command_rate_hz_ <= 0.0) {
    throw std::runtime_error("command_rate_hz must be > 0");
  }

  start_test();

  const auto period = std::chrono::duration<double>(1.0 / command_rate_hz_);
  command_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    std::bind(&DmMotorTestNode::command_timer_callback, this));

  RCLCPP_INFO(
    this->get_logger(),
    "dm_motor_test_node ready, target=%s, motors=%zu, rate=%.2f Hz",
    target_motor_.c_str(),
    target_motors_.size(),
    command_rate_hz_);
}

DmMotorTestNode::~DmMotorTestNode()
{
  if (auto_disable_on_shutdown_ && manager_) {
    stop_all_motors();
  }
}

std::string DmMotorTestNode::resolve_config_path(const std::string & configured_path) const
{
  if (!configured_path.empty()) {
    return configured_path;
  }

  const auto share_dir = ament_index_cpp::get_package_share_directory("dm_motor");
  return share_dir + "/config/dm_motor_29.yaml";
}

std::vector<std::string> DmMotorTestNode::resolve_target_motors(const std::string & target_motor) const
{
  if (target_motor == kAllMotorsKeyword) {
    return manager_->motor_names();
  }

  const auto & motor_names = manager_->motor_names();
  const auto it = std::find(motor_names.begin(), motor_names.end(), target_motor);
  if (it == motor_names.end()) {
    throw std::runtime_error("target_motor not found in config: " + target_motor);
  }

  return {target_motor};
}

MitCommand DmMotorTestNode::build_mit_command_from_parameters() const
{
  MitCommand command;
  command.position = static_cast<float>(this->get_parameter("mit_position").as_double());
  command.velocity = static_cast<float>(this->get_parameter("mit_velocity").as_double());
  command.kp = static_cast<float>(this->get_parameter("mit_kp").as_double());
  command.kd = static_cast<float>(this->get_parameter("mit_kd").as_double());
  command.torque = static_cast<float>(this->get_parameter("mit_torque").as_double());

  if (target_motor_ != kAllMotorsKeyword) {
    const auto config = manager_->motor_config(target_motor_);
    if (config.has_value()) {
      if (command.kp < 0.0F) {
        command.kp = config->default_kp;
      }
      if (command.kd < 0.0F) {
        command.kd = config->default_kd;
      }
    }
  }
  return command;
}

void DmMotorTestNode::start_test()
{
  if (!auto_enable_on_start_) {
    RCLCPP_WARN(
      this->get_logger(),
      "auto_enable_on_start is false, node will only send MIT commands without an enable step");
    return;
  }

  std::vector<MotorCommandResult> results;
  if (target_motor_ == kAllMotorsKeyword) {
    results = manager_->enable_all(std::chrono::milliseconds(command_timeout_ms_));
  } else {
    results.emplace_back(
      manager_->enable_motor(target_motor_, std::chrono::milliseconds(command_timeout_ms_)));
  }

  log_results("enable", results, true);
}

void DmMotorTestNode::command_timer_callback()
{
  const auto command = build_mit_command_from_parameters();

  std::vector<MotorCommandResult> results;
  if (target_motor_ == kAllMotorsKeyword) {
    std::vector<NamedMitCommand> commands;
    commands.reserve(target_motors_.size());
    for (const auto & name : target_motors_) {
      auto per_motor_command = command;
      const auto config = manager_->motor_config(name);
      if (config.has_value()) {
        if (per_motor_command.kp < 0.0F) {
          per_motor_command.kp = config->default_kp;
        }
        if (per_motor_command.kd < 0.0F) {
          per_motor_command.kd = config->default_kd;
        }
      }
      commands.push_back(NamedMitCommand {name, per_motor_command});
    }
    results = manager_->command_group_mit(
      commands,
      std::chrono::milliseconds(command_timeout_ms_));
  } else {
    results.emplace_back(
      manager_->command_motor_mit(
        target_motor_,
        command,
        std::chrono::milliseconds(command_timeout_ms_)));
  }

  if (!sent_first_command_) {
    log_results("first_mit_command", results, true);
    sent_first_command_ = true;
  } else {
    log_results("mit_command", results, false);
  }

  (void)manager_->poll_once(std::chrono::milliseconds(0));
}

void DmMotorTestNode::stop_all_motors()
{
  std::vector<MotorCommandResult> results;
  if (target_motor_ == kAllMotorsKeyword) {
    results = manager_->disable_all(std::chrono::milliseconds(command_timeout_ms_));
  } else {
    results.emplace_back(
      manager_->disable_motor(target_motor_, std::chrono::milliseconds(command_timeout_ms_)));
  }

  log_results("disable", results, true);
}

void DmMotorTestNode::log_results(
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
      RCLCPP_WARN(
        this->get_logger(),
        "[%s] %s: %s",
        title.c_str(),
        result.name.c_str(),
        result.message.c_str());
    }
  }

  if (log_successes || success_count != results.size()) {
    RCLCPP_INFO(
      this->get_logger(),
      "[%s] summary: %zu/%zu success",
      title.c_str(),
      success_count,
      results.size());
  }
}

}  // namespace dm_motor
