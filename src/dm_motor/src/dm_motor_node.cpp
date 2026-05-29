#include "dm_motor/dm_motor_node.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include <chrono>
#include <stdexcept>

namespace dm_motor
{

using namespace std::chrono_literals;

DmMotorNode::DmMotorNode()
: Node("dm_motor_node")
{
  this->declare_parameter<std::string>("config_path", "");
  this->declare_parameter<double>("state_publish_hz", 100.0);
  this->declare_parameter<int>("rx_poll_timeout_ms", 0);

  state_publish_hz_ = this->get_parameter("state_publish_hz").as_double();
  rx_poll_timeout_ms_ = this->get_parameter("rx_poll_timeout_ms").as_int();

  const auto configured_path = this->get_parameter("config_path").as_string();
  const auto config_path = resolve_config_path(configured_path);

  manager_ = std::make_unique<DmMotorManager>(config_path);
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 20);

  const auto period = std::chrono::duration<double>(1.0 / state_publish_hz_);
  publish_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    std::bind(&DmMotorNode::publish_states, this));

  RCLCPP_INFO(
    this->get_logger(),
    "dm_motor node loaded %zu motors from %s",
    manager_->motor_names().size(),
    config_path.c_str());
}

std::string DmMotorNode::resolve_config_path(const std::string & configured_path) const
{
  if (!configured_path.empty()) {
    return configured_path;
  }

  const auto share_dir = ament_index_cpp::get_package_share_directory("dm_motor");
  return share_dir + "/config/dm_motor_29.yaml";
}

void DmMotorNode::publish_states()
{
  (void)manager_->poll_once(std::chrono::milliseconds(rx_poll_timeout_ms_));
  const auto states = manager_->snapshot_states();

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

  joint_state_pub_->publish(message);
}

}  // namespace dm_motor
