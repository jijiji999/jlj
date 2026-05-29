#ifndef DM_MOTOR__DM_MOTOR_NODE_HPP_
#define DM_MOTOR__DM_MOTOR_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "dm_motor/dm_motor_manager.hpp"

namespace dm_motor
{

class DmMotorNode : public rclcpp::Node
{
public:
  DmMotorNode();

private:
  std::string resolve_config_path(const std::string & configured_path) const;
  void publish_states();

  std::unique_ptr<DmMotorManager> manager_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  double state_publish_hz_ {100.0};
  int rx_poll_timeout_ms_ {0};
};

}  // namespace dm_motor

#endif  // DM_MOTOR__DM_MOTOR_NODE_HPP_
