#ifndef DM_MOTOR__DM_MOTOR_TEST_NODE_HPP_
#define DM_MOTOR__DM_MOTOR_TEST_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "dm_motor/dm_motor_manager.hpp"

namespace dm_motor
{

class DmMotorTestNode : public rclcpp::Node
{
public:
  DmMotorTestNode();
  ~DmMotorTestNode() override;

private:
  std::string resolve_config_path(const std::string & configured_path) const;
  std::vector<std::string> resolve_target_motors(const std::string & target_motor) const;
  MitCommand build_mit_command_from_parameters() const;
  void start_test();
  void command_timer_callback();
  void stop_all_motors();
  void log_results(
    const std::string & title,
    const std::vector<MotorCommandResult> & results,
    bool log_successes);

  std::unique_ptr<DmMotorManager> manager_;
  rclcpp::TimerBase::SharedPtr command_timer_;

  std::string target_motor_;
  std::vector<std::string> target_motors_;
  double command_rate_hz_ {50.0};
  int command_timeout_ms_ {20};
  bool auto_enable_on_start_ {true};
  bool auto_disable_on_shutdown_ {true};
  bool sent_first_command_ {false};
};

}  // namespace dm_motor

#endif  // DM_MOTOR__DM_MOTOR_TEST_NODE_HPP_
