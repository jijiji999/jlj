#ifndef DM_TEST__DM_TRAJECTORY_TEST_NODE_HPP_
#define DM_TEST__DM_TRAJECTORY_TEST_NODE_HPP_

#include <chrono>
#include <optional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "dm_motor/dm_motor_manager.hpp"

namespace dm_test
{

class DmTrajectoryTestNode : public rclcpp::Node
{
public:
  DmTrajectoryTestNode();
  ~DmTrajectoryTestNode() override;

private:
  enum class ExecutionPhase
  {
    kWaitingForState = 0,
    kPrePositioning = 1,
    kTracking = 2,
  };

  struct JointGain
  {
    double kp {25.0};
    double kd {1.0};
  };

  struct GroupConfig
  {
    std::string name;
    std::vector<std::string> joints;
    double default_kp {25.0};
    double default_kd {1.0};
    std::unordered_map<std::string, JointGain> joint_gains;
    std::unordered_map<std::string, std::string> csv_column_names;
  };

  struct RawTrajectoryPoint
  {
    double time_from_start {0.0};
    std::unordered_map<std::string, double> offsets;
  };

  struct TrajectoryPoint
  {
    double time_from_start {0.0};
    std::vector<double> offsets;
  };

  struct TrajectoryConfig
  {
    std::string name;
    std::string group_name;
    std::string description;
    bool loop {false};
    bool use_absolute_positions {false};
    std::vector<TrajectoryPoint> points;
    double duration {0.0};
  };

  struct SampledTrajectory
  {
    std::vector<double> offsets;
    std::vector<double> velocities;
  };

  std::string resolve_test_config_path(const std::string & configured_path) const;
  std::string resolve_path_relative_to_test_config(const std::string & path) const;
  void load_test_config();
  void load_csv_trajectory(const std::string & csv_path);
  void validate_active_group_against_motor_config();
  void start_test_session();
  void stop_test_session();

  void timer_callback();
  bool capture_initial_positions();
  SampledTrajectory build_pre_position_sample(double progress_ratio) const;
  SampledTrajectory sample_trajectory(double time_from_start) const;
  std::vector<dm_motor::NamedMitCommand> build_commands(
    const SampledTrajectory & sample,
    std::vector<double> & target_positions,
    std::vector<double> & target_velocities);
  void update_latest_states();
  void update_latest_states_from_results(const std::vector<dm_motor::MotorCommandResult> & results);
  void publish_tracking_messages(
    const std::vector<double> & target_positions,
    const std::vector<double> & target_velocities);
  void log_command_results(const std::vector<dm_motor::MotorCommandResult> & results);
  JointGain gain_for_joint(const std::string & joint_name) const;
  std::vector<std::string> split_csv_line(const std::string & line) const;
  std::optional<size_t> find_time_column_index(const std::vector<std::string> & headers) const;
  std::vector<size_t> resolve_csv_joint_column_indices(
    const std::vector<std::string> & headers) const;
  static std::string trim_copy(const std::string & value);
  static std::string to_lower_copy(const std::string & value);

  std::string test_config_path_;
  std::string motor_config_path_;
  std::string trajectory_name_;
  std::string trajectory_csv_path_;
  std::string time_column_name_;

  GroupConfig active_group_;
  TrajectoryConfig active_trajectory_;
  std::unordered_map<std::string, dm_motor::MotorConfig> motor_configs_by_name_;
  std::unordered_map<std::string, dm_motor::MotorState> latest_states_by_name_;
  std::unordered_map<std::string, double> initial_positions_by_name_;

  std::unique_ptr<dm_motor::DmMotorManager> manager_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr reference_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr actual_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr error_pub_;

  double command_rate_hz_ {100.0};
  double playback_speed_ {1.0};
  double csv_row_rate_hz_ {0.0};
  double pre_position_duration_sec_ {5.0};
  bool enable_pre_positioning_ {true};
  int command_timeout_ms_ {20};
  double start_delay_sec_ {1.0};
  bool auto_enable_on_start_ {true};
  bool auto_disable_on_shutdown_ {false};

  ExecutionPhase phase_ {ExecutionPhase::kWaitingForState};
  bool initial_positions_ready_ {false};
  bool completion_logged_ {false};
  std::chrono::steady_clock::time_point warmup_start_time_ {};
  std::chrono::steady_clock::time_point pre_position_start_time_ {};
  std::chrono::steady_clock::time_point trajectory_start_time_ {};
};

}  // namespace dm_test

#endif  // DM_TEST__DM_TRAJECTORY_TEST_NODE_HPP_
