#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"

#include <chrono>
#include <optional>
#include <string>
#include <vector>

namespace
{

using namespace std::chrono_literals;

struct TopicStatus
{
  std::string name;
  bool required {true};
  bool received {false};
  rclcpp::Time last_stamp {0, 0, RCL_ROS_TIME};
};

template<typename MessageT>
void mark_received(TopicStatus & status, const typename MessageT::SharedPtr & message)
{
  status.received = true;
  status.last_stamp = message->header.stamp;
}

void mark_received(TopicStatus & status, const std_msgs::msg::String::SharedPtr &)
{
  status.received = true;
}

class DmHumanoidSystemCheckNode : public rclcpp::Node
{
public:
  DmHumanoidSystemCheckNode()
  : Node("dm_humanoid_system_check_node")
  {
    this->declare_parameter<std::string>("joy_topic", "joy");
    this->declare_parameter<std::string>("imu_topic", "imu/data");
    this->declare_parameter<std::string>("raw_motor_topic", "raw_motor_states");
    this->declare_parameter<std::string>("joint_states_topic", "joint_states");
    this->declare_parameter<std::string>("mode_topic", "humanoid_control/mode");
    this->declare_parameter<double>("startup_grace_period_sec", 5.0);
    this->declare_parameter<double>("check_period_sec", 1.0);
    this->declare_parameter<bool>("require_joy", true);
    this->declare_parameter<bool>("require_imu", true);
    this->declare_parameter<bool>("require_raw_motor_states", true);
    this->declare_parameter<bool>("require_joint_states", true);
    this->declare_parameter<bool>("require_mode_topic", true);

    joy_status_.name = this->get_parameter("joy_topic").as_string();
    imu_status_.name = this->get_parameter("imu_topic").as_string();
    raw_motor_status_.name = this->get_parameter("raw_motor_topic").as_string();
    joint_state_status_.name = this->get_parameter("joint_states_topic").as_string();
    mode_status_.name = this->get_parameter("mode_topic").as_string();

    joy_status_.required = this->get_parameter("require_joy").as_bool();
    imu_status_.required = this->get_parameter("require_imu").as_bool();
    raw_motor_status_.required = this->get_parameter("require_raw_motor_states").as_bool();
    joint_state_status_.required = this->get_parameter("require_joint_states").as_bool();
    mode_status_.required = this->get_parameter("require_mode_topic").as_bool();

    startup_grace_period_sec_ = this->get_parameter("startup_grace_period_sec").as_double();
    check_period_sec_ = this->get_parameter("check_period_sec").as_double();

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      joy_status_.name, 20,
      [this](const sensor_msgs::msg::Joy::SharedPtr msg) {mark_received<sensor_msgs::msg::Joy>(joy_status_, msg);});
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_status_.name, 20,
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) {mark_received<sensor_msgs::msg::Imu>(imu_status_, msg);});
    raw_motor_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      raw_motor_status_.name, 20,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {mark_received<sensor_msgs::msg::JointState>(raw_motor_status_, msg);});
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      joint_state_status_.name, 20,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {mark_received<sensor_msgs::msg::JointState>(joint_state_status_, msg);});
    mode_sub_ = this->create_subscription<std_msgs::msg::String>(
      mode_status_.name, 20,
      [this](const std_msgs::msg::String::SharedPtr msg) {mark_received(mode_status_, msg);});

    start_time_ = this->now();
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(check_period_sec_)),
      std::bind(&DmHumanoidSystemCheckNode::check_once, this));

    RCLCPP_INFO(
      this->get_logger(),
      "dm_humanoid_system_check_node started. joy=%s imu=%s raw_motor=%s joint_states=%s mode=%s",
      joy_status_.name.c_str(),
      imu_status_.name.c_str(),
      raw_motor_status_.name.c_str(),
      joint_state_status_.name.c_str(),
      mode_status_.name.c_str());
  }

private:
  void check_once()
  {
    if ((this->now() - start_time_).seconds() < startup_grace_period_sec_) {
      return;
    }

    const auto failures = collect_failures();
    if (failures.empty()) {
      if (!reported_ready_) {
        reported_ready_ = true;
        RCLCPP_INFO(this->get_logger(), "System check passed: all required modules are online");
      }
      return;
    }

    reported_ready_ = false;
    std::string joined;
    for (size_t i = 0; i < failures.size(); ++i) {
      if (i > 0) {
        joined += "; ";
      }
      joined += failures[i];
    }
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 3000,
      "System check failed: %s",
      joined.c_str());
  }

  std::vector<std::string> collect_failures() const
  {
    std::vector<std::string> failures;
    append_failure(joy_status_, "joystick topic is offline", failures);
    append_failure(imu_status_, "imu topic is offline", failures);
    append_failure(raw_motor_status_, "raw motor state topic is offline", failures);
    append_failure(joint_state_status_, "logical joint state topic is offline", failures);
    append_failure(mode_status_, "mode topic is offline", failures);
    return failures;
  }

  static void append_failure(
    const TopicStatus & status,
    const std::string & message,
    std::vector<std::string> & failures)
  {
    if (status.required && !status.received) {
      failures.push_back(message + " (" + status.name + ")");
    }
  }

  TopicStatus joy_status_ {};
  TopicStatus imu_status_ {};
  TopicStatus raw_motor_status_ {};
  TopicStatus joint_state_status_ {};
  TopicStatus mode_status_ {};
  double startup_grace_period_sec_ {5.0};
  double check_period_sec_ {1.0};
  bool reported_ready_ {false};
  rclcpp::Time start_time_ {0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr raw_motor_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DmHumanoidSystemCheckNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
