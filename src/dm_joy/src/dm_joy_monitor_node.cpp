#include <cmath>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace
{
std::vector<std::string> defaultButtonNames()
{
  return {
    "A",
    "B",
    "X",
    "Y",
    "L1",
    "R1",
    "select",
    "start",
    "ps",
    "L3",
    "R3"
  };
}

std::vector<std::string> defaultAxisNames()
{
  return {
    "left_stick_x",
    "left_stick_y_up_positive",
    "L2_trigger",
    "right_stick_x",
    "right_stick_y_up_positive",
    "R2_trigger",
    "dpad_x",
    "dpad_y_up_positive"
  };
}
}  // namespace

class DmJoyMonitorNode : public rclcpp::Node
{
public:
  DmJoyMonitorNode()
  : Node("dm_joy_monitor_node")
  {
    this->declare_parameter<std::string>("joy_topic", "joy");
    this->declare_parameter<double>("axis_report_threshold", 0.5);

    joy_topic_ = this->get_parameter("joy_topic").as_string();
    axis_report_threshold_ = this->get_parameter("axis_report_threshold").as_double();

    sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_, 20,
      std::bind(&DmJoyMonitorNode::joyCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "dm_joy_monitor_node listening on %s", joy_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Monitor uses the labels printed on your controller.");
    RCLCPP_INFO(this->get_logger(), "L2/R2 are reported as analog axes, not digital buttons.");
    printMapping();
  }

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (!initialized_) {
      last_buttons_ = msg->buttons;
      last_axes_ = msg->axes;
      initialized_ = true;
      reportActiveAxes(*msg, true);
      return;
    }

    for (std::size_t i = 0; i < msg->buttons.size(); ++i) {
      const int previous = i < last_buttons_.size() ? last_buttons_[i] : 0;
      if (previous == msg->buttons[i]) {
        continue;
      }

      const std::string name = lookupButtonName(i);
      if (msg->buttons[i]) {
        RCLCPP_INFO(this->get_logger(), "Pressed  button[%zu] = %s", i, name.c_str());
      } else {
        RCLCPP_INFO(this->get_logger(), "Released button[%zu] = %s", i, name.c_str());
      }
    }

    reportActiveAxes(*msg, false);

    last_buttons_ = msg->buttons;
    last_axes_ = msg->axes;
  }

  void reportActiveAxes(const sensor_msgs::msg::Joy & msg, bool first_report)
  {
    std::vector<std::string> active_axes;

    for (std::size_t i = 0; i < msg.axes.size(); ++i) {
      const float current = msg.axes[i];
      const float previous = i < last_axes_.size() ? last_axes_[i] : 0.0F;
      const bool changed_enough = std::fabs(current - previous) >= 0.2F;
      const bool active_enough = std::fabs(current) >= axis_report_threshold_;

      if ((first_report || changed_enough) && active_enough) {
        char buffer[128];
        std::snprintf(
          buffer, sizeof(buffer),
          "axis[%zu]=%s value=%.2f", i, lookupAxisName(i).c_str(), current);
        active_axes.emplace_back(buffer);
      }
    }

    for (const auto & axis_state : active_axes) {
      RCLCPP_INFO(this->get_logger(), "%s", axis_state.c_str());
    }
  }

  void printMapping() const
  {
    const auto buttons = defaultButtonNames();
    const auto axes = defaultAxisNames();

    for (std::size_t i = 0; i < buttons.size(); ++i) {
      RCLCPP_INFO(this->get_logger(), "button[%zu] -> %s", i, buttons[i].c_str());
    }
    for (std::size_t i = 0; i < axes.size(); ++i) {
      RCLCPP_INFO(this->get_logger(), "axis[%zu] -> %s", i, axes[i].c_str());
    }
  }

  std::string lookupButtonName(std::size_t index) const
  {
    const auto names = defaultButtonNames();
    if (index < names.size()) {
      return names[index];
    }
    return "unknown_button";
  }

  std::string lookupAxisName(std::size_t index) const
  {
    const auto names = defaultAxisNames();
    if (index < names.size()) {
      return names[index];
    }
    return "unknown_axis";
  }

  std::string joy_topic_;
  double axis_report_threshold_{0.5};
  bool initialized_{false};
  std::vector<int32_t> last_buttons_;
  std::vector<float> last_axes_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DmJoyMonitorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
