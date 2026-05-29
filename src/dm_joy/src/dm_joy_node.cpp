#include <cerrno>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>

#include <fcntl.h>
#include <linux/joystick.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace
{
double clampToUnitRange(double value)
{
  if (value > 1.0) {
    return 1.0;
  }
  if (value < -1.0) {
    return -1.0;
  }
  return value;
}

float normalizeAxisValue(int16_t raw_value, double deadzone)
{
  const double raw = clampToUnitRange(static_cast<double>(raw_value) / 32767.0);
  const double magnitude = std::abs(raw);
  if (magnitude <= deadzone) {
    return 0.0F;
  }

  const double scaled = (magnitude - deadzone) / (1.0 - deadzone);
  return static_cast<float>(std::copysign(scaled, raw));
}
}  // namespace

class DmJoyNode : public rclcpp::Node
{
public:
  DmJoyNode()
  : Node("dm_joy_node")
  {
    this->declare_parameter<std::string>(
      "device_path", "/dev/input/by-id/usb-S_TGZ_Controller_3E529620-joystick");
    this->declare_parameter<std::string>("joy_topic", "joy");
    this->declare_parameter<std::string>("frame_id", "joy_link");
    this->declare_parameter<double>("deadzone", 0.05);
    this->declare_parameter<double>("autorepeat_rate", 20.0);
    this->declare_parameter<int>("poll_interval_ms", 10);
    this->declare_parameter<int>("reconnect_interval_ms", 1000);

    device_path_ = this->get_parameter("device_path").as_string();
    joy_topic_ = this->get_parameter("joy_topic").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();
    deadzone_ = this->get_parameter("deadzone").as_double();
    autorepeat_rate_ = this->get_parameter("autorepeat_rate").as_double();
    poll_interval_ms_ = this->get_parameter("poll_interval_ms").as_int();
    reconnect_interval_ms_ = this->get_parameter("reconnect_interval_ms").as_int();

    if (deadzone_ < 0.0) {
      deadzone_ = 0.0;
    }
    if (deadzone_ >= 1.0) {
      deadzone_ = 0.99;
    }

    joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>(joy_topic_, 20);

    last_reconnect_attempt_ = this->now();
    last_publish_time_ = this->now();

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(poll_interval_ms_),
      std::bind(&DmJoyNode::pollJoystick, this));

    RCLCPP_INFO(
      this->get_logger(),
      "dm_joy_node started. device=%s topic=%s",
      device_path_.c_str(), joy_topic_.c_str());
  }

  ~DmJoyNode() override
  {
    closeJoystick();
  }

private:
  void pollJoystick()
  {
    if (!is_open_) {
      tryOpenJoystick();
      return;
    }

    bool state_changed = false;
    js_event event{};

    while (true) {
      const ssize_t bytes_read = read(joy_fd_, &event, sizeof(event));
      if (bytes_read == static_cast<ssize_t>(sizeof(event))) {
        state_changed = handleEvent(event) || state_changed;
        continue;
      }

      if (bytes_read < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        break;
      }

      if (bytes_read == 0 || (bytes_read < 0 && errno == ENODEV)) {
        RCLCPP_WARN(this->get_logger(), "Joystick disconnected: %s", device_path_.c_str());
        closeJoystick();
        return;
      }

      if (bytes_read < 0) {
        RCLCPP_WARN(
          this->get_logger(),
          "Joystick read error on %s: %s",
          device_path_.c_str(), std::strerror(errno));
        closeJoystick();
        return;
      }

      break;
    }

    const rclcpp::Time now = this->now();
    const double repeat_period = autorepeat_rate_ > 0.0 ? (1.0 / autorepeat_rate_) : 0.0;
    const bool should_autorepeat = repeat_period > 0.0 &&
      (now - last_publish_time_).seconds() >= repeat_period;

    if (state_changed || should_autorepeat) {
      publishJoyMessage();
    }
  }

  void tryOpenJoystick()
  {
    const rclcpp::Time now = this->now();
    if ((now - last_reconnect_attempt_).nanoseconds() / 1000000 < reconnect_interval_ms_) {
      return;
    }
    last_reconnect_attempt_ = now;

    joy_fd_ = open(device_path_.c_str(), O_RDONLY | O_NONBLOCK);
    if (joy_fd_ < 0) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Unable to open joystick device %s: %s",
        device_path_.c_str(), std::strerror(errno));
      return;
    }

    char joystick_name[128] = "Unknown Joystick";
    if (ioctl(joy_fd_, JSIOCGNAME(sizeof(joystick_name)), joystick_name) < 0) {
      std::strncpy(joystick_name, "Unknown Joystick", sizeof(joystick_name) - 1);
      joystick_name[sizeof(joystick_name) - 1] = '\0';
    }

    uint8_t axis_count = 0;
    uint8_t button_count = 0;
    ioctl(joy_fd_, JSIOCGAXES, &axis_count);
    ioctl(joy_fd_, JSIOCGBUTTONS, &button_count);

    joy_msg_.axes.assign(axis_count, 0.0F);
    joy_msg_.buttons.assign(button_count, 0);
    joy_msg_.header.frame_id = frame_id_;
    joystick_name_ = joystick_name;
    is_open_ = true;

    RCLCPP_INFO(
      this->get_logger(),
      "Opened joystick %s (%s). axes=%u buttons=%u",
      device_path_.c_str(), joystick_name_.c_str(), axis_count, button_count);

    publishJoyMessage();
  }

  bool handleEvent(const js_event & raw_event)
  {
    const uint8_t event_type = raw_event.type & ~JS_EVENT_INIT;
    if (event_type == JS_EVENT_AXIS) {
      if (raw_event.number >= joy_msg_.axes.size()) {
        return false;
      }
      joy_msg_.axes[raw_event.number] = normalizeAxisValue(raw_event.value, deadzone_);
      return true;
    }

    if (event_type == JS_EVENT_BUTTON) {
      if (raw_event.number >= joy_msg_.buttons.size()) {
        return false;
      }
      joy_msg_.buttons[raw_event.number] = raw_event.value ? 1 : 0;
      return true;
    }

    return false;
  }

  void publishJoyMessage()
  {
    joy_msg_.header.stamp = this->now();
    joy_pub_->publish(joy_msg_);
    last_publish_time_ = joy_msg_.header.stamp;
  }

  void closeJoystick()
  {
    if (joy_fd_ >= 0) {
      close(joy_fd_);
      joy_fd_ = -1;
    }
    is_open_ = false;
  }

  std::string device_path_;
  std::string joy_topic_;
  std::string frame_id_;
  std::string joystick_name_;
  double deadzone_{0.05};
  double autorepeat_rate_{20.0};
  int poll_interval_ms_{10};
  int reconnect_interval_ms_{1000};
  int joy_fd_{-1};
  bool is_open_{false};
  rclcpp::Time last_reconnect_attempt_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};
  sensor_msgs::msg::Joy joy_msg_;

  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DmJoyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
