#include <cmath>
#include <cstdio>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace
{
geometry_msgs::msg::Point makePoint(double x, double y, double z)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

std_msgs::msg::ColorRGBA makeColor(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}
}  // namespace

class ImuVisualTestNode : public rclcpp::Node
{
public:
  ImuVisualTestNode()
  : Node("imu_visual_test_node")
  {
    this->declare_parameter<std::string>("imu_topic", "imu/data");
    this->declare_parameter<std::string>("marker_topic", "imu_visual_test/markers");
    this->declare_parameter<std::string>("frame_id", "imu_link");
    this->declare_parameter<std::string>("euler_topic", "imu_visual_test/euler");
    this->declare_parameter<std::string>("accel_topic", "imu_visual_test/linear_acceleration");
    this->declare_parameter<std::string>("gyro_topic", "imu_visual_test/angular_velocity");
    this->declare_parameter<std::string>("roll_topic", "imu_visual_test/roll_deg");
    this->declare_parameter<std::string>("pitch_topic", "imu_visual_test/pitch_deg");
    this->declare_parameter<std::string>("yaw_topic", "imu_visual_test/yaw_deg");
    this->declare_parameter<double>("timeout_sec", 0.5);
    this->declare_parameter<double>("body_length", 0.30);
    this->declare_parameter<double>("body_width", 0.16);
    this->declare_parameter<double>("body_height", 0.08);
    this->declare_parameter<double>("axis_length", 0.45);

    imu_topic_ = this->get_parameter("imu_topic").as_string();
    marker_topic_ = this->get_parameter("marker_topic").as_string();
    default_frame_id_ = this->get_parameter("frame_id").as_string();
    euler_topic_ = this->get_parameter("euler_topic").as_string();
    accel_topic_ = this->get_parameter("accel_topic").as_string();
    gyro_topic_ = this->get_parameter("gyro_topic").as_string();
    roll_topic_ = this->get_parameter("roll_topic").as_string();
    pitch_topic_ = this->get_parameter("pitch_topic").as_string();
    yaw_topic_ = this->get_parameter("yaw_topic").as_string();
    timeout_sec_ = this->get_parameter("timeout_sec").as_double();
    body_length_ = this->get_parameter("body_length").as_double();
    body_width_ = this->get_parameter("body_width").as_double();
    body_height_ = this->get_parameter("body_height").as_double();
    axis_length_ = this->get_parameter("axis_length").as_double();

    marker_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);
    euler_pub_ =
      this->create_publisher<geometry_msgs::msg::Vector3Stamped>(euler_topic_, 20);
    accel_pub_ =
      this->create_publisher<geometry_msgs::msg::Vector3Stamped>(accel_topic_, 20);
    gyro_pub_ =
      this->create_publisher<geometry_msgs::msg::Vector3Stamped>(gyro_topic_, 20);
    roll_pub_ =
      this->create_publisher<std_msgs::msg::Float64>(roll_topic_, 20);
    pitch_pub_ =
      this->create_publisher<std_msgs::msg::Float64>(pitch_topic_, 20);
    yaw_pub_ =
      this->create_publisher<std_msgs::msg::Float64>(yaw_topic_, 20);

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, 20,
      std::bind(&ImuVisualTestNode::imuCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&ImuVisualTestNode::publishFromTimer, this));

    last_orientation_.setRPY(0.0, 0.0, 0.0);
    last_orientation_.normalize();
    last_msg_receive_time_ = this->now();
    last_msg_stamp_ = this->now();

    publishMarkers(false, true);
    RCLCPP_INFO(
      this->get_logger(),
      "IMU visual test node started. Subscribe: %s, publish markers: %s",
      imu_topic_.c_str(), marker_topic_.c_str());
  }

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    tf2::Quaternion orientation;
    tf2::fromMsg(msg->orientation, orientation);

    if (orientation.length2() < 1e-12) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Received IMU orientation is near zero quaternion, skip this frame.");
      return;
    }

    orientation.normalize();

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      last_orientation_ = orientation;
      last_roll_ = roll;
      last_pitch_ = pitch;
      last_yaw_ = yaw;
      last_frame_id_ = msg->header.frame_id.empty() ? default_frame_id_ : msg->header.frame_id;
      last_msg_receive_time_ = this->now();
      last_msg_stamp_ = msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0 ?
        this->now() : rclcpp::Time(msg->header.stamp);
      has_imu_data_ = true;
    }

    publishPlotTopics(msg, roll, pitch, yaw);
    publishMarkers(false, false);
  }

  void publishFromTimer()
  {
    bool has_imu_data = false;
    bool timed_out = false;

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      has_imu_data = has_imu_data_;
      timed_out = has_imu_data_ &&
        (this->now() - last_msg_receive_time_).seconds() > timeout_sec_;
    }

    if (!has_imu_data) {
      publishMarkers(false, true);
      return;
    }

    if (timed_out) {
      publishMarkers(true, false);
    }
  }

  void publishMarkers(bool timed_out, bool waiting_for_data)
  {
    tf2::Quaternion orientation;
    std::string frame_id;
    rclcpp::Time stamp;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      orientation = last_orientation_;
      frame_id = last_frame_id_.empty() ? default_frame_id_ : last_frame_id_;
      stamp = last_msg_stamp_;
      roll = last_roll_;
      pitch = last_pitch_;
      yaw = last_yaw_;
    }

    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker delete_all;
    delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(delete_all);

    const auto online_body_color = makeColor(0.10f, 0.75f, 0.35f, 0.90f);
    const auto timeout_body_color = makeColor(0.95f, 0.60f, 0.10f, 0.90f);
    const auto waiting_body_color = makeColor(0.50f, 0.50f, 0.50f, 0.70f);
    const auto reference_color = makeColor(0.75f, 0.75f, 0.75f, 0.55f);

    const auto body_color = waiting_for_data ? waiting_body_color :
      (timed_out ? timeout_body_color : online_body_color);

    markers.markers.push_back(makeBodyMarker(frame_id, stamp, orientation, body_color));
    appendReferenceAxes(markers, frame_id, stamp, reference_color);
    appendMeasuredAxes(markers, frame_id, stamp, orientation, timed_out, waiting_for_data);
    markers.markers.push_back(makeTextMarker(frame_id, stamp, roll, pitch, yaw, timed_out, waiting_for_data));

    marker_pub_->publish(markers);
  }

  void publishPlotTopics(
    const sensor_msgs::msg::Imu::SharedPtr & msg,
    double roll,
    double pitch,
    double yaw)
  {
    geometry_msgs::msg::Vector3Stamped euler_msg;
    euler_msg.header = msg->header;
    if (euler_msg.header.frame_id.empty()) {
      euler_msg.header.frame_id = default_frame_id_;
    }
    euler_msg.vector.x = roll * 180.0 / M_PI;
    euler_msg.vector.y = pitch * 180.0 / M_PI;
    euler_msg.vector.z = yaw * 180.0 / M_PI;
    euler_pub_->publish(euler_msg);

    geometry_msgs::msg::Vector3Stamped accel_msg;
    accel_msg.header = euler_msg.header;
    accel_msg.vector = msg->linear_acceleration;
    accel_pub_->publish(accel_msg);

    geometry_msgs::msg::Vector3Stamped gyro_msg;
    gyro_msg.header = euler_msg.header;
    gyro_msg.vector = msg->angular_velocity;
    gyro_pub_->publish(gyro_msg);

    std_msgs::msg::Float64 roll_msg;
    roll_msg.data = euler_msg.vector.x;
    roll_pub_->publish(roll_msg);

    std_msgs::msg::Float64 pitch_msg;
    pitch_msg.data = euler_msg.vector.y;
    pitch_pub_->publish(pitch_msg);

    std_msgs::msg::Float64 yaw_msg;
    yaw_msg.data = euler_msg.vector.z;
    yaw_pub_->publish(yaw_msg);
  }

  visualization_msgs::msg::Marker makeBodyMarker(
    const std::string & frame_id,
    const rclcpp::Time & stamp,
    const tf2::Quaternion & orientation,
    const std_msgs::msg::ColorRGBA & color) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.ns = "imu_body";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = makePoint(0.0, 0.0, 0.0);
    marker.pose.orientation = tf2::toMsg(orientation);
    marker.scale.x = body_length_;
    marker.scale.y = body_width_;
    marker.scale.z = body_height_;
    marker.color = color;
    return marker;
  }

  void appendReferenceAxes(
    visualization_msgs::msg::MarkerArray & markers,
    const std::string & frame_id,
    const rclcpp::Time & stamp,
    const std_msgs::msg::ColorRGBA & color) const
  {
    const std::vector<geometry_msgs::msg::Point> endpoints = {
      makePoint(axis_length_, 0.0, 0.0),
      makePoint(0.0, axis_length_, 0.0),
      makePoint(0.0, 0.0, axis_length_)
    };

    for (int i = 0; i < 3; ++i) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = frame_id;
      marker.header.stamp = stamp;
      marker.ns = "reference_axes";
      marker.id = i;
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = 0.015;
      marker.scale.y = 0.03;
      marker.scale.z = 0.04;
      marker.color = color;
      marker.points.push_back(makePoint(0.0, 0.0, 0.0));
      marker.points.push_back(endpoints[static_cast<std::size_t>(i)]);
      markers.markers.push_back(marker);
    }
  }

  void appendMeasuredAxes(
    visualization_msgs::msg::MarkerArray & markers,
    const std::string & frame_id,
    const rclcpp::Time & stamp,
    const tf2::Quaternion & orientation,
    bool timed_out,
    bool waiting_for_data) const
  {
    const std::vector<tf2::Vector3> basis = {
      tf2::Vector3(axis_length_, 0.0, 0.0),
      tf2::Vector3(0.0, axis_length_, 0.0),
      tf2::Vector3(0.0, 0.0, axis_length_)
    };

    std::vector<std_msgs::msg::ColorRGBA> colors = {
      makeColor(0.95f, 0.15f, 0.15f, 0.95f),
      makeColor(0.10f, 0.85f, 0.10f, 0.95f),
      makeColor(0.15f, 0.35f, 0.95f, 0.95f)
    };

    if (timed_out || waiting_for_data) {
      for (auto & color : colors) {
        color.a = waiting_for_data ? 0.35f : 0.55f;
      }
    }

    for (int i = 0; i < 3; ++i) {
      const tf2::Vector3 rotated = tf2::quatRotate(orientation, basis[static_cast<std::size_t>(i)]);

      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = frame_id;
      marker.header.stamp = stamp;
      marker.ns = "measured_axes";
      marker.id = i;
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = 0.03;
      marker.scale.y = 0.05;
      marker.scale.z = 0.06;
      marker.color = colors[static_cast<std::size_t>(i)];
      marker.points.push_back(makePoint(0.0, 0.0, 0.0));
      marker.points.push_back(makePoint(rotated.x(), rotated.y(), rotated.z()));
      markers.markers.push_back(marker);
    }
  }

  visualization_msgs::msg::Marker makeTextMarker(
    const std::string & frame_id,
    const rclcpp::Time & stamp,
    double roll,
    double pitch,
    double yaw,
    bool timed_out,
    bool waiting_for_data) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.ns = "imu_text";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = makePoint(0.0, 0.0, 0.30);
    marker.pose.orientation.w = 1.0;
    marker.scale.z = 0.08;

    if (waiting_for_data) {
      marker.color = makeColor(0.95f, 0.95f, 0.95f, 0.95f);
      marker.text = "IMU status: waiting for imu/data";
      return marker;
    }

    if (timed_out) {
      marker.color = makeColor(1.0f, 0.45f, 0.10f, 0.95f);
    } else {
      marker.color = makeColor(1.0f, 1.0f, 1.0f, 0.98f);
    }

    const double roll_deg = roll * 180.0 / M_PI;
    const double pitch_deg = pitch * 180.0 / M_PI;
    const double yaw_deg = yaw * 180.0 / M_PI;

    char text_buffer[256];
    std::snprintf(
      text_buffer, sizeof(text_buffer),
      "IMU status: %s\nRoll : %7.2f deg\nPitch: %7.2f deg\nYaw  : %7.2f deg",
      timed_out ? "timeout" : "online",
      roll_deg, pitch_deg, yaw_deg);
    marker.text = text_buffer;
    return marker;
  }

  std::string imu_topic_;
  std::string marker_topic_;
  std::string default_frame_id_;
  std::string euler_topic_;
  std::string accel_topic_;
  std::string gyro_topic_;
  std::string roll_topic_;
  std::string pitch_topic_;
  std::string yaw_topic_;
  std::string last_frame_id_{};
  double timeout_sec_{0.5};
  double body_length_{0.30};
  double body_width_{0.16};
  double body_height_{0.08};
  double axis_length_{0.45};
  bool has_imu_data_{false};
  double last_roll_{0.0};
  double last_pitch_{0.0};
  double last_yaw_{0.0};
  tf2::Quaternion last_orientation_;
  rclcpp::Time last_msg_receive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_msg_stamp_{0, 0, RCL_ROS_TIME};
  std::mutex data_mutex_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr euler_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr accel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr gyro_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr roll_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pitch_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImuVisualTestNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
