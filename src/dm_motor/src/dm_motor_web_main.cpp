#include "rclcpp/rclcpp.hpp"

#include "dm_motor/dm_motor_web_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dm_motor::DmMotorWebNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
