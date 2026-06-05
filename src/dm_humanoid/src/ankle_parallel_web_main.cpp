#include "rclcpp/rclcpp.hpp"

#include "dm_humanoid/ankle_parallel_web_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dm_humanoid::AnkleParallelWebNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
