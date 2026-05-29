#include "dm_test/dm_trajectory_test_node.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dm_test::DmTrajectoryTestNode>());
  rclcpp::shutdown();
  return 0;
}
