#include "rclcpp/rclcpp.hpp"
#include "dm_imu/imu_driver.h"
#include <memory>

int main(int argc, char **argv)
{
    // 初始化 ROS 2
    rclcpp::init(argc, argv);
    
    // 实例化 IMU 节点
    auto imu_node = std::make_shared<dmbot_serial::DmImu>();
    
    // 保持节点运行，替代 ROS 1 的 ros::Rate 循环
    rclcpp::spin(imu_node);
    
    // 关闭节点
    rclcpp::shutdown();
    return 0;
}
