cmake_minimum_required(VERSION 3.8)
project(dm_imu)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 寻找核心依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# 寻找第三方串口库
find_package(serial REQUIRED)

# 2. 包含头文件目录
include_directories(
  include
)

# 3. 声明可执行文件（把你所有的 .cpp 都编译进这一个节点里）
add_executable(dm_imu_node
  src/bsp_crc.cpp
  src/imu_driver.cpp
  src/test_imu.cpp
)

add_executable(imu_visual_test_node
  src/imu_visual_test_node.cpp
)

# 4. 链接 ROS 2 的依赖库
ament_target_dependencies(dm_imu_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  visualization_msgs
  serial
)

ament_target_dependencies(imu_visual_test_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  visualization_msgs
)

# 5. 链接第三方串口库
#target_link_libraries(dm_imu_node serial)

# 6. 安装规则（告诉 ROS 2 编译后把生成的文件放到哪里）
install(TARGETS
  dm_imu_node
  imu_visual_test_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
