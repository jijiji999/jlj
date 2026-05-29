cmake_minimum_required(VERSION 3.8)
project(dm_joy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(dm_joy_node
  src/dm_joy_node.cpp
)

add_executable(dm_joy_monitor_node
  src/dm_joy_monitor_node.cpp
)

ament_target_dependencies(dm_joy_node
  rclcpp
  sensor_msgs
)

ament_target_dependencies(dm_joy_monitor_node
  rclcpp
  sensor_msgs
)

install(TARGETS
  dm_joy_node
  dm_joy_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
