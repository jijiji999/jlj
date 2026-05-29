cmake_minimum_required(VERSION 3.8)
project(dm_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(dm_motor REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(dm_trajectory_test_node
  src/dm_trajectory_test_node.cpp
  src/dm_trajectory_test_main.cpp
)

target_compile_features(dm_trajectory_test_node PUBLIC cxx_std_17)

target_include_directories(dm_trajectory_test_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(dm_trajectory_test_node
  ament_index_cpp
  dm_motor
  rclcpp
  sensor_msgs
  std_msgs
)

target_link_libraries(dm_trajectory_test_node
  dm_motor::dm_motor_core
  yaml-cpp
)

install(
  TARGETS dm_trajectory_test_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
