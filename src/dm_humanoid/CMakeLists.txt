cmake_minimum_required(VERSION 3.8)
project(dm_humanoid)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(dm_motor REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(dm_humanoid_parallel
  src/parallel_kinematics.cpp
  src/parallel_joint_adapter.cpp
)

target_compile_features(dm_humanoid_parallel PUBLIC cxx_std_17)

target_include_directories(dm_humanoid_parallel
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(dm_humanoid_parallel
  dm_motor
)

target_link_libraries(dm_humanoid_parallel
  dm_motor::dm_motor_core
  yaml-cpp
)

add_executable(dm_humanoid_control_node
  src/dm_humanoid_control_node.cpp
)

add_executable(ankle_parallel_web_node
  src/ankle_parallel_web_node.cpp
  src/ankle_parallel_web_main.cpp
)

target_compile_features(dm_humanoid_control_node PUBLIC cxx_std_17)
target_compile_features(ankle_parallel_web_node PUBLIC cxx_std_17)

target_include_directories(dm_humanoid_control_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_include_directories(ankle_parallel_web_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(dm_humanoid_control_node
  ament_index_cpp
  dm_motor
  rclcpp
  sensor_msgs
  std_msgs
  tf2
)

ament_target_dependencies(ankle_parallel_web_node
  ament_index_cpp
  dm_motor
  rclcpp
)

target_link_libraries(dm_humanoid_control_node
  dm_humanoid_parallel
  dm_motor::dm_motor_core
  yaml-cpp
)

target_link_libraries(ankle_parallel_web_node
  dm_humanoid_parallel
  dm_motor::dm_motor_core
  yaml-cpp
)

install(
  TARGETS dm_humanoid_control_node ankle_parallel_web_node dm_humanoid_parallel
  EXPORT export_dm_humanoid_parallel
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY web
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
