cmake_minimum_required(VERSION 3.8)
project(dm_motor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(dm_motor_core
  src/damiao_protocol.cpp
  src/socketcan_transport.cpp
  src/dm_motor_driver.cpp
  src/dm_motor_manager.cpp
)

target_compile_features(dm_motor_core PUBLIC cxx_std_17)

target_include_directories(dm_motor_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(dm_motor_core
  PUBLIC
    yaml-cpp
)

add_executable(dm_motor_node
  src/dm_motor_node.cpp
  src/dm_motor_main.cpp
)

add_executable(dm_motor_test_node
  src/dm_motor_test_node.cpp
  src/dm_motor_test_main.cpp
)

add_executable(dm_motor_web_node
  src/dm_motor_web_node.cpp
  src/dm_motor_web_main.cpp
)

target_compile_features(dm_motor_node PUBLIC cxx_std_17)

target_include_directories(dm_motor_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(dm_motor_node
  ament_index_cpp
  rclcpp
  sensor_msgs
)

target_link_libraries(dm_motor_node
  dm_motor_core
)

target_compile_features(dm_motor_test_node PUBLIC cxx_std_17)

target_include_directories(dm_motor_test_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(dm_motor_test_node
  ament_index_cpp
  rclcpp
  sensor_msgs
)

target_link_libraries(dm_motor_test_node
  dm_motor_core
)

target_compile_features(dm_motor_web_node PUBLIC cxx_std_17)

target_include_directories(dm_motor_web_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(dm_motor_web_node
  ament_index_cpp
  rclcpp
  sensor_msgs
)

target_link_libraries(dm_motor_web_node
  dm_motor_core
)

install(
  TARGETS dm_motor_core
  EXPORT export_dm_motor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  TARGETS dm_motor_node dm_motor_test_node dm_motor_web_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY web
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md USAGE_GUIDE.md
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/start_can_bridge.sh
  DESTINATION share/${PROJECT_NAME}/scripts
)

ament_export_include_directories(include)
ament_export_libraries(dm_motor_core)
ament_export_targets(export_dm_motor HAS_LIBRARY_TARGET)

ament_package()
