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

add_executable(dm_humanoid_system_check_node
  src/dm_humanoid_system_check_node.cpp
)

target_compile_features(dm_humanoid_control_node PUBLIC cxx_std_17)
target_compile_features(ankle_parallel_web_node PUBLIC cxx_std_17)
target_compile_features(dm_humanoid_system_check_node PUBLIC cxx_std_17)

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

target_include_directories(dm_humanoid_system_check_node
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

ament_target_dependencies(dm_humanoid_system_check_node
  rclcpp
  sensor_msgs
  std_msgs
)

set(ONNXRUNTIME_WORKSPACE_HINT "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/onnxruntime")
set(ONNXRUNTIME_ROOT_DIR "" CACHE PATH "Optional ONNX Runtime installation root")
find_path(ONNXRUNTIME_INCLUDE_DIR
  NAMES onnxruntime_cxx_api.h
  HINTS
    ${ONNXRUNTIME_WORKSPACE_HINT}
    ${ONNXRUNTIME_ROOT_DIR}
    $ENV{ONNXRUNTIME_ROOT_DIR}
  PATH_SUFFIXES include include/onnxruntime include/onnxruntime/core/session
)
find_library(ONNXRUNTIME_LIBRARY
  NAMES onnxruntime
  HINTS
    ${ONNXRUNTIME_WORKSPACE_HINT}
    ${ONNXRUNTIME_ROOT_DIR}
    $ENV{ONNXRUNTIME_ROOT_DIR}
  PATH_SUFFIXES lib lib64
)

target_link_libraries(dm_humanoid_control_node
  dm_humanoid_parallel
  dm_motor::dm_motor_core
  yaml-cpp
)

if(ONNXRUNTIME_INCLUDE_DIR AND ONNXRUNTIME_LIBRARY)
  message(STATUS "Found ONNX Runtime: ${ONNXRUNTIME_LIBRARY}")
  get_filename_component(ONNXRUNTIME_LIBRARY_DIR ${ONNXRUNTIME_LIBRARY} DIRECTORY)
  target_compile_definitions(dm_humanoid_control_node PRIVATE DM_MOTOR_HAS_ONNXRUNTIME=1)
  target_include_directories(dm_humanoid_control_node PRIVATE ${ONNXRUNTIME_INCLUDE_DIR})
  target_link_libraries(dm_humanoid_control_node ${ONNXRUNTIME_LIBRARY})
  set_target_properties(dm_humanoid_control_node PROPERTIES
    BUILD_RPATH "${ONNXRUNTIME_LIBRARY_DIR}"
    INSTALL_RPATH "${ONNXRUNTIME_LIBRARY_DIR}"
  )
else()
  message(STATUS "ONNX Runtime not found, loco mode will use fallback backend")
endif()

target_link_libraries(ankle_parallel_web_node
  dm_humanoid_parallel
  dm_motor::dm_motor_core
  yaml-cpp
)

ament_export_targets(export_dm_humanoid_parallel HAS_LIBRARY_TARGET)
ament_export_dependencies(dm_motor Eigen3 yaml-cpp)

install(
  TARGETS dm_humanoid_parallel
  EXPORT export_dm_humanoid_parallel
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  TARGETS dm_humanoid_control_node ankle_parallel_web_node dm_humanoid_system_check_node
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
