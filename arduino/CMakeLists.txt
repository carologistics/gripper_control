cmake_minimum_required(VERSION 3.8)
project(arduino)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(UDEV REQUIRED libudev)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(diagnostic_updater REQUIRED)

include_directories(
  include
  ${UDEV_INCLUDE_DIRS}
)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Status.msg"
  "srv/GetStatus.srv"
  "srv/ResetDevice.srv"
  "action/Home.action"
  "action/Calibrate.action"
)

# Create a target for the generated interfaces
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# Arduino node
add_executable(arduino_node
  src/arduino_node.cpp
  src/serial_port.cpp
)

# Ensure the node can find its own messages
target_include_directories(arduino_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
)

ament_target_dependencies(arduino_node
  rclcpp
  rclcpp_action
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  diagnostic_updater
)

target_link_libraries(arduino_node
  ${UDEV_LIBRARIES}
  boost_system
  boost_thread
  "${cpp_typesupport_target}"
)

install(TARGETS
  arduino_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  action
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
