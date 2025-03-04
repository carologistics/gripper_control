cmake_minimum_required(VERSION 3.8)
project(gigatino_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gigatino_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(msgpack REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/gigatino_ros.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_definitions(${PROJECT_NAME}
  PRIVATE "CUSTOM_ACTION_CPP_BUILDING_DLL")

ament_target_dependencies(${PROJECT_NAME}
  "gigatino_msgs"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "rclcpp_lifecycle"
  "lifecycle_msgs"
  "nav2_util"
  "msgpack"
  "geometry_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs")

install(DIRECTORY
  launch params
  DESTINATION
  share/${PROJECT_NAME})

rclcpp_components_register_node(${PROJECT_NAME} PLUGIN "gigatino_ros::GigatinoROS" EXECUTABLE gigatino_node)
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(PROGRAMS
  scripts/gigatino_mockup.py
  scripts/test_ethernet.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
