cmake_minimum_required(VERSION 3.8)
project(hdr_simulation_gz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(gz_ros2_control REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(ros_gz_interfaces REQUIRED)

find_package(moveit_visual_tools REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)

find_package(xacro REQUIRED)

install(DIRECTORY config launch examples
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
  DESTINATION lib/${PROJECT_NAME})

ament_package()