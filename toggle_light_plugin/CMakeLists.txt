cmake_minimum_required(VERSION 3.8)
project(toggle_light_plugin)

# Set CMake policy for IN_LIST operator
if(POLICY CMP0057)
  cmake_policy(SET CMP0057 NEW)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_dev REQUIRED)

# Include directories
include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${gazebo_ros_INCLUDE_DIRS}
  ${gazebo_dev_INCLUDE_DIRS}
)

# Add library
add_library(toggle_light_plugin SHARED src/toggle_light_plugin.cpp)
ament_target_dependencies(toggle_light_plugin rclcpp std_msgs gazebo_ros)

# Link libraries
target_link_libraries(toggle_light_plugin ${GAZEBO_LIBRARIES})

# Install target
install(TARGETS
  toggle_light_plugin
  LIBRARY DESTINATION lib/${PROJECT_NAME})

ament_package()
