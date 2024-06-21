cmake_minimum_required(VERSION 3.8)
project(toggle_greenzone_plugin)

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
# Depending on your Gazebo version and setup, you might need the following line if rendering is a separate lib:
# find_package(gazebo_rendering REQUIRED)

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${gazebo_ros_INCLUDE_DIRS}
  ${gazebo_dev_INCLUDE_DIRS}
  # Add gazebo_rendering include directory if separate:
  ${gazebo_rendering_INCLUDE_DIRS}
)

# Add library
add_library(toggle_greenzone_plugin SHARED src/toggle_greenzone_plugin.cpp)
ament_target_dependencies(toggle_greenzone_plugin rclcpp std_msgs gazebo_ros gazebo_dev)

# Link libraries
target_link_libraries(toggle_greenzone_plugin 
  ${GAZEBO_LIBRARIES}
  ${gazebo_ros_LIBRARIES}
  # Link against gazebo_rendering if necessary:
  ${gazebo_rendering_LIBRARIES}
)

# Install target
install(TARGETS
  toggle_greenzone_plugin
  LIBRARY DESTINATION lib/${PROJECT_NAME})

ament_package()
