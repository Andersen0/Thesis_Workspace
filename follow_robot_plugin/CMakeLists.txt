cmake_minimum_required(VERSION 3.8)
project(follow_robot_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include ${gazebo_dev_INCLUDE_DIRS} ${gazebo_ros_INCLUDE_DIRS})

add_library(follow_robot_plugin SHARED
  src/follow_robot_plugin.cpp
)

ament_target_dependencies(follow_robot_plugin
  gazebo_dev
  gazebo_ros
)

install(TARGETS
  follow_robot_plugin
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()