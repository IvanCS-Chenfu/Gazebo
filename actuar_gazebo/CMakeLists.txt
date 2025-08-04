cmake_minimum_required(VERSION 3.8)
project(fuerza_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(xacro REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(ignition-math6 REQUIRED)



include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${gazebo_ros_INCLUDE_DIRS}
  ${ignition-math6_INCLUDE_DIRS}
)

include_directories(
  include
)

add_library(led_color_plugin SHARED src/light_plugin.cpp)
ament_target_dependencies(led_color_plugin
  gazebo_ros
  rclcpp
  geometry_msgs
  ignition-math6
)
install(TARGETS led_color_plugin
  LIBRARY DESTINATION lib
)



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

add_executable(sim_URDF src/sim_URDF.cpp)

ament_target_dependencies(sim_URDF
  rclcpp
  gazebo_msgs
  ament_index_cpp
)

install(TARGETS sim_URDF
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch urdf worlds
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()