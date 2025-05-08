cmake_minimum_required(VERSION 3.8)
project(my_box_robot)

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

include_directories(include ${gazebo_INCLUDE_DIRS})

include_directories(
  include
)


add_library(diff_drive_plugin SHARED src/diff_drive_plugin.cpp)

ament_target_dependencies(diff_drive_plugin
  rclcpp
  geometry_msgs
  gazebo_ros
)

install(TARGETS diff_drive_plugin
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include/
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

add_executable(sim_box_bot src/sim_box_bot.cpp)

ament_target_dependencies(sim_box_bot
  rclcpp
  gazebo_msgs
  ament_index_cpp
)

install(TARGETS sim_box_bot
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch urdf worlds
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
