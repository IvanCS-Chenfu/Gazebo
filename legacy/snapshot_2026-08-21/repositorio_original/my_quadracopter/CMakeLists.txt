cmake_minimum_required(VERSION 3.8)
project(my_quadracopter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 🔍 Buscar dependencias necesarias
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(xacro REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(std_msgs REQUIRED)

# 📦 Si hay tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 📂 Instalar carpetas de recursos
install(DIRECTORY
  launch urdf worlds config
  DESTINATION share/${PROJECT_NAME}/
)

# ▶️ Nodo para insertar el robot en Gazebo
add_executable(sim_quadcopter src/sim_quadcopter.cpp)

ament_target_dependencies(sim_quadcopter
  rclcpp
  gazebo_msgs
  ament_index_cpp
)

target_link_libraries(sim_quadcopter yaml-cpp)

install(TARGETS sim_quadcopter
  DESTINATION lib/${PROJECT_NAME}
)

# 🧩 Plugin de Gazebo Classic
add_library(quadcopter_plugin SHARED src/plugins/quadcopter_plugin.cpp)

ament_target_dependencies(quadcopter_plugin
  rclcpp
  std_msgs
  gazebo_ros
  ament_index_cpp
)

target_include_directories(quadcopter_plugin PRIVATE
  include
  ${gazebo_ros_INCLUDE_DIRS}
)

target_link_libraries(quadcopter_plugin
  ${gazebo_ros_LIBRARIES}
  yaml-cpp
)

install(TARGETS quadcopter_plugin
  DESTINATION lib
)

add_executable(prueba src/prueba.cpp)
ament_target_dependencies(prueba rclcpp std_msgs)
install(TARGETS prueba DESTINATION lib/${PROJECT_NAME})



ament_package()
