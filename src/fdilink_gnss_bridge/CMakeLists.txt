cmake_minimum_required(VERSION 3.8)
project(fdilink_gnss_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

add_executable(gnss_bridge_node src/gnss_bridge_node.cpp)
ament_target_dependencies(gnss_bridge_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  builtin_interfaces
)

add_executable(gnss_gate_node src/gnss_gate_node.cpp)
ament_target_dependencies(gnss_gate_node
  rclcpp
  sensor_msgs
  nav_msgs
)

install(TARGETS
  gnss_bridge_node
  gnss_gate_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
