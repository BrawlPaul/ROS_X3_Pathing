cmake_minimum_required(VERSION 3.8)
project(x3_bt)

# Use C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# --- Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(x3_interfaces REQUIRED)

# --- Executable ---
add_executable(bt src/bt.cpp)

ament_target_dependencies(bt
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  behaviortree_cpp
  behaviortree_ros2
  x3_interfaces
)

# --- Install target ---
install(TARGETS
  bt
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
