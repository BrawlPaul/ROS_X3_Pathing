cmake_minimum_required(VERSION 3.5)
project(x3_pathing)
#CMake so .action file will gen into cpp and python
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/SetHeight.action"
  DEPENDENCIES std_msgs  #
)

ament_package()
