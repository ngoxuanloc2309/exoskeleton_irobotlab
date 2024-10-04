cmake_minimum_required(VERSION 3.8)
project(control_exos)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Install Python files
install(
  PROGRAMS src/node_control.py
  PROGRAMS src/push_angle.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
