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
  PROGRAMS src/hi.py
  PROGRAMS src/plot.py
  PROGRAMS src/node_control_true.py
  PROGRAMS src/contact_mc.py
  PROGRAMS src/serial_monitor.py
  PROGRAMS src/check_contact.py
  PROGRAMS src/check_data_ros.py
  PROGRAMS src/contact.py
  PROGRAMS src/donghoc.py
  PROGRAMS src/mathplot.py
  PROGRAMS src/control_angle.py
  PROGRAMS src/thaythe.py
  PROGRAMS src/receive_control.py

  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
