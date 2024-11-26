cmake_minimum_required(VERSION 3.8)
project(arduino_pkg)

# Ensure Python3 is used
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(
  PROGRAMS
  src/arduino_control_manual.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install any additional non-Python files (e.g., configuration files)
install(
  DIRECTORY src/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
