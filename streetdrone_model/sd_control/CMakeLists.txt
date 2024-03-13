cmake_minimum_required(VERSION 3.5)
project(sd_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sd_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  include
  ${gazebo_INCLUDE_DIRS}
)

# Add Gazebo executable
add_library(SdControlPlugin SHARED
  src/sd_control_plugin.cpp
)
ament_target_dependencies(SdControlPlugin
  gazebo_ros
  rclcpp
  tf2_ros
  nav_msgs
  sd_msgs
  geometry_msgs
)

# Install Gazebo plugin
install(TARGETS SdControlPlugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install Python executables
install(PROGRAMS
  src/sd_teleop_keyboard.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
