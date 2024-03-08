cmake_minimum_required(VERSION 3.5)
project(sd_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Generate message files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SDControl.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
