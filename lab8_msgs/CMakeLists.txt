cmake_minimum_required(VERSION 3.5)
project(lab8_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Explore.action"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_package()
