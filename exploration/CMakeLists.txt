cmake_minimum_required(VERSION 3.5)
project(exploration)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ClusterInfo.msg"
  "msg/ClusterArray.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_package()

