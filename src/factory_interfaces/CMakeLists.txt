# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.8)
project(factory_interfaces)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteFactoryMission.action"
  "msg/StationDetection.msg"
  "msg/ProtocolEvent.msg"
  "srv/TransferPart.srv"
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
