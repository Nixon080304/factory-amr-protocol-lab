# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.8)
project(factory_interfaces)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteFactoryMission.action"
  "action/ExecuteFleetMission.action"
  "action/DockRobot.action"
  "msg/StationDetection.msg"
  "msg/RobotState.msg"
  "msg/ProtocolEvent.msg"
  "srv/TransferPart.srv"
  "srv/EstimateMissionCost.srv"
  "srv/AcquireResource.srv"
  "srv/RenewResource.srv"
  "srv/ReleaseResource.srv"
  "srv/CancelResourceWait.srv"
  "srv/SetFault.srv"
  "msg/FaultCommand.msg"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
