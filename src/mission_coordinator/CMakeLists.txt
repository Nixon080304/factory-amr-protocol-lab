# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.8)
project(mission_coordinator)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(factory_interfaces REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(mission_state_machine src/mission_state_machine.cpp)
target_compile_features(mission_state_machine PUBLIC cxx_std_17)
target_include_directories(mission_state_machine PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(mission_state_machine PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(DIRECTORY include/ DESTINATION include)
install(TARGETS mission_state_machine
  EXPORT mission_state_machine_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_targets(mission_state_machine_targets HAS_LIBRARY_TARGET)
add_library(mission_ros src/navigation_adapter.cpp src/mission_coordinator_node.cpp)
target_link_libraries(mission_ros mission_state_machine)
ament_target_dependencies(mission_ros rclcpp rclcpp_action factory_interfaces nav2_msgs geometry_msgs std_msgs)
add_executable(mission_coordinator src/main.cpp)
target_link_libraries(mission_coordinator mission_ros)
install(TARGETS mission_ros mission_coordinator ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mission_state_machine test/test_mission_state_machine.cpp)
  target_link_libraries(test_mission_state_machine mission_state_machine)
  find_package(rosgraph_msgs REQUIRED)
  ament_add_gtest(test_mission_coordinator_node test/test_mission_coordinator_node.cpp)
  target_link_libraries(test_mission_coordinator_node mission_ros)
  ament_target_dependencies(test_mission_coordinator_node rclcpp rclcpp_action factory_interfaces nav2_msgs geometry_msgs rosgraph_msgs)
  set_tests_properties(test_mission_coordinator_node PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=76" TIMEOUT 90)
endif()

ament_package()
