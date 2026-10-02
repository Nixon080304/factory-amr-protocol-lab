# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.8)
project(mission_coordinator)

find_package(ament_cmake REQUIRED)

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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mission_state_machine test/test_mission_state_machine.cpp)
  target_link_libraries(test_mission_state_machine mission_state_machine)
endif()

ament_package()
