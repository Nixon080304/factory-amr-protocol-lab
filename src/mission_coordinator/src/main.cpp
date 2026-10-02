// SPDX-License-Identifier: Apache-2.0
#include "mission_coordinator/mission_coordinator_node.hpp"
int main(int argc, char ** argv) {
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<mission_coordinator::MissionCoordinatorNode>());
  rclcpp::shutdown(); return 0;
}
