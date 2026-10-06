// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "mission_coordinator/resource_adapter.hpp"
namespace rclcpp {
class Node;
}
namespace mission_coordinator {
std::shared_ptr<ResourceTransport> make_resource_transport(rclcpp::Node *node);
}
