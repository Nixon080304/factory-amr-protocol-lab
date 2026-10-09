// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>

namespace mission_coordinator {
// All callbacks use the node's mutually exclusive default callback group.
class NavigationAdapter {
public:
  using Diagnostic = std::function<void(const std::string &, const std::string &)>;
  explicit NavigationAdapter(rclcpp::Node *node, Diagnostic diagnostic = {});
  void navigate(const geometry_msgs::msg::PoseStamped &pose,
                std::function<void(bool)> done);
  void clear_costmaps(std::function<void(bool)> done,
                      std::function<void(const std::string &)> on_cleared = {});
  void cancel();

private:
  using Nav = nav2_msgs::action::NavigateToPose;
  using Clear = nav2_msgs::srv::ClearEntireCostmap;
  rclcpp_action::Client<Nav>::SharedPtr client_;
  rclcpp_action::ClientGoalHandle<Nav>::SharedPtr goal_;
  rclcpp::Client<Clear>::SharedPtr local_, global_;
  uint64_t generation_{0};
  Diagnostic diagnostic_;
};
} // namespace mission_coordinator
