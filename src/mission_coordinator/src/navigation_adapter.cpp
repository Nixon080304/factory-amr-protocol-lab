// SPDX-License-Identifier: Apache-2.0
#include "mission_coordinator/navigation_adapter.hpp"

namespace mission_coordinator {
NavigationAdapter::NavigationAdapter(rclcpp::Node *node) {
  client_ = rclcpp_action::create_client<Nav>(node, "/navigate_to_pose");
  local_ = node->create_client<Clear>("/local_costmap/clear_entirely_local_costmap");
  global_ = node->create_client<Clear>("/global_costmap/clear_entirely_global_costmap");
}
void NavigationAdapter::navigate(const geometry_msgs::msg::PoseStamped &pose,
                                 std::function<void(bool)> done) {
  const auto generation = ++generation_;
  goal_.reset();
  if (!client_->action_server_is_ready()) {
    done(false);
    return;
  }
  Nav::Goal request;
  request.pose = pose;
  rclcpp_action::Client<Nav>::SendGoalOptions options;
  options.goal_response_callback = [this, generation, done](auto goal) {
    if (generation != generation_) {
      if (goal) {
        client_->async_cancel_goal(goal);
      }
      return;
    }
    goal_ = goal;
    if (!goal) {
      done(false);
    }
  };
  options.result_callback = [this, generation, done](const auto &result) {
    if (generation != generation_ || !goal_ || result.goal_id != goal_->get_goal_id()) {
      return;
    }
    goal_.reset();
    done(result.code == rclcpp_action::ResultCode::SUCCEEDED);
  };
  try {
    client_->async_send_goal(request, options);
  } catch (const std::exception &) {
    ++generation_;
    done(false);
  }
}
void NavigationAdapter::clear_costmaps(
    std::function<void(bool)> done,
    std::function<void(const std::string &)> on_cleared) {
  const auto generation = ++generation_;
  if (!local_->service_is_ready() || !global_->service_is_ready()) {
    done(false);
    return;
  }
  auto remaining = std::make_shared<int>(2);
  auto success = std::make_shared<bool>(true);
  auto callback = [this, generation, remaining, success, done,
                   on_cleared](const std::string &service,
                               rclcpp::Client<Clear>::SharedFuture future) {
    if (generation != generation_) {
      return;
    }
    try {
      future.get();
      if (on_cleared) {
        on_cleared(service);
      }
    } catch (const std::exception &) {
      *success = false;
    }
    if (--*remaining == 0) {
      done(*success);
    }
  };
  try {
    local_->async_send_request(std::make_shared<Clear::Request>(),
                               [callback](rclcpp::Client<Clear>::SharedFuture future) {
                                 callback("/local_costmap/clear_entirely_local_costmap",
                                          future);
                               });
    global_->async_send_request(
        std::make_shared<Clear::Request>(),
        [callback](rclcpp::Client<Clear>::SharedFuture future) {
          callback("/global_costmap/clear_entirely_global_costmap", future);
        });
  } catch (const std::exception &) {
    ++generation_;
    done(false);
  }
}
void NavigationAdapter::cancel() {
  ++generation_;
  if (goal_) {
    try {
      client_->async_cancel_goal(goal_);
    } catch (const std::exception &) {
    }
    goal_.reset();
  }
}
} // namespace mission_coordinator
