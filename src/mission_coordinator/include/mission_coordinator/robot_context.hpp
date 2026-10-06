// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <regex>
#include <stdexcept>
#include <string>
#include <utility>

namespace mission_coordinator {
// Identity stays fixed for the lifetime of one robot stack. ROS resolves local
// endpoints against the node namespace; shared factory resources stay global.
struct RobotContext {
  const std::string robot_id;
  const std::string frame_prefix;
  static constexpr const char *mission_action = "factory/execute_mission";
  static constexpr const char *navigation_action = "navigate_to_pose";
  static constexpr const char *pose_topic = "amcl_pose";
  static constexpr const char *detection_topic = "factory/station_detection";
  static constexpr const char *state_topic = "factory/mission_state";
  static constexpr const char *local_costmap =
      "local_costmap/clear_entirely_local_costmap";
  static constexpr const char *global_costmap =
      "global_costmap/clear_entirely_global_costmap";
  static constexpr const char *map_frame = "map";

  RobotContext(std::string id, std::string prefix)
      : robot_id(std::move(id)), frame_prefix(std::move(prefix)) {
    if (!std::regex_match(robot_id, std::regex("[A-Za-z_][A-Za-z0-9_]*"))) {
      throw std::invalid_argument("robot_id must be a nonempty ROS identifier");
    }
    if (!frame_prefix.empty() &&
        !std::regex_match(frame_prefix, std::regex("([A-Za-z_][A-Za-z0-9_]*/)+"))) {
      throw std::invalid_argument(
          "frame_prefix must be relative and end with / (or empty for V1)");
    }
  }

  bool accepts_robot(const std::string &requested) const {
    return requested == robot_id;
  }

  std::string local_frame(const std::string &frame) const {
    if (frame == map_frame ||
        frame.compare(0, frame_prefix.size(), frame_prefix) == 0) {
      return frame;
    }
    return frame_prefix + frame;
  }
};
} // namespace mission_coordinator
