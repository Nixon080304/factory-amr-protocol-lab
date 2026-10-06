// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <rclcpp/expand_topic_or_service_name.hpp>
#include <array>
#include <string>

#include "mission_coordinator/robot_context.hpp"

using mission_coordinator::RobotContext;

TEST(RobotContextTest, RobotLocalEndpointsResolveIndependently) {
  const RobotContext first("amr_01", "amr_01/");
  const RobotContext second("amr_02", "amr_02/");
  const std::array<std::pair<const char *, const char *>, 7> endpoints = {
      {{RobotContext::mission_action, "factory/execute_mission"},
       {RobotContext::navigation_action, "navigate_to_pose"},
       {RobotContext::pose_topic, "amcl_pose"},
       {RobotContext::detection_topic, "factory/station_detection"},
       {RobotContext::state_topic, "factory/mission_state"},
       {RobotContext::local_costmap, "local_costmap/clear_entirely_local_costmap"},
       {RobotContext::global_costmap, "global_costmap/clear_entirely_global_costmap"}}};
  for (const auto &endpoint : endpoints) {
    EXPECT_EQ(rclcpp::expand_topic_or_service_name(endpoint.first, "coordinator",
                                                   "/amr_01", false),
              std::string("/amr_01/") + endpoint.second);
    EXPECT_EQ(rclcpp::expand_topic_or_service_name(endpoint.first, "coordinator",
                                                   "/amr_02", false),
              std::string("/amr_02/") + endpoint.second);
  }
  EXPECT_TRUE(first.accepts_robot("amr_01"));
  EXPECT_FALSE(first.accepts_robot("amr_02"));
  EXPECT_TRUE(second.accepts_robot("amr_02"));
  EXPECT_FALSE(second.accepts_robot("amr_01"));
  EXPECT_FALSE(second.accepts_robot(""));
  EXPECT_EQ(RobotContext::map_frame, std::string("map"));
}

TEST(RobotContextTest, FutureRobotUsesSameIdentityAndFrameRules) {
  const RobotContext robot("warehouse_10", "floor/warehouse_10/");
  EXPECT_TRUE(robot.accepts_robot("warehouse_10"));
  EXPECT_FALSE(robot.accepts_robot("amr_01"));
  EXPECT_EQ(robot.local_frame("camera_optical_frame"),
            "floor/warehouse_10/camera_optical_frame");
  EXPECT_EQ(robot.local_frame("floor/warehouse_10/camera_optical_frame"),
            "floor/warehouse_10/camera_optical_frame");
  EXPECT_EQ(robot.local_frame("map"), "map");
  EXPECT_EQ(RobotContext("amr_01", "").local_frame("camera_optical_frame"),
            "camera_optical_frame");
}

TEST(RobotContextTest, RejectsInvalidIdentityAndFramePrefixes) {
  for (const std::string id : {"", "01", "amr/01", "amr-01", "amr 01"}) {
    EXPECT_THROW(RobotContext(id, "amr_01/"), std::invalid_argument);
  }
  for (const std::string prefix :
       {"/amr_01/", "amr_01", "amr_01//", "../", "amr-01/", "01/"}) {
    EXPECT_THROW(RobotContext("amr_01", prefix), std::invalid_argument);
  }
}
