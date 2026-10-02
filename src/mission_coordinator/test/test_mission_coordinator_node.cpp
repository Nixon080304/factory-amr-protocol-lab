// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <chrono>
#include <thread>
#include <algorithm>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>
#include <factory_interfaces/action/execute_factory_mission.hpp>
#include <factory_interfaces/srv/transfer_part.hpp>
#include <factory_interfaces/msg/station_detection.hpp>
#include <factory_interfaces/msg/protocol_event.hpp>
#include <std_msgs/msg/string.hpp>
#include "mission_coordinator/mission_coordinator_node.hpp"

using Mission = factory_interfaces::action::ExecuteFactoryMission;
using Nav = nav2_msgs::action::NavigateToPose;
using namespace std::chrono_literals;

class CoordinatorTest : public testing::Test {
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override {
    if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
    coordinator = std::make_shared<mission_coordinator::MissionCoordinatorNode>(
      rclcpp::NodeOptions().parameter_overrides({
        {"use_sim_time", true}, {"perception_timeout_sec", 2.0}}));
    peer = std::make_shared<rclcpp::Node>("mission_test_peers");
    clock_pub = peer->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
    pose_pub = peer->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose", rclcpp::SensorDataQoS());
    detection_pub = peer->create_publisher<factory_interfaces::msg::StationDetection>("/factory/station_detection", rclcpp::SensorDataQoS());
    events_sub = peer->create_subscription<factory_interfaces::msg::ProtocolEvent>(
      "/factory/protocol_events", rclcpp::QoS(100).reliable(),
      [this](factory_interfaces::msg::ProtocolEvent::SharedPtr event) {events.push_back(*event);});
    states_sub=peer->create_subscription<std_msgs::msg::String>("/factory/mission_state",rclcpp::QoS(100).reliable(),
      [this](std_msgs::msg::String::SharedPtr message) {states.push_back(message->data);});
    nav = rclcpp_action::create_server<Nav>(peer, "/navigate_to_pose",
      [](auto, auto) {return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
      [](auto) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](auto handle) {
        nav_handles.push_back(handle);
        clears_on_navigation.push_back(clear_count);
        target = handle->get_goal()->pose;
        if (!hold_navigation) {
          auto result = std::make_shared<Nav::Result>();
          if (fail_navigation-- > 0) {handle->abort(result);} else {handle->succeed(result);}
        }
      });
    for (const auto & name : {"/local_costmap/clear_entirely_local_costmap", "/global_costmap/clear_entirely_global_costmap"}) {
      clears.push_back(peer->create_service<nav2_msgs::srv::ClearEntireCostmap>(name,
        [this](std::shared_ptr<nav2_msgs::srv::ClearEntireCostmap::Request>, std::shared_ptr<nav2_msgs::srv::ClearEntireCostmap::Response>) {++clear_count;}));
    }
    transfer = peer->create_service<factory_interfaces::srv::TransferPart>("/factory/transfer_part",
      [this](std::shared_ptr<rmw_request_id_t> header, std::shared_ptr<factory_interfaces::srv::TransferPart::Request> request) {
        transfers.push_back(*request);
        if (request->station_id==hold_transfer_station) {pending_transfer=header; return;}
        auto response=std::make_shared<factory_interfaces::srv::TransferPart::Response>();
        response->accepted = transfer_error.empty();
        response->error_code = transfer_error;
        response->message = "fake PLC completion";
        transfer->send_response(*header, *response);
      });
    client = rclcpp_action::create_client<Mission>(peer, "/factory/execute_mission");
    executor.add_node(coordinator);
    executor.add_node(peer);
    ASSERT_TRUE(client->wait_for_action_server(2s));
    pump(20);
  }
  void TearDown() override {
    executor.remove_node(coordinator);
    executor.remove_node(peer);
    coordinator.reset(); peer.reset();
  }
  void pump(int count=1, bool detect=true, bool localized=true, int marker=0) {
    for (int i=0; i<count; ++i) {
      sim_time += 0.05;
      builtin_interfaces::msg::Time stamp = rclcpp::Time(static_cast<int64_t>(sim_time * 1e9));
      rosgraph_msgs::msg::Clock clock; clock.clock=stamp; clock_pub->publish(clock);
      geometry_msgs::msg::PoseWithCovarianceStamped pose;
      pose.header.stamp=stamp; pose.header.frame_id="map";
      pose.pose.pose=target.pose;
      if (!localized) {pose.pose.pose.position.x += 3.0;}
      if (invalid_pose) {pose.pose.pose.position.z=std::numeric_limits<double>::quiet_NaN();}
      pose_pub->publish(pose);
      if (detect && !feedback.empty() && feedback.back().find("VERIFYING") == 0) {
        factory_interfaces::msg::StationDetection detection;
        detection.header.stamp=stamp;
        if (repeated_stamp<0) {detection.header.stamp.sec=-1; detection.header.stamp.nanosec=0;}
        else if (repeated_stamp) {detection.header.stamp=rclcpp::Time(static_cast<int64_t>(repeated_stamp*1e9));}
        bool pickup=feedback.back()=="VERIFYING_PICKUP";
        detection.station_id=pickup ? "assembly" : "inspection";
        detection.marker_id=marker ? marker : (pickup ? 10 : 20);
        detection.confidence=0.5;
        detection_pub->publish(detection);
      }
      executor.spin_some();
      std::this_thread::sleep_for(5ms);
    }
  }
  std::shared_ptr<rclcpp_action::ClientGoalHandle<Mission>> send(std::string id="test", std::string robot="amr_01", std::string pickup="assembly", std::string dropoff="inspection", std::string part="motor") {
    Mission::Goal goal;
    goal.mission_id=id; goal.robot_id=robot; goal.pickup_station=pickup;
    goal.dropoff_station=dropoff; goal.part=part;
    rclcpp_action::Client<Mission>::SendGoalOptions options;
    options.feedback_callback=[this](auto, auto message) {feedback.push_back(message->state); progress.push_back(message->progress);};
    auto future=client->async_send_goal(goal, options);
    for (int i=0; i<200 && future.wait_for(0s)!=std::future_status::ready; ++i) {pump();}
    EXPECT_EQ(future.wait_for(0s), std::future_status::ready);
    if (future.wait_for(0s)!=std::future_status::ready) {throw std::runtime_error("goal acceptance timeout");}
    return future.get();
  }
  rclcpp_action::ClientGoalHandle<Mission>::WrappedResult finish(std::shared_ptr<rclcpp_action::ClientGoalHandle<Mission>> handle, bool detect=true, bool localized=true, int marker=0) {
    auto future=client->async_get_result(handle);
    for (int i=0; i<300 && future.wait_for(0s)!=std::future_status::ready; ++i) {pump(1, detect, localized, marker);}
    EXPECT_EQ(future.wait_for(0s), std::future_status::ready);
    if (future.wait_for(0s)!=std::future_status::ready) {throw std::runtime_error("mission result timeout");}
    return future.get();
  }
  rclcpp::executors::SingleThreadedExecutor executor;
  std::shared_ptr<mission_coordinator::MissionCoordinatorNode> coordinator;
  rclcpp::Node::SharedPtr peer;
  rclcpp_action::Client<Mission>::SharedPtr client;
  rclcpp_action::Server<Nav>::SharedPtr nav;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub;
  rclcpp::Publisher<factory_interfaces::msg::StationDetection>::SharedPtr detection_pub;
  rclcpp::Subscription<factory_interfaces::msg::ProtocolEvent>::SharedPtr events_sub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr states_sub;
  rclcpp::Service<factory_interfaces::srv::TransferPart>::SharedPtr transfer;
  std::vector<rclcpp::Service<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr> clears;
  std::vector<std::shared_ptr<rclcpp_action::ServerGoalHandle<Nav>>> nav_handles;
  std::vector<int> clears_on_navigation;
  std::vector<factory_interfaces::srv::TransferPart::Request> transfers;
  std::vector<factory_interfaces::msg::ProtocolEvent> events;
  std::vector<std::string> feedback;
  std::vector<float> progress;
  std::vector<std::string> states;
  geometry_msgs::msg::PoseStamped target;
  double sim_time=10.0;
  int fail_navigation=0, clear_count=0;
  bool hold_navigation=false;
  bool invalid_pose=false;
  std::string transfer_error;
  std::string hold_transfer_station;
  std::shared_ptr<rmw_request_id_t> pending_transfer;
  double repeated_stamp=0;
};

TEST_F(CoordinatorTest, ValidMissionCompletesWithOrderedFeedbackAndPhases) {
  auto handle=send(); ASSERT_NE(handle, nullptr);
  auto result=finish(handle);
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_TRUE(result.result->success);
  EXPECT_EQ(result.result->final_state, "COMPLETED");
  EXPECT_EQ(transfers.size(), 2u);
  pump(10);
  const std::vector<std::string> expected={"RECEIVED", "NAVIGATING_TO_PICKUP", "VERIFYING_PICKUP", "LOADING", "NAVIGATING_TO_DROPOFF", "VERIFYING_DROPOFF", "UNLOADING", "COMPLETED"};
  EXPECT_EQ(states,expected);
  std::vector<std::string> event_states;
  for (const auto & e:events) {if(e.event=="state_changed") {event_states.push_back(e.detail);}}
  EXPECT_EQ(event_states,expected);
  // Humble drops feedback arriving before the goal response registers its ID.
  ASSERT_GE(feedback.size(),3u);
  auto position=expected.begin();
  for (const auto & state:feedback) {
    position=std::find(position,expected.end(),state);
    ASSERT_NE(position,expected.end()); ++position;
  }
  EXPECT_NE(std::find(feedback.begin(),feedback.end(),"LOADING"),feedback.end());
  EXPECT_NE(std::find(feedback.begin(),feedback.end(),"UNLOADING"),feedback.end());
  EXPECT_EQ(std::count_if(events.begin(), events.end(), [](auto e) {return e.event=="mission_finished";}), 1);
}
TEST_F(CoordinatorTest, RejectsInvalidAndBusyGoals) {
  EXPECT_EQ(send("bad", "other"), nullptr);
  EXPECT_EQ(send("bad", "amr_01", "inspection"), nullptr);
  EXPECT_EQ(send("bad", "amr_01", "assembly", "assembly"), nullptr);
  EXPECT_EQ(send("bad", "amr_01", "assembly", "inspection", "other"), nullptr);
  EXPECT_EQ(send(std::string(65,'a')), nullptr);
  hold_navigation=true;
  ASSERT_NE(send("first"), nullptr);
  EXPECT_EQ(send("second"), nullptr);
  pump(10);
  EXPECT_TRUE(std::any_of(events.begin(), events.end(), [](auto e) {return e.mission_id=="second" && e.detail=="ROBOT_BUSY";}));
}
TEST_F(CoordinatorTest, CompletedPayloadRequiresRestartBeforeAnotherExecution) {
  ASSERT_TRUE(finish(send("first")).result->success);
  auto second=send("second"); ASSERT_NE(second,nullptr);
  auto result=finish(second);
  EXPECT_EQ(result.code,rclcpp_action::ResultCode::ABORTED);
  EXPECT_FALSE(result.result->success);
  EXPECT_EQ(result.result->final_state,"FAILED");
  EXPECT_EQ(result.result->error_code,"RESTART_REQUIRED");
  EXPECT_EQ(result.result->message,"Restart the full simulation before another payload mission");
  pump(10);
  EXPECT_EQ(transfers.size(),2u);
  EXPECT_EQ(nav_handles.size(),2u);
  EXPECT_EQ(std::count_if(events.begin(),events.end(),[](auto e){return e.event=="mission_started";}),1);
  EXPECT_TRUE(std::any_of(events.begin(),events.end(),[](auto e){return e.mission_id=="second" && e.event=="mission_rejected" && e.detail=="RESTART_REQUIRED";}));
}
TEST_F(CoordinatorTest, PrePickupTransferFailureAllowsNewMission) {
  transfer_error="PLC_FAULT";
  EXPECT_EQ(finish(send("failed")).result->error_code,"PLC_FAULT");
  transfer_error.clear();
  EXPECT_TRUE(finish(send("retry")).result->success);
  EXPECT_EQ(transfers.size(),3u);
}
TEST_F(CoordinatorTest, CancellationAfterSuccessfulPickupRequiresRestart) {
  hold_navigation=true;
  auto handle=send("first");
  for(int i=0;i<50 && nav_handles.empty();++i) {pump();}
  ASSERT_EQ(nav_handles.size(),1u);
  nav_handles.front()->succeed(std::make_shared<Nav::Result>());
  for(int i=0;i<100 && nav_handles.size()<2;++i) {pump();}
  ASSERT_EQ(transfers.size(),1u);
  ASSERT_EQ(nav_handles.size(),2u);
  client->async_cancel_goal(handle); pump(10);
  EXPECT_EQ(finish(handle).result->error_code,"MISSION_CANCELED");
  auto result=finish(send("second"));
  EXPECT_EQ(result.code,rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(result.result->error_code,"RESTART_REQUIRED");
  EXPECT_EQ(transfers.size(),1u);
  EXPECT_EQ(nav_handles.size(),2u);
}
TEST_F(CoordinatorTest, ClearsBothCostmapsBeforeSoleRetry) {
  fail_navigation=1;
  auto result=finish(send());
  EXPECT_TRUE(result.result->success);
  EXPECT_EQ(clear_count, 2);
  EXPECT_EQ(nav_handles.size(), 3u);
  EXPECT_EQ(clears_on_navigation,(std::vector<int>{0,2,2}));
}
TEST_F(CoordinatorTest, SecondNavigationFailureIsTerminal) {
  fail_navigation=2;
  auto result=finish(send());
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(result.result->error_code, "NAVIGATION_FAILED");
  EXPECT_TRUE(transfers.empty());
}
TEST_F(CoordinatorTest, WrongMarkerNeverContactsPLC) {
  auto result=finish(send(), true, true, 20);
  EXPECT_EQ(result.result->error_code, "STATION_NOT_CONFIRMED");
  EXPECT_TRUE(transfers.empty());
}
TEST_F(CoordinatorTest, LocalizationMismatchNeverContactsPLC) {
  auto result=finish(send(), true, false);
  EXPECT_EQ(result.result->error_code, "STATION_NOT_CONFIRMED");
  EXPECT_TRUE(transfers.empty());
  EXPECT_EQ(std::find(states.begin(),states.end(),"VERIFYING_PICKUP"),states.end());
}
TEST_F(CoordinatorTest, PropagatesTransferFailure) {
  transfer_error="STALE_PLC_STATE";
  auto result=finish(send());
  EXPECT_EQ(result.result->error_code, "STALE_PLC_STATE");
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_TRUE(std::all_of(progress.begin(),progress.end(),[](float value){return value>=0.0f && value<=1.0f;}));
}
TEST_F(CoordinatorTest, NonfiniteLocalizationNeverStartsVerification) {
  invalid_pose=true;
  auto result=finish(send());
  EXPECT_EQ(result.result->error_code,"STATION_NOT_CONFIRMED");
  EXPECT_TRUE(transfers.empty());
}
TEST_F(CoordinatorTest, CancelsNavigationAndRejectsStaleCompletion) {
  hold_navigation=true;
  auto handle=send("cancel");
  pump(10);
  auto cancellation=client->async_cancel_goal(handle);
  pump(20);
  auto result=finish(handle);
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_EQ(result.result->error_code, "MISSION_CANCELED");
  auto old=nav_handles.front();
  if (old->is_active()) {old->succeed(std::make_shared<Nav::Result>());}
  hold_navigation=false;
  EXPECT_TRUE(finish(send("new")).result->success);
}
TEST_F(CoordinatorTest, RepeatedImageTimestampsNeverConfirmStation) {
  auto handle=send();
  pump(10, false);
  repeated_stamp=sim_time;
  auto result=finish(handle);
  EXPECT_EQ(result.result->error_code,"STATION_NOT_CONFIRMED");
  EXPECT_TRUE(transfers.empty());
}
TEST_F(CoordinatorTest, OldAndFutureImageTimestampsNeverConfirmStation) {
  auto handle=send("old");
  repeated_stamp=1;
  EXPECT_EQ(finish(handle).result->error_code,"STATION_NOT_CONFIRMED");
  repeated_stamp=sim_time+100;
  EXPECT_EQ(finish(send("future")).result->error_code,"STATION_NOT_CONFIRMED");
  EXPECT_TRUE(transfers.empty());
}
TEST_F(CoordinatorTest, StationaryLocalizationWaitsForClockThenRemainsUsable) {
  hold_navigation=true;
  auto handle=send("clock_pose");
  pump(10,false,false);
  const double source_stamp=sim_time+0.08;
  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.header.frame_id="map";
  pose.header.stamp=rclcpp::Time(static_cast<int64_t>(source_stamp*1e9));
  pose.pose.pose=target.pose;
  pose_pub->publish(pose);
  for (int i=0;i<20;++i) {executor.spin_some(); std::this_thread::sleep_for(5ms);}
  nav_handles.front()->succeed(std::make_shared<Nav::Result>());
  for (int i=0;i<20;++i) {executor.spin_some(); std::this_thread::sleep_for(5ms);}
  EXPECT_EQ(std::find(states.begin(),states.end(),"VERIFYING_PICKUP"),states.end());
  EXPECT_TRUE(transfers.empty());
  sim_time=source_stamp+0.02;
  rosgraph_msgs::msg::Clock clock;
  clock.clock=rclcpp::Time(static_cast<int64_t>(sim_time*1e9));
  clock_pub->publish(clock);
  for (int i=0;i<20;++i) {executor.spin_some(); std::this_thread::sleep_for(5ms);}
  EXPECT_NE(std::find(states.begin(),states.end(),"VERIFYING_PICKUP"),states.end());
  EXPECT_TRUE(transfers.empty());
  hold_navigation=false;
  EXPECT_TRUE(finish(handle).result->success);
}
TEST_F(CoordinatorTest, FutureImagesWaitForClockWithoutEarlyTransfer) {
  hold_transfer_station="assembly";
  auto handle=send("clock_images");
  for (int i=0;i<50 && std::find(states.begin(),states.end(),"VERIFYING_PICKUP")==states.end();++i) {pump(1,false);}
  ASSERT_NE(std::find(states.begin(),states.end(),"VERIFYING_PICKUP"),states.end());
  for (int i=1;i<=5;++i) {
    factory_interfaces::msg::StationDetection image;
    image.header.stamp=rclcpp::Time(static_cast<int64_t>((sim_time+i*0.02)*1e9));
    image.station_id="assembly"; image.marker_id=10;
    detection_pub->publish(image);
    for (int j=0;j<4;++j) {executor.spin_some(); std::this_thread::sleep_for(5ms);}
  }
  EXPECT_TRUE(transfers.empty());
  sim_time+=0.12;
  rosgraph_msgs::msg::Clock clock;
  clock.clock=rclcpp::Time(static_cast<int64_t>(sim_time*1e9));
  clock_pub->publish(clock);
  for (int i=0;i<30;++i) {executor.spin_some(); std::this_thread::sleep_for(5ms);}
  ASSERT_EQ(transfers.size(),1u);
  ASSERT_NE(pending_transfer,nullptr);
  factory_interfaces::srv::TransferPart::Response response; response.accepted=true;
  transfer->send_response(*pending_transfer,response);
  EXPECT_TRUE(finish(handle).result->success);
}
TEST_F(CoordinatorTest, FarFutureImagesTimeOutAndDoNotSurviveReset) {
  auto handle=send("far_future");
  for (int i=0;i<50 && std::find(states.begin(),states.end(),"VERIFYING_PICKUP")==states.end();++i) {pump(1,false);}
  ASSERT_NE(std::find(states.begin(),states.end(),"VERIFYING_PICKUP"),states.end());
  for (int i=0;i<8;++i) {
    factory_interfaces::msg::StationDetection image;
    image.header.stamp=rclcpp::Time(static_cast<int64_t>((sim_time+100+i*0.02)*1e9));
    image.station_id="assembly"; image.marker_id=10;
    detection_pub->publish(image);
    for (int j=0;j<4;++j) {executor.spin_some(); std::this_thread::sleep_for(5ms);}
  }
  EXPECT_EQ(finish(handle,false).result->error_code,"STATION_NOT_CONFIRMED");
  EXPECT_TRUE(transfers.empty());
  EXPECT_TRUE(finish(send("after_future_reset")).result->success);
  EXPECT_EQ(transfers.size(),2u);
}
TEST_F(CoordinatorTest, InvalidQueuedImagesClearPartialConfirmation) {
  for (const auto & invalid : {"wrong", "repeated", "backward", "negative"}) {
    const auto count=states.size();
    auto handle=send(std::string("queued_")+invalid);
    for (int i=0;i<50 && std::find(states.begin()+count,states.end(),"VERIFYING_PICKUP")==states.end();++i) {pump(1,false);}
    ASSERT_NE(std::find(states.begin()+count,states.end(),"VERIFYING_PICKUP"),states.end());
    auto publish=[this](double source,int marker=10) {
      factory_interfaces::msg::StationDetection image;
      image.header.stamp=rclcpp::Time(static_cast<int64_t>(source*1e9));
      image.station_id="assembly"; image.marker_id=marker;
      detection_pub->publish(image);
      for (int j=0;j<4;++j) {executor.spin_some(); std::this_thread::sleep_for(5ms);}
    };
    for (int i=1;i<=4;++i) {publish(sim_time+i*0.02);}
    if (std::string(invalid)=="wrong") {publish(sim_time+0.10,20);}
    else if (std::string(invalid)=="repeated") {publish(sim_time+0.08);}
    else if (std::string(invalid)=="backward") {publish(sim_time+0.07);}
    else {
      factory_interfaces::msg::StationDetection image;
      image.header.stamp.sec=-1; image.station_id="assembly"; image.marker_id=10;
      detection_pub->publish(image);
      for (int j=0;j<4;++j) {executor.spin_some(); std::this_thread::sleep_for(5ms);}
    }
    publish(sim_time+0.12);
    sim_time+=0.14;
    rosgraph_msgs::msg::Clock clock;
    clock.clock=rclcpp::Time(static_cast<int64_t>(sim_time*1e9));
    clock_pub->publish(clock);
    for (int i=0;i<20;++i) {executor.spin_some(); std::this_thread::sleep_for(5ms);}
    EXPECT_TRUE(transfers.empty()) << invalid;
    EXPECT_EQ(finish(handle,false).result->error_code,"STATION_NOT_CONFIRMED") << invalid;
  }
}
TEST_F(CoordinatorTest, NegativeImageTimestampsAreRejectedWithoutException) {
  repeated_stamp=-1;
  auto result=finish(send());
  EXPECT_EQ(result.result->error_code,"STATION_NOT_CONFIRMED");
  EXPECT_TRUE(transfers.empty());
}
TEST_F(CoordinatorTest, WrongMarkerAdvancesWatermarkBeforeBackdatedImages) {
  auto handle=send();
  for (int i=0; i<50 && (feedback.empty() || feedback.back()!="VERIFYING_PICKUP"); ++i) {pump(1,false);}
  ASSERT_EQ(feedback.back(),"VERIFYING_PICKUP");
  auto publish=[this](double stamp,int marker) {
    factory_interfaces::msg::StationDetection message;
    message.header.stamp=rclcpp::Time(static_cast<int64_t>(stamp*1e9));
    message.station_id="assembly"; message.marker_id=marker;
    detection_pub->publish(message);
    pump(1,false);
  };
  for (int i=0;i<4;++i) {pump(1,false);publish(sim_time,10);}
  const double last_correct=sim_time;
  pump(1,false);publish(sim_time,20);
  for (int i=1;i<=5;++i) {publish(last_correct+i*0.008,10);}
  auto result=finish(handle,false);
  EXPECT_EQ(result.result->error_code,"STATION_NOT_CONFIRMED");
  EXPECT_TRUE(transfers.empty());
}
TEST_F(CoordinatorTest, PausedSimulationClockDoesNotExpireVerification) {
  auto handle=send();
  for (int i=0; i<50 && (feedback.empty() || feedback.back()!="VERIFYING_PICKUP"); ++i) {pump(1,false);}
  ASSERT_EQ(feedback.back(),"VERIFYING_PICKUP");
  auto result=client->async_get_result(handle);
  const auto deadline=std::chrono::steady_clock::now()+2500ms;
  while(std::chrono::steady_clock::now()<deadline) {executor.spin_some(); std::this_thread::sleep_for(5ms);}
  EXPECT_NE(result.wait_for(0s),std::future_status::ready);
  EXPECT_TRUE(transfers.empty());
  EXPECT_TRUE(finish(handle).result->success);
}
TEST_F(CoordinatorTest, FiveImagesOutsideWindowNeverConfirmStation) {
  auto handle=send();
  for (int i=0; i<50 && (feedback.empty() || feedback.back()!="VERIFYING_PICKUP"); ++i) {pump(1,false);}
  ASSERT_EQ(feedback.back(),"VERIFYING_PICKUP");
  for(int i=0;i<5;++i) {
    factory_interfaces::msg::StationDetection message;
    message.header.stamp=rclcpp::Time(static_cast<int64_t>(sim_time*1e9));
    message.station_id="assembly"; message.marker_id=10;
    detection_pub->publish(message);
    pump(i==4 ? 1 : 8,false);
  }
  EXPECT_TRUE(transfers.empty());
  EXPECT_EQ(finish(handle,false).result->error_code,"STATION_NOT_CONFIRMED");
}
TEST_F(CoordinatorTest, SuccessfulTransferCompletesPendingCancelAsRosCanceled) {
  hold_transfer_station="assembly";
  auto handle=send();
  for (int i=0; i<100 && !pending_transfer; ++i) {pump();}
  ASSERT_NE(pending_transfer,nullptr);
  auto result=client->async_get_result(handle);
  client->async_cancel_goal(handle); pump(10);
  EXPECT_NE(result.wait_for(0s),std::future_status::ready);
  factory_interfaces::srv::TransferPart::Response response; response.accepted=true;
  transfer->send_response(*pending_transfer,response);
  auto finished=finish(handle);
  EXPECT_EQ(finished.code,rclcpp_action::ResultCode::CANCELED);
  EXPECT_EQ(finished.result->error_code,"MISSION_CANCELED");
  EXPECT_EQ(transfers.size(),1u);
  hold_transfer_station.clear();
  EXPECT_EQ(finish(send("after_cancel")).result->error_code,"RESTART_REQUIRED");
  EXPECT_EQ(transfers.size(),1u);
}
TEST_F(CoordinatorTest, FailedTransferPreservesPlcErrorDuringPendingCancel) {
  hold_transfer_station="inspection";
  auto handle=send();
  for (int i=0; i<200 && !pending_transfer; ++i) {pump();}
  ASSERT_NE(pending_transfer,nullptr);
  client->async_cancel_goal(handle); pump(10);
  factory_interfaces::srv::TransferPart::Response response;
  response.accepted=false; response.error_code="PLC_FAULT";
  transfer->send_response(*pending_transfer,response);
  auto finished=finish(handle);
  EXPECT_EQ(finished.code,rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(finished.result->error_code,"PLC_FAULT");
  hold_transfer_station.clear();
  EXPECT_EQ(finish(send("after_failure")).result->error_code,"RESTART_REQUIRED");
  EXPECT_EQ(transfers.size(),2u);
}
