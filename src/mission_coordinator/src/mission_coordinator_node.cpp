// SPDX-License-Identifier: Apache-2.0
#include "mission_coordinator/mission_coordinator_node.hpp"
#include <algorithm>
#include <cmath>
#include <regex>
#include <stdexcept>

namespace mission_coordinator {
MissionCoordinatorNode::MissionCoordinatorNode(const rclcpp::NodeOptions & options)
: Node("mission_coordinator", options), navigation_(this) {
  if (!has_parameter("use_sim_time")) {declare_parameter("use_sim_time", true);} else {set_parameter({"use_sim_time", true});}
  poses_["assembly"]=declare_parameter<std::vector<double>>("stations.assembly.pose", {-3.0, 0.8, 1.5707963267948966});
  poses_["inspection"]=declare_parameter<std::vector<double>>("stations.inspection.pose", {3.0, 0.8, 1.5707963267948966});
  for (const auto & pair : poses_) {
    if (pair.second.size()!=3 || !std::all_of(pair.second.begin(), pair.second.end(), [](double v){return std::isfinite(v);})) {throw std::invalid_argument("station pose requires three finite values");}
  }
  timeout_=declare_parameter("perception_timeout_sec", 10.0);
  navigation_timeout_=declare_parameter("navigation_timeout_sec", 120.0);
  if (!std::isfinite(timeout_) || timeout_<=0 || !std::isfinite(navigation_timeout_) || navigation_timeout_<=0) {throw std::invalid_argument("timeouts must be finite and positive");}
  events_=create_publisher<factory_interfaces::msg::ProtocolEvent>("/factory/protocol_events", rclcpp::QoS(100).reliable());
  states_=create_publisher<std_msgs::msg::String>("/factory/mission_state", rclcpp::QoS(100).reliable());
  transfer_client_=create_client<Transfer>("/factory/transfer_part");
  localization_=create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose", rclcpp::SensorDataQoS(),
    [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message) {
      const auto & p=message->pose.pose;
      if (message->header.stamp.sec<0 || message->header.stamp.nanosec>=1000000000u) {pose_valid_=false; return;}
      const double stamp=rclcpp::Time(message->header.stamp).seconds();
      pose_valid_=goal_ && stamp>=localization_started_ && stamp<=now().seconds() && message->header.frame_id=="map" && std::isfinite(p.position.x) && std::isfinite(p.position.y) && std::isfinite(p.position.z) &&
        std::isfinite(p.orientation.x) && std::isfinite(p.orientation.y) && std::isfinite(p.orientation.z) && std::isfinite(p.orientation.w) &&
        std::abs(p.orientation.x*p.orientation.x+p.orientation.y*p.orientation.y+p.orientation.z*p.orientation.z+p.orientation.w*p.orientation.w-1.0)<0.01;
      pose_=p;
    });
  detections_=create_subscription<factory_interfaces::msg::StationDetection>("/factory/station_detection", rclcpp::SensorDataQoS(),
    [this](factory_interfaces::msg::StationDetection::SharedPtr message) {detection(*message);});
  server_=rclcpp_action::create_server<Mission>(this, "/factory/execute_mission",
    [this](auto, auto request) {
      std::string error;
      if (!std::regex_match(request->mission_id, std::regex("[A-Za-z0-9_-]{1,64}")) || request->robot_id!="amr_01" || request->pickup_station!="assembly" || request->dropoff_station!="inspection" || request->part!="motor") {error="INVALID_MISSION";}
      else if (reserved_) {error="ROBOT_BUSY";}
      if (!error.empty()) {event("mission_rejected", "FAILED", error, request->mission_id); return rclcpp_action::GoalResponse::REJECT;}
      reserved_=true;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [this](auto handle) {
      if (handle!=goal_) {return rclcpp_action::CancelResponse::REJECT;}
      auto result=machine_.cancel();
      if (result.state==MissionState::Failed) {navigation_.cancel(); deferred_error_="MISSION_CANCELED";}
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [this](auto handle) {
      goal_=handle; ++mission_generation_; pose_valid_=false;
      auto request=handle->get_goal();
      event("mission_started");
      transition(machine_.start({request->mission_id, request->robot_id, request->pickup_station, request->dropoff_station, request->part}));
      transition(machine_.begin_navigation()); navigate();
    });
  timer_=create_wall_timer(std::chrono::milliseconds(20), [this] {tick();});
}
std::string MissionCoordinatorNode::state_name() const {
  static const std::vector<std::string> names={"IDLE","RECEIVED","NAVIGATING_TO_PICKUP","VERIFYING_PICKUP","LOADING","NAVIGATING_TO_DROPOFF","VERIFYING_DROPOFF","UNLOADING","RECOVERING","COMPLETED","FAILED"};
  return names.at(static_cast<size_t>(machine_.state()));
}
std::string MissionCoordinatorNode::phase(const std::string & prefix) const {
  return prefix+(machine_.current_station()=="assembly" ? "_pickup" : "_dropoff");
}
void MissionCoordinatorNode::event(const std::string & name, const std::string & outcome, const std::string & detail, const std::string & mission) {
  factory_interfaces::msg::ProtocolEvent message;
  message.stamp=now(); message.mission_id=mission.empty() && goal_ ? goal_->get_goal()->mission_id : mission;
  message.protocol="ROS"; message.direction="INTERNAL"; message.event=name; message.outcome=outcome; message.detail=detail;
  events_->publish(message);
}
void MissionCoordinatorNode::transition(const TransitionResult & result) {
  if (!result.accepted || !goal_) {return;}
  RCLCPP_INFO(get_logger(), "mission_id=%s robot_id=%s station=%s state=%s error_code=%s",
    machine_.mission().mission_id.c_str(), machine_.mission().robot_id.c_str(), machine_.current_station().c_str(), state_name().c_str(), result.error_code.c_str());
  auto feedback=std::make_shared<Mission::Feedback>();
  feedback->state=state_name(); feedback->station=machine_.current_station(); feedback->detail=result.error_code;
  feedback->progress=std::min(1.0f, static_cast<float>(static_cast<int>(result.state))/9.0f);
  goal_->publish_feedback(feedback);
  std_msgs::msg::String state; state.data=state_name(); states_->publish(state);
  event("state_changed", "", state_name());
}
void MissionCoordinatorNode::navigate() {
  navigation_completed_=false;
  pose_valid_=false; stamps_.clear(); last_stamp_=-1;
  active_phase_=phase("navigation"); phase_started_=now().seconds(); event(active_phase_+"_started");
  localization_started_=phase_started_;
  auto coordinates=poses_.at(machine_.current_station());
  geometry_msgs::msg::PoseStamped pose; pose.header.frame_id="map"; pose.header.stamp=now();
  pose.pose.position.x=coordinates[0]; pose.pose.position.y=coordinates[1];
  pose.pose.orientation.z=std::sin(coordinates[2]/2); pose.pose.orientation.w=std::cos(coordinates[2]/2);
  const auto generation=mission_generation_;
  navigation_.navigate(pose, [this,generation](bool success) {
    if (!goal_ || generation!=mission_generation_ || active_phase_.find("navigation_")!=0) {return;}
    event(active_phase_+"_finished", success ? "SUCCEEDED" : "FAILED"); active_phase_.clear();
    if (success) {
      navigation_completed_=true; phase_started_=now().seconds();
      if (localized()) {verify_station();}
    } else {
      auto result=machine_.navigation_failed(); transition(result);
      if (result.state==MissionState::Failed) {finish(result.error_code); return;}
      phase_started_=now().seconds();
      navigation_.clear_costmaps([this,generation](bool cleared) {
        if (!goal_ || generation!=mission_generation_ || machine_.state()!=MissionState::Recovering) {return;}
        if (!cleared) {finish("NAVIGATION_FAILED"); return;}
        event("retry"); transition(machine_.recovery_ready()); navigate();
      });
    }
  });
}
void MissionCoordinatorNode::verify_station() {
  navigation_completed_=false;
  transition(machine_.navigation_succeeded());
  active_phase_=phase("perception"); phase_started_=now().seconds(); stamps_.clear(); last_stamp_=-1;
  event(active_phase_+"_started");
}
bool MissionCoordinatorNode::localized() const {
  if (!pose_valid_) {return false;}
  auto target=poses_.at(machine_.current_station());
  double yaw=std::atan2(2*(pose_.orientation.w*pose_.orientation.z+pose_.orientation.x*pose_.orientation.y), 1-2*(pose_.orientation.y*pose_.orientation.y+pose_.orientation.z*pose_.orientation.z));
  return std::hypot(pose_.position.x-target[0],pose_.position.y-target[1])<=0.25 && std::abs(std::remainder(yaw-target[2],2*M_PI))<=0.25;
}
void MissionCoordinatorNode::detection(const factory_interfaces::msg::StationDetection & message) {
  if (!goal_ || (machine_.state()!=MissionState::VerifyingPickup && machine_.state()!=MissionState::VerifyingDropoff)) {return;}
  if (message.header.stamp.sec<0 || message.header.stamp.nanosec>=1000000000u) {stamps_.clear(); return;}
  const double stamp=rclcpp::Time(message.header.stamp).seconds(), current=now().seconds();
  const int marker=machine_.current_station()=="assembly" ? 10 : 20;
  if (stamp<phase_started_ || stamp>current || current-stamp>1.5 || stamp<=last_stamp_) {stamps_.clear(); return;}
  last_stamp_=stamp;
  if (message.station_id!=machine_.current_station() || message.marker_id!=marker) {stamps_.clear(); return;}
  stamps_.push_back(stamp);
  while (!stamps_.empty() && stamp-stamps_.front()>1.5) {stamps_.pop_front();}
  while (stamps_.size()>5) {stamps_.pop_front();}
  if (stamps_.size()>=5 && localized()) {
    event(active_phase_+"_finished", "SUCCEEDED"); active_phase_.clear();
    transition(machine_.station_confirmed()); transfer();
  }
}
void MissionCoordinatorNode::transfer() {
  if (!transfer_client_->service_is_ready()) {transition(machine_.transfer_failed("PLC_TIMEOUT")); finish("PLC_TIMEOUT"); return;}
  auto request=std::make_shared<Transfer::Request>(); request->mission_id=machine_.mission().mission_id;
  request->station_id=machine_.current_station(); request->part=machine_.mission().part;
  const auto generation=mission_generation_; phase_started_=now().seconds();
  try {transfer_client_->async_send_request(request, [this,generation](rclcpp::Client<Transfer>::SharedFuture future) {
    if (!goal_ || generation!=mission_generation_) {return;}
    TransitionResult result;
    try {auto response=future.get(); event("transfer_result",response->accepted ? "SUCCEEDED" : "FAILED",response->error_code); result=response->accepted ? machine_.transfer_succeeded() : machine_.transfer_failed(response->error_code.empty() ? "PLC_TIMEOUT" : response->error_code);}
    catch (const std::exception & error) {RCLCPP_ERROR(get_logger(), "Mission %s: %s", machine_.mission().mission_id.c_str(),error.what()); result=machine_.transfer_failed("PLC_TIMEOUT");}
    transition(result);
    if (result.state==MissionState::Completed || result.state==MissionState::Failed) {finish(result.error_code);} else {navigate();}
  });} catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Mission %s: transfer dispatch failed: %s", request->mission_id.c_str(), error.what());
    transition(machine_.transfer_failed("PLC_TIMEOUT")); finish("PLC_TIMEOUT");
  }
}
void MissionCoordinatorNode::tick() {
  if (!goal_) {return;}
  if (!deferred_error_.empty()) {auto error=deferred_error_; deferred_error_.clear(); transition({true, machine_.state(),error}); finish(error); return;}
  const double elapsed=now().seconds()-phase_started_;
  if (navigation_completed_) {
    if (localized()) {verify_station();}
    else if (elapsed>=timeout_ || elapsed<0) {finish("STATION_NOT_CONFIRMED");}
    return;
  }
  if (machine_.state()==MissionState::VerifyingPickup || machine_.state()==MissionState::VerifyingDropoff) {
    if (elapsed>=timeout_ || elapsed<0) {auto result=machine_.perception_timed_out(); transition(result); finish(result.error_code);}
  } else if ((machine_.state()==MissionState::Recovering || active_phase_.find("navigation_")==0) && (elapsed>=navigation_timeout_ || elapsed<0)) {finish("NAVIGATION_FAILED");}
}
void MissionCoordinatorNode::finish(const std::string & error) {
  if (!goal_) {return;}
  if (!error.empty() && machine_.state()!=MissionState::Failed) {
    auto result=(machine_.state()==MissionState::Loading || machine_.state()==MissionState::Unloading) ? machine_.transfer_failed(error) : machine_.cancel();
    result.error_code=error; transition(result);
  }
  if (!active_phase_.empty()) {event(active_phase_+"_finished", "FAILED", error); active_phase_.clear();}
  navigation_.cancel();
  auto result=std::make_shared<Mission::Result>(); result->success=error.empty(); result->final_state=error.empty() ? "COMPLETED" : "FAILED"; result->error_code=error; result->message=error.empty() ? "Mission completed" : error;
  event("mission_finished", result->final_state, error);
  if (error.empty()) {goal_->succeed(result);} else if (error=="MISSION_CANCELED" && goal_->is_canceling()) {goal_->canceled(result);} else {goal_->abort(result);}
  goal_.reset(); reserved_=false; pose_valid_=false; navigation_completed_=false; ++mission_generation_;
  machine_.reset();
}
}  // namespace mission_coordinator
