// SPDX-License-Identifier: Apache-2.0
#include "mission_coordinator/mission_coordinator_node.hpp"
#include "mission_coordinator/resource_ros_transport.hpp"
#include <algorithm>
#include <cmath>
#include <regex>
#include <stdexcept>
#include <iomanip>
#include <sstream>

namespace mission_coordinator {
namespace {
double wall_time() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
RobotContext local_robot(rclcpp::Node *node) {
  rcl_interfaces::msg::ParameterDescriptor identity;
  identity.read_only = true;
  const auto id = node->declare_parameter<std::string>("robot_id", "amr_01", identity);
  const bool namespaced = std::string(node->get_namespace()) != "/";
  const auto prefix = node->declare_parameter<std::string>(
      "frame_prefix", namespaced ? id + "/" : "", identity);
  if (namespaced && prefix.empty()) {
    throw std::invalid_argument("namespaced robots require a nonempty frame_prefix");
  }
  return RobotContext(id, prefix);
}
} // namespace
MissionCoordinatorNode::MissionCoordinatorNode(const rclcpp::NodeOptions &options)
    : Node("mission_coordinator", options), robot_(local_robot(this)),
      navigation_(this, [this](const auto &name,
                               const auto &detail) { event(name, "", detail); }),
      resources_(
          make_resource_transport(this), robot_.robot_id,
          [] {
            return std::chrono::duration<double>(
                       std::chrono::steady_clock::now().time_since_epoch())
                .count();
          },
          declare_parameter("resource_wait_timeout_sec", 120.0)),
      resource_gate_(resources_, machine_) {
  if (!has_parameter("use_sim_time")) {
    declare_parameter("use_sim_time", true);
  } else {
    set_parameter({"use_sim_time", true});
  }
  poses_["assembly"] = declare_parameter<std::vector<double>>(
      "stations.assembly.pose", {-3.0, 0.8, 1.5707963267948966});
  poses_["inspection"] = declare_parameter<std::vector<double>>(
      "stations.inspection.pose", {3.0, 0.8, 1.5707963267948966});
  for (const auto &pair : poses_) {
    if (pair.second.size() != 3 ||
        !std::all_of(pair.second.begin(), pair.second.end(),
                     [](double v) { return std::isfinite(v); })) {
      throw std::invalid_argument("station pose requires three finite values");
    }
  }
  leases_enabled_ =
      declare_parameter("resource_leases_enabled", std::string(get_namespace()) != "/");
  if (leases_enabled_) {
    // Pair separation is 2 * (configured robot radius + arrival tolerance).
    // The default preserves the existing 0.35 m + 0.25 m conservative envelope.
    station_clearance_distance_ =
        declare_parameter("stations.clearance_distance", 1.20);
    if (!std::isfinite(station_clearance_distance_) || station_clearance_distance_ <= 0)
      throw std::invalid_argument(
          "station clearance distance must be finite and positive");
    for (const auto &station : poses_) {
      station_bounds_.emplace(
          station.first,
          TrafficBoundary(declare_parameter<std::vector<double>>(
              "stations." + station.first + ".bounds", std::vector<double>{})));
      auto staging = declare_parameter<std::vector<double>>(
          "stations." + station.first + ".staging_pose", std::vector<double>{});
      if (staging.size() != 3 ||
          !std::all_of(staging.begin(), staging.end(),
                       [](double v) { return std::isfinite(v); }) ||
          !station_bounds_.at(station.first)
               .outside(staging[0], staging[1], station_clearance_distance_ / 2))
        throw std::invalid_argument("station staging must clear the station footprint");
      station_staging_[station.first] = staging;
      station_exit_[station.first] = declare_parameter<std::vector<double>>(
          "stations." + station.first + ".exit_pose", std::vector<double>{});
    }
  }
  const auto route_names = declare_parameter<std::vector<std::string>>(
      "route_names", {"to_assembly", "assembly_to_inspection"});
  for (const auto &name : route_names) {
    auto ids = declare_parameter<std::vector<std::string>>(
        "routes." + name + ".resources", std::vector<std::string>{});
    if (leases_enabled_ && ids.size() > 1)
      throw std::invalid_argument("leased routes require one traffic segment until "
                                  "handoff bays are configured");
    auto &route = routes_[name];
    for (const auto &id : ids) {
      auto staging = declare_parameter<std::vector<double>>(
          "routes." + name + "." + id + ".staging_pose", std::vector<double>{});
      auto exit = declare_parameter<std::vector<double>>(
          "routes." + name + "." + id + ".exit_pose", std::vector<double>{});
      TrafficBoundary boundary(declare_parameter<std::vector<double>>(
          "routes." + name + "." + id + ".bounds", std::vector<double>{}));
      for (const auto &pose : {staging, exit}) {
        if (pose.size() != 3 || !std::all_of(pose.begin(), pose.end(),
                                             [](double v) { return std::isfinite(v); }))
          throw std::invalid_argument(
              "route staging and exit require three finite values");
      }
      if (id.empty() || std::hypot(staging[0] - exit[0], staging[1] - exit[1]) < 0.5)
        throw std::invalid_argument("traffic route requires distinct staging and exit");
      if (!boundary.outside(staging[0], staging[1], 0.60) ||
          !boundary.outside(exit[0], exit[1], 0.60))
        throw std::invalid_argument(
            "traffic staging and exit must clear the zone footprint");
      route.push_back({id, staging, exit, boundary});
    }
  }
  if (leases_enabled_) {
    for (const auto &station : poses_) {
      auto &exit = station_exit_.at(station.first);
      const auto &staging = station_staging_.at(station.first);
      if (exit.size() != 3 ||
          !std::all_of(exit.begin(), exit.end(),
                       [](double v) { return std::isfinite(v); }) ||
          !station_bounds_.at(station.first)
               .outside(exit[0], exit[1], station_clearance_distance_ / 2) ||
          std::hypot(staging[0] - exit[0], staging[1] - exit[1]) <=
              station_clearance_distance_)
        throw std::invalid_argument(
            "explicit station parking must clear bounds and the waiting pose");
      for (const auto &route : routes_) {
        for (const auto &segment : route.second) {
          if (!segment.boundary.segment_outside(staging[0], staging[1],
                                                station.second[0], station.second[1],
                                                station_clearance_distance_ / 2))
            throw std::invalid_argument(
                "station approach must clear all traffic bounds");
          if (!segment.boundary.segment_outside(station.second[0], station.second[1],
                                                exit[0], exit[1],
                                                station_clearance_distance_ / 2))
            throw std::invalid_argument(
                "station parking path must clear traffic bounds");
          for (const auto &point : {segment.staging, segment.exit}) {
            if (std::hypot(staging[0] - point[0], staging[1] - point[1]) <=
                station_clearance_distance_)
              throw std::invalid_argument(
                  "station wait pose must be separated from traffic waypoints");
            if (std::hypot(exit[0] - point[0], exit[1] - point[1]) <=
                station_clearance_distance_)
              throw std::invalid_argument(
                  "station parking must be separated from traffic waypoints");
          }
        }
      }
    }
  }
  timeout_ = declare_parameter("perception_timeout_sec", 10.0);
  refresh_timeout_ = declare_parameter("localization_refresh_timeout_sec", 1.0);
  navigation_timeout_ = declare_parameter("navigation_timeout_sec", 120.0);
  if (!std::isfinite(timeout_) || timeout_ <= 0 ||
      !std::isfinite(navigation_timeout_) || navigation_timeout_ <= 0 ||
      !std::isfinite(refresh_timeout_) || refresh_timeout_ <= 0) {
    throw std::invalid_argument("timeouts must be finite and positive");
  }
  events_ = create_publisher<factory_interfaces::msg::ProtocolEvent>(
      "/factory/protocol_events", rclcpp::QoS(100).reliable());
  fault_acks_ = create_publisher<factory_interfaces::msg::FaultCommand>(
      "/factory/faults/acknowledgements", rclcpp::QoS(100).reliable());
  fault_commands_ = create_subscription<factory_interfaces::msg::FaultCommand>(
      "/factory/faults/commands", rclcpp::QoS(100).reliable(),
      [this](factory_interfaces::msg::FaultCommand::SharedPtr message) {
        fault_command(*message);
      });
  states_ = create_publisher<std_msgs::msg::String>(RobotContext::state_topic,
                                                    rclcpp::QoS(100).reliable());
  transfer_client_ = create_client<Transfer>("/factory/transfer_part");
  // All coordinator callbacks use the default mutually exclusive callback group.
  // Refresh is asynchronous; no callback waits for this robot-local service.
  refresh_client_ = create_client<std_srvs::srv::Empty>("request_nomotion_update");
  localization_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      RobotContext::pose_topic, rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message) {
        const auto &p = message->pose.pose;
        if (message->header.stamp.sec < 0 ||
            message->header.stamp.nanosec >= 1000000000u) {
          pose_valid_ = false;
          return;
        }
        const double stamp = rclcpp::Time(message->header.stamp).seconds();
        // A sensor can arrive before the matching /clock update. Keep one
        // current-leg candidate; localized() still forbids future evidence.
        const bool source_valid =
            message->header.frame_id == robot_.local_frame(RobotContext::map_frame) &&
            std::isfinite(p.position.x) && std::isfinite(p.position.y) &&
            std::isfinite(p.position.z) && std::isfinite(p.orientation.x) &&
            std::isfinite(p.orientation.y) && std::isfinite(p.orientation.z) &&
            std::isfinite(p.orientation.w) &&
            std::abs(p.orientation.x * p.orientation.x +
                     p.orientation.y * p.orientation.y +
                     p.orientation.z * p.orientation.z +
                     p.orientation.w * p.orientation.w - 1.0) < 0.01;
        pose_valid_ = source_valid && goal_ && stamp >= localization_started_ &&
                      (!leases_enabled_ || stamp > mission_pose_floor_);
        if (source_valid && stamp <= now().seconds())
          last_current_pose_stamp_ = std::max(last_current_pose_stamp_, stamp);
        pose_receipt_ = wall_time();
        pose_ = p;
        pose_stamp_ = stamp;
        refresh_buffered_pose_ = refresh_active_ && source_valid &&
                                 stamp > refresh_source_stamp_ &&
                                 stamp < refresh_epoch_ && stamp <= now().seconds() &&
                                 pose_receipt_ >= refresh_sent_;
        if (refresh_active_ && owns_goal(mission_generation_)) {
          std::ostringstream detail;
          detail << std::setprecision(17) << "{\"source_stamp\":" << stamp
                 << ",\"epoch\":" << refresh_epoch_
                 << ",\"source_floor\":" << refresh_source_stamp_
                 << ",\"mission_floor\":" << mission_pose_floor_
                 << ",\"receipt_after_sec\":" << pose_receipt_ - refresh_sent_
                 << ",\"acknowledged\":" << (refresh_ready_ ? "true" : "false")
                 << ",\"source_valid\":" << (source_valid ? "true" : "false")
                 << ",\"pose_current\":" << (pose_current() ? "true" : "false") << "}";
          event("localization_refresh_pose", "", detail.str());
        }
      });
  detections_ = create_subscription<factory_interfaces::msg::StationDetection>(
      RobotContext::detection_topic, rclcpp::SensorDataQoS(),
      [this](factory_interfaces::msg::StationDetection::SharedPtr message) {
        queue_detection(*message);
      });
  server_ = rclcpp_action::create_server<Mission>(
      this, RobotContext::mission_action,
      [this](auto, auto request) {
        if (!get_node_base_interface()->get_context()->is_valid())
          return rclcpp_action::GoalResponse::REJECT;
        std::string error;
        if (!std::regex_match(request->mission_id, std::regex("[A-Za-z0-9_-]{1,64}")) ||
            !robot_.accepts_robot(request->robot_id) ||
            request->pickup_station != "assembly" ||
            request->dropoff_station != "inspection" || request->part != "motor") {
          error = "INVALID_MISSION";
        } else if (reserved_) {
          error = "ROBOT_BUSY";
        }
        if (!error.empty()) {
          event("mission_rejected", "FAILED", error, request->mission_id);
          return rclcpp_action::GoalResponse::REJECT;
        }
        reserved_ = true;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](auto handle) {
        if (handle != goal_ || !owns_goal(mission_generation_)) {
          return rclcpp_action::CancelResponse::REJECT;
        }
        if (station_exiting_) {
          navigation_.cancel();
          deferred_error_ = "MISSION_CANCELED";
          return rclcpp_action::CancelResponse::ACCEPT;
        }
        auto result = machine_.cancel();
        if (waiting_resource_) {
          resource_gate_.cancel();
          waiting_resource_ = false;
          deferred_error_ = "MISSION_CANCELED";
        }
        if (result.state == MissionState::Failed) {
          navigation_.cancel();
          deferred_error_ = "MISSION_CANCELED";
        }
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](auto handle) {
        if (!get_node_base_interface()->get_context()->is_valid()) {
          server_.reset();
          return;
        }
        if (restart_required_) {
          // Goal rejection cannot carry a typed reason. Acknowledge transport,
          // then return the lifecycle error without starting physical execution.
          auto result = std::make_shared<Mission::Result>();
          result->success = false;
          result->final_state = "FAILED";
          result->error_code = "RESTART_REQUIRED";
          result->message =
              "Restart the full simulation before another payload mission";
          event("mission_rejected", "FAILED", result->error_code,
                handle->get_goal()->mission_id);
          try {
            handle->abort(result);
          } catch (const std::exception &) {
            if (get_node_base_interface()->get_context()->is_valid())
              throw;
            server_.reset();
          }
          reserved_ = false;
          return;
        }
        goal_ = handle;
        drop_pose_refresh();
        localization_started_ = now().seconds();
        mission_pose_floor_ = pose_stamp_;
        finishing_ = waiting_resource_ = crossing_resource_ = false;
        station_staged_ = station_exiting_ = station_terminal_exit_ = false;
        waiting_traffic_pose_ = clearing_traffic_ = false;
        waiting_navigation_pose_ = false;
        pending_navigation_target_.clear();
        departure_station_.clear();
        held_resource_.clear();
        active_route_.clear();
        route_index_ = 0;
        ++mission_generation_;
        pose_valid_ = false;
        auto request = handle->get_goal();
        event("mission_started");
        transition(machine_.start({request->mission_id, request->robot_id,
                                   request->pickup_station, request->dropoff_station,
                                   request->part}));
        transition(machine_.begin_navigation());
        navigate();
      });
  timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] { tick(); });
}
MissionCoordinatorNode::~MissionCoordinatorNode() {
  ++mission_generation_;
  drop_pose_refresh();
  if (!get_node_base_interface()->get_context()->is_valid()) {
    // Humble automatically publishes cancellation from an unfinished goal's
    // destructor. Retire its server first after shutdown, so that callback's
    // weak server reference cannot publish through an invalid ROS context.
    server_.reset();
  }
  goal_.reset();
}
bool MissionCoordinatorNode::owns_goal(uint64_t generation) {
  return goal_ && server_ && generation == mission_generation_ &&
         get_node_base_interface()->get_context()->is_valid();
}
std::string MissionCoordinatorNode::state_name() const {
  static const std::vector<std::string> names = {"IDLE",
                                                 "RECEIVED",
                                                 "NAVIGATING_TO_PICKUP",
                                                 "VERIFYING_PICKUP",
                                                 "LOADING",
                                                 "NAVIGATING_TO_DROPOFF",
                                                 "VERIFYING_DROPOFF",
                                                 "UNLOADING",
                                                 "RECOVERING",
                                                 "COMPLETED",
                                                 "FAILED"};
  return names.at(static_cast<size_t>(machine_.state()));
}
std::string MissionCoordinatorNode::phase(const std::string &prefix) const {
  return prefix + (machine_.current_station() == "assembly" ? "_pickup" : "_dropoff");
}
void MissionCoordinatorNode::event(const std::string &name, const std::string &outcome,
                                   const std::string &detail,
                                   const std::string &mission) {
  factory_interfaces::msg::ProtocolEvent message;
  message.stamp = now();
  message.mission_id =
      mission.empty() && goal_ ? goal_->get_goal()->mission_id : mission;
  message.robot_id = robot_.robot_id;
  message.protocol = "ROS";
  message.direction = "INTERNAL";
  message.event = name;
  message.outcome = outcome;
  message.detail = detail;
  events_->publish(message);
}
void MissionCoordinatorNode::transition(const TransitionResult &result) {
  if (!result.accepted || !owns_goal(mission_generation_)) {
    return;
  }
  RCLCPP_INFO(get_logger(),
              "mission_id=%s robot_id=%s station=%s state=%s error_code=%s",
              machine_.mission().mission_id.c_str(),
              machine_.mission().robot_id.c_str(), machine_.current_station().c_str(),
              state_name().c_str(), result.error_code.c_str());
  auto feedback = std::make_shared<Mission::Feedback>();
  feedback->state = state_name();
  feedback->station = machine_.current_station();
  feedback->detail = result.error_code;
  feedback->progress =
      std::min(1.0f, static_cast<float>(static_cast<int>(result.state)) / 9.0f);
  goal_->publish_feedback(feedback);
  std_msgs::msg::String state;
  state.data = state_name();
  states_->publish(state);
  event("state_changed", "", state_name());
}
void MissionCoordinatorNode::navigate() {
  if (leases_enabled_) {
    const auto route_name =
        machine_.state() == MissionState::NavigatingToPickup
            ? "to_" + machine_.current_station()
            : machine_.mission().pickup_station + "_to_" + machine_.current_station();
    if (active_route_.empty() && route_index_ == 0) {
      auto route = routes_.find(route_name);
      if (route == routes_.end()) {
        finish("RESOURCE_ROUTE_MISSING");
        return;
      }
      active_route_ = route->second;
    }
    if (!held_resource_.empty() && station_bounds_.count(held_resource_) &&
        held_resource_ != machine_.current_station()) {
      station_exiting_ = true;
      departure_station_ = held_resource_;
      if (!active_route_.empty()) {
        const auto &segment = active_route_[route_index_];
        if (!traffic_pose_clear(segment.boundary)) {
          waiting_traffic_pose_ = true;
          request_pose_refresh();
          return;
        }
        waiting_traffic_pose_ = false;
        drop_pose_refresh();
        wait_for_resource(
            segment.resource, [this] { navigate(); }, departure_station_);
        return;
      }
      const auto &exit = station_exit_.at(held_resource_);
      if (exit.size() != 3 ||
          !station_bounds_.at(held_resource_)
               .outside(exit[0], exit[1], station_clearance_distance_ / 2)) {
        finish("RESOURCE_EXIT_UNCONFIRMED");
        return;
      }
      navigate_target(exit);
      return;
    }
    if (route_index_ < active_route_.size()) {
      const auto &segment = active_route_[route_index_];
      if (held_resource_ != segment.resource) {
        if (!traffic_pose_clear(segment.boundary)) {
          waiting_traffic_pose_ = true;
          request_pose_refresh();
          return;
        }
        waiting_traffic_pose_ = false;
        drop_pose_refresh();
        wait_for_resource(segment.resource, [this] { navigate(); });
      } else {
        if (!crossing_resource_ && !traffic_pose_clear(segment.boundary)) {
          // Localization and /clock may arrive on different callback turns.
          // Retain both leases, but dispatch no motion until evidence is current.
          waiting_traffic_pose_ = true;
          request_pose_refresh();
          return;
        }
        waiting_traffic_pose_ = false;
        drop_pose_refresh();
        navigate_target(clearing_traffic_
                            ? station_staging_.at(machine_.current_station())
                        : crossing_resource_ ? segment.exit
                                             : segment.staging);
      }
      return;
    }
    const auto station = machine_.current_station();
    if (held_resource_ != station) {
      if (!station_staged_) {
        const auto staging = station_staging_.at(station);
        if (staging.size() != 3) {
          finish("RESOURCE_ROUTE_MISSING");
          return;
        }
        navigate_target(staging);
      } else
        wait_for_resource(station, [this] { navigate(); });
      return;
    }
  }
  navigate_target(poses_.at(machine_.current_station()));
}
void MissionCoordinatorNode::navigate_target(const std::vector<double> &coordinates) {
  if ((!held_resource_.empty() &&
       !resources_.authorized(held_resource_, machine_.mission().mission_id)) ||
      (!departure_station_.empty() &&
       !resources_.authorized(departure_station_, machine_.mission().mission_id))) {
    finish("RESOURCE_LEASE_LOST");
    return;
  }
  if (leases_enabled_ && !pose_current()) {
    pending_navigation_target_ = coordinates;
    waiting_navigation_pose_ = true;
    request_pose_refresh();
    return;
  }
  waiting_navigation_pose_ = false;
  pending_navigation_target_.clear();
  drop_pose_refresh();
  navigation_target_ = coordinates;
  navigation_completed_ = false;
  pose_valid_ = false;
  stamps_.clear();
  pending_detections_.clear();
  last_stamp_ = received_stamp_ = -1;
  active_phase_ = phase("navigation");
  phase_started_ = now().seconds();
  event(active_phase_ + "_started");
  localization_started_ = phase_started_;
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = robot_.local_frame(RobotContext::map_frame);
  pose.header.stamp = now();
  pose.pose.position.x = coordinates[0];
  pose.pose.position.y = coordinates[1];
  pose.pose.orientation.z = std::sin(coordinates[2] / 2);
  pose.pose.orientation.w = std::cos(coordinates[2] / 2);
  const auto generation = mission_generation_;
  std::ostringstream detail;
  detail << std::setprecision(17) << "{\"station\":\"" << machine_.current_station()
         << "\",\"pose\":[" << coordinates[0] << "," << coordinates[1] << ","
         << coordinates[2] << "]}";
  event("navigation_goal_requested", "", detail.str());
  auto done = [this, generation](bool success) {
    if (!owns_goal(generation) || active_phase_.find("navigation_") != 0) {
      return;
    }
    event(active_phase_ + "_finished", success ? "SUCCEEDED" : "FAILED");
    active_phase_.clear();
    if (success) {
      navigation_completed_ = true;
      phase_started_ = now().seconds();
      if (localized()) {
        navigation_arrived();
      } else if (leases_enabled_ && !pose_current())
        request_pose_refresh();
    } else {
      if (station_exiting_) {
        finish("NAVIGATION_FAILED");
        return;
      }
      auto result = machine_.navigation_failed();
      transition(result);
      if (result.state == MissionState::Failed) {
        finish(result.error_code);
        return;
      }
      phase_started_ = now().seconds();
      event("navigation_recovery_started");
      navigation_.clear_costmaps(
          [this, generation](bool cleared) {
            if (!owns_goal(generation) ||
                machine_.state() != MissionState::Recovering) {
              return;
            }
            event("navigation_recovery_finished", cleared ? "SUCCEEDED" : "FAILED");
            if (!cleared) {
              finish("NAVIGATION_FAILED");
              return;
            }
            event("retry");
            transition(machine_.recovery_ready());
            navigate();
          },
          [this, generation](const std::string &service) {
            if (owns_goal(generation) && machine_.state() == MissionState::Recovering) {
              event("costmap_cleared", "SUCCEEDED",
                    "{\"service\":\"" + service + "\"}");
            }
          });
    }
  };
  if (reject_navigation()) {
    done(false);
  } else {
    navigation_.navigate(pose, done);
  }
}
bool MissionCoordinatorNode::traffic_pose_clear(const TrafficBoundary &boundary) const {
  return pose_current() && boundary.outside(pose_.position.x, pose_.position.y, 0.35);
}
bool MissionCoordinatorNode::pose_current() const {
  const auto time = now().seconds();
  return pose_valid_ && pose_stamp_ >= localization_started_ && pose_stamp_ <= time &&
         time - pose_stamp_ <= 1.5 && wall_time() - pose_receipt_ <= 1.5 &&
         (!refresh_active_ ||
          (refresh_ready_ && pose_stamp_ > refresh_source_stamp_ &&
           pose_stamp_ >= refresh_epoch_ && pose_receipt_ >= refresh_sent_ &&
           pose_receipt_ - refresh_sent_ <= refresh_timeout_ &&
           refresh_ack_receipt_ >= refresh_sent_ &&
           refresh_ack_receipt_ - refresh_sent_ <= refresh_timeout_));
}
void MissionCoordinatorNode::drop_pose_refresh() {
  ++refresh_sequence_;
  if (refresh_request_id_ >= 0 && refresh_client_)
    refresh_client_->remove_pending_request(refresh_request_id_);
  refresh_request_id_ = -1;
  refresh_active_ = refresh_ready_ = false;
  refresh_ack_receipt_ = -1;
  refresh_buffered_pose_ = false;
  refresh_attempts_ = 0;
}
void MissionCoordinatorNode::request_pose_refresh() {
  if (!leases_enabled_ || !owns_goal(mission_generation_) || finishing_)
    return;
  if (!refresh_active_) {
    refresh_active_ = true;
    refresh_ready_ = false;
    refresh_ack_receipt_ = -1;
    refresh_buffered_pose_ = false;
    refresh_attempts_ = 0;
    refresh_sent_ = wall_time();
    refresh_epoch_ = now().seconds();
    refresh_source_stamp_ = std::max(mission_pose_floor_, last_current_pose_stamp_);
    if (pose_valid_ && pose_stamp_ <= refresh_epoch_)
      refresh_source_stamp_ = std::max(refresh_source_stamp_, pose_stamp_);
    ++refresh_sequence_;
    event("localization_refresh_started");
  }
  if (refresh_request_id_ >= 0 || !refresh_client_->service_is_ready())
    return;
  if (refresh_ready_) {
    // AMCL can consume a buffered scan on an acknowledged no-motion update.
    // Reissue only that valid-but-pre-epoch candidate. Never move the original
    // epoch, source floor or steady deadline, and never authorize stale poses.
    if (!refresh_buffered_pose_ || refresh_attempts_ >= 10 ||
        wall_time() - refresh_last_attempt_ < 0.1)
      return;
    refresh_ready_ = false;
    refresh_ack_receipt_ = -1;
    ++refresh_sequence_;
    event("localization_refresh_reissued");
  }
  refresh_last_attempt_ = wall_time();
  ++refresh_attempts_;
  std::ostringstream attempt_detail;
  attempt_detail << std::setprecision(17) << "{\"attempt\":" << refresh_attempts_
                 << ",\"elapsed_sec\":" << refresh_last_attempt_ - refresh_sent_ << "}";
  event("localization_refresh_requested", "", attempt_detail.str());
  const auto generation = mission_generation_, sequence = refresh_sequence_;
  try {
    auto pending = refresh_client_->async_send_request(
        std::make_shared<std_srvs::srv::Empty::Request>(),
        [this, generation,
         sequence](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture reply) {
          if (!owns_goal(generation) || finishing_ || sequence != refresh_sequence_ ||
              !refresh_active_)
            return;
          refresh_request_id_ = -1;
          try {
            if (!reply.get())
              throw std::runtime_error("empty localization refresh reply");
            refresh_ready_ = true;
            refresh_ack_receipt_ = wall_time();
            std::ostringstream detail;
            detail << std::setprecision(17) << "{\"acknowledgement_after_sec\":"
                   << refresh_ack_receipt_ - refresh_sent_ << "}";
            event("localization_refresh_finished", "SUCCEEDED", detail.str());
          } catch (const std::exception &) {
            event("localization_refresh_finished", "FAILED");
            finish("LOCALIZATION_REFRESH_FAILED");
          }
        });
    refresh_request_id_ = pending.request_id;
  } catch (const std::exception &) {
    finish("LOCALIZATION_REFRESH_FAILED");
  }
}
void MissionCoordinatorNode::wait_for_resource(const std::string &resource,
                                               std::function<void()> effect,
                                               const std::string &departure_source) {
  waiting_resource_ = true;
  const auto generation = mission_generation_;
  resource_gate_.acquire(
      resource, std::move(effect),
      [this, generation, resource](const ResourceNotice &notice) {
        if (!owns_goal(generation) || finishing_)
          return;
        event("resource_lease", "", notice.reason);
        if (notice.state == ResourceState::Waiting) {
          auto feedback = std::make_shared<Mission::Feedback>();
          feedback->state = "WAITING_FOR_RESOURCE";
          feedback->station = machine_.current_station();
          feedback->detail = resource + ": " + notice.reason;
          goal_->publish_feedback(feedback);
          std_msgs::msg::String state;
          state.data = feedback->state;
          states_->publish(state);
        } else if (notice.state == ResourceState::Granted) {
          waiting_resource_ = false;
          held_resource_ = resource;
        } else if (notice.state == ResourceState::Lost) {
          navigation_.cancel();
          const bool wait_timeout = notice.reason == "RESOURCE_WAIT_TIMEOUT";
          if (!wait_timeout)
            restart_required_ = true;
          finish(wait_timeout ? "RESOURCE_WAIT_TIMEOUT" : "RESOURCE_LEASE_LOST");
        }
      },
      departure_source);
}
void MissionCoordinatorNode::navigation_arrived() {
  drop_pose_refresh();
  if (leases_enabled_ && station_exiting_) {
    navigation_completed_ = false;
    const auto source =
        departure_station_.empty() ? held_resource_ : departure_station_;
    if (!resources_.authorized(source, machine_.mission().mission_id) ||
        !station_bounds_.at(source).outside(pose_.position.x, pose_.position.y,
                                            station_clearance_distance_ / 2)) {
      finish("RESOURCE_EXIT_UNCONFIRMED");
      return;
    }
    const auto generation = mission_generation_;
    resources_.exited(source, machine_.mission().mission_id);
    waiting_resource_ = true;
    resource_gate_.release(source, [this, generation, source](bool cleared) {
      if (!owns_goal(generation) || finishing_)
        return;
      if (!cleared) {
        finish("RESOURCE_RELEASE_FAILED");
        return;
      }
      if (held_resource_ == source)
        held_resource_.clear();
      departure_station_.clear();
      waiting_resource_ = station_exiting_ = false;
      if (station_terminal_exit_) {
        const auto result = machine_.transfer_succeeded();
        transition(result);
        finish(result.error_code);
      } else {
        crossing_resource_ = !active_route_.empty();
        navigate();
      }
    });
    return;
  }
  if (leases_enabled_ && route_index_ < active_route_.size()) {
    navigation_completed_ = false;
    auto segment = active_route_[route_index_];
    if (!segment.boundary.outside(pose_.position.x, pose_.position.y, 0.35)) {
      finish("RESOURCE_EXIT_UNCONFIRMED");
      return;
    }
    if (!crossing_resource_) {
      crossing_resource_ = true;
      navigate();
    } else if (!clearing_traffic_) {
      clearing_traffic_ = true;
      navigate();
    } else {
      if (!station_bounds_.at(machine_.current_station())
               .outside(pose_.position.x, pose_.position.y,
                        station_clearance_distance_ / 2)) {
        finish("RESOURCE_EXIT_UNCONFIRMED");
        return;
      }
      const auto generation = mission_generation_;
      resources_.exited(segment.resource, machine_.mission().mission_id);
      waiting_resource_ = true;
      resource_gate_.release(segment.resource, [this, generation](bool cleared) {
        if (!owns_goal(generation) || finishing_)
          return;
        if (!cleared) {
          restart_required_ = true;
          finish("RESOURCE_RELEASE_FAILED");
          return;
        }
        held_resource_.clear();
        waiting_resource_ = crossing_resource_ = clearing_traffic_ = false;
        ++route_index_;
        if (route_index_ == active_route_.size()) {
          const auto station = machine_.current_station();
          if (!station_bounds_.at(station).outside(pose_.position.x, pose_.position.y,
                                                   station_clearance_distance_ / 2)) {
            finish("RESOURCE_EXIT_UNCONFIRMED");
            return;
          }
          station_staged_ = true;
        }
        navigate();
      });
    }
  } else if (leases_enabled_ && held_resource_ != machine_.current_station()) {
    navigation_completed_ = false;
    if (!station_bounds_.at(machine_.current_station())
             .outside(pose_.position.x, pose_.position.y,
                      station_clearance_distance_ / 2)) {
      finish("RESOURCE_EXIT_UNCONFIRMED");
      return;
    }
    station_staged_ = true;
    navigate();
  } else
    verify_station();
}
void MissionCoordinatorNode::fault_event(
    const std::string &name, const factory_interfaces::msg::FaultCommand &control) {
  factory_interfaces::msg::ProtocolEvent message;
  message.stamp = now();
  message.mission_id = control.mission_id;
  message.robot_id = robot_.robot_id;
  message.protocol = "FAULT";
  message.direction = "INTERNAL";
  message.event = name;
  message.outcome = "SUCCEEDED";
  message.detail =
      "{\"name\":\"" + control.name + "\",\"station\":\"" + control.station + "\"}";
  events_->publish(message);
}
void MissionCoordinatorNode::fault_command(
    const factory_interfaces::msg::FaultCommand &message) {
  if (!message.robot_id.empty() && !robot_.accepts_robot(message.robot_id)) {
    return;
  }
  if (!message.owner.empty() && message.owner != "mission_coordinator") {
    return;
  }
  if (message.reset) {
    for (const auto &fault : navigation_faults_) {
      fault_event("fault_reset", fault.command);
    }
    navigation_faults_.clear();
  } else {
    if ((message.name != "nav_reject_once" && message.name != "nav_reject_twice") ||
        !std::regex_match(message.mission_id, std::regex("[A-Za-z0-9_-]{1,64}")) ||
        (!message.station.empty() && message.station != "assembly" &&
         message.station != "inspection") ||
        message.activation_point != "navigation_start" ||
        !std::isfinite(message.duration) || message.duration <= 0) {
      return;
    }
    navigation_faults_.erase(
        std::remove_if(navigation_faults_.begin(), navigation_faults_.end(),
                       [&message](const auto &entry) {
                         return entry.command.name == message.name &&
                                entry.command.mission_id == message.mission_id &&
                                entry.command.station == message.station;
                       }),
        navigation_faults_.end());
    navigation_faults_.push_back(
        {message, message.name == "nav_reject_once" ? 1 : 2, false, {}});
  }
  factory_interfaces::msg::FaultCommand ack;
  ack.command_id = message.command_id;
  ack.owner = "mission_coordinator";
  ack.robot_id = robot_.robot_id;
  ack.mission_id = message.mission_id;
  ack.station = message.station;
  ack.acknowledged = true;
  fault_acks_->publish(ack);
}
bool MissionCoordinatorNode::reject_navigation() {
  const auto current = std::chrono::steady_clock::now();
  for (auto entry = navigation_faults_.begin(); entry != navigation_faults_.end();) {
    if (entry->activated &&
        std::chrono::duration<double>(current - entry->started).count() >=
            entry->command.duration) {
      fault_event("fault_reset", entry->command);
      entry = navigation_faults_.erase(entry);
      continue;
    }
    if (entry->command.mission_id == machine_.mission().mission_id &&
        (entry->command.station.empty() ||
         entry->command.station == machine_.current_station())) {
      if (!entry->activated) {
        entry->activated = true;
        entry->started = current;
        fault_event("fault_activated", entry->command);
        if (entry->command.one_shot) {
          fault_event("fault_consumed", entry->command);
        }
      }
      if (--entry->remaining == 0) {
        if (entry->command.one_shot) {
          fault_event("fault_reset", entry->command);
          navigation_faults_.erase(entry);
        } else {
          entry->remaining = entry->command.name == "nav_reject_once" ? 1 : 2;
        }
      }
      return true;
    }
    ++entry;
  }
  return false;
}
void MissionCoordinatorNode::verify_station() {
  navigation_completed_ = false;
  transition(machine_.navigation_succeeded());
  active_phase_ = phase("perception");
  phase_started_ = now().seconds();
  stamps_.clear();
  pending_detections_.clear();
  last_stamp_ = received_stamp_ = -1;
  event(active_phase_ + "_started");
}
bool MissionCoordinatorNode::localized() const {
  const auto time = now().seconds();
  if (!pose_valid_ || pose_stamp_ > time || pose_stamp_ < localization_started_ ||
      (leases_enabled_ && !pose_current())) {
    return false;
  }
  const auto &target = navigation_target_;
  if (target.size() != 3)
    return false;
  double yaw = std::atan2(2 * (pose_.orientation.w * pose_.orientation.z +
                               pose_.orientation.x * pose_.orientation.y),
                          1 - 2 * (pose_.orientation.y * pose_.orientation.y +
                                   pose_.orientation.z * pose_.orientation.z));
  return std::hypot(pose_.position.x - target[0], pose_.position.y - target[1]) <=
             0.25 &&
         std::abs(std::remainder(yaw - target[2], 2 * M_PI)) <= 0.25;
}
void MissionCoordinatorNode::queue_detection(
    const factory_interfaces::msg::StationDetection &message) {
  if (!goal_ || (machine_.state() != MissionState::VerifyingPickup &&
                 machine_.state() != MissionState::VerifyingDropoff)) {
    return;
  }
  auto clear = [this] {
    stamps_.clear();
    pending_detections_.clear();
  };
  if (message.header.stamp.sec < 0 || message.header.stamp.nanosec >= 1000000000u) {
    clear();
    return;
  }
  const double stamp = rclcpp::Time(message.header.stamp).seconds(),
               current = now().seconds();
  if (stamp < phase_started_ || current - stamp > 1.5 || stamp <= received_stamp_) {
    clear();
    return;
  }
  received_stamp_ = stamp;
  const int marker = machine_.current_station() == "assembly" ? 10 : 20;
  if ((!message.header.frame_id.empty() &&
       message.header.frame_id != robot_.local_frame("camera_optical_frame")) ||
      message.station_id != machine_.current_station() || message.marker_id != marker) {
    last_stamp_ = stamp;
    clear();
    return;
  }
  // Bound unconsumed observations, including a paused or delayed shared clock.
  // Overflow invalidates the partial window rather than silently dropping a
  // disruptive observation and retaining an old confirmation sequence.
  if (pending_detections_.size() >= 5) {
    clear();
  }
  pending_detections_.push_back(message);
  consume_detections();
}
void MissionCoordinatorNode::consume_detections() {
  while (!pending_detections_.empty()) {
    if (!goal_ || (machine_.state() != MissionState::VerifyingPickup &&
                   machine_.state() != MissionState::VerifyingDropoff)) {
      pending_detections_.clear();
      return;
    }
    const double current = now().seconds();
    if (current < phase_started_ || current - phase_started_ >= timeout_) {
      return;
    }
    if (rclcpp::Time(pending_detections_.front().header.stamp).seconds() > current) {
      return;
    }
    auto message = pending_detections_.front();
    pending_detections_.pop_front();
    detection(message);
  }
}
void MissionCoordinatorNode::detection(
    const factory_interfaces::msg::StationDetection &message) {
  if (!goal_ || (machine_.state() != MissionState::VerifyingPickup &&
                 machine_.state() != MissionState::VerifyingDropoff)) {
    return;
  }
  if (message.header.stamp.sec < 0 || message.header.stamp.nanosec >= 1000000000u) {
    stamps_.clear();
    return;
  }
  const double stamp = rclcpp::Time(message.header.stamp).seconds(),
               current = now().seconds();
  const int marker = machine_.current_station() == "assembly" ? 10 : 20;
  if (stamp < phase_started_ || stamp > current || current - stamp > 1.5 ||
      stamp <= last_stamp_) {
    stamps_.clear();
    return;
  }
  last_stamp_ = stamp;
  if (message.station_id != machine_.current_station() || message.marker_id != marker) {
    stamps_.clear();
    return;
  }
  stamps_.push_back(stamp);
  while (!stamps_.empty() && stamp - stamps_.front() > 1.5) {
    stamps_.pop_front();
  }
  while (stamps_.size() > 5) {
    stamps_.pop_front();
  }
  if (stamps_.size() >= 5 && localized()) {
    event(active_phase_ + "_finished", "SUCCEEDED");
    active_phase_.clear();
    transition(machine_.station_confirmed());
    transfer();
  }
}
void MissionCoordinatorNode::dispatch_transfer(
    std::shared_ptr<Transfer::Request> request, TransferCallback callback) {
  transfer_client_->async_send_request(request, std::move(callback));
}
void MissionCoordinatorNode::transfer() { transfer_effect(); }
void MissionCoordinatorNode::transfer_effect() {
  if (leases_enabled_ && !resources_.authorized(machine_.current_station(),
                                                machine_.mission().mission_id)) {
    finish("RESOURCE_LEASE_LOST");
    return;
  }
  if (!transfer_client_->service_is_ready()) {
    transition(machine_.transfer_failed("PLC_TIMEOUT"));
    finish("PLC_TIMEOUT");
    return;
  }
  auto request = std::make_shared<Transfer::Request>();
  request->mission_id = machine_.mission().mission_id;
  request->robot_id = robot_.robot_id;
  request->station_id = machine_.current_station();
  request->part = machine_.mission().part;
  const auto generation = mission_generation_;
  phase_started_ = now().seconds();
  try {
    dispatch_transfer(request, [this, generation](
                                   rclcpp::Client<Transfer>::SharedFuture future) {
      if (!owns_goal(generation)) {
        return;
      }
      TransitionResult result;
      try {
        auto response = future.get();
        auto error_code = response->error_code;
        bool unsafe_outcome = false;
        for (const std::string suffix : {"_TRANSFER_COMPLETED", "_TRANSFER_UNKNOWN"}) {
          if (error_code.size() >= suffix.size() &&
              error_code.compare(error_code.size() - suffix.size(), suffix.size(),
                                 suffix) == 0) {
            unsafe_outcome = true;
            error_code.resize(error_code.size() - suffix.size());
            break;
          }
        }
        // Latch before publishing a result or completing pending cancellation.
        // Cleanup failure never proves that the sole part remains available.
        if ((response->accepted || unsafe_outcome) &&
            machine_.state() == MissionState::Loading) {
          restart_required_ = true;
        }
        event("transfer_result", response->accepted ? "SUCCEEDED" : "FAILED",
              response->error_code);
        if (!response->accepted)
          result =
              machine_.transfer_failed(error_code.empty() ? "PLC_TIMEOUT" : error_code);
        else if (leases_enabled_ && machine_.state() == MissionState::Unloading &&
                 !goal_->is_canceling())
          // Keep the public mission nonterminal until physical clearance and
          // exact lease release are acknowledged. Cancellation still fences exit.
          result = {true, MissionState::Completed, ""};
        else
          result = machine_.transfer_succeeded();
      } catch (const std::exception &error) {
        RCLCPP_ERROR(get_logger(), "Mission %s: %s",
                     machine_.mission().mission_id.c_str(), error.what());
        if (machine_.state() == MissionState::Loading) {
          restart_required_ = true;
        }
        result = machine_.transfer_failed("PLC_TIMEOUT");
      }
      auto continue_mission = [this, generation, result](bool cleared) {
        if (!owns_goal(generation) || finishing_)
          return;
        if (!cleared) {
          restart_required_ = true;
          finish("RESOURCE_RELEASE_FAILED");
          return;
        }
        held_resource_.clear();
        waiting_resource_ = false;
        transition(result);
        if (result.state == MissionState::Completed ||
            result.state == MissionState::Failed)
          finish(result.error_code);
        else {
          active_route_.clear();
          route_index_ = 0;
          navigate();
        }
      };
      if (leases_enabled_) {
        // Only confirmed success proves the station transaction completed safely.
        bool confirmed = false;
        try {
          confirmed = future.get()->accepted;
        } catch (const std::exception &) {
        }
        if (!confirmed || result.state == MissionState::Failed) {
          restart_required_ = true;
          finish(result.error_code);
          return;
        }
        station_staged_ = false;
        if (result.state == MissionState::Completed) {
          station_exiting_ = station_terminal_exit_ = true;
          navigate_target(station_exit_.at(held_resource_));
        } else {
          transition(result);
          active_route_.clear();
          route_index_ = 0;
          navigate();
        }
      } else
        continue_mission(true);
    });
  } catch (const std::exception &error) {
    RCLCPP_ERROR(get_logger(), "Mission %s: transfer dispatch failed: %s",
                 request->mission_id.c_str(), error.what());
    // A dispatch exception cannot establish that the request stayed local.
    if (machine_.state() == MissionState::Loading) {
      restart_required_ = true;
    }
    transition(machine_.transfer_failed("PLC_TIMEOUT"));
    finish("PLC_TIMEOUT");
  }
}
void MissionCoordinatorNode::tick() {
  if (!get_node_base_interface()->get_context()->is_valid())
    return;
  resources_.tick();
  if (!goal_) {
    return;
  }
  if (!held_resource_.empty() && !waiting_resource_ &&
      !resources_.authorized(held_resource_, machine_.mission().mission_id)) {
    finish("RESOURCE_LEASE_LOST");
    return;
  }
  if (!departure_station_.empty() && !waiting_resource_ &&
      !resources_.authorized(departure_station_, machine_.mission().mission_id)) {
    finish("RESOURCE_LEASE_LOST");
    return;
  }
  if (finishing_) {
    if (!resources_.cleanup_pending(machine_.mission().mission_id))
      finish(pending_finish_error_);
    return;
  }
  if (refresh_active_) {
    // Timely complete DDS proof remains valid if executor pressure delays this
    // timer. ACK and pose receipts, not consumption time, own the deadline.
    if (pose_current() && pose_receipt_ - refresh_sent_ <= refresh_timeout_) {
      drop_pose_refresh();
    } else if (wall_time() - refresh_sent_ >= refresh_timeout_) {
      finish(refresh_request_id_ < 0 && !refresh_ready_
                 ? "LOCALIZATION_REFRESH_FAILED"
                 : "LOCALIZATION_REFRESH_TIMEOUT");
      return;
    } else
      request_pose_refresh();
  }
  if (!deferred_error_.empty()) {
    auto error = deferred_error_;
    deferred_error_.clear();
    transition({true, machine_.state(), error});
    finish(error);
    return;
  }
  consume_detections();
  if (waiting_resource_ || finishing_)
    return;
  if (waiting_navigation_pose_) {
    const auto target = pending_navigation_target_;
    navigate_target(target);
    return;
  }
  if (waiting_traffic_pose_) {
    navigate();
    return;
  }
  const double elapsed = now().seconds() - phase_started_;
  if (navigation_completed_) {
    if (localized()) {
      navigation_arrived();
    } else if (elapsed >= timeout_ || elapsed < 0) {
      finish("STATION_NOT_CONFIRMED");
    } else if (leases_enabled_ && !pose_current())
      request_pose_refresh();
    return;
  }
  if (machine_.state() == MissionState::VerifyingPickup ||
      machine_.state() == MissionState::VerifyingDropoff) {
    if (leases_enabled_ && !pose_current())
      request_pose_refresh();
    if (elapsed >= timeout_ || elapsed < 0) {
      auto result = machine_.perception_timed_out();
      transition(result);
      finish(result.error_code);
    }
  } else if ((machine_.state() == MissionState::Recovering ||
              active_phase_.find("navigation_") == 0) &&
             (elapsed >= navigation_timeout_ || elapsed < 0)) {
    finish("NAVIGATION_FAILED");
  }
}
void MissionCoordinatorNode::finish(const std::string &error) {
  if (!owns_goal(mission_generation_)) {
    return;
  }
  finishing_ = true;
  drop_pose_refresh();
  const bool safe_cleanup = resource_gate_.cancel();
  navigation_.cancel();
  if (!safe_cleanup && resources_.cleanup_pending(machine_.mission().mission_id)) {
    pending_finish_error_ = error;
    return;
  }
  const bool recovery =
      leases_enabled_ && (!safe_cleanup || error == "RESOURCE_LEASE_LOST" ||
                          error == "RESOURCE_RELEASE_FAILED");
  if (recovery)
    restart_required_ = true;
  if (!error.empty() && machine_.state() != MissionState::Failed) {
    auto result = (machine_.state() == MissionState::Loading ||
                   machine_.state() == MissionState::Unloading)
                      ? machine_.transfer_failed(error)
                      : machine_.cancel();
    result.error_code = error;
    transition(result);
  }
  if (!active_phase_.empty()) {
    event(active_phase_ + "_finished", "FAILED", error);
    active_phase_.clear();
  }
  navigation_.cancel();
  auto result = std::make_shared<Mission::Result>();
  result->success = error.empty();
  result->final_state = recovery        ? "RECOVERY_REQUIRED"
                        : error.empty() ? "COMPLETED"
                                        : "FAILED";
  result->error_code = error;
  result->message = error.empty() ? "Mission completed" : error;
  event("mission_finished", result->final_state, error);
  // Revoke this generation before publication; no continuation can publish a
  // second result or apply an effect to this retiring goal.
  auto handle = std::move(goal_);
  ++mission_generation_;
  try {
    if (get_node_base_interface()->get_context()->is_valid()) {
      if (error.empty()) {
        handle->succeed(result);
      } else if (error == "MISSION_CANCELED" && handle->is_canceling()) {
        handle->canceled(result);
      } else {
        handle->abort(result);
      }
    } else {
      server_.reset();
    }
  } catch (const std::exception &) {
    // Shutdown can invalidate the context between the ownership check and the
    // action transition/publication. Never mask errors in a live context.
    if (get_node_base_interface()->get_context()->is_valid())
      throw;
    server_.reset();
  }
  reserved_ = false;
  pose_valid_ = false;
  navigation_completed_ = false;
  waiting_resource_ = crossing_resource_ = finishing_ = false;
  station_staged_ = station_exiting_ = station_terminal_exit_ = false;
  waiting_traffic_pose_ = clearing_traffic_ = false;
  waiting_navigation_pose_ = false;
  pending_navigation_target_.clear();
  departure_station_.clear();
  pending_finish_error_.clear();
  active_route_.clear();
  route_index_ = 0;
  held_resource_.clear();
  stamps_.clear();
  pending_detections_.clear();
  last_stamp_ = received_stamp_ = -1;
  machine_.reset();
}
} // namespace mission_coordinator
