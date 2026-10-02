// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <deque>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <factory_interfaces/action/execute_factory_mission.hpp>
#include <factory_interfaces/srv/transfer_part.hpp>
#include <factory_interfaces/msg/station_detection.hpp>
#include <factory_interfaces/msg/protocol_event.hpp>
#include <factory_interfaces/msg/fault_command.hpp>
#include <chrono>
#include <std_msgs/msg/string.hpp>
#include "mission_coordinator/navigation_adapter.hpp"
#include "mission_coordinator/mission_state_machine.hpp"

namespace mission_coordinator {
class MissionCoordinatorNode : public rclcpp::Node {
public:
  explicit MissionCoordinatorNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  using Mission = factory_interfaces::action::ExecuteFactoryMission;
  using Handle = rclcpp_action::ServerGoalHandle<Mission>;
  using Transfer = factory_interfaces::srv::TransferPart;
  void transition(const TransitionResult &result);
  void navigate();
  void verify_station();
  void detection(const factory_interfaces::msg::StationDetection &message);
  void queue_detection(const factory_interfaces::msg::StationDetection &message);
  void consume_detections();
  bool localized() const;
  void transfer();
  void tick();
  void finish(const std::string &error);
  void fault_command(const factory_interfaces::msg::FaultCommand &message);
  bool reject_navigation();
  void fault_event(const std::string &name,
                   const factory_interfaces::msg::FaultCommand &control);
  void event(const std::string &name, const std::string &outcome = "",
             const std::string &detail = "", const std::string &mission = "");
  std::string state_name() const;
  std::string phase(const std::string &prefix) const;
  MissionStateMachine machine_;
  NavigationAdapter navigation_;
  rclcpp_action::Server<Mission>::SharedPtr server_;
  std::shared_ptr<Handle> goal_;
  rclcpp::Client<Transfer>::SharedPtr transfer_client_;
  rclcpp::Publisher<factory_interfaces::msg::ProtocolEvent>::SharedPtr events_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr states_;
  rclcpp::Subscription<factory_interfaces::msg::StationDetection>::SharedPtr
      detections_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      localization_;
  rclcpp::Subscription<factory_interfaces::msg::FaultCommand>::SharedPtr
      fault_commands_;
  rclcpp::Publisher<factory_interfaces::msg::FaultCommand>::SharedPtr fault_acks_;
  struct NavigationFault {
    factory_interfaces::msg::FaultCommand command;
    int remaining;
    bool activated{false};
    std::chrono::steady_clock::time_point started;
  };
  std::vector<NavigationFault> navigation_faults_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::map<std::string, std::vector<double>> poses_;
  geometry_msgs::msg::Pose pose_;
  bool pose_valid_{false}, reserved_{false}, navigation_completed_{false};
  bool restart_required_{false};
  uint64_t mission_generation_{0};
  double phase_started_{0}, localization_started_{0}, timeout_{10},
      navigation_timeout_{120};
  double last_stamp_{-1};
  double pose_stamp_{0}, received_stamp_{-1};
  std::deque<double> stamps_;
  std::deque<factory_interfaces::msg::StationDetection> pending_detections_;
  std::string active_phase_, deferred_error_;
};
} // namespace mission_coordinator
