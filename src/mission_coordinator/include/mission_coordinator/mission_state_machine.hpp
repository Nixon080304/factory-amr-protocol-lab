// SPDX-License-Identifier: Apache-2.0
#ifndef MISSION_COORDINATOR__MISSION_STATE_MACHINE_HPP_
#define MISSION_COORDINATOR__MISSION_STATE_MACHINE_HPP_

#include <string>

namespace mission_coordinator {

enum class MissionState {
  Idle,
  Received,
  NavigatingToPickup,
  VerifyingPickup,
  Loading,
  NavigatingToDropoff,
  VerifyingDropoff,
  Unloading,
  Recovering,
  Completed,
  Failed,
};

struct MissionRequest {
  std::string mission_id;
  std::string robot_id;
  std::string pickup_station;
  std::string dropoff_station;
  std::string part;
};

struct TransitionResult {
  bool accepted;
  MissionState state;
  std::string error_code;
};

class MissionStateMachine {
public:
  // Callers serialize events and validate requests before starting a mission.
  TransitionResult start(const MissionRequest &request);
  TransitionResult begin_navigation();
  TransitionResult navigation_succeeded();
  TransitionResult station_confirmed();
  TransitionResult transfer_succeeded();
  TransitionResult navigation_failed();
  TransitionResult recovery_ready();
  TransitionResult perception_timed_out();
  TransitionResult transfer_failed(const std::string &error_code);
  // Transfer cancellation remains pending until a transfer outcome is received.
  TransitionResult cancel();
  // Terminal missions require an explicit reset before another start.
  TransitionResult reset();

  MissionState state() const;
  const std::string &current_station() const;
  const MissionRequest &mission() const;
  // The current navigation leg owns its own single retry budget.
  unsigned int navigation_retry_count() const;

private:
  TransitionResult transition_to(MissionState state,
                                 const std::string &error_code = "");
  TransitionResult invalid_transition() const;

  MissionState state_{MissionState::Idle};
  MissionRequest mission_;
  std::string current_station_;
  MissionState navigation_leg_{MissionState::NavigatingToPickup};
  unsigned int pickup_navigation_retries_{0};
  unsigned int dropoff_navigation_retries_{0};
  bool cancel_pending_{false};
};

} // namespace mission_coordinator

#endif // MISSION_COORDINATOR__MISSION_STATE_MACHINE_HPP_
