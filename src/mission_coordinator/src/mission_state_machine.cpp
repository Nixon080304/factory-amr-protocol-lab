// SPDX-License-Identifier: Apache-2.0
#include "mission_coordinator/mission_state_machine.hpp"

namespace mission_coordinator {

TransitionResult MissionStateMachine::start(const MissionRequest &request) {
  switch (state_) {
  case MissionState::Idle:
    mission_ = request;
    current_station_ = mission_.pickup_station;
    return transition_to(MissionState::Received);
  case MissionState::Completed:
  case MissionState::Failed:
    return invalid_transition();
  default:
    return {false, state_, "ROBOT_BUSY"};
  }
}

TransitionResult MissionStateMachine::begin_navigation() {
  switch (state_) {
  case MissionState::Received:
    return transition_to(MissionState::NavigatingToPickup);
  default:
    return invalid_transition();
  }
}

TransitionResult MissionStateMachine::navigation_succeeded() {
  switch (state_) {
  case MissionState::NavigatingToPickup:
    return transition_to(MissionState::VerifyingPickup);
  case MissionState::NavigatingToDropoff:
    return transition_to(MissionState::VerifyingDropoff);
  default:
    return invalid_transition();
  }
}

TransitionResult MissionStateMachine::station_confirmed() {
  switch (state_) {
  case MissionState::VerifyingPickup:
    return transition_to(MissionState::Loading);
  case MissionState::VerifyingDropoff:
    return transition_to(MissionState::Unloading);
  default:
    return invalid_transition();
  }
}

TransitionResult MissionStateMachine::transfer_succeeded() {
  switch (state_) {
  case MissionState::Loading:
    if (cancel_pending_) {
      return transition_to(MissionState::Failed, "MISSION_CANCELED");
    }
    current_station_ = mission_.dropoff_station;
    navigation_leg_ = MissionState::NavigatingToDropoff;
    return transition_to(MissionState::NavigatingToDropoff);
  case MissionState::Unloading:
    if (cancel_pending_) {
      return transition_to(MissionState::Failed, "MISSION_CANCELED");
    }
    return transition_to(MissionState::Completed);
  default:
    return invalid_transition();
  }
}

TransitionResult MissionStateMachine::navigation_failed() {
  switch (state_) {
  case MissionState::NavigatingToPickup:
    if (pickup_navigation_retries_ == 0) {
      ++pickup_navigation_retries_;
      return transition_to(MissionState::Recovering);
    }
    return transition_to(MissionState::Failed, "NAVIGATION_FAILED");
  case MissionState::NavigatingToDropoff:
    if (dropoff_navigation_retries_ == 0) {
      ++dropoff_navigation_retries_;
      return transition_to(MissionState::Recovering);
    }
    return transition_to(MissionState::Failed, "NAVIGATION_FAILED");
  default:
    return invalid_transition();
  }
}

TransitionResult MissionStateMachine::recovery_ready() {
  switch (state_) {
  case MissionState::Recovering:
    return transition_to(navigation_leg_);
  default:
    return invalid_transition();
  }
}

TransitionResult MissionStateMachine::perception_timed_out() {
  switch (state_) {
  case MissionState::VerifyingPickup:
  case MissionState::VerifyingDropoff:
    return transition_to(MissionState::Failed, "STATION_NOT_CONFIRMED");
  default:
    return invalid_transition();
  }
}

TransitionResult MissionStateMachine::transfer_failed(const std::string &error_code) {
  switch (state_) {
  case MissionState::Loading:
  case MissionState::Unloading:
    // A transfer failure takes precedence over a pending cancellation.
    return transition_to(MissionState::Failed, error_code);
  default:
    return invalid_transition();
  }
}

TransitionResult MissionStateMachine::cancel() {
  switch (state_) {
  case MissionState::Loading:
  case MissionState::Unloading:
    cancel_pending_ = true;
    return {true, state_, ""};
  case MissionState::Received:
  case MissionState::NavigatingToPickup:
  case MissionState::VerifyingPickup:
  case MissionState::NavigatingToDropoff:
  case MissionState::VerifyingDropoff:
  case MissionState::Recovering:
    return transition_to(MissionState::Failed, "MISSION_CANCELED");
  default:
    return invalid_transition();
  }
}

TransitionResult MissionStateMachine::reset() {
  switch (state_) {
  case MissionState::Completed:
  case MissionState::Failed:
    mission_ = {};
    current_station_.clear();
    navigation_leg_ = MissionState::NavigatingToPickup;
    pickup_navigation_retries_ = 0;
    dropoff_navigation_retries_ = 0;
    cancel_pending_ = false;
    return transition_to(MissionState::Idle);
  default:
    return invalid_transition();
  }
}

MissionState MissionStateMachine::state() const { return state_; }

const std::string &MissionStateMachine::current_station() const {
  return current_station_;
}

const MissionRequest &MissionStateMachine::mission() const { return mission_; }

unsigned int MissionStateMachine::navigation_retry_count() const {
  switch (navigation_leg_) {
  case MissionState::NavigatingToDropoff:
    return dropoff_navigation_retries_;
  default:
    return pickup_navigation_retries_;
  }
}

TransitionResult MissionStateMachine::transition_to(MissionState state,
                                                    const std::string &error_code) {
  state_ = state;
  return {true, state_, error_code};
}

TransitionResult MissionStateMachine::invalid_transition() const {
  return {false, state_, "INVALID_TRANSITION"};
}

} // namespace mission_coordinator
