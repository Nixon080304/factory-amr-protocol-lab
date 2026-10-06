// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "mission_coordinator/mission_state_machine.hpp"
#include "mission_coordinator/resource_adapter.hpp"

namespace mission_coordinator {
// Production dispatch gate shared by the ROS coordinator and DDS-free contracts.
// The caller supplies the physical effect; it runs only after exact lease authority.
class MissionResourceGate {
public:
  MissionResourceGate(ResourceAdapter &resources, MissionStateMachine &machine)
      : state_(std::make_shared<State>(resources, machine)) {}
  void acquire(const std::string &resource, std::function<void()> effect,
               ResourceAdapter::Notice notice) {
    const auto mission = state_->machine.mission();
    if (!state_->active.empty() || !state_->pending.empty()) {
      notice({ResourceState::Lost,
              {mission.robot_id, mission.mission_id, resource, ""},
              0,
              "hold-and-wait forbidden"});
      return;
    }
    state_->pending = resource;
    const auto generation = state_->generation;
    std::weak_ptr<State> weak = state_;
    state_->resources.acquire(
        resource, mission.mission_id,
        [weak, generation, mission, resource, effect,
         notice](const ResourceNotice &value) {
          auto state = weak.lock();
          if (!state || generation != state->generation ||
              state->machine.mission().mission_id != mission.mission_id)
            return;
          if (value.state == ResourceState::Released)
            state->active.clear();
          const auto phase = state->machine.state();
          if (phase == MissionState::Idle || phase == MissionState::Completed ||
              phase == MissionState::Failed)
            return;
          if (value.state == ResourceState::Granted) {
            if (!state->resources.authorized(resource, mission.mission_id)) {
              auto failed = value;
              failed.state = ResourceState::Lost;
              failed.reason = "lease cannot authorize physical entry";
              notice(failed);
              return;
            }
            state->pending.clear();
            state->active = resource;
          }
          notice(value);
          const auto current_phase = state->machine.state();
          if (value.state == ResourceState::Granted &&
              generation == state->generation &&
              current_phase != MissionState::Failed &&
              current_phase != MissionState::Completed &&
              current_phase != MissionState::Idle) {
            if (state->resources.enter(resource, mission.mission_id))
              effect();
            else {
              auto failed = value;
              failed.state = ResourceState::Lost;
              failed.reason = "lease lost before physical effect";
              notice(failed);
            }
          }
        });
  }
  bool cancel() {
    ++state_->generation;
    state_->active.clear();
    state_->pending.clear();
    return state_->resources.release_all(state_->machine.mission().mission_id);
  }

private:
  struct State {
    State(ResourceAdapter &adapter, MissionStateMachine &local_machine)
        : resources(adapter), machine(local_machine) {}
    ResourceAdapter &resources;
    MissionStateMachine &machine;
    uint64_t generation{0};
    std::string active, pending;
  };
  std::shared_ptr<State> state_;
};
} // namespace mission_coordinator
