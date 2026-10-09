// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "mission_coordinator/mission_state_machine.hpp"
#include "mission_coordinator/resource_adapter.hpp"
#include <set>

namespace mission_coordinator {
// Production dispatch gate shared by the ROS coordinator and DDS-free contracts.
// The caller supplies the physical effect; it runs only after exact lease authority.
class MissionResourceGate {
public:
  MissionResourceGate(ResourceAdapter &resources, MissionStateMachine &machine)
      : state_(std::make_shared<State>(resources, machine)) {}
  void acquire(const std::string &resource, std::function<void()> effect,
               ResourceAdapter::Notice notice,
               const std::string &departure_source = "") {
    const auto mission = state_->machine.mission();
    // Only the coordinator's station-to-traffic departure may hold two leases.
    // This keeps the source protected until physical clearance is proven.
    const bool departure =
        !departure_source.empty() && state_->active.size() == 1 &&
        state_->active.count(departure_source) && resource != departure_source &&
        state_->resources.authorized(departure_source, mission.mission_id);
    if ((!state_->active.empty() && !departure) || !state_->pending.empty()) {
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
        [weak, generation, mission, resource, departure_source, effect,
         notice](const ResourceNotice &value) {
          auto state = weak.lock();
          if (!state || generation != state->generation ||
              state->machine.mission().mission_id != mission.mission_id)
            return;
          if (value.state == ResourceState::Released)
            state->active.erase(value.lease.resource_id);
          const auto phase = state->machine.state();
          if (phase == MissionState::Idle || phase == MissionState::Completed ||
              phase == MissionState::Failed)
            return;
          if (value.state == ResourceState::Granted) {
            if (!state->resources.authorized(resource, mission.mission_id) ||
                (!departure_source.empty() &&
                 !state->resources.authorized(departure_source, mission.mission_id))) {
              auto failed = value;
              failed.state = ResourceState::Lost;
              failed.reason = "lease cannot authorize physical entry";
              notice(failed);
              return;
            }
            state->pending.clear();
            state->active.insert(resource);
          }
          notice(value);
          const auto current_phase = state->machine.state();
          if (value.state == ResourceState::Granted &&
              generation == state->generation &&
              current_phase != MissionState::Failed &&
              current_phase != MissionState::Completed &&
              current_phase != MissionState::Idle) {
            if ((departure_source.empty() ||
                 state->resources.authorized(departure_source, mission.mission_id)) &&
                state->resources.enter(resource, mission.mission_id))
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
  void release(const std::string &resource, std::function<void(bool)> continuation) {
    const auto mission = state_->machine.mission().mission_id;
    const auto generation = state_->generation;
    std::weak_ptr<State> weak = state_;
    state_->resources.release(resource, mission,
                              [weak, generation, mission, continuation](bool cleared) {
                                auto state = weak.lock();
                                if (state && generation == state->generation &&
                                    state->machine.mission().mission_id == mission)
                                  continuation(cleared);
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
    std::set<std::string> active;
    std::string pending;
  };
  std::shared_ptr<State> state_;
};
} // namespace mission_coordinator
