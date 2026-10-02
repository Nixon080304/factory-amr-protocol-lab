// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <array>

#include "mission_coordinator/mission_state_machine.hpp"

namespace mission_coordinator {
namespace {

const MissionRequest kRequest{"M-001", "amr_01", "assembly", "inspection", "motor"};

void expect_transition(const TransitionResult & result, MissionState state)
{
  EXPECT_TRUE(result.accepted);
  EXPECT_EQ(result.state, state);
  EXPECT_TRUE(result.error_code.empty());
}

TEST(MissionStateMachine, HappyPathVisitsEachStateAndStation)
{
  MissionStateMachine machine;
  EXPECT_EQ(machine.state(), MissionState::Idle);
  expect_transition(machine.start(kRequest), MissionState::Received);
  EXPECT_EQ(machine.mission().mission_id, "M-001");
  EXPECT_EQ(machine.mission().robot_id, "amr_01");
  EXPECT_EQ(machine.mission().pickup_station, "assembly");
  EXPECT_EQ(machine.mission().dropoff_station, "inspection");
  EXPECT_EQ(machine.mission().part, "motor");
  EXPECT_EQ(machine.current_station(), "assembly");
  expect_transition(machine.begin_navigation(), MissionState::NavigatingToPickup);
  expect_transition(machine.navigation_succeeded(), MissionState::VerifyingPickup);
  expect_transition(machine.station_confirmed(), MissionState::Loading);
  expect_transition(machine.transfer_succeeded(), MissionState::NavigatingToDropoff);
  EXPECT_EQ(machine.current_station(), "inspection");
  expect_transition(machine.navigation_succeeded(), MissionState::VerifyingDropoff);
  expect_transition(machine.station_confirmed(), MissionState::Unloading);
  expect_transition(machine.transfer_succeeded(), MissionState::Completed);
}

TEST(MissionStateMachine, TransferSuccessCannotSkipNavigation)
{
  MissionStateMachine machine;
  ASSERT_TRUE(machine.start(kRequest).accepted);
  ASSERT_TRUE(machine.begin_navigation().accepted);
  const auto result = machine.transfer_succeeded();
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.state, MissionState::NavigatingToPickup);
  EXPECT_EQ(machine.state(), MissionState::NavigatingToPickup);
  EXPECT_EQ(result.error_code, "INVALID_TRANSITION");
}

MissionStateMachine machine_at(MissionState target)
{
  MissionStateMachine machine;
  if (target == MissionState::Idle) {
    return machine;
  }
  expect_transition(machine.start(kRequest), MissionState::Received);
  if (target == MissionState::Received) {
    return machine;
  }
  expect_transition(machine.begin_navigation(), MissionState::NavigatingToPickup);
  if (target == MissionState::NavigatingToPickup) {
    return machine;
  }
  if (target == MissionState::Recovering) {
    expect_transition(machine.navigation_failed(), MissionState::Recovering);
    return machine;
  }
  expect_transition(machine.navigation_succeeded(), MissionState::VerifyingPickup);
  if (target == MissionState::VerifyingPickup) {
    return machine;
  }
  if (target == MissionState::Failed) {
    const auto result = machine.perception_timed_out();
    EXPECT_TRUE(result.accepted);
    EXPECT_EQ(result.state, MissionState::Failed);
    EXPECT_EQ(result.error_code, "STATION_NOT_CONFIRMED");
    return machine;
  }
  expect_transition(machine.station_confirmed(), MissionState::Loading);
  if (target == MissionState::Loading) {
    return machine;
  }
  expect_transition(machine.transfer_succeeded(), MissionState::NavigatingToDropoff);
  if (target == MissionState::NavigatingToDropoff) {
    return machine;
  }
  expect_transition(machine.navigation_succeeded(), MissionState::VerifyingDropoff);
  if (target == MissionState::VerifyingDropoff) {
    return machine;
  }
  expect_transition(machine.station_confirmed(), MissionState::Unloading);
  if (target == MissionState::Unloading) {
    return machine;
  }
  expect_transition(machine.transfer_succeeded(), MissionState::Completed);
  return machine;
}

TEST(MissionStateMachine, EachNavigationLegRecoversOnceAtItsCurrentStation)
{
  for (const auto state : {MissionState::NavigatingToPickup, MissionState::NavigatingToDropoff}) {
    auto machine = machine_at(state);
    const std::string station = state == MissionState::NavigatingToPickup ? "assembly" : "inspection";
    EXPECT_EQ(machine.navigation_retry_count(), 0U);
    expect_transition(machine.navigation_failed(), MissionState::Recovering);
    EXPECT_EQ(machine.current_station(), station);
    EXPECT_EQ(machine.navigation_retry_count(), 1U);
    expect_transition(machine.recovery_ready(), state);
    EXPECT_EQ(machine.current_station(), station);
    EXPECT_EQ(machine.navigation_retry_count(), 1U);
  }
}

TEST(MissionStateMachine, SecondFailureOnEitherNavigationLegIsTerminal)
{
  for (const auto state : {MissionState::NavigatingToPickup, MissionState::NavigatingToDropoff}) {
    auto machine = machine_at(state);
    ASSERT_TRUE(machine.navigation_failed().accepted);
    ASSERT_TRUE(machine.recovery_ready().accepted);
    const auto result = machine.navigation_failed();
    EXPECT_TRUE(result.accepted);
    EXPECT_EQ(result.state, MissionState::Failed);
    EXPECT_EQ(result.error_code, "NAVIGATION_FAILED");
    EXPECT_EQ(machine.navigation_retry_count(), 1U);
  }
}

TEST(MissionStateMachine, PickupRetryDoesNotConsumeDropoffRetry)
{
  auto machine = machine_at(MissionState::NavigatingToPickup);
  expect_transition(machine.navigation_failed(), MissionState::Recovering);
  expect_transition(machine.recovery_ready(), MissionState::NavigatingToPickup);
  expect_transition(machine.navigation_succeeded(), MissionState::VerifyingPickup);
  expect_transition(machine.station_confirmed(), MissionState::Loading);
  expect_transition(machine.transfer_succeeded(), MissionState::NavigatingToDropoff);
  EXPECT_EQ(machine.navigation_retry_count(), 0U);
  expect_transition(machine.navigation_failed(), MissionState::Recovering);
  expect_transition(machine.recovery_ready(), MissionState::NavigatingToDropoff);
  expect_transition(machine.navigation_succeeded(), MissionState::VerifyingDropoff);
  expect_transition(machine.station_confirmed(), MissionState::Unloading);
  expect_transition(machine.transfer_succeeded(), MissionState::Completed);
}

TEST(MissionStateMachine, PerceptionTimeoutFailsEitherStationVerification)
{
  for (const auto state : {MissionState::VerifyingPickup, MissionState::VerifyingDropoff}) {
    auto machine = machine_at(state);
    const auto result = machine.perception_timed_out();
    EXPECT_TRUE(result.accepted);
    EXPECT_EQ(result.state, MissionState::Failed);
    EXPECT_EQ(machine.state(), MissionState::Failed);
    EXPECT_EQ(result.error_code, "STATION_NOT_CONFIRMED");
  }
}

TEST(MissionStateMachine, TransferFailurePreservesStableCodeAtEitherStation)
{
  for (const auto state : {MissionState::Loading, MissionState::Unloading}) {
    for (const auto * code : {"PLC_TIMEOUT", "STATION_FAULT", "HANDSHAKE_INVALID"}) {
      auto machine = machine_at(state);
      const auto result = machine.transfer_failed(code);
      EXPECT_TRUE(result.accepted);
      EXPECT_EQ(result.state, MissionState::Failed);
      EXPECT_EQ(machine.state(), MissionState::Failed);
      EXPECT_EQ(result.error_code, code);
    }
  }
}

TEST(MissionStateMachine, CancellationBeforeTransferEndsImmediately)
{
  for (const auto state : {MissionState::Received, MissionState::NavigatingToPickup,
    MissionState::VerifyingPickup, MissionState::NavigatingToDropoff,
    MissionState::VerifyingDropoff, MissionState::Recovering})
  {
    auto machine = machine_at(state);
    const auto station = machine.current_station();
    const auto result = machine.cancel();
    EXPECT_TRUE(result.accepted);
    EXPECT_EQ(result.state, MissionState::Failed);
    EXPECT_EQ(machine.state(), MissionState::Failed);
    EXPECT_EQ(result.error_code, "MISSION_CANCELED");
    EXPECT_EQ(machine.current_station(), station);
  }
}

TEST(MissionStateMachine, CancellationWaitsForTransferSuccessAtEitherStation)
{
  for (const auto state : {MissionState::Loading, MissionState::Unloading}) {
    auto machine = machine_at(state);
    const auto station = machine.current_station();
    expect_transition(machine.cancel(), state);
    EXPECT_EQ(machine.state(), state);
    EXPECT_EQ(machine.current_station(), station);
    expect_transition(machine.cancel(), state);
    const auto result = machine.transfer_succeeded();
    EXPECT_TRUE(result.accepted);
    EXPECT_EQ(result.state, MissionState::Failed);
    EXPECT_EQ(result.error_code, "MISSION_CANCELED");
    EXPECT_EQ(machine.current_station(), station);
  }
}

TEST(MissionStateMachine, CancellationWaitsForTransferFailureAndPreservesError)
{
  for (const auto state : {MissionState::Loading, MissionState::Unloading}) {
    auto machine = machine_at(state);
    expect_transition(machine.cancel(), state);
    EXPECT_EQ(machine.state(), state);
    const auto result = machine.transfer_failed("PLC_TIMEOUT");
    EXPECT_TRUE(result.accepted);
    EXPECT_EQ(result.state, MissionState::Failed);
    EXPECT_EQ(result.error_code, "PLC_TIMEOUT");
  }
}

TEST(MissionStateMachine, ActiveMissionRejectsStartWithoutReplacingRequest)
{
  const MissionRequest other{"M-002", "amr_02", "inspection", "assembly", "wheel"};
  for (const auto state : {MissionState::Received, MissionState::NavigatingToPickup,
    MissionState::VerifyingPickup, MissionState::Loading, MissionState::NavigatingToDropoff,
    MissionState::VerifyingDropoff, MissionState::Unloading, MissionState::Recovering})
  {
    auto machine = machine_at(state);
    const auto station = machine.current_station();
    const auto result = machine.start(other);
    EXPECT_FALSE(result.accepted);
    EXPECT_EQ(result.state, state);
    EXPECT_EQ(result.error_code, "ROBOT_BUSY");
    EXPECT_EQ(machine.mission().mission_id, "M-001");
    EXPECT_EQ(machine.current_station(), station);
  }
}

using Event = TransitionResult (MissionStateMachine::*)();

struct GuardCase {
  Event event;
  std::array<MissionState, 2> allowed;
};

TEST(MissionStateMachine, EventsRejectEveryUnrelatedStateWithoutMutation)
{
  const std::array<MissionState, 11> states{{MissionState::Idle, MissionState::Received,
    MissionState::NavigatingToPickup, MissionState::VerifyingPickup, MissionState::Loading,
    MissionState::NavigatingToDropoff, MissionState::VerifyingDropoff, MissionState::Unloading,
    MissionState::Recovering, MissionState::Completed, MissionState::Failed}};
  const std::array<GuardCase, 7> guards{{
    {&MissionStateMachine::begin_navigation, {MissionState::Received, MissionState::Received}},
    {&MissionStateMachine::navigation_succeeded,
      {MissionState::NavigatingToPickup, MissionState::NavigatingToDropoff}},
    {&MissionStateMachine::station_confirmed,
      {MissionState::VerifyingPickup, MissionState::VerifyingDropoff}},
    {&MissionStateMachine::transfer_succeeded, {MissionState::Loading, MissionState::Unloading}},
    {&MissionStateMachine::navigation_failed,
      {MissionState::NavigatingToPickup, MissionState::NavigatingToDropoff}},
    {&MissionStateMachine::recovery_ready, {MissionState::Recovering, MissionState::Recovering}},
    {&MissionStateMachine::perception_timed_out,
      {MissionState::VerifyingPickup, MissionState::VerifyingDropoff}},
  }};
  for (const auto state : states) {
    for (const auto & guard : guards) {
      if (state == guard.allowed[0] || state == guard.allowed[1]) {
        continue;
      }
      auto machine = machine_at(state);
      const auto station = machine.current_station();
      const auto retry_count = machine.navigation_retry_count();
      const auto result = (machine.*guard.event)();
      EXPECT_FALSE(result.accepted) << static_cast<int>(state);
      EXPECT_EQ(result.state, state);
      EXPECT_EQ(result.error_code, "INVALID_TRANSITION");
      EXPECT_EQ(machine.state(), state);
      EXPECT_EQ(machine.current_station(), station);
      EXPECT_EQ(machine.navigation_retry_count(), retry_count);
    }
    if (state != MissionState::Loading && state != MissionState::Unloading) {
      auto machine = machine_at(state);
      const auto result = machine.transfer_failed("PLC_TIMEOUT");
      EXPECT_FALSE(result.accepted);
      EXPECT_EQ(result.state, state);
      EXPECT_EQ(result.error_code, "INVALID_TRANSITION");
    }
  }
}

TEST(MissionStateMachine, TerminalAndIdleStatesRejectCancel)
{
  for (const auto state : {MissionState::Idle, MissionState::Completed, MissionState::Failed}) {
    auto machine = machine_at(state);
    const auto result = machine.cancel();
    EXPECT_FALSE(result.accepted);
    EXPECT_EQ(result.state, state);
    EXPECT_EQ(result.error_code, "INVALID_TRANSITION");
  }
}

TEST(MissionStateMachine, TerminalStatesRejectStartUntilExplicitReset)
{
  for (const auto state : {MissionState::Completed, MissionState::Failed}) {
    auto machine = machine_at(state);
    const auto result = machine.start(kRequest);
    EXPECT_FALSE(result.accepted);
    EXPECT_EQ(result.state, state);
    EXPECT_EQ(result.error_code, "INVALID_TRANSITION");
    expect_transition(machine.reset(), MissionState::Idle);
    EXPECT_TRUE(machine.current_station().empty());
    EXPECT_TRUE(machine.mission().mission_id.empty());
    EXPECT_TRUE(machine.mission().robot_id.empty());
    EXPECT_TRUE(machine.mission().pickup_station.empty());
    EXPECT_TRUE(machine.mission().dropoff_station.empty());
    EXPECT_TRUE(machine.mission().part.empty());
    expect_transition(machine.start(kRequest), MissionState::Received);
  }
}

TEST(MissionStateMachine, ResetClearsPendingCancelAndBothNavigationRetries)
{
  auto machine = machine_at(MissionState::NavigatingToPickup);
  ASSERT_TRUE(machine.navigation_failed().accepted);
  ASSERT_TRUE(machine.recovery_ready().accepted);
  ASSERT_TRUE(machine.navigation_succeeded().accepted);
  ASSERT_TRUE(machine.station_confirmed().accepted);
  ASSERT_TRUE(machine.transfer_succeeded().accepted);
  ASSERT_TRUE(machine.navigation_failed().accepted);
  ASSERT_TRUE(machine.recovery_ready().accepted);
  ASSERT_TRUE(machine.navigation_succeeded().accepted);
  ASSERT_TRUE(machine.station_confirmed().accepted);
  ASSERT_TRUE(machine.cancel().accepted);
  ASSERT_TRUE(machine.transfer_succeeded().accepted);
  expect_transition(machine.reset(), MissionState::Idle);
  EXPECT_EQ(machine.navigation_retry_count(), 0U);
  expect_transition(machine.start(kRequest), MissionState::Received);
  expect_transition(machine.begin_navigation(), MissionState::NavigatingToPickup);
  expect_transition(machine.navigation_failed(), MissionState::Recovering);
  expect_transition(machine.recovery_ready(), MissionState::NavigatingToPickup);
  expect_transition(machine.navigation_succeeded(), MissionState::VerifyingPickup);
  expect_transition(machine.station_confirmed(), MissionState::Loading);
  expect_transition(machine.transfer_succeeded(), MissionState::NavigatingToDropoff);
  EXPECT_EQ(machine.navigation_retry_count(), 0U);
  expect_transition(machine.navigation_failed(), MissionState::Recovering);
  expect_transition(machine.recovery_ready(), MissionState::NavigatingToDropoff);
  expect_transition(machine.navigation_succeeded(), MissionState::VerifyingDropoff);
  expect_transition(machine.station_confirmed(), MissionState::Unloading);
  expect_transition(machine.transfer_succeeded(), MissionState::Completed);
}

TEST(MissionStateMachine, ResetCannotDiscardAnActiveMission)
{
  for (const auto state : {MissionState::Idle, MissionState::Received,
    MissionState::NavigatingToPickup, MissionState::VerifyingPickup, MissionState::Loading,
    MissionState::NavigatingToDropoff, MissionState::VerifyingDropoff, MissionState::Unloading,
    MissionState::Recovering})
  {
    auto machine = machine_at(state);
    const auto result = machine.reset();
    EXPECT_FALSE(result.accepted);
    EXPECT_EQ(result.state, state);
    EXPECT_EQ(result.error_code, "INVALID_TRANSITION");
    EXPECT_EQ(machine.state(), state);
  }
}

}  // namespace
}  // namespace mission_coordinator
