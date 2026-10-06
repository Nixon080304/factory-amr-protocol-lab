// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include "mission_coordinator/resource_adapter.hpp"
#include "mission_coordinator/mission_state_machine.hpp"
#include "mission_coordinator/mission_resource_gate.hpp"
#include "mission_coordinator/traffic_boundary.hpp"
#include <deque>
using namespace mission_coordinator;
TEST(TrafficBoundary, ClearanceIncludesFootprintAndArrivalTolerance) {
  TrafficBoundary boundary({-0.8, -2.65, 0.8, -1.75});
  EXPECT_TRUE(boundary.outside(-1.5, -2.2, 0.60));
  EXPECT_TRUE(boundary.outside(1.5, -2.2, 0.60));
  EXPECT_FALSE(boundary.outside(1.1, -2.2, 0.35));
  EXPECT_FALSE(boundary.outside(0, -2.2, 0.35));
  EXPECT_THROW(TrafficBoundary({0, 1, 0, 2}), std::invalid_argument);
}

// Replace only service I/O and time. Every lease decision uses the production adapter.
struct Wire : ResourceTransport {
  struct Call {
    ResourceOperation operation;
    LeaseIdentity key;
    Reply callback;
  };
  std::deque<Call> calls;
  bool ready{true};
  unsigned canceled{0};
  bool available(ResourceOperation) const override { return ready; }
  Cancel send(ResourceOperation operation, const LeaseIdentity &key,
              Reply callback) override {
    calls.push_back({operation, key, std::move(callback)});
    return [this] { ++canceled; };
  }
  Call take() {
    auto call = calls.front();
    calls.pop_front();
    return call;
  }
  void answer(bool ok, std::string id = "unguessable-central-token", double ttl = 10,
              std::string reason = "") {
    take().callback({ok, std::move(id), ttl, std::move(reason)});
  }
};
struct LeaseTest : testing::Test {
  double time{100};
  std::shared_ptr<Wire> wire{std::make_shared<Wire>()};
  ResourceAdapter adapter{wire, "cart_10", [this] { return time; }};
  std::vector<ResourceNotice> notices;
  void acquire(std::string resource = "central_aisle", std::string mission = "m1") {
    adapter.acquire(resource, mission,
                    [this](const auto &notice) { notices.push_back(notice); });
  }
  void advance(double seconds) {
    time += seconds;
    adapter.tick();
  }
};
TEST_F(LeaseTest, CentralGrantAuthorizesOnlyExactMissionResourceAndRobot) {
  acquire();
  ASSERT_EQ(wire->calls.size(), 1u);
  EXPECT_EQ(wire->calls.front().key.robot_id, "cart_10");
  EXPECT_EQ(wire->calls.front().key.mission_id, "m1");
  EXPECT_EQ(wire->calls.front().key.resource_id, "central_aisle");
  EXPECT_FALSE(adapter.authorized("central_aisle", "m1"));
  wire->answer(true);
  EXPECT_TRUE(adapter.authorized("central_aisle", "m1"));
  EXPECT_FALSE(adapter.authorized("central_aisle", "other"));
  EXPECT_EQ(notices.back().state, ResourceState::Granted);
  EXPECT_DOUBLE_EQ(notices.back().expires_at, 110);
}
TEST_F(LeaseTest, DenialRetriesAtPacedIntervalsWithoutBlocking) {
  acquire();
  wire->answer(false, "", 0, "owned by cart_11");
  EXPECT_EQ(notices.back().state, ResourceState::Waiting);
  for (unsigned i = 0; i < 100; ++i)
    adapter.tick();
  EXPECT_TRUE(wire->calls.empty());
  advance(0.25);
  ASSERT_EQ(wire->calls.size(), 1u);
  wire->answer(true);
  EXPECT_TRUE(adapter.authorized("central_aisle", "m1"));
}
TEST_F(LeaseTest, UnavailableServiceHasBoundedPacedAttempts) {
  wire->ready = false;
  acquire();
  for (unsigned i = 0; i < 120; ++i)
    advance(0.25);
  EXPECT_TRUE(wire->calls.empty());
  EXPECT_EQ(notices.back().state, ResourceState::Lost);
  EXPECT_FALSE(adapter.authorized("central_aisle", "m1"));
}
TEST_F(LeaseTest, HealthyOwnerCanCrossForFifteenSecondsBeforeWaiterGetsGrant) {
  acquire();
  wire->answer(false, "", 0, "owned");
  for (unsigned i = 0; i < 60; ++i) {
    advance(0.25);
    if (!wire->calls.empty())
      wire->answer(false, "", 0, "owned");
  }
  EXPECT_EQ(notices.back().state, ResourceState::Waiting);
  advance(0.25);
  ASSERT_EQ(wire->calls.size(), 1u);
  wire->answer(true);
  EXPECT_TRUE(adapter.authorized("central_aisle", "m1"));
}
TEST_F(LeaseTest, RenewalUsesExactCentralLeaseAtOneThirdTtl) {
  acquire();
  wire->answer(true, "central-random-identity", 9);
  advance(2.9);
  EXPECT_TRUE(wire->calls.empty());
  advance(0.1);
  ASSERT_EQ(wire->calls.size(), 1u);
  EXPECT_EQ(wire->calls.front().operation, ResourceOperation::Renew);
  EXPECT_EQ(wire->calls.front().key.lease_id, "central-random-identity");
  wire->answer(true, "", 9);
  advance(3);
  ASSERT_EQ(wire->calls.size(), 1u);
  EXPECT_EQ(wire->calls.front().operation, ResourceOperation::Renew);
}
TEST_F(LeaseTest, FailedRenewalStopsAuthorityAndNeverReleasesOccupiedResource) {
  acquire();
  wire->answer(true);
  ASSERT_TRUE(adapter.enter("central_aisle", "m1"));
  advance(4);
  wire->answer(false, "", 0, "lease mismatch");
  EXPECT_FALSE(adapter.authorized("central_aisle", "m1"));
  EXPECT_EQ(notices.back().state, ResourceState::Lost);
  EXPECT_FALSE(adapter.release_all("m1"));
  EXPECT_TRUE(wire->calls.empty());
}
TEST_F(LeaseTest, ExpiryFencesLateRenewalWithoutFabricatingClearance) {
  acquire();
  wire->answer(true, "lease", 3);
  ASSERT_TRUE(adapter.enter("central_aisle", "m1"));
  advance(1);
  auto renewal = wire->take();
  advance(2);
  renewal.callback({true, "", 10, "renewed"});
  EXPECT_FALSE(adapter.authorized("central_aisle", "m1"));
  EXPECT_FALSE(adapter.release_all("m1"));
  EXPECT_EQ(notices.back().state, ResourceState::Lost);
}
TEST_F(LeaseTest, ServiceLossRevokesAuthorityBeforeNextEffect) {
  acquire();
  wire->answer(true);
  wire->ready = false;
  adapter.tick();
  EXPECT_FALSE(adapter.enter("central_aisle", "m1"));
  EXPECT_EQ(notices.back().state, ResourceState::Lost);
}
TEST_F(LeaseTest, CancellationFencesLateGrantAndCleansExactSafeLease) {
  acquire();
  auto pending = wire->take();
  EXPECT_TRUE(adapter.release_all("m1"));
  ASSERT_EQ(wire->calls.size(), 1u);
  EXPECT_EQ(wire->calls.front().operation, ResourceOperation::CancelWait);
  EXPECT_TRUE(wire->calls.front().key.lease_id.empty());
  wire->answer(false, "", 0, "waiter cancelled");
  pending.callback({true, "late-central-token", 10, "granted"});
  ASSERT_EQ(wire->calls.size(), 1u);
  EXPECT_EQ(wire->calls.front().key.lease_id, "late-central-token");
  EXPECT_FALSE(adapter.authorized("central_aisle", "m1"));
  EXPECT_EQ(notices.back().state, ResourceState::Cancelled);
}
TEST_F(LeaseTest, CancellationFromWaitingNoticeCannotDispatchAcquireAfterCleanup) {
  adapter.acquire("central_aisle", "m1", [&](const auto &notice) {
    if (notice.state == ResourceState::Waiting)
      adapter.release_all("m1");
  });
  ASSERT_EQ(wire->calls.size(), 1u);
  EXPECT_EQ(wire->calls.front().operation, ResourceOperation::CancelWait);
}
TEST_F(LeaseTest, VerifiedExitRequiresConfirmedExternalReleaseBeforeNextEffect) {
  acquire();
  wire->answer(true);
  ASSERT_TRUE(adapter.enter("central_aisle", "m1"));
  bool cleared = false;
  adapter.release("central_aisle", "m1", [&](bool ok) { cleared = ok; });
  EXPECT_FALSE(cleared);
  EXPECT_TRUE(wire->calls.empty());
  adapter.exited("central_aisle", "m1");
  adapter.release("central_aisle", "m1", [&](bool ok) { cleared = ok; });
  EXPECT_FALSE(cleared);
  ASSERT_EQ(wire->calls.size(), 1u);
  EXPECT_EQ(wire->calls.front().key.lease_id, "unguessable-central-token");
  wire->answer(true, "", 0);
  EXPECT_TRUE(cleared);
  EXPECT_FALSE(adapter.authorized("central_aisle", "m1"));
}
TEST_F(LeaseTest, WrongLeaseReleaseCannotReportSafeClearance) {
  acquire();
  wire->answer(true);
  bool cleared = true;
  adapter.release("central_aisle", "m1", [&](bool ok) { cleared = ok; });
  wire->answer(false, "", 0, "lease mismatch");
  EXPECT_FALSE(cleared);
  EXPECT_EQ(notices.back().state, ResourceState::Lost);
}
TEST_F(LeaseTest, DelayedGrantUsesRequestTimeForExpiry) {
  acquire();
  time += 2;
  wire->answer(true, "lease", 1);
  EXPECT_FALSE(adapter.enter("central_aisle", "m1"));
  EXPECT_EQ(notices.back().state, ResourceState::Lost);
}
TEST_F(LeaseTest, MissingResponseTimesOutAndFencesPreviousAttempt) {
  acquire();
  auto old = wire->take();
  advance(1);
  advance(0.25);
  ASSERT_EQ(wire->calls.size(), 1u);
  wire->answer(true, "new-lease");
  old.callback({false, "", 0, "denied"});
  EXPECT_TRUE(adapter.authorized("central_aisle", "m1"));
  EXPECT_GE(wire->canceled, 1u);
}
TEST_F(LeaseTest, StaleGrantDuringRetryMustNotReleaseThePendingIdempotentLease) {
  // Releasing the old reply would revoke the same central token returned to the retry.
  acquire();
  auto old = wire->take();
  advance(1);
  advance(0.25);
  old.callback({true, "shared-central-token", 10, "granted"});
  ASSERT_EQ(wire->calls.size(), 1u);
  EXPECT_EQ(wire->calls.front().operation, ResourceOperation::Acquire);
  wire->answer(true, "shared-central-token");
  EXPECT_TRUE(adapter.enter("central_aisle", "m1"));
  EXPECT_TRUE(wire->calls.empty());
}
TEST_F(LeaseTest, CancelledMissionCannotReacquireAndRaceItsLateCleanup) {
  acquire();
  auto old = wire->take();
  adapter.release_all("m1");
  wire->take();
  acquire();
  EXPECT_TRUE(wire->calls.empty());
  EXPECT_EQ(notices.back().state, ResourceState::Cancelled);
  old.callback({true, "late-token", 10, "granted"});
  ASSERT_EQ(wire->calls.size(), 1u);
  EXPECT_EQ(wire->calls.front().operation, ResourceOperation::Release);
}
TEST_F(LeaseTest, CancelledAcquireDeniedAfterCancellationRemovesItsLateWaiter) {
  // Cross-service reordering can enqueue an acquisition after cancel_wait completed.
  acquire();
  auto old = wire->take();
  adapter.release_all("m1");
  wire->answer(false, "", 0, "no waiter");
  old.callback({false, "", 0, "owned by other robot"});
  ASSERT_EQ(wire->calls.size(), 1u);
  EXPECT_EQ(wire->calls.front().operation, ResourceOperation::CancelWait);
  EXPECT_EQ(notices.back().state, ResourceState::Cancelled);
}
TEST_F(LeaseTest, CancelWaitCleanupRetriesUnavailableServiceAtPacedIntervals) {
  acquire();
  wire->take();
  wire->ready = false;
  adapter.release_all("m1");
  EXPECT_TRUE(wire->calls.empty());
  wire->ready = true;
  advance(0.24);
  EXPECT_TRUE(wire->calls.empty());
  advance(0.01);
  ASSERT_EQ(wire->calls.size(), 1u);
  EXPECT_EQ(wire->calls.front().operation, ResourceOperation::CancelWait);
  wire->answer(true, "", 0, "waiter cancelled");
  advance(1);
  EXPECT_TRUE(wire->calls.empty());
}
TEST_F(LeaseTest, ReleaseAllCleansSafeStationButPreservesUncertainTraffic) {
  acquire();
  wire->answer(true);
  adapter.enter("central_aisle", "m1");
  acquire("assembly");
  wire->answer(true, "station-token");
  EXPECT_FALSE(adapter.release_all("m1"));
  ASSERT_EQ(wire->calls.size(), 1u);
  EXPECT_EQ(wire->calls.front().key.resource_id, "assembly");
  EXPECT_EQ(wire->calls.front().key.lease_id, "station-token");
}
TEST_F(LeaseTest, TerminalCleanupCannotClaimClearanceBeforeExternalAcknowledgement) {
  acquire("assembly");
  wire->answer(true, "station-token");
  EXPECT_FALSE(adapter.release_all("m1"));
  ASSERT_EQ(wire->calls.size(), 1u);
  EXPECT_EQ(wire->calls.front().operation, ResourceOperation::Release);
  wire->answer(false, "", 0, "central release unavailable");
  EXPECT_FALSE(adapter.release_all("m1"));
  EXPECT_EQ(notices.back().state, ResourceState::Lost);
  EXPECT_FALSE(adapter.authorized("assembly", "m1"));
}
TEST(ResourceMissionContract,
     TwoRobotContextsContendBeforeNavigationAndStationTransaction) {
  // Removing the acquire gate would let the second production machine navigate.
  double time = 100;
  auto first_wire = std::make_shared<Wire>(), second_wire = std::make_shared<Wire>();
  ResourceAdapter first(first_wire, "amr_01", [&] { return time; });
  ResourceAdapter second(second_wire, "amr_02", [&] { return time; });
  MissionStateMachine first_machine, second_machine;
  first_machine.start({"m1", "amr_01", "assembly", "inspection", "motor"});
  second_machine.start({"m2", "amr_02", "assembly", "inspection", "motor"});
  unsigned first_navigation = 0, second_navigation = 0;
  MissionResourceGate first_gate(first, first_machine),
      second_gate(second, second_machine);
  first_gate.acquire(
      "central_aisle",
      [&] {
        first_machine.begin_navigation();
        ++first_navigation;
      },
      [](const auto &) {});
  second_gate.acquire(
      "central_aisle",
      [&] {
        second_machine.begin_navigation();
        ++second_navigation;
      },
      [](const auto &) {});
  first_wire->answer(true, "first-token");
  second_wire->answer(false, "", 0, "owned");
  EXPECT_EQ(first_navigation, 1u);
  EXPECT_EQ(second_navigation, 0u);
  EXPECT_EQ(second_machine.state(), MissionState::Received);
  first_machine.navigation_succeeded();
  first.exited("central_aisle", "m1");
  first.release("central_aisle", "m1", [](bool) {});
  first_wire->answer(true, "", 0);
  time += 0.25;
  second.tick();
  second_wire->answer(true, "second-token");
  EXPECT_EQ(second_navigation, 1u);
  unsigned transfers = 0;
  first_gate.acquire(
      "assembly",
      [&] {
        first_machine.station_confirmed();
        ++transfers;
      },
      [](const auto &) {});
  EXPECT_EQ(transfers, 0u);
  first_wire->answer(true, "station-token");
  EXPECT_EQ(transfers, 1u);
  EXPECT_EQ(first_machine.state(), MissionState::Loading);
  EXPECT_FALSE(first.release_all("m1")); // An unconfirmed transaction remains occupied.
}
TEST(ResourceMissionContract, CancellationDuringGrantNoticeFencesNavigationEffect) {
  double time = 100;
  auto wire = std::make_shared<Wire>();
  ResourceAdapter adapter(wire, "cart_10", [&] { return time; });
  MissionStateMachine machine;
  machine.start({"cancel", "cart_10", "assembly", "inspection", "motor"});
  machine.begin_navigation();
  MissionResourceGate gate(adapter, machine);
  unsigned goals = 0;
  gate.acquire(
      "central_aisle", [&] { ++goals; },
      [&](const auto &notice) {
        if (notice.state == ResourceState::Granted)
          machine.cancel();
      });
  wire->answer(true);
  EXPECT_EQ(goals, 0u);
  EXPECT_EQ(machine.state(), MissionState::Failed);
  EXPECT_FALSE(gate.cancel()); // Safe to request release, but not yet acknowledged.
  ASSERT_EQ(wire->calls.size(), 1u);
  EXPECT_EQ(wire->calls.front().operation, ResourceOperation::Release);
}
TEST(ResourceMissionContract, NoHoldAndWaitOrNavigationAfterCancelledPendingGrant) {
  double time = 100;
  auto wire = std::make_shared<Wire>();
  ResourceAdapter adapter(wire, "cart_10", [&] { return time; });
  MissionStateMachine machine;
  machine.start({"cancel", "cart_10", "assembly", "inspection", "motor"});
  MissionResourceGate gate(adapter, machine);
  unsigned effects = 0;
  gate.acquire(
      "central_aisle", [&] { ++effects; }, [](const auto &) {});
  auto pending = wire->take();
  ResourceState rejected = ResourceState::Waiting;
  gate.acquire(
      "another_aisle", [&] { ++effects; },
      [&](const auto &notice) { rejected = notice.state; });
  EXPECT_EQ(rejected, ResourceState::Lost);
  EXPECT_TRUE(wire->calls.empty());
  machine.cancel();
  gate.cancel();
  wire->take();
  pending.callback({true, "late-token", 10, "granted"});
  EXPECT_EQ(effects, 0u);
  ASSERT_EQ(wire->calls.size(), 1u);
  EXPECT_EQ(wire->calls.front().operation, ResourceOperation::Release);
  EXPECT_EQ(wire->calls.front().key.lease_id, "late-token");
}
TEST(ResourceMissionContract, ServiceLossBetweenGrantNoticeAndEffectReportsRecovery) {
  double time = 100;
  auto wire = std::make_shared<Wire>();
  ResourceAdapter adapter(wire, "cart_10", [&] { return time; });
  MissionStateMachine machine;
  machine.start({"lost", "cart_10", "assembly", "inspection", "motor"});
  MissionResourceGate gate(adapter, machine);
  unsigned effects = 0;
  ResourceState last = ResourceState::Waiting;
  gate.acquire(
      "central_aisle", [&] { ++effects; },
      [&](const auto &notice) {
        last = notice.state;
        if (notice.state == ResourceState::Granted)
          wire->ready = false;
      });
  wire->answer(true);
  EXPECT_EQ(effects, 0u);
  EXPECT_EQ(last, ResourceState::Lost);
}
