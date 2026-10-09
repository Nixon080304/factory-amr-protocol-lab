"""Exercise robot docking and real resource expiry under a shared steady clock."""

from types import SimpleNamespace

from fleet_manager.config import ResourceConfig
from fleet_manager.resources import LeaseKey, LeaseRequest, ResourceManager
import pytest

from test_docking import arrive, rig as dock_rig, stage


def rig():
    r = dock_rig()
    _, wall, wire, *_ = r
    manager = ResourceManager((ResourceConfig("charger", "dock", 1),))

    def resource(operation, key, callback):
        wire.requests.append((operation, key, callback))
        identity = (key.robot_id, key.mission_id, key.resource_id)
        if operation == "release":
            callback(
                SimpleNamespace(
                    released=manager.release(LeaseKey(*identity, key.lease_id), wall[0])
                )
            )
        else:
            decision = (
                manager.acquire(LeaseRequest(*identity), wall[0])
                if operation == "acquire"
                else manager.renew(LeaseKey(*identity, key.lease_id), wall[0])
            )
            callback(
                SimpleNamespace(
                    granted=decision.granted,
                    renewed=decision.granted,
                    lease_id=decision.lease.lease_id if decision.lease else "",
                    lease_ttl_sec=decision.lease.expires_at - wall[0]
                    if decision.lease
                    else 0.0,
                )
            )

    wire.resource = resource
    return r, manager


def advance(r, elapsed, *, contact=False):
    agent, wall, _, _, _, controller = r
    wall[0] += elapsed
    stamp = max(agent._pose_stamp, agent._odom_stamp) + 1
    agent.odometry(stamp, 0, 0, agent.frame_prefix + "odom")
    agent.localization(stamp, *agent.pose, agent.frame_prefix + "map")
    if contact:
        controller.contact(True)
    else:
        controller.tick()


@pytest.mark.parametrize("phase", ["ENTERING", "CHARGING", "EXITING"])
def test_healthy_long_dock_leg_renews_same_exact_resource_until_verified_exit(phase):
    r, manager = rig()
    stage(r)
    agent, wall, wire, results, _, controller = r
    identity = controller.key
    assert identity.lease_id
    if phase != "ENTERING":
        arrive(r, 1, controller.charging)
        controller.contact(True)
        if phase == "EXITING":
            agent.energy.battery_percent = 80
            controller.tick()
    for _ in range(35):
        advance(r, 1.0, contact=phase == "CHARGING")
        snapshot = manager.snapshot(wall[0])[0]
        assert snapshot.lease is not None
        assert snapshot.lease.lease_id == identity.lease_id
        assert snapshot.lease.mission_id == identity.mission_id
        assert not snapshot.reconciliation_required
        assert controller.active and controller.state == phase
    renewals = [key for operation, key, _ in wire.requests if operation == "renew"]
    assert len(renewals) >= 6 and all(key == identity for key in renewals)
    assert not any(operation == "release" for operation, *_ in wire.requests)
    if phase == "ENTERING":
        arrive(r, 1, controller.charging)
        controller.contact(True)
    if phase != "EXITING":
        agent.energy.battery_percent = 80
        controller.tick()
    arrive(r, 2, controller.exit_pose)
    snapshot = manager.snapshot(wall[0])[0]
    assert snapshot.lease is None and not snapshot.reconciliation_required
    assert results[0].success and agent.mode == "AVAILABLE"


def test_absent_physical_contact_times_out_before_renewed_lease_then_quarantines():
    r, manager = rig()
    stage(r)
    agent, wall, wire, results, _, controller = r
    identity = controller.key
    for _ in range(11):
        advance(r, 1.0)
    arrive(r, 1, controller.charging)
    assert controller.state == "WAITING_FOR_CONTACT"
    before = agent.energy.battery_percent
    for _ in range(5):
        advance(r, 1.0)
    assert results[0].error_code == "CONTACT_TIMEOUT"
    assert agent.mode == "RECOVERY_REQUIRED" and agent.energy.battery_percent < before
    snapshot = manager.snapshot(wall[0])[0]
    assert snapshot.lease is not None and snapshot.lease.lease_id == identity.lease_id
    assert wall[0] < snapshot.lease.expires_at
    assert not any(operation == "release" for operation, *_ in wire.requests)
    wall[0] = snapshot.lease.expires_at
    expired = manager.snapshot(wall[0])[0]
    assert (
        expired.reconciliation_required
        and expired.former_lease.lease_id == identity.lease_id
    )
    controller.contact(True)
    assert len(results) == 1 and agent.mode == "RECOVERY_REQUIRED"
    assert not agent.charge_authorized()
