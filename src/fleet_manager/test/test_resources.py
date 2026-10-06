# SPDX-License-Identifier: Apache-2.0
"""Exercise capacity, fairness, fencing, and physical uncertainty on real cores."""

from dataclasses import FrozenInstanceError
from concurrent.futures import ThreadPoolExecutor
import importlib
from pathlib import Path
import sys
from threading import Barrier

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def resource_api():
    try:
        return importlib.import_module("fleet_manager.resources")
    except ModuleNotFoundError:
        pytest.fail("resource lease core is not implemented")


def make_manager(api):
    from fleet_manager.config import ResourceConfig

    return api.ResourceManager(
        (
            ResourceConfig("assembly", "station", 1),
            ResourceConfig("aisle", "traffic_zone", 1),
            ResourceConfig("dock", "dock", 1),
        ),
        lease_ttl_sec=10.0,
    )


@pytest.fixture
def api():
    return resource_api()


@pytest.fixture
def manager(api):
    return make_manager(api)


def request(api, robot="r1", mission="m1", resource="assembly"):
    return api.LeaseRequest(robot, mission, resource)


def key(api, lease, **changes):
    fields = {
        "robot_id": lease.robot_id,
        "mission_id": lease.mission_id,
        "resource_id": lease.resource_id,
        "lease_id": lease.lease_id,
    }
    fields.update(changes)
    return api.LeaseKey(**fields)


def test_acquire_creates_immutable_finite_lease_and_idempotent_retry():
    api = resource_api()
    manager = make_manager(api)
    original = manager.acquire(request(api), 100.0)
    assert original.granted
    assert original.lease.robot_id == "r1"
    assert original.lease.mission_id == "m1"
    assert original.lease.resource_id == "assembly"
    assert original.lease.expires_at == 110.0
    assert original.current_owner == "r1"
    assert manager.acquire(request(api), 102.0) == original
    with pytest.raises(FrozenInstanceError):
        original.lease.robot_id = "r2"
    snapshots = manager.snapshot(102.0)
    assert isinstance(snapshots, tuple)
    assert snapshots[0].lease == original.lease
    assert snapshots[0].waiters == ()
    with pytest.raises(FrozenInstanceError):
        snapshots[0].capacity = 2


def test_cancel_resolution_atomically_reports_exact_auto_handoff_without_release(
    api, manager
):
    owner = manager.acquire(request(api), 100).lease
    waiter = request(api, "r2", "cancelled")
    assert not manager.acquire(waiter, 101).granted
    assert manager.release(key(api, owner), 102)
    resolver = getattr(manager, "resolve_waiter", None)
    assert resolver is not None, "atomic cancellation ownership resolution missing"
    resolution = resolver(waiter, 102)
    assert not resolution.cancelled
    assert resolution.lease is not None
    assert resolution.lease.robot_id == "r2"
    assert resolution.lease.mission_id == "cancelled"
    assert manager.snapshot(102)[0].lease == resolution.lease
    assert resolver(request(api, "r3", "other"), 102).lease is None
    assert manager.release(key(api, resolution.lease), 102)
    assert manager.snapshot(102)[0].lease is None


def test_cancel_resolution_reports_matching_quarantine_without_exposing_foreign_token(
    api, manager
):
    manager.acquire(request(api), 100)
    resolver = getattr(manager, "resolve_waiter", None)
    assert resolver is not None, "atomic cancellation ownership resolution missing"
    resolution = resolver(request(api), 110)
    assert resolution.reconciliation_required
    assert resolution.lease is None
    foreign = resolver(request(api, "r2", "m2"), 110)
    assert foreign.lease is None
    assert not foreign.reconciliation_required


def test_queue_orders_time_then_robot_then_stable_insertion(api, manager):
    held = manager.acquire(request(api), 0.0).lease
    requests = (
        request(api, "z", "first"),
        request(api, "b", "second"),
        request(api, "a", "z_mission"),
        request(api, "a", "a_mission"),
    )
    for queued, now in zip(requests, (1.0, 2.0, 2.0, 2.0)):
        decision = manager.acquire(queued, now)
        assert not decision.granted
        assert decision.lease is None
        assert decision.current_owner == "r1"
    ordered = (requests[0], requests[2], requests[3], requests[1])
    assert manager.snapshot(2.0)[0].waiters == ordered
    retry = manager.acquire(requests[0], 3.0)
    assert not retry.granted
    assert manager.snapshot(3.0)[0].waiters == ordered
    assert manager.release(key(api, held), 3.0)
    for queued in ordered:
        lease = manager.acquire(queued, 3.0).lease
        assert (lease.robot_id, lease.mission_id) == (
            queued.robot_id,
            queued.mission_id,
        )
        assert lease.expires_at == 13.0
        assert manager.release(key(api, lease), 3.0)
    assert manager.snapshot(3.0)[0].lease is None


def test_new_acquirer_cannot_bypass_waiter_after_release(api, manager):
    held = manager.acquire(request(api), 0.0).lease
    waiter = request(api, "waiting", "m2")
    manager.acquire(waiter, 1.0)
    manager.release(key(api, held), 2.0)
    newcomer = manager.acquire(request(api, "new", "m3"), 2.0)
    assert not newcomer.granted
    assert newcomer.current_owner == "waiting"
    assert manager.acquire(waiter, 2.0).granted


@pytest.mark.parametrize(
    "field,value",
    [
        ("robot_id", "wrong"),
        ("mission_id", "wrong"),
        ("lease_id", "stale"),
        ("resource_id", "aisle"),
    ],
)
def test_wrong_identity_cannot_renew_or_release(api, manager, field, value):
    held = manager.acquire(request(api), 0.0).lease
    wrong = key(api, held, **{field: value})
    assert not manager.renew(wrong, 1.0).granted
    assert not manager.release(wrong, 1.0)
    assert manager.snapshot(1.0)[0].lease == held


def test_renewal_keeps_token_and_advances_expiry(api, manager):
    held = manager.acquire(request(api), 0.0).lease
    renewed = manager.renew(key(api, held), 9.0)
    assert renewed.granted
    assert renewed.lease.lease_id == held.lease_id
    assert renewed.lease.expires_at == 19.0
    assert held.expires_at == 10.0
    assert manager.expire(10.0) == ()
    assert manager.expire(19.0) == (renewed.lease,)


def test_cancel_waiter_removes_only_matching_queued_request(api, manager):
    held = manager.acquire(request(api), 0.0).lease
    cancelled = request(api, "r2", "cancel")
    survivor = request(api, "r2", "keep")
    manager.acquire(cancelled, 1.0)
    manager.acquire(survivor, 2.0)
    assert not manager.cancel_waiter(request(api), 3.0)
    assert manager.cancel_waiter(cancelled, 3.0)
    assert not manager.cancel_waiter(cancelled, 3.0)
    assert manager.snapshot(3.0)[0].waiters == (survivor,)
    manager.release(key(api, held), 3.0)
    assert manager.acquire(survivor, 3.0).granted


def test_expiry_quarantines_until_exact_former_lease_is_cleared(api, manager):
    held = manager.acquire(request(api), 0.0).lease
    waiter = request(api, "r2", "m2")
    manager.acquire(waiter, 1.0)
    assert manager.expire(9.999) == ()
    assert manager.expire(10.0) == (held,)
    assert manager.expire(11.0) == ()
    state = manager.snapshot(11.0)[0]
    assert state.lease is None
    assert state.reconciliation_required
    assert state.former_lease == held
    assert state.waiters == (waiter,)
    assert not manager.acquire(waiter, 11.0).granted
    assert not manager.renew(key(api, held), 11.0).granted
    assert not manager.release(key(api, held), 11.0)
    for field in ("robot_id", "mission_id", "lease_id"):
        assert not manager.clear_reconciliation(
            key(api, held, **{field: "wrong"}), 11.0
        )
    assert manager.clear_reconciliation(key(api, held), 11.0)
    assert not manager.clear_reconciliation(key(api, held), 11.0)
    state = manager.snapshot(11.0)[0]
    assert not state.reconciliation_required
    assert state.former_lease is None
    assert state.lease.robot_id == "r2"
    assert state.lease.expires_at == 21.0
    assert state.lease.lease_id != held.lease_id


@pytest.mark.parametrize("operation", ["acquire", "renew", "release", "snapshot"])
def test_operations_never_treat_expired_lease_as_available(api, manager, operation):
    held = manager.acquire(request(api), 0.0).lease
    if operation == "acquire":
        assert not manager.acquire(request(api, "r2", "m2"), 10.0).granted
    elif operation == "renew":
        assert not manager.renew(key(api, held), 10.0).granted
    elif operation == "release":
        assert not manager.release(key(api, held), 10.0)
    else:
        manager.snapshot(10.0)
    assert manager.snapshot(10.0)[0].reconciliation_required


def test_offline_owner_quarantines_holds_and_removes_all_its_waits(api, manager):
    station = manager.acquire(request(api), 0.0).lease
    dock = manager.acquire(request(api, resource="dock"), 0.0).lease
    other = manager.acquire(request(api, "r2", "m2", "aisle"), 0.0).lease
    manager.acquire(request(api, resource="aisle"), 1.0)
    waiting = request(api, "r2", "m2")
    manager.acquire(waiting, 1.0)
    assert manager.release_owner("r1", 2.0) == (station, dock)
    assert manager.release_owner("r1", 2.0) == ()
    station_state, aisle_state, dock_state = manager.snapshot(2.0)
    assert station_state.reconciliation_required
    assert dock_state.reconciliation_required
    assert aisle_state.lease == other
    assert aisle_state.waiters == ()
    assert station_state.waiters == (waiting,)
    assert not manager.acquire(waiting, 2.0).granted
    assert manager.clear_reconciliation(key(api, station), 2.0)
    assert manager.acquire(waiting, 2.0).granted


def test_old_token_cannot_affect_new_same_owner_lease(api, manager):
    old = manager.acquire(request(api), 0.0).lease
    manager.release(key(api, old), 1.0)
    new = manager.acquire(request(api), 1.0).lease
    assert new.lease_id != old.lease_id
    assert len(new.lease_id) >= 32
    assert not manager.renew(key(api, old), 2.0).granted
    assert not manager.release(key(api, old), 2.0)
    assert manager.snapshot(2.0)[0].lease == new


@pytest.mark.parametrize(
    "operation",
    [
        "acquire",
        "renew",
        "release",
        "expire",
        "release_owner",
        "snapshot",
        "cancel_waiter",
        "clear_reconciliation",
    ],
)
@pytest.mark.parametrize(
    "invalid", [float("nan"), float("inf"), -float("inf"), True, "1"]
)
def test_invalid_time_rejected_without_changing_state(api, manager, operation, invalid):
    held = manager.acquire(request(api), 0.0).lease
    args = {
        "acquire": (request(api, "r2", "m2"),),
        "renew": (key(api, held),),
        "release": (key(api, held),),
        "expire": (),
        "release_owner": ("r1",),
        "snapshot": (),
        "cancel_waiter": (request(api),),
        "clear_reconciliation": (key(api, held),),
    }
    with pytest.raises(ValueError, match="finite"):
        getattr(manager, operation)(*args[operation], invalid)
    assert manager.snapshot(0.0)[0].lease == held
    assert manager.snapshot(0.0)[0].waiters == ()


@pytest.mark.parametrize("ttl", [0, -1, float("nan"), float("inf"), True, "10"])
def test_invalid_ttl_rejected(api, ttl):
    from fleet_manager.config import ResourceConfig

    with pytest.raises(ValueError):
        api.ResourceManager((ResourceConfig("assembly", "station", 1),), ttl)


def test_overflowing_expiry_rejected_without_grant(api):
    from fleet_manager.config import ResourceConfig

    manager = api.ResourceManager((ResourceConfig("assembly", "station", 1),), 1e308)
    with pytest.raises(ValueError, match="finite"):
        manager.acquire(request(api), 1e308)
    assert manager.snapshot(0.0)[0].lease is None


@pytest.mark.parametrize("capacity", [0, 2, True, 1.0])
def test_capacity_other_than_integer_one_is_rejected(api, capacity):
    from fleet_manager.config import ResourceConfig

    with pytest.raises(ValueError, match="capacity"):
        api.ResourceManager((ResourceConfig("assembly", "station", capacity),))


def test_duplicate_resource_ids_are_rejected(api):
    from fleet_manager.config import ResourceConfig

    with pytest.raises(ValueError, match="duplicate"):
        api.ResourceManager(
            (
                ResourceConfig("assembly", "station", 1),
                ResourceConfig("assembly", "dock", 1),
            )
        )


def test_unknown_resource_kind_is_rejected(api):
    from fleet_manager.config import ResourceConfig

    with pytest.raises(ValueError, match="kind"):
        api.ResourceManager((ResourceConfig("assembly", "unsupported", 1),))


@pytest.mark.parametrize(
    "operation",
    [
        "acquire",
        "renew",
        "release",
        "cancel_waiter",
        "clear_reconciliation",
    ],
)
def test_unknown_resource_rejected_before_state_changes(api, manager, operation):
    held = manager.acquire(request(api), 0.0).lease
    arg = (
        request(api, resource="unknown")
        if operation in ("acquire", "cancel_waiter")
        else key(api, held, resource_id="unknown")
    )
    with pytest.raises(KeyError, match="unknown"):
        getattr(manager, operation)(arg, 10.0)
    assert manager.snapshot(0.0)[0].lease == held


@pytest.mark.parametrize("field", ["robot_id", "mission_id", "resource_id"])
@pytest.mark.parametrize("invalid", ["", None])
def test_empty_request_identity_rejected(api, manager, field, invalid):
    fields = {"robot_id": "r1", "mission_id": "m1", "resource_id": "assembly"}
    fields[field] = invalid
    with pytest.raises(ValueError, match=field):
        manager.acquire(api.LeaseRequest(**fields), 0.0)
    assert manager.snapshot(0.0)[0].lease is None


@pytest.mark.parametrize("trial", range(100))
def test_concurrent_acquire_release_preserves_capacity_and_queue(api, manager, trial):
    requests = tuple(request(api, f"r{i}", f"m{trial}") for i in range(8))
    barrier = Barrier(8)

    def acquire(queued):
        barrier.wait(timeout=5)
        return manager.acquire(queued, 1.0)

    with ThreadPoolExecutor(max_workers=8) as workers:
        results = tuple(workers.map(acquire, requests))
    granted = [result.lease for result in results if result.granted]
    assert len(granted) == 1
    initial = manager.snapshot(1.0)[0]
    waiting = tuple(
        sorted(
            (queued for queued in requests if queued.robot_id != granted[0].robot_id),
            key=lambda queued: queued.robot_id,
        )
    )
    assert initial.waiters == waiting
    barrier = Barrier(8)

    def release_or_retry(queued):
        barrier.wait(timeout=5)
        if queued.robot_id == granted[0].robot_id:
            return manager.release(key(api, granted[0]), 2.0)
        return manager.acquire(queued, 2.0)

    with ThreadPoolExecutor(max_workers=8) as workers:
        tuple(workers.map(release_or_retry, requests))
    state = manager.snapshot(2.0)[0]
    assert state.lease.robot_id == waiting[0].robot_id
    assert state.waiters == waiting[1:]
    tokens = {granted[0].lease_id}
    for queued in waiting:
        decision = manager.acquire(queued, 2.0)
        assert decision.granted
        assert decision.lease.robot_id == queued.robot_id
        assert decision.lease.lease_id not in tokens
        tokens.add(decision.lease.lease_id)
        assert manager.release(key(api, decision.lease), 2.0)
    assert manager.snapshot(2.0)[0].lease is None
