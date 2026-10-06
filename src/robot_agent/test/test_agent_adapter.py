"""Catch stale health, distance replay, payload lies, and late path completions."""

import importlib
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def api():
    try:
        return importlib.import_module("robot_agent.adapter")
    except ModuleNotFoundError:
        pytest.fail("AgentAdapter behavior is missing")


class Paths:
    def __init__(self):
        self.calls = []
        self.cancelled = []

    def compute(self, start, goal, frame, callback):
        index = len(self.calls)
        self.calls.append((start, goal, frame, callback))
        return lambda: self.cancelled.append(index)

    def finish(self, index, length=None, reason=""):
        self.calls[index][3](length, reason)


def rig():
    module = api()
    wall = [100.0]
    paths = Paths()
    agent = module.AgentAdapter(
        "cart_1",
        "floor/cart_1/",
        {"pick": (-3, 1, 0), "drop": (3, 1, 0)},
        paths,
        battery_percent=80,
        clock=lambda: wall[0],
    )
    return agent, paths, wall


def ready(agent):
    agent.odometry(1, 0, 0, "floor/cart_1/odom")
    agent.localization(1, 0, -3, 0, "floor/cart_1/map")
    agent.payload(
        json.dumps(
            {
                "robot_id": "cart_1",
                "mission_id": "",
                "state": "AT_ASSEMBLY",
                "transfer_kind": "",
                "cycle_counter": None,
            }
        )
    )


def mission(name="m1"):
    return SimpleNamespace(
        mission_id=name, pickup_station="pick", dropoff_station="drop", part="motor"
    )


def event(name, detail="", outcome="", robot="cart_1", mission_id="m1", protocol="ROS"):
    return SimpleNamespace(
        robot_id=robot,
        mission_id=mission_id,
        protocol=protocol,
        event=name,
        detail=detail,
        outcome=outcome,
    )


def test_wall_heartbeat_continues_with_unknown_and_stale_pose():
    agent, _, wall = rig()
    state = agent.heartbeat()
    assert state.mode == "UNHEALTHY" and state.pose is None
    assert state.payload_state == "UNKNOWN" and state.health_detail
    ready(agent)
    assert agent.heartbeat().mode == "AVAILABLE"
    wall[0] += 3.1
    state = agent.heartbeat()
    assert state.mode == "UNHEALTHY" and "stale" in state.health_detail
    assert state.battery_percent == pytest.approx(79.9969)


def test_odom_replay_regression_jump_and_frame_change_never_double_count():
    agent, _, wall = rig()
    ready(agent)
    agent.odometry(2, 3, 4, "floor/cart_1/odom")
    assert agent.energy.battery_percent == 79.5
    for stamp, x, y, frame in [
        (2, 3, 4, "floor/cart_1/odom"),
        (1, 30, 40, "floor/cart_1/odom"),
        (3, 1000, 1000, "floor/cart_1/odom"),
        (4, 0, 0, "other/odom"),
    ]:
        agent.odometry(stamp, x, y, frame)
    assert agent.energy.battery_percent == 79.5
    wall[0] = 99
    agent.heartbeat()
    wall[0] = 100
    assert agent.heartbeat().battery_percent == 79.5
    wall[0] = 101
    assert agent.heartbeat().battery_percent == pytest.approx(79.499)


@pytest.mark.parametrize(
    "payload",
    [
        "null",
        "[]",
        "{",
        '{"robot_id":"foreign","state":"IN_TRANSIT"}',
        '{"robot_id":"cart_1","mission_id":"m2","state":"AT_INSPECTION","transfer_kind":"UNLOADING","cycle_counter":2}',
    ],
)
def test_malformed_foreign_or_wrong_mission_payload_never_changes_ownership(payload):
    agent, _, _ = rig()
    ready(agent)
    agent.protocol(event("state_changed", "LOADING"))
    assert agent.heartbeat().payload_state == "UNKNOWN"
    agent.payload(payload)
    assert agent.heartbeat().payload_state == "UNKNOWN"


def test_confirmed_payload_counts_operation_once_and_terminal_never_invents_empty():
    agent, _, _ = rig()
    ready(agent)
    agent.protocol(event("state_changed", "LOADING"))
    loaded = json.dumps(
        {
            "robot_id": "cart_1",
            "mission_id": "m1",
            "state": "IN_TRANSIT",
            "transfer_kind": "LOADING",
            "cycle_counter": 1,
        }
    )
    agent.payload(loaded)
    agent.payload(loaded)
    assert agent.heartbeat().payload_state == "LOADED"
    assert agent.energy.battery_percent == 79.5
    agent.protocol(event("mission_finished", outcome="COMPLETED"))
    state = agent.heartbeat()
    assert state.mode == "RECOVERY_REQUIRED" and state.payload_state == "LOADED"
    assert state.mission_id == "m1"


def test_mission_wait_resume_unknown_failure_and_foreign_event():
    agent, _, _ = rig()
    ready(agent)
    agent.protocol(event("state_changed", "NAVIGATING_TO_PICKUP", robot="foreign"))
    assert agent.heartbeat().mode == "AVAILABLE"
    agent.protocol(event("state_changed", "NAVIGATING_TO_PICKUP"))
    assert agent.heartbeat().mode == "EXECUTING"
    agent.mission_state("WAITING_FOR_RESOURCE")
    assert agent.heartbeat().mode == "WAITING_FOR_RESOURCE"
    agent.protocol(event("state_changed", "LOADING"))
    agent.protocol(event("mission_finished", outcome="FAILED"))
    assert agent.heartbeat().mode == "RECOVERY_REQUIRED"
    assert agent.heartbeat().payload_state == "UNKNOWN"


def test_successful_unload_is_only_safe_terminal_empty_evidence():
    agent, _, _ = rig()
    ready(agent)
    agent.protocol(event("state_changed", "LOADING"))
    for state, kind, counter in [
        ("IN_TRANSIT", "LOADING", 1),
        ("AT_INSPECTION", "UNLOADING", 2),
    ]:
        agent.payload(
            json.dumps(
                {
                    "robot_id": "cart_1",
                    "mission_id": "m1",
                    "state": state,
                    "transfer_kind": kind,
                    "cycle_counter": counter,
                }
            )
        )
    agent.protocol(event("mission_finished", outcome="COMPLETED"))
    state = agent.heartbeat()
    assert (
        state.mode == "AVAILABLE"
        and state.payload_state == "EMPTY"
        and state.mission_id == ""
    )
    assert state.battery_percent == 79


def test_cost_uses_localized_map_pose_two_paths_and_dock_allowance():
    agent, paths, _ = rig()
    ready(agent)
    results = []
    agent.estimate(mission(), results.append)
    assert results == []
    assert [(c[0], c[1], c[2]) for c in paths.calls] == [
        (None, (-3, 1, 0), "floor/cart_1/map"),
        ((-3, 1, 0), (3, 1, 0), "floor/cart_1/map"),
    ]
    paths.finish(0, 7)
    paths.finish(1, 8)
    assert len(results) == 1 and results[0].feasible
    assert results[0].path_cost == 7 and results[0].predicted_final_battery == 76.5


@pytest.mark.parametrize(
    "reason", ["unavailable", "rejected", "path failed", "timeout"]
)
def test_path_failure_excludes_candidate_without_euclidean_success(reason):
    agent, paths, _ = rig()
    ready(agent)
    results = []
    agent.estimate(mission(), results.append)
    paths.finish(0, None, reason)
    paths.finish(1, 6)
    assert len(results) == 1 and not results[0].feasible and reason in results[0].reason


def test_cost_timeout_and_concurrent_requests_fence_stale_callbacks():
    agent, paths, wall = rig()
    ready(agent)
    first, second = [], []
    agent.estimate(mission(), first.append)
    agent.estimate(mission("m2"), second.append)
    assert len(first) == 1 and not first[0].feasible
    paths.finish(0, 1)
    paths.finish(1, 1)
    assert second == []
    wall[0] += 0.8
    agent.tick()
    assert len(second) == 1 and not second[0].feasible and "timeout" in second[0].reason
    paths.finish(2, 1)
    paths.finish(3, 1)
    assert len(second) == 1
    assert sorted(paths.cancelled) == [0, 1, 2, 3]


def test_cost_unknown_pose_stale_pose_wrong_frame_and_bad_mission_are_infeasible():
    agent, paths, wall = rig()
    results = []
    agent.estimate(mission(), results.append)
    assert not results[-1].feasible and not paths.calls
    ready(agent)
    agent.localization(2, 1, 2, 0, "floor/cart_1/odom")
    agent.estimate(mission(), results.append)
    assert not results[-1].feasible and not paths.calls
    ready(agent)
    agent.estimate(
        SimpleNamespace(
            mission_id="", pickup_station="bad", dropoff_station="drop", part="motor"
        ),
        results.append,
    )
    assert not results[-1].feasible and not paths.calls
    wall[0] += 4
    agent.estimate(mission(), results.append)
    assert not results[-1].feasible and not paths.calls


@pytest.mark.parametrize(
    "field,value",
    [("state", []), ("transfer_kind", {}), ("mission_id", []), ("robot_id", {})],
)
def test_payload_wrong_json_types_are_rejected_without_callback_exception(field, value):
    agent, _, _ = rig()
    ready(agent)
    agent.protocol(event("state_changed", "LOADING"))
    fields = {
        "robot_id": "cart_1",
        "mission_id": "m1",
        "state": "IN_TRANSIT",
        "transfer_kind": "LOADING",
        "cycle_counter": 1,
    }
    fields[field] = value
    assert agent.payload(json.dumps(fields)) is False
    assert agent.heartbeat().payload_state == "UNKNOWN"


def test_malformed_state_does_not_bind_mission_or_accept_old_terminal_replay():
    agent, _, _ = rig()
    ready(agent)
    assert agent.protocol(event("state_changed", "INVALID")) is False
    assert agent.heartbeat().mission_id == ""
    agent.protocol(event("state_changed", "NAVIGATING_TO_PICKUP"))
    agent.protocol(event("mission_finished", outcome="FAILED"))
    assert agent.heartbeat().mode == "AVAILABLE"
    agent.protocol(event("state_changed", "LOADING"))
    assert agent.heartbeat().mode == "AVAILABLE" and agent.heartbeat().mission_id == ""


def test_duplicate_loading_state_cannot_erase_confirmed_payload():
    agent, _, _ = rig()
    ready(agent)
    agent.protocol(event("state_changed", "LOADING"))
    agent.payload(
        json.dumps(
            {
                "robot_id": "cart_1",
                "mission_id": "m1",
                "state": "IN_TRANSIT",
                "transfer_kind": "LOADING",
                "cycle_counter": 1,
            }
        )
    )
    agent.mission_state("LOADING")
    agent.protocol(event("state_changed", "LOADING"))
    assert agent.heartbeat().payload_state == "LOADED"


def test_late_confirmed_unload_updates_payload_without_clearing_recovery():
    agent, _, _ = rig()
    ready(agent)
    agent.protocol(event("state_changed", "LOADING"))
    agent.payload(
        json.dumps(
            {
                "robot_id": "cart_1",
                "mission_id": "m1",
                "state": "IN_TRANSIT",
                "transfer_kind": "LOADING",
                "cycle_counter": 1,
            }
        )
    )
    agent.protocol(event("mission_finished", outcome="COMPLETED"))
    agent.payload(
        json.dumps(
            {
                "robot_id": "cart_1",
                "mission_id": "m1",
                "state": "AT_INSPECTION",
                "transfer_kind": "UNLOADING",
                "cycle_counter": 2,
            }
        )
    )
    assert agent.heartbeat().payload_state == "EMPTY"
    assert agent.heartbeat().mode == "RECOVERY_REQUIRED"


def test_odometry_becoming_stale_during_path_estimate_rejects_success():
    agent, paths, wall = rig()
    ready(agent)
    wall[0] += 2.8
    results = []
    agent.estimate(mission(), results.append)
    wall[0] += 0.3
    paths.finish(0, 7)
    paths.finish(1, 8)
    assert (
        len(results) == 1 and not results[0].feasible and "stale" in results[0].reason
    )


def test_charge_authorization_expiry_and_mode_transition_split_wall_elapsed():
    agent, _, wall = rig()
    ready(agent)
    agent.mode = "CHARGING"
    agent.set_charge_authorization(lease_until=101, contact=True)
    wall[0] = 102
    assert agent.heartbeat().battery_percent == pytest.approx(80.998)
    agent.mode = "AVAILABLE"
    wall[0] = 103
    assert agent.heartbeat().battery_percent == pytest.approx(80.997)


def test_idle_robot_keeps_valid_localization_with_continuous_odom_and_no_new_amcl():
    agent, paths, wall = rig()
    ready(agent)
    wall[0] += 30
    agent.odometry(31, 0, 0, "floor/cart_1/odom")
    state = agent.heartbeat()
    assert state.mode == "AVAILABLE" and state.pose == (0.0, -3.0, 0.0)
    results = []
    agent.estimate(mission(), results.append)
    assert not results and len(paths.calls) == 2
    assert paths.calls[0][0] is None
    paths.finish(0, 7)
    paths.finish(1, 8)
    assert results[0].feasible


def test_cancellation_exception_still_completes_cost_response():
    agent, paths, wall = rig()
    ready(agent)
    original = paths.compute

    def compute(*args):
        original(*args)

        def cancel():
            raise RuntimeError("wire closed")

        return cancel

    paths.compute = compute
    results = []
    agent.estimate(mission(), results.append)
    wall[0] += 0.8
    agent.tick()
    assert len(results) == 1 and not results[0].feasible


def test_new_charge_authorization_cannot_apply_to_elapsed_time_before_grant():
    agent, _, wall = rig()
    ready(agent)
    agent.mode = "CHARGING"
    wall[0] = 102
    agent.set_charge_authorization(lease_until=103, contact=True)
    assert agent.heartbeat().battery_percent == pytest.approx(79.998)
    wall[0] = 103
    assert agent.heartbeat().battery_percent == pytest.approx(80.997)


def test_delayed_older_mission_state_cannot_erase_confirmed_loaded_payload():
    agent, _, _ = rig()
    ready(agent)
    agent.protocol(event("state_changed", "LOADING"))
    agent.payload(
        json.dumps(
            {
                "robot_id": "cart_1",
                "mission_id": "m1",
                "state": "IN_TRANSIT",
                "transfer_kind": "LOADING",
                "cycle_counter": 1,
            }
        )
    )
    agent.protocol(event("state_changed", "NAVIGATING_TO_DROPOFF"))
    agent.mission_state("LOADING")
    assert agent.heartbeat().payload_state == "LOADED"


def test_local_state_before_identity_marks_executing_without_inventing_mission():
    agent, _, _ = rig()
    ready(agent)
    agent.mission_state("LOADING")
    state = agent.heartbeat()
    assert state.mode == "EXECUTING" and state.payload_state == "UNKNOWN"
    assert state.mission_id == ""


def test_cached_estimate_is_pending_then_uses_paths_with_current_energy():
    agent, paths, wall = rig()
    ready(agent)
    first = agent.cached_estimate(mission())
    assert not first.feasible and "pending" in first.reason
    assert not agent.cached_estimate(mission()).feasible and len(paths.calls) == 2
    paths.finish(0, 7)
    paths.finish(1, 8)
    wall[0] += 1
    result = agent.cached_estimate(mission("m2"))
    assert result.feasible and result.path_cost == 7
    assert result.predicted_final_battery == pytest.approx(76.499)
    assert len(paths.calls) == 2


def test_cache_age_and_movement_refresh_current_nav2_leg():
    agent, paths, wall = rig()
    ready(agent)
    agent.cached_estimate(mission())
    paths.finish(0, 7)
    paths.finish(1, 8)
    wall[0] += 2.1
    agent.odometry(3, 0, 0, "floor/cart_1/odom")
    assert not agent.cached_estimate(mission()).feasible and len(paths.calls) == 4
    paths.finish(2, 7)
    paths.finish(3, 8)
    assert agent.cached_estimate(mission()).feasible
    agent.odometry(4, 0.2, 0, "floor/cart_1/odom")
    assert not agent.cached_estimate(mission()).feasible and len(paths.calls) == 6


def test_cache_superseded_route_and_late_timeout_callbacks_cannot_win():
    agent, paths, wall = rig()
    ready(agent)
    assert not agent.cached_estimate(mission()).feasible
    reverse = SimpleNamespace(
        mission_id="m2", pickup_station="drop", dropoff_station="pick", part="motor"
    )
    assert not agent.cached_estimate(reverse).feasible
    paths.finish(0, 1)
    paths.finish(1, 1)
    assert not agent.cached_estimate(reverse).feasible
    wall[0] += 0.8
    agent.tick()
    paths.finish(2, 1)
    paths.finish(3, 1)
    result = agent.cached_estimate(reverse)
    assert not result.feasible and "timeout" in result.reason


def test_cache_movement_during_request_and_path_error_remain_infeasible():
    agent, paths, _ = rig()
    ready(agent)
    agent.cached_estimate(mission())
    agent.odometry(2, 1, 0, "floor/cart_1/odom")
    paths.finish(0, 7)
    paths.finish(1, 8)
    assert not agent.cached_estimate(mission()).feasible
    paths.finish(2, None, "rejected")
    result = agent.cached_estimate(mission())
    assert not result.feasible and "rejected" in result.reason


@pytest.mark.parametrize("seconds", [0, 0.5, 1.0])
def test_cache_age_must_survive_fleet_one_second_retry(seconds):
    with pytest.raises(ValueError):
        api().AgentAdapter(
            "cart_1",
            "floor/cart_1/",
            {"pick": (0, 0, 0), "drop": (3, 0, 0)},
            Paths(),
            cache_max_age_sec=seconds,
        )


def test_map_relocalization_invalidates_cached_current_leg_without_odom_motion():
    agent, paths, _ = rig()
    ready(agent)
    agent.cached_estimate(mission())
    paths.finish(0, 7)
    paths.finish(1, 8)
    agent.localization(2, 3, 2, 0, "floor/cart_1/map")
    assert not agent.cached_estimate(mission()).feasible and len(paths.calls) == 4
