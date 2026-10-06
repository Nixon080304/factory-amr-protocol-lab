"""Production fleet/agent scheduling against controlled external transport."""

from pathlib import Path
import sys
import json

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "fleet_manager"))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "fleet_manager/test"))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "robot_agent"))
from fleet_manager.adapter import FleetAdapter
from fleet_manager.config import Pose2D, load_fleet_config
from fleet_manager.journal import MissionJournal
from fleet_manager.models import CostEstimate, MissionRequest, RobotSnapshot
from fake_robot_agent import FakeRobots
from robot_agent.adapter import AgentAdapter
from test_agent_adapter import Paths

ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture
def integrated(tmp_path):
    wall = [100.0]
    config = load_fleet_config(ROOT / "factory_bringup/config/fleet.yaml")
    journal = MissionJournal(tmp_path / "missions.sqlite3")
    agents, planners = {}, {}
    for robot in config.robots:
        planner = planners[robot.robot_id] = Paths()
        agent = agents[robot.robot_id] = AgentAdapter(
            robot.robot_id,
            robot.frame_prefix,
            {"assembly": (-3, 1, 0), "inspection": (3, 1, 0)},
            planner,
            battery_percent=80,
            clock=lambda: wall[0],
        )
        agent.odometry(1, 0, 0, robot.frame_prefix + "odom")
        agent.localization(1, 0, 0, 0, robot.frame_prefix + "map")
        agent.payload(
            json.dumps(
                dict(
                    robot_id=robot.robot_id,
                    mission_id="",
                    state="AT_ASSEMBLY",
                    transfer_kind="",
                    cycle_counter=None,
                )
            )
        )

    class Transport(FakeRobots):
        def estimate(self, robot_id, request, callback):
            result = agents[robot_id].cached_estimate(request)
            callback(
                CostEstimate(
                    result.feasible,
                    result.path_cost,
                    result.predicted_final_battery,
                    result.reason,
                )
            )

    transport = Transport(journal)
    fleet = FleetAdapter(config, journal, transport, clock=lambda: wall[0])

    def observe():
        for robot in config.robots:
            fleet.observe(
                RobotSnapshot(robot.robot_id, "AVAILABLE", Pose2D(0, 0, 0), 80, "EMPTY")
            )

    observe()
    yield fleet, journal, transport, agents, planners, wall, observe
    journal.close()


def complete(planner):
    index = 0
    while index < len(planner.calls):
        planner.finish(index, 5)
        index += 1


def test_pinned_cold_cache_waits_for_bounded_paths_then_dispatches(integrated):
    fleet, journal, transport, _, planners, wall, observe = integrated
    fleet.submit(MissionRequest("pinned", "assembly", "inspection", "motor", "amr_01"))
    fleet.tick()
    fleet.tick()
    assert journal.get("pinned").state == "QUEUED"
    complete(planners["amr_01"])
    wall[0] += 1.01
    observe()
    fleet.tick()
    fleet.tick()
    assert len(transport.goals) == 1 and transport.goals[0].robot_id == "amr_01"


def test_mixed_routes_each_reach_execution_without_cache_starvation(integrated):
    fleet, journal, transport, _, planners, wall, observe = integrated
    fleet.submit(MissionRequest("forward", "assembly", "inspection", "motor"))
    fleet.submit(MissionRequest("reverse", "inspection", "assembly", "motor"))
    fleet.tick()
    fleet.tick()
    for planner in planners.values():
        complete(planner)
    wall[0] += 1.01
    observe()
    fleet.tick()
    fleet.tick()
    assert len(transport.goals) == 2
    assert {g.request.mission_id for g in transport.goals} == {"forward", "reverse"}
    assert {journal.get(m).state for m in ("forward", "reverse")} == {"ASSIGNED"}


def test_pinned_permanent_pending_has_firm_deadline(integrated):
    fleet, journal, transport, _, _, wall, observe = integrated
    transport.estimate = lambda robot, request, callback: callback(
        CostEstimate(False, 0, 0, "path pending")
    )
    fleet.submit(MissionRequest("pinned", "assembly", "inspection", "motor", "amr_01"))
    fleet.tick()
    fleet.tick()
    assert journal.get("pinned").state == "QUEUED"
    for _ in range(4):
        wall[0] += 1.01
        observe()
        fleet.tick()
        fleet.tick()
    assert journal.get("pinned").state == "FAILED"
    assert journal.get("pinned").result["error_code"] == "REQUESTED_ROBOT_UNAVAILABLE"
    assert not transport.goals


def test_cancelling_pending_pin_clears_bounded_wait_state(integrated):
    fleet, journal, transport, _, planners, wall, observe = integrated
    fleet.submit(MissionRequest("pinned", "assembly", "inspection", "motor", "amr_01"))
    fleet.tick()
    fleet.tick()
    fleet.cancel("pinned")
    assert fleet._path_pending_since == {}
    complete(planners["amr_01"])
    wall[0] += 4
    observe()
    fleet.tick()
    assert journal.get("pinned").state == "CANCELLED" and not transport.goals


@pytest.mark.parametrize(
    "mode,health,pose,payload",
    [
        ("AVAILABLE", "OFFLINE", Pose2D(0, 0, 0), "EMPTY"),
        ("AVAILABLE", "UNHEALTHY", Pose2D(0, 0, 0), "EMPTY"),
        ("EXECUTING", "ONLINE", Pose2D(0, 0, 0), "EMPTY"),
        ("AVAILABLE", "ONLINE", None, "EMPTY"),
        ("AVAILABLE", "ONLINE", Pose2D(0, 0, 0), "UNKNOWN"),
    ],
)
def test_pending_response_cannot_override_current_pinned_robot_unavailability(
    integrated, mode, health, pose, payload
):
    fleet, journal, transport, _, _, _, _ = integrated
    fleet.submit(MissionRequest("pinned", "assembly", "inspection", "motor", "amr_01"))
    fleet.tick()  # Production agent responds path pending, callback awaits decision.
    fleet.observe(RobotSnapshot("amr_01", mode, pose, 80, payload, health=health))
    fleet.tick()
    record = journal.get("pinned")
    assert record.state == "FAILED"
    assert record.result["error_code"] == "REQUESTED_ROBOT_UNAVAILABLE"
    assert not transport.goals
    assert fleet._path_pending_since == {} and fleet._retry_at == {}
