"""Two configured agents exercise fleet queueing, actual leases, and dock handoff."""

from dataclasses import replace
import importlib
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "src" / "fleet_manager"))
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "src" / "robot_agent"))

from fleet_manager.adapter import FleetAdapter
from fleet_manager.config import Pose2D, load_fleet_config
from fleet_manager.journal import MissionJournal
from fleet_manager.models import CostEstimate, MissionRequest, RobotSnapshot
from robot_agent.adapter import AgentAdapter

ROOT = Path(__file__).resolve().parents[3]


class Scenario:
    def __init__(self, config, tmp_path):
        try:
            docking = importlib.import_module("robot_agent.docking")
        except ModuleNotFoundError:
            pytest.fail("production docking controller is missing")
        self.now = 100.0
        self.journal = MissionJournal(tmp_path / "missions.sqlite3")
        self.fleet = FleetAdapter(config, self.journal, self, clock=lambda: self.now)
        self.agents, self.controllers, self.moves, self.starts = {}, {}, {}, []
        dock = config.docks[config.energy.dock_id]
        for robot in config.robots:
            agent = AgentAdapter(
                robot.robot_id,
                robot.frame_prefix,
                {"assembly": (-3, 1, 0), "inspection": (3, 1, 0)},
                None,
                battery_percent=20,
                clock=lambda: self.now,
            )
            agent.odometry(1, 0, 0, robot.frame_prefix + "odom")
            agent.localization(
                1,
                robot.spawn.x,
                robot.spawn.y,
                robot.spawn.yaw,
                robot.frame_prefix + "map",
            )
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
            self.agents[robot.robot_id] = agent
            wire = SimpleNamespace(
                navigate=lambda pose, frame, callback, robot_id=robot.robot_id: (
                    self.navigate(robot_id, pose, frame, callback)
                ),
                resource=lambda operation, key, callback: callback(
                    SimpleNamespace(**self.fleet.resource(operation, key))
                ),
            )
            self.controllers[robot.robot_id] = docking.DockingController(
                agent,
                wire,
                config.energy.dock_id,
                (dock.staging_pose.x, dock.staging_pose.y, dock.staging_pose.yaw),
                (dock.charging_pose.x, dock.charging_pose.y, dock.charging_pose.yaw),
                clock=lambda: self.now,
            )
        self.observe()

    def observe(self):
        for agent in self.agents.values():
            state = agent.heartbeat()
            self.fleet.observe(
                RobotSnapshot(
                    state.robot_id,
                    state.mode,
                    Pose2D(*state.pose) if state.pose else None,
                    state.battery_percent,
                    state.payload_state,
                    state.mission_id or None,
                    health_detail=state.health_detail,
                )
            )

    def navigate(self, robot_id, pose, frame, callback):
        self.moves.setdefault(robot_id, []).append((pose, frame, callback))
        return lambda: None

    def send_dock_goal(self, robot_id, charge, feedback, result, accepted):
        self.starts.append(robot_id)
        controller = self.controllers[robot_id]
        accepted(controller, "")
        assert controller.start(charge.target_percent, feedback, result)

    def estimate(self, robot_id, request, callback):
        callback(CostEstimate(True, 1, 40))

    def send_goal(self, *args):
        pytest.fail("charging robot received mission work")

    def publish(self, *args):
        pass

    def cancel(self, handle, callback):
        handle.cancel()
        callback(True)

    def arrive(self, robot_id, index):
        agent = self.agents[robot_id]
        pose, frame, callback = self.moves[robot_id][index]
        stamp = agent._pose_stamp + 1
        agent.odometry(stamp, 0, 0, agent.frame_prefix + "odom")
        agent.localization(stamp, *pose, frame)
        callback(True, "")
        self.controllers[robot_id].tick()
        self.observe()
        self.fleet.tick()


@pytest.mark.parametrize("renamed", [False, True])
def test_two_robots_stage_without_lease_then_charge_and_handoff(tmp_path, renamed):
    config = load_fleet_config(ROOT / "src/factory_bringup/config/fleet.yaml")
    if renamed:
        config = replace(
            config,
            robots=tuple(
                replace(
                    robot,
                    robot_id="cart_" + str(index),
                    namespace="/warehouse/cart_" + str(index),
                    frame_prefix="warehouse/cart_" + str(index) + "/",
                )
                for index, robot in enumerate(config.robots)
            ),
        )
    scenario = Scenario(config, tmp_path)
    first, second = sorted(scenario.agents)
    try:
        scenario.fleet.tick()
        scenario.fleet.tick()
        assert scenario.starts == [first]
        resource = next(
            r
            for r in scenario.fleet.resources.snapshot(scenario.now)
            if r.kind == "dock"
        )
        assert resource.lease is None  # Staging travel does not acquire the dock.
        assert (
            second not in scenario.moves
        )  # Its initial safe pose stays clear of staging.
        scenario.arrive(first, 0)
        resource = next(
            r
            for r in scenario.fleet.resources.snapshot(scenario.now)
            if r.kind == "dock"
        )
        assert resource.lease.robot_id == first
        scenario.arrive(first, 1)
        scenario.controllers[first].contact(True)
        scenario.fleet.tick()
        assert scenario.starts == [first, second]
        scenario.arrive(second, 0)
        assert len(scenario.moves[second]) == 1  # Wait outside the held dock.
        scenario.agents[first].energy.battery_percent = 79.8
        scenario.fleet.submit(MissionRequest("work", "assembly", "inspection", "motor"))
        scenario.fleet.tick()
        scenario.fleet.tick()
        assert scenario.journal.get("work").assigned_robot_id is None
        scenario.now += 0.3
        scenario.controllers[first].contact(True)
        scenario.arrive(first, 2)
        scenario.fleet.tick()
        scenario.controllers[second].tick()
        resource = next(
            r
            for r in scenario.fleet.resources.snapshot(scenario.now)
            if r.kind == "dock"
        )
        assert resource.lease.robot_id == second
        assert len(scenario.moves[second]) == 2
        scenario.arrive(second, 1)
        scenario.controllers[second].contact(True)
        scenario.agents[second].energy.battery_percent = 79.8
        scenario.now += 0.3
        scenario.controllers[second].contact(True)
        # Cancel work before freshly available first robot can receive it.
        scenario.fleet.cancel("work")
        scenario.arrive(second, 2)
        scenario.fleet.tick()
        resource = next(
            r
            for r in scenario.fleet.resources.snapshot(scenario.now)
            if r.kind == "dock"
        )
        assert resource.lease is None and resource.waiters == ()
        assert scenario.fleet.core.charging_snapshot() == ()
    finally:
        scenario.journal.close()
