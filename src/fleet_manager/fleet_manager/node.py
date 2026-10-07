# SPDX-License-Identifier: Apache-2.0
"""ROS transport for durable fleet orchestration."""

from pathlib import Path
from dataclasses import replace

from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from factory_interfaces.action import (
    DockRobot,
    ExecuteFactoryMission,
    ExecuteFleetMission,
)
from geometry_msgs.msg import PoseStamped
import math
from factory_interfaces.msg import RobotState
from factory_interfaces.srv import (
    AcquireResource,
    CancelResourceWait,
    EstimateMissionCost,
    ReleaseResource,
    RenewResource,
)
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from fleet_manager.adapter import (
    FleetAdapter,
    RobotFeedback,
    RobotReply,
    robot_endpoints,
    robot_snapshot,
)
from fleet_manager.config import load_fleet_config
from fleet_manager.journal import MissionJournal, MissionState
from fleet_manager.models import CostEstimate, MissionRequest


def _request(goal):
    return MissionRequest(
        goal.mission_id,
        goal.pickup_station,
        goal.dropoff_station,
        goal.part,
        goal.requested_robot_id or None,
    )


def manager_config(fleet_file, legacy_single_robot=False):
    """V1 simulation has one root-frame robot; normal fleet stays unchanged."""
    if type(legacy_single_robot) is not bool:
        raise ValueError("legacy_single_robot must be bool")
    config = load_fleet_config(fleet_file)
    if legacy_single_robot:
        config = replace(config, robots=(replace(config.robots[0], frame_prefix=""),))
    return config


class FleetManagerNode(Node):
    """Run with a SingleThreadedExecutor: SQLite belongs to its creating thread.

    Future callbacks touch transport values only. FleetAdapter queues their work
    for a timer on the shared callback group. Accepted northbound handles publish
    feedback without polling. Their execute callback runs only after a terminal
    record commits; it never accesses FleetCore or the journal.
    """

    def __init__(self, **kwargs):
        super().__init__("fleet_manager", **kwargs)
        fleet_file = self.declare_parameter(
            "fleet_file",
            str(
                Path(get_package_share_directory("factory_bringup"))
                / "config/fleet.yaml"
            ),
        ).value
        journal_path = Path(
            self.declare_parameter(
                "journal_path",
                str(Path.home() / ".local/share/factory_fleet/missions.sqlite3"),
            ).value
        ).expanduser()
        journal_path.parent.mkdir(parents=True, exist_ok=True)
        config = manager_config(
            fleet_file, self.declare_parameter("legacy_single_robot", False).value
        )
        self.protocol_group = MutuallyExclusiveCallbackGroup()
        self._waiters = {}
        self._results = {}
        self.actions, self.costs, self.states, self.docks = {}, {}, [], {}
        self.journal = MissionJournal(journal_path)
        self.adapter = FleetAdapter(config, self.journal, self)
        for robot in config.robots:
            endpoints = robot_endpoints(robot)
            self.actions[robot.robot_id] = ActionClient(
                self,
                ExecuteFactoryMission,
                endpoints["action"],
                callback_group=self.protocol_group,
            )
            self.costs[robot.robot_id] = self.create_client(
                EstimateMissionCost,
                endpoints["cost"],
                callback_group=self.protocol_group,
            )
            self.docks[robot.robot_id] = ActionClient(
                self, DockRobot, endpoints["dock"], callback_group=self.protocol_group
            )
            self.states.append(
                self.create_subscription(
                    RobotState,
                    endpoints["state"],
                    lambda message, robot_id=robot.robot_id: self._observe(
                        robot_id, message
                    ),
                    qos_profile_sensor_data,
                    callback_group=self.protocol_group,
                )
            )
        self.resource_services = [
            self.create_service(
                service,
                f"/factory/resources/{operation}",
                lambda request, response, operation=operation: self._resource(
                    operation, request, response
                ),
                callback_group=self.protocol_group,
            )
            for operation, service in (
                ("acquire", AcquireResource),
                ("renew", RenewResource),
                ("release", ReleaseResource),
                ("cancel_wait", CancelResourceWait),
            )
        ]
        self.server = ActionServer(
            self,
            ExecuteFleetMission,
            "/factory/execute_fleet_mission",
            self._execute,
            goal_callback=self._goal,
            cancel_callback=self._cancel,
            handle_accepted_callback=self._accepted,
            callback_group=self.protocol_group,
        )
        self.timer = self.create_timer(
            0.05, self.adapter.tick, callback_group=self.protocol_group, clock=Clock()
        )

    def _observe(self, robot_id, message):
        try:
            self.adapter.observe(robot_snapshot(robot_id, message))
        except ValueError as error:
            self.get_logger().warning(str(error))

    def _goal(self, goal):
        stations = {
            resource.resource_id
            for resource in self.adapter.config.resources
            if resource.kind == "station"
        }
        if (
            not goal.mission_id
            or not goal.part
            or goal.pickup_station not in stations
            or goal.dropoff_station not in stations
            or goal.pickup_station == goal.dropoff_station
        ):
            return GoalResponse.REJECT
        try:
            self.adapter.submit(_request(goal))
        except ValueError as error:
            self.get_logger().warning(str(error))
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _accepted(self, handle):
        self._waiters.setdefault(handle.request.mission_id, []).append(handle)
        self.adapter.submit(
            _request(handle.request)
        )  # Replay durable state for this subscriber.

    def _execute(self, handle):
        record = self._results.pop(id(handle))
        if record.state == MissionState.COMPLETED:
            handle.succeed()
        elif record.state == MissionState.CANCELLED and handle.is_cancel_requested:
            handle.canceled()
        else:
            handle.abort()
        result = record.result or {}
        return ExecuteFleetMission.Result(
            success=record.state == MissionState.COMPLETED,
            final_state=record.state.value,
            assigned_robot_id=record.assigned_robot_id or "",
            error_code=str(
                result.get(
                    "error_code",
                    "RECOVERY_REQUIRED"
                    if record.state == MissionState.RECOVERY_REQUIRED
                    else "",
                )
            ),
            message=str(
                result.get(
                    "message",
                    "Recovery required"
                    if record.state == MissionState.RECOVERY_REQUIRED
                    else "",
                )
            ),
        )

    def _cancel(self, handle):
        self.adapter.cancel(handle.request.mission_id)
        return CancelResponse.ACCEPT

    def publish(self, record, state, detail, progress):
        waiters = self._waiters.get(record.request.mission_id, ())
        terminal = record.state in (
            MissionState.COMPLETED,
            MissionState.FAILED,
            MissionState.CANCELLED,
            MissionState.RECOVERY_REQUIRED,
        )
        for handle in waiters:
            if terminal:
                self._results[id(handle)] = record
                handle.execute()
            else:
                handle.publish_feedback(
                    ExecuteFleetMission.Feedback(
                        state=state,
                        assigned_robot_id=record.assigned_robot_id or "",
                        detail=detail,
                        progress=float(progress),
                    )
                )
        if terminal:
            self._waiters.pop(record.request.mission_id, None)

    def estimate(self, robot_id, request, callback):
        client = self.costs[robot_id]
        if (
            not client.service_is_ready()
            or not self.actions[robot_id].server_is_ready()
        ):
            callback(None)
            return
        future = client.call_async(
            EstimateMissionCost.Request(
                mission_id=request.mission_id,
                pickup_station=request.pickup_station,
                dropoff_station=request.dropoff_station,
                part=request.part,
            )
        )

        def estimated(future):
            try:
                response = future.result()
                estimate = CostEstimate(
                    response.feasible,
                    response.path_cost,
                    response.predicted_final_battery,
                    response.reason,
                )
            except Exception:
                estimate = None
            callback(estimate)

        future.add_done_callback(estimated)

        def abandon():
            client.remove_pending_request(future)
            future.cancel()

        return abandon

    def send_goal(self, robot_id, request, feedback, result, accepted):
        client = self.actions[robot_id]
        if not client.server_is_ready():
            accepted(None, "")
            return
        future = client.send_goal_async(
            ExecuteFactoryMission.Goal(
                mission_id=request.mission_id,
                robot_id=robot_id,
                pickup_station=request.pickup_station,
                dropoff_station=request.dropoff_station,
                part=request.part,
            ),
            feedback_callback=lambda message: feedback(
                RobotFeedback(
                    message.feedback.state,
                    message.feedback.detail,
                    message.feedback.progress,
                )
            ),
        )

        def completed(future):
            try:
                outcome = future.result()
                reply = outcome.result
                result(
                    RobotReply(
                        reply.success,
                        reply.error_code,
                        reply.message,
                        cancelled=outcome.status == GoalStatus.STATUS_CANCELED,
                        recovery_required=reply.final_state == "RECOVERY_REQUIRED",
                    )
                )
            except Exception as error:
                accepted(handle, str(error))

        def responded(future):
            nonlocal handle
            try:
                handle = future.result()
                if not handle.accepted:
                    accepted(None, "")
                    return
                accepted(handle, "")
                handle.get_result_async().add_done_callback(completed)
            except Exception as error:
                accepted(handle, str(error))

        handle = None
        future.add_done_callback(responded)

    def cancel(self, handle, callback):
        def cancelled(future):
            try:
                acknowledged = bool(future.result().goals_canceling)
            except Exception:
                acknowledged = False
            callback(acknowledged)

        handle.cancel_goal_async().add_done_callback(cancelled)

    def send_dock_goal(self, robot_id, charge, feedback, result, accepted):
        client = self.docks[robot_id]
        if not client.server_is_ready():
            accepted(None, "dock action unavailable")
            return
        robot = next(
            robot for robot in self.adapter.config.robots if robot.robot_id == robot_id
        )
        dock = self.adapter.config.docks[charge.dock_id]

        def pose(value):
            message = PoseStamped()
            message.header.frame_id = robot.frame_prefix + "map"
            message.pose.position.x, message.pose.position.y = (
                float(value.x),
                float(value.y),
            )
            message.pose.orientation.z, message.pose.orientation.w = (
                math.sin(value.yaw / 2),
                math.cos(value.yaw / 2),
            )
            return message

        handle = None

        def completed(future):
            try:
                outcome = future.result()
                reply = outcome.result
                result(
                    RobotReply(
                        reply.success and outcome.status == GoalStatus.STATUS_SUCCEEDED,
                        reply.error_code,
                        reply.message,
                        cancelled=outcome.status == GoalStatus.STATUS_CANCELED,
                    )
                )
            except Exception as error:
                result(RobotReply(False, "DOCK_RESULT_UNKNOWN", str(error)))

        def responded(future):
            nonlocal handle
            try:
                handle = future.result()
                if not handle.accepted:
                    accepted(None, "dock goal rejected")
                    return
                accepted(handle, "")
                handle.get_result_async().add_done_callback(completed)
            except Exception as error:
                accepted(handle, str(error))

        future = client.send_goal_async(
            DockRobot.Goal(
                dock_id=charge.dock_id,
                staging_pose=pose(dock.staging_pose),
                charging_pose=pose(dock.charging_pose),
                target_percent=float(charge.target_percent),
            ),
            feedback_callback=lambda message: feedback(
                RobotFeedback(message.feedback.state, message.feedback.detail)
            ),
        )
        future.add_done_callback(responded)

    def _resource(self, operation, request, response):
        try:
            values = self.adapter.resource(operation, request)
        except (ValueError, KeyError) as error:
            values = {"reason": str(error)}
            if operation == "cancel_wait":
                # Validation failure is not atomic proof that ownership is absent.
                values["reconciliation_required"] = True
        for field, value in values.items():
            setattr(response, field, value)
        return response

    def destroy_node(self):
        self.server.destroy()
        for client in self.actions.values():
            client.destroy()
        for client in self.docks.values():
            client.destroy()
        self.journal.close()
        return super().destroy_node()
