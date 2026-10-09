# SPDX-License-Identifier: Apache-2.0
"""ROS transport for durable fleet orchestration."""

from pathlib import Path
from dataclasses import replace
import fcntl
import json
import time
import sqlite3

from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from factory_interfaces.action import (
    DockRobot,
    ExecuteFactoryMission,
    ExecuteFleetMission,
)
from geometry_msgs.msg import PoseStamped
import math
from factory_interfaces.msg import ProtocolEvent, RobotState
from factory_interfaces.srv import (
    AcquireResource,
    CancelResourceWait,
    EstimateMissionCost,
    GetFleetMission,
    ReleaseResource,
    RenewResource,
)
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import CallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool

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
from fleet_manager.web import FleetDashboard


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


class _ObservationGroup(CallbackGroup):
    """Let the steady timer take DDS samples with their original metadata.

    Humble's executor discards MessageInfo before invoking subscriptions. Native
    take_message supplies source_timestamp, which still advances while ROS time
    is paused. Keep these subscriptions out of executor callback dispatch.
    """

    def can_execute(self, entity):
        return False

    def beginning_execution(self, entity):
        return False

    def ending_execution(self, entity):
        pass


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
        legacy_single_robot = self.declare_parameter("legacy_single_robot", False).value
        config = manager_config(fleet_file, legacy_single_robot)
        self._journal_lock = Path(str(journal_path.resolve()) + ".manager.lock").open(
            "a"
        )
        try:
            fcntl.flock(self._journal_lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            self._journal_lock.close()
            super().destroy_node()
            raise RuntimeError("journal already has a fleet manager writer") from error
        self.protocol_group = MutuallyExclusiveCallbackGroup()
        self.observation_group = _ObservationGroup()
        self._waiters = {}
        self._results = {}
        self.actions, self.costs, self.states, self.docks = {}, {}, [], {}
        self.contacts = []
        try:
            self.journal = MissionJournal(journal_path)
        except BaseException:
            self._journal_lock.close()
            super().destroy_node()
            raise
        self.adapter = FleetAdapter(
            config,
            self.journal,
            self,
            diagnostic=lambda row: self.get_logger().info(
                json.dumps(row, sort_keys=True, allow_nan=False)
            ),
            reconciliation_timeout=self.declare_parameter(
                "reconciliation_timeout_sec", 5.0
            ).value,
        )
        for robot in config.robots:
            endpoints = robot_endpoints(robot)
            self.actions[robot.robot_id] = ActionClient(
                self,
                ExecuteFactoryMission,
                "/factory/execute_mission"
                if legacy_single_robot
                else endpoints["action"],
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
                    lambda _: None,
                    qos_profile_sensor_data,
                    callback_group=self.observation_group,
                )
            )
            self.contacts.append(
                self.create_subscription(
                    Bool,
                    robot.namespace.rstrip("/") + "/factory/dock_contact",
                    lambda _: None,
                    qos_profile_sensor_data,
                    callback_group=self.observation_group,
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
        self.mission_query = self.create_service(
            GetFleetMission,
            "/factory/get_fleet_mission",
            self._query_mission,
            callback_group=self.protocol_group,
        )
        self.timer = self.create_timer(
            0.05,
            self._tick,
            callback_group=self.protocol_group,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )
        self.dashboard = None
        enabled = self.declare_parameter("dashboard_enabled", False).value
        host = self.declare_parameter("dashboard_host", "127.0.0.1").value
        port = self.declare_parameter("dashboard_port", 8080).value
        if enabled:
            dashboard = None
            try:
                dashboard = FleetDashboard(
                    host=host, port=port, journal_path=journal_path
                )
                dashboard.capture(self.adapter, self._dispatch_readiness())
                dashboard.start()
                self.dashboard = dashboard
                self.dashboard_events = self.create_subscription(
                    ProtocolEvent,
                    "/factory/protocol_events",
                    self._dashboard_event,
                    100,
                    callback_group=self.protocol_group,
                )
            except Exception as error:
                if dashboard is not None:
                    try:
                        dashboard.stop()
                    except Exception as cleanup_error:
                        self.get_logger().warning(
                            f"Dashboard cleanup failed: {cleanup_error}"
                        )
                self.dashboard = None
                self.get_logger().warning(f"Dashboard unavailable: {error}")

    def _dispatch_readiness(self):
        """Read only this participant's bounded dispatch endpoint discovery."""
        return {
            robot.robot_id: {
                "cost_service_ready": self.costs[robot.robot_id].service_is_ready(),
                "mission_action_ready": self.actions[robot.robot_id].server_is_ready(),
            }
            for robot in self.adapter.config.robots
        }

    def _dashboard_event(self, message):
        try:
            if self.dashboard is not None:
                self.dashboard.observe_event(message)
        except Exception as error:
            self.get_logger().warning(f"Dashboard observation failed: {error}")

    def _take_message(self, subscription, message_type):
        if not self.context.ok():
            return None
        try:
            with subscription.handle:
                return subscription.handle.take_message(message_type, False)
        except RuntimeError as error:
            # SIGINT can interrupt the generated message conversion after the
            # RMW take starts. Active-context and unrelated failures stay fatal.
            if not self.context.ok() and str(error) == (
                "Unable to convert call argument to Python object "
                "(compile in debug mode for details)"
            ):
                return None
            raise

    def _tick(self):
        if not self.context.ok():
            return
        for robot, subscription in zip(self.adapter.config.robots, self.contacts):
            for _ in range(20):
                sample = self._take_message(subscription, Bool)
                if sample is None:
                    break
                self.adapter.observe_contact(
                    robot.robot_id,
                    sample[0].data,
                    source_time_ns=sample[1].get("source_timestamp", 0),
                )
        for robot, subscription in zip(self.adapter.config.robots, self.states):
            for _ in range(20):
                sample = self._take_message(subscription, RobotState)
                if sample is None:
                    break
                self._observe(robot.robot_id, sample[0], sample[1])
        if not self.context.ok():
            return
        try:
            self.adapter.tick()
        except sqlite3.Error as error:
            self.get_logger().error(str(error))
        try:
            if self.dashboard is not None:
                self.dashboard.capture(self.adapter, self._dispatch_readiness())
        except Exception as error:
            self.get_logger().warning(f"Dashboard observation failed: {error}")

    def _observe(self, robot_id, message, info=None):
        try:
            robot = next(
                robot
                for robot in self.adapter.config.robots
                if robot.robot_id == robot_id
            )
            snapshot = robot_snapshot(robot_id, message)
            if message.frame_id != robot.frame_prefix + "map":
                snapshot = replace(
                    snapshot,
                    pose=None,
                    health="UNHEALTHY",
                    health_detail="pose frame mismatch",
                )
            self.adapter.observe(
                snapshot,
                source_time_ns=info.get("source_timestamp", 0)
                if info is not None
                else None,
            )
        except (ValueError, sqlite3.Error) as error:
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
        except (ValueError, sqlite3.Error) as error:
            self.get_logger().warning(str(error))
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _query_mission(self, request, response):
        """Read durable state without registering work or restoring authority."""
        try:
            record = self.journal.get(request.mission_id)
        except KeyError:
            return response
        response.found = True
        response.matches = record.request == _request(request)
        if not response.matches:
            return response
        response.ready = self.adapter.state == "RUNNING" or not any(
            active.request.mission_id == request.mission_id
            for active in self.journal.load_active()
        )
        response.state = record.state.value
        response.assigned_robot_id = record.assigned_robot_id or ""
        response.success = record.state == MissionState.COMPLETED
        result = record.result or {}
        recovery = record.state == MissionState.RECOVERY_REQUIRED
        response.error_code = str(
            result.get("error_code", "RECOVERY_REQUIRED" if recovery else "")
        )
        response.message = str(
            result.get("message", "Recovery required" if recovery else "")
        )
        return response

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

    def _cost_diagnostic(self, event, robot_id, mission_id, **fields):
        # Diagnostics must not change action/service behavior or error handling.
        try:
            self.adapter.diagnose_cost(event, robot_id, mission_id, **fields)
        except Exception:
            pass

    def estimate(self, robot_id, request, callback):
        client = self.costs[robot_id]
        service_ready = client.service_is_ready()
        action_ready = self.actions[robot_id].server_is_ready()
        self._cost_diagnostic(
            "fleet_cost_dispatch",
            robot_id,
            request.mission_id,
            cost_service_ready=service_ready,
            mission_action_ready=action_ready,
        )
        if not service_ready or not action_ready:
            callback(None)
            return
        started = time.monotonic()
        future = client.call_async(
            EstimateMissionCost.Request(
                mission_id=request.mission_id,
                pickup_station=request.pickup_station,
                dropoff_station=request.dropoff_station,
                part=request.part,
            )
        )

        def estimated(future):
            error = ""
            try:
                response = future.result()
                estimate = CostEstimate(
                    response.feasible,
                    response.path_cost,
                    response.predicted_final_battery,
                    response.reason,
                )
            except Exception as exception:
                estimate = None
                error = str(exception)
            self._cost_diagnostic(
                "fleet_cost_result",
                robot_id,
                request.mission_id,
                feasible=None if estimate is None else estimate.feasible,
                path_cost=None if estimate is None else estimate.path_cost,
                predicted_final_battery=None
                if estimate is None
                else estimate.predicted_final_battery,
                reason=error if estimate is None else estimate.reason,
                elapsed_sec=time.monotonic() - started,
            )
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
        from fleet_manager.adapter import DockRequestUnsent

        client = self.docks[robot_id]
        ready = client.server_is_ready()
        reported = getattr(self, "_dock_dispatch_diagnostics", None)
        if reported is None:
            reported = self._dock_dispatch_diagnostics = {}
        signature = (charge.generation, ready)
        if reported.get(robot_id) != signature:
            reported[robot_id] = signature
            self.adapter.diagnose_dock(
                "fleet_dock_dispatch", charge, server_ready=ready
            )
        if not ready:
            raise DockRequestUnsent("dock action unavailable")
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
                self.adapter.diagnose_dock(
                    "fleet_dock_action_status",
                    charge,
                    status=outcome.status,
                    success=reply.success,
                    error_code=reply.error_code,
                    message=reply.message,
                )
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
        if getattr(self, "dashboard", None) is not None:
            try:
                self.dashboard.stop()
            except Exception as error:
                self.get_logger().warning(f"Dashboard cleanup failed: {error}")
        self.server.destroy()
        for client in self.actions.values():
            client.destroy()
        for client in self.docks.values():
            client.destroy()
        self.journal.close()
        self._journal_lock.close()
        return super().destroy_node()
