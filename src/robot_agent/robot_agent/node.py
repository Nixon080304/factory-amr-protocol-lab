# SPDX-License-Identifier: Apache-2.0
"""Steady health telemetry and bounded asynchronous robot-local cost service."""

from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import re
import time

from factory_interfaces.msg import ProtocolEvent, RobotState
from factory_interfaces.action import DockRobot
from factory_interfaces.srv import (
    AcquireResource,
    CancelResourceWait,
    EstimateMissionCost,
    ReleaseResource,
    RenewResource,
)
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.impl.implementation_singleton import rclpy_implementation
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty

from robot_agent.adapter import AgentAdapter, finite_pose
from robot_agent.energy_model import EnergyConfig
from robot_agent.nav2_paths import Nav2Paths
from robot_agent.nav2_paths import pose_message
from robot_agent.docking import DockingController, DockResult


@dataclass(frozen=True)
class AgentConfig:
    robot_id: str
    frame_prefix: str
    stations: dict
    battery_start_percent: float = 100
    energy: EnergyConfig = field(default_factory=EnergyConfig)
    stale_sec: float = 3.0
    estimate_timeout_sec: float = 0.75
    max_odom_step_m: float = 10.0
    cache_max_age_sec: float = 2.0
    cache_move_tolerance_m: float = 0.1
    localization_move_tolerance_m: float = 0.5
    cache_max_entries: int = 16


def stamp_seconds(stamp):
    if stamp.sec < 0 or stamp.nanosec >= 1000000000:
        return float("nan")
    return stamp.sec + stamp.nanosec / 1e9


def valid_pose(pose):
    p, q = pose.position, pose.orientation
    return (
        all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        and abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1) <= 0.01
    )


def optional_exit_pose(node):
    """Declare an immutable DOUBLE_ARRAY with an empty V1 default on Humble."""
    name = "dock.exit_pose"
    parameter = node.declare_parameter(name, Parameter.Type.DOUBLE_ARRAY)
    value = parameter.value
    if value is None:
        # Humble infers [] as BYTE_ARRAY, so initialize the typed empty default
        # explicitly before making the descriptor immutable.
        result = node.set_parameters([Parameter(name, Parameter.Type.DOUBLE_ARRAY, [])])
        if not result[0].successful:
            raise ValueError("dock exit pose default could not be initialized")
        value = []
    node.set_descriptor(
        name,
        ParameterDescriptor(read_only=True, type=Parameter.Type.DOUBLE_ARRAY.value),
    )
    if len(value) == 0:
        return None
    if not finite_pose(value):
        raise ValueError("dock exit pose requires zero or three finite values")
    return tuple(value)


class AgentRuntime:
    """Production ROS boundary, also runnable with an external wire substitute."""

    def __init__(self, node, config, paths, *, clock=time.monotonic):
        self.node = node
        self.adapter = AgentAdapter(
            config.robot_id,
            config.frame_prefix,
            config.stations,
            paths,
            battery_percent=config.battery_start_percent,
            energy_config=config.energy,
            clock=clock,
            stale_sec=config.stale_sec,
            estimate_timeout_sec=config.estimate_timeout_sec,
            max_odom_step_m=config.max_odom_step_m,
            cache_max_age_sec=config.cache_max_age_sec,
            cache_move_tolerance_m=config.cache_move_tolerance_m,
            localization_move_tolerance_m=config.localization_move_tolerance_m,
            cache_max_entries=config.cache_max_entries,
        )
        self.states = node.create_publisher(RobotState, "factory/robot_state", 10)
        self.batteries = node.create_publisher(BatteryState, "battery_state", 10)
        payload_qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.subscriptions = [
            node.create_subscription(
                Odometry, "odom", self.odometry, qos_profile_sensor_data
            ),
            node.create_subscription(
                PoseWithCovarianceStamped,
                "amcl_pose",
                self.localization,
                QoSProfile(
                    depth=1,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                ),
            ),
            node.create_subscription(
                String,
                "factory/mission_state",
                lambda message: self.adapter.mission_state(message.data),
                100,
            ),
            node.create_subscription(
                String,
                "/factory/payload_state",
                lambda message: self.adapter.payload(message.data),
                payload_qos,
            ),
            node.create_subscription(
                ProtocolEvent, "/factory/protocol_events", self.adapter.protocol, 100
            ),
        ]
        self.service = node.create_service(
            EstimateMissionCost,
            "factory/estimate_mission_cost",
            self.estimate,
        )
        self.heartbeat_timer = node.create_timer(
            0.5, self.heartbeat, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )
        self.deadline_timer = node.create_timer(
            0.02, self.adapter.tick, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )

    def odometry(self, message):
        p = message.pose.pose
        stamp = stamp_seconds(message.header.stamp)
        if (
            not valid_pose(p)
            or message.child_frame_id != self.adapter.frame_prefix + "base_footprint"
        ):
            stamp = float("nan")
        self.adapter.odometry(
            stamp, p.position.x, p.position.y, message.header.frame_id
        )

    def localization(self, message):
        p, q = message.pose.pose, message.pose.pose.orientation
        stamp = stamp_seconds(message.header.stamp) if valid_pose(p) else float("nan")
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.adapter.localization(
            stamp, p.position.x, p.position.y, yaw, message.header.frame_id
        )

    def heartbeat(self):
        state, stamp = self.adapter.heartbeat(), self.node.get_clock().now().to_msg()
        message = RobotState(
            stamp=stamp,
            robot_id=state.robot_id,
            mode=state.mode,
            frame_id=state.frame_id,
            battery_percent=float(state.battery_percent),
            payload_state=state.payload_state,
            mission_id=state.mission_id,
            health_detail=state.health_detail,
        )
        if state.pose is not None:
            message.pose.position.x, message.pose.position.y = state.pose[:2]
            message.pose.orientation.z = math.sin(state.pose[2] / 2)
            message.pose.orientation.w = math.cos(state.pose[2] / 2)
        battery = BatteryState()
        battery.header.stamp, battery.header.frame_id = (
            stamp,
            self.adapter.frame_prefix + "base_link",
        )
        battery.percentage, battery.present = state.battery_percent / 100, True
        battery.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        if (
            self.adapter.mode == "CHARGING"
            and self.adapter.charge_authorized()
            and self.adapter.clock()
            < min(self.adapter._lease_until, self.adapter._dock_contact_until)
            and not state.health_detail
        ):
            battery.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        for name in (
            "voltage",
            "temperature",
            "current",
            "charge",
            "capacity",
            "design_capacity",
        ):
            setattr(battery, name, float("nan"))
        self.states.publish(message)
        self.batteries.publish(battery)

    def estimate(self, request, response):
        result = self.adapter.cached_estimate(request)
        response.feasible, response.path_cost = result.feasible, result.path_cost
        response.predicted_final_battery, response.reason = (
            result.predicted_final_battery,
            result.reason,
        )
        return response


class DockTransport:
    """Preserve late goal acceptance and lease replies so cleanup can resolve them."""

    def __init__(
        self,
        node,
        navigation,
        resources,
        *,
        behavior_tree="",
        diagnostic=None,
        root_nav2=False,
    ):
        if not isinstance(behavior_tree, str) or (
            behavior_tree
            and (
                not Path(behavior_tree).is_absolute()
                or not Path(behavior_tree).is_file()
            )
        ):
            raise ValueError(
                "dock behavior tree must be empty or an existing absolute file path"
            )
        self.node, self.navigation, self.resources = node, navigation, resources
        self.behavior_tree = behavior_tree
        self.diagnostic = diagnostic
        self.localization_client = node.create_client(
            Empty,
            "/request_nomotion_update" if root_nav2 else "request_nomotion_update",
        )

    def localization_epoch(self):
        return stamp_seconds(self.node.get_clock().now().to_msg())

    def _diagnose(self, event, **fields):
        if self.diagnostic is not None:
            try:
                self.diagnostic(event, **fields)
            except Exception:
                pass

    def request_localization(self, callback):
        """Request robot-local AMCL evidence without waiting in a ROS callback."""
        client = self.localization_client
        ready = client.service_is_ready()
        self._diagnose("dock_localization_ready", service_ready=ready)
        if not ready:
            callback(False)
            return None
        future = client.call_async(Empty.Request())
        active = True

        def responded(done):
            nonlocal active
            if not active:
                return
            active = False
            try:
                success = isinstance(done.result(), Empty.Response)
            except Exception:
                success = False
            callback(success)

        def cancel():
            nonlocal active
            if active:
                active = False
                remove = getattr(client, "remove_pending_request", None)
                if remove is not None:
                    remove(future)

        future.add_done_callback(responded)
        return cancel

    def navigate(self, pose, frame, callback, *, precise=False):
        behavior_tree = self.behavior_tree if precise else ""
        if behavior_tree and not Path(behavior_tree).is_file():
            callback(False, "dock behavior tree unavailable")
            return lambda: None
        ready = self.navigation.server_is_ready()
        self._diagnose(
            "dock_navigation_ready",
            server_ready=ready,
            precise=precise,
            behavior_tree_configured=bool(behavior_tree),
        )
        if not ready:
            callback(False, "navigation unavailable")
            return lambda: None
        handle, cancelled, finished, stopped = None, False, False, False

        def cancel():
            nonlocal cancelled
            if not cancelled and not stopped:
                cancelled = True
                if handle is not None and handle.accepted:
                    handle.cancel_goal_async()

        def complete(success, reason, terminal=True):
            nonlocal finished, stopped
            if not finished:
                finished = True
                stopped = terminal
                if not terminal:
                    try:
                        cancel()
                    except Exception as error:
                        reason += "; cancel failed: " + str(error)
                callback(success, reason, terminal)

        def result(future):
            try:
                status = future.result().status
                if status not in (
                    GoalStatus.STATUS_SUCCEEDED,
                    GoalStatus.STATUS_CANCELED,
                    GoalStatus.STATUS_ABORTED,
                ):
                    complete(False, "navigation result is not terminal", False)
                    return
                complete(
                    not cancelled and status == GoalStatus.STATUS_SUCCEEDED,
                    "navigation cancelled"
                    if cancelled
                    else ""
                    if status == GoalStatus.STATUS_SUCCEEDED
                    else "navigation failed",
                )
            except Exception as error:
                complete(False, str(error), False)

        def accepted(future):
            nonlocal handle
            try:
                handle = future.result()
                if not handle.accepted:
                    complete(False, "navigation rejected")
                    return
                if cancelled:
                    handle.cancel_goal_async()
                handle.get_result_async().add_done_callback(result)
            except Exception as error:
                complete(False, str(error), False)

        future = self.navigation.send_goal_async(
            NavigateToPose.Goal(
                pose=pose_message(pose, frame, self.node.get_clock().now().to_msg()),
                behavior_tree=behavior_tree,
            )
        )
        future.add_done_callback(accepted)

        return cancel

    def resource(self, operation, key, callback):
        types = {
            "acquire": AcquireResource,
            "renew": RenewResource,
            "release": ReleaseResource,
            "cancel_wait": CancelResourceWait,
        }
        service, client = types[operation], self.resources[operation]
        if not client.service_is_ready():
            response = service.Response()
            if operation == "cancel_wait":
                response.reconciliation_required = True
            callback(response)
            return
        fields = dict(
            robot_id=key.robot_id,
            mission_id=key.mission_id,
            resource_id=key.resource_id,
        )
        if operation in ("release", "renew"):
            fields["lease_id"] = key.lease_id
        future = client.call_async(service.Request(**fields))

        def responded(future):
            try:
                response = future.result()
            except Exception:
                response = service.Response()
                if operation == "cancel_wait":
                    response.reconciliation_required = True
            callback(response)

        future.add_done_callback(responded)


class DockRuntime:
    """Defer action execution until controller supplies its terminal result."""

    def __init__(
        self,
        node,
        agent,
        transport,
        dock_id,
        staging,
        charging,
        *,
        server_factory=ActionServer,
        robot_radius=0.15,
        arrival_tolerance=0.15,
        exit_pose=None,
    ):
        self.node = node
        self.controller = DockingController(
            agent,
            transport,
            dock_id,
            staging,
            charging,
            clock=agent.clock,
            robot_radius=robot_radius,
            tolerance=arrival_tolerance,
            exit_pose=exit_pose,
            diagnostic=self._diagnostic,
        )
        self._reserved, self._handle, self._results = False, None, {}
        self._last_goal_diagnostic = None
        if isinstance(transport, DockTransport):
            transport.diagnostic = self.controller.diagnose
        self.server = server_factory(
            node,
            DockRobot,
            "factory/dock_robot",
            self.execute,
            goal_callback=self.goal,
            cancel_callback=self.cancel,
            handle_accepted_callback=self.accepted,
        )
        self.contact_subscription = node.create_subscription(
            Bool,
            "factory/dock_contact",
            lambda message: self.controller.contact(message.data),
            10,
        )
        self.timer = node.create_timer(
            0.02, self.controller.tick, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )

    def _diagnostic(self, row):
        try:
            key = self.controller.key

            def clean(value):
                if isinstance(value, str):
                    for token in () if key is None else (key.lease_id, key.mission_id):
                        if token:
                            value = value.replace(token, "[redacted]")
                    return value[:256]
                if isinstance(value, float) and not math.isfinite(value):
                    return None
                if isinstance(value, (tuple, list)):
                    return [clean(item) for item in value[:3]]
                return value

            row = {name: clean(value) for name, value in row.items()}
            if "message" in row:
                row["message"] = re.sub(
                    r"[A-Za-z0-9_-]{16,}", "[redacted]", row["message"]
                )
            self.node.get_logger().info(
                json.dumps(row, sort_keys=True, allow_nan=False)
            )
        except Exception:
            pass

    def _goal_decision(self, accepted, reason):
        evidence = self.controller.evidence()
        signature = (
            accepted,
            reason,
            evidence["mode"],
            evidence["payload_state"],
            evidence["health_detail"],
            self.controller._generation,
        )
        if signature != self._last_goal_diagnostic:
            self._last_goal_diagnostic = signature
            self.controller.diagnose(
                "dock_goal",
                decision="ACCEPT" if accepted else "REJECT",
                reason=reason,
                reserved=self._reserved,
                **evidence,
            )
        return GoalResponse.ACCEPT if accepted else GoalResponse.REJECT

    def goal(self, goal):
        if self._reserved:
            return self._goal_decision(False, "reserved")
        if not self.controller.can_start():
            return self._goal_decision(False, "robot_unavailable")
        if goal.dock_id != self.controller.dock_id:
            return self._goal_decision(False, "dock_identity")
        if not math.isfinite(goal.target_percent) or not 0 < goal.target_percent <= 100:
            return self._goal_decision(False, "target_percent")
        for message, expected in (
            (goal.staging_pose, self.controller.staging),
            (goal.charging_pose, self.controller.charging),
        ):
            if (
                message.header.frame_id != self.controller.agent.frame_prefix + "map"
                or not valid_pose(message.pose)
            ):
                return self._goal_decision(False, "pose_frame_or_geometry")
            p, q = message.pose.position, message.pose.orientation
            yaw = math.atan2(
                2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)
            )
            if (
                abs(p.x - expected[0]) > 1e-6
                or abs(p.y - expected[1]) > 1e-6
                or abs(p.z) > 1e-6
                or abs(math.remainder(yaw - expected[2], 2 * math.pi)) > 1e-6
            ):
                return self._goal_decision(False, "pose_config_mismatch")
        self._reserved = True
        return self._goal_decision(True, "validated")

    def accepted(self, handle):
        self._handle = handle

        def feedback(value):
            if self._handle is handle:
                handle.publish_feedback(
                    DockRobot.Feedback(
                        state=value.state,
                        battery_percent=float(value.battery_percent),
                        detail=value.detail,
                    )
                )

        def result(value):
            if self._handle is handle:
                self.controller.diagnose(
                    "dock_result",
                    success=value.success,
                    error_code=value.error_code,
                    message=value.message,
                    **self.controller.evidence(),
                )
                self._results[id(handle)] = value
                self._handle, self._reserved = None, False
                try:
                    handle.execute()
                except rclpy_implementation.RCLError as error:
                    # A signal-invalidated action publisher cannot schedule a
                    # terminal result. Keep every active or unrelated error.
                    if self.node.context.ok() or str(error) != (
                        "Failed get goal status array: feedback publisher is invalid, "
                        "at ./src/rcl_action/action_server.c:919"
                    ):
                        raise
                    self._results.pop(id(handle), None)

        if not self.controller.start(handle.request.target_percent, feedback, result):
            result(DockResult(False, "ROBOT_UNAVAILABLE", "Robot cannot start docking"))

    def execute(self, handle):
        outcome = self._results.pop(id(handle))
        if outcome.success:
            handle.succeed()
        elif outcome.error_code == "CANCELLED" and handle.is_cancel_requested:
            handle.canceled()
        else:
            handle.abort()
        return DockRobot.Result(
            success=outcome.success,
            error_code=outcome.error_code,
            message=outcome.message,
        )

    def cancel(self, handle):
        if self._handle is not handle:
            return CancelResponse.REJECT
        self.controller.cancel()
        return CancelResponse.ACCEPT

    def shutdown(self):
        self.controller.shutdown()
        self.server.destroy()


class RobotAgentNode(Node):
    def __init__(self, **kwargs):
        super().__init__("robot_agent", **kwargs)
        identity = ParameterDescriptor(read_only=True)
        default_id = self.get_namespace().rstrip("/").split("/")[-1] or "amr_01"
        robot_id = self.declare_parameter("robot_id", default_id, identity).value
        prefix = self.declare_parameter(
            "frame_prefix",
            robot_id + "/" if self.get_namespace() != "/" else "",
            identity,
        ).value
        legacy = self.declare_parameter(
            "legacy_unprefixed_frames", False, identity
        ).value
        if type(legacy) is not bool or (legacy and prefix):
            raise ValueError(
                "legacy_unprefixed_frames requires bool and empty frame_prefix"
            )
        if self.get_namespace() != "/" and not prefix and not legacy:
            raise ValueError("namespaced robots require frame_prefix")
        stations = {}
        for station in self.declare_parameter(
            "station_names", ["assembly", "inspection"], identity
        ).value:
            stations[station] = tuple(
                self.declare_parameter(
                    f"stations.{station}.pose",
                    [-3.0, 0.8, math.pi / 2]
                    if station == "assembly"
                    else [3.0, 0.8, math.pi / 2],
                    identity,
                ).value
            )
        energy = EnergyConfig(
            **{
                name: self.declare_parameter("energy." + name, value, identity).value
                for name, value in vars(EnergyConfig()).items()
            }
        )
        self.dock_id = self.declare_parameter("dock_id", "dock_01", identity).value
        self.dock_radius = self.declare_parameter(
            "dock.robot_radius", 0.15, identity
        ).value
        self.dock_tolerance = self.declare_parameter(
            "dock.arrival_tolerance", 0.15, identity
        ).value
        self.dock_staging = self.declare_parameter(
            "dock.staging_pose", [3.5, -2.0, 0.0], identity
        ).value
        self.dock_charging = self.declare_parameter(
            "dock.charging_pose", [4.0, -2.0, 0.0], identity
        ).value
        self.dock_exit = optional_exit_pose(self)
        self.dock_behavior_tree = self.declare_parameter(
            "dock.behavior_tree", "", identity
        ).value
        config = AgentConfig(
            robot_id,
            prefix,
            stations,
            self.declare_parameter("battery_start_percent", 100.0, identity).value,
            energy,
            self.declare_parameter("stale_sec", 3.0, identity).value,
            self.declare_parameter("estimate_timeout_sec", 0.75, identity).value,
            self.declare_parameter("max_odom_step_m", 10.0, identity).value,
            self.declare_parameter("cache_max_age_sec", 2.0, identity).value,
            self.declare_parameter("cache_move_tolerance_m", 0.1, identity).value,
            self.declare_parameter(
                "localization_move_tolerance_m", 0.5, identity
            ).value,
            self.declare_parameter("cache_max_entries", 16, identity).value,
        )
        self.path_client = ActionClient(
            self,
            ComputePathToPose,
            "/compute_path_to_pose" if legacy else "compute_path_to_pose",
        )
        self.runtime = AgentRuntime(
            self,
            config,
            Nav2Paths(self.path_client, lambda: self.get_clock().now().to_msg()),
        )
        self.navigation_client = ActionClient(
            self, NavigateToPose, "/navigate_to_pose" if legacy else "navigate_to_pose"
        )
        self.resource_clients = {
            operation: self.create_client(service, "/factory/resources/" + operation)
            for operation, service in (
                ("acquire", AcquireResource),
                ("renew", RenewResource),
                ("release", ReleaseResource),
                ("cancel_wait", CancelResourceWait),
            )
        }
        self.docking = DockRuntime(
            self,
            self.runtime.adapter,
            DockTransport(
                self,
                self.navigation_client,
                self.resource_clients,
                behavior_tree=self.dock_behavior_tree,
                root_nav2=legacy,
            ),
            self.dock_id,
            self.dock_staging,
            self.dock_charging,
            robot_radius=self.dock_radius,
            arrival_tolerance=self.dock_tolerance,
            exit_pose=self.dock_exit,
        )

    def destroy_node(self):
        if hasattr(self, "docking"):
            self.docking.shutdown()
        if hasattr(self, "runtime"):
            self.runtime.adapter.shutdown()
        if hasattr(self, "path_client"):
            self.path_client.destroy()
        if hasattr(self, "navigation_client"):
            self.navigation_client.destroy()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = SingleThreadedExecutor()
    try:
        node = RobotAgentNode()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as error:
        # SIGINT can interrupt generated message conversion inside native take.
        # Keep active-context and unrelated robot agent errors visible.
        if rclpy.ok() or str(error) != (
            "Unable to convert call argument to Python object "
            "(compile in debug mode for details)"
        ):
            raise
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
