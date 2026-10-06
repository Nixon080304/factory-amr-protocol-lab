# SPDX-License-Identifier: Apache-2.0
"""Steady health telemetry and bounded asynchronous robot-local cost service."""

from dataclasses import dataclass, field
import math
import time

from factory_interfaces.msg import ProtocolEvent, RobotState
from factory_interfaces.srv import EstimateMissionCost
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

from robot_agent.adapter import AgentAdapter
from robot_agent.energy_model import EnergyConfig
from robot_agent.nav2_paths import Nav2Paths


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
                qos_profile_sensor_data,
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
            and self.adapter._dock_contact
            and self.adapter.clock() < self.adapter._lease_until
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
        # Dock geometry is held for Task 13's contact/lease controller.
        self.dock_id = self.declare_parameter("dock_id", "dock_01", identity).value
        self.dock_staging = self.declare_parameter(
            "dock.staging_pose", [3.5, -2.0, 0.0], identity
        ).value
        self.dock_charging = self.declare_parameter(
            "dock.charging_pose", [4.0, -2.0, 0.0], identity
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
        self.path_client = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        self.runtime = AgentRuntime(
            self,
            config,
            Nav2Paths(self.path_client, lambda: self.get_clock().now().to_msg()),
        )

    def destroy_node(self):
        if hasattr(self, "runtime"):
            self.runtime.adapter.shutdown()
        if hasattr(self, "path_client"):
            self.path_client.destroy()
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
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
