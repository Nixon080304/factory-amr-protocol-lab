# SPDX-License-Identifier: Apache-2.0
"""Immutable fleet values shared by pure policies and transport adapters."""

from dataclasses import dataclass
from enum import Enum

from fleet_manager.config import Pose2D


class RobotMode(str, Enum):
    AVAILABLE = "AVAILABLE"
    RESERVED = "RESERVED"
    EXECUTING = "EXECUTING"
    WAITING_FOR_RESOURCE = "WAITING_FOR_RESOURCE"
    DOCKING = "DOCKING"
    CHARGING = "CHARGING"
    UNHEALTHY = "UNHEALTHY"
    OFFLINE = "OFFLINE"
    RECOVERY_REQUIRED = "RECOVERY_REQUIRED"


class RobotHealth(str, Enum):
    ONLINE = "ONLINE"
    UNHEALTHY = "UNHEALTHY"
    OFFLINE = "OFFLINE"


@dataclass(frozen=True)
class RobotSnapshot:
    robot_id: str
    mode: RobotMode = RobotMode.OFFLINE
    pose: Pose2D | None = None
    battery_percent: float | None = None
    payload_state: str | None = "UNKNOWN"
    mission_id: str | None = None
    health: RobotHealth = RobotHealth.ONLINE
    fault: str = ""
    health_detail: str = ""
    stamp: float | None = None


@dataclass(frozen=True)
class CostEstimate:
    feasible: bool
    path_cost: float
    predicted_final_battery: float
    reason: str = ""


@dataclass(frozen=True)
class MissionRequest:
    mission_id: str
    pickup_station: str
    dropoff_station: str
    part: str
    requested_robot_id: str | None = None


@dataclass(frozen=True)
class AssignmentDecision:
    robot_id: str | None
    reason: str


def robot_is_ready(state: RobotSnapshot) -> bool:
    """Check readiness shared by registry filtering and assignment defense."""
    return (
        state.health == RobotHealth.ONLINE
        and state.mode == RobotMode.AVAILABLE
        and state.pose is not None
        and bool(state.payload_state)
        and state.payload_state != "UNKNOWN"
        and not state.fault
    )
