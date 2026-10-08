# SPDX-License-Identifier: Apache-2.0
"""Pure, conservative restart decisions from current robot and lease evidence."""

from dataclasses import dataclass
import math
from typing import Mapping, Sequence

from fleet_manager.journal import MissionRecord, MissionState, PayloadOwnership
from fleet_manager.models import RobotHealth, RobotMode, RobotSnapshot
from fleet_manager.resources import Lease, ResourceSnapshot


@dataclass(frozen=True)
class RecoveryDecision:
    mission_id: str
    state: MissionState
    robot_id: str | None
    payload_ownership: PayloadOwnership
    reason: str


class RecoveryPlanner:
    """Observations passed here must already meet the startup freshness boundary.

    No clock, transport, journal mutation, or robot-specific branch belongs here.
    A missing geometry is not physical clearance. A fresh idle, empty observation
    establishes that an old robot goal is no longer executing; an offline timeout
    cannot establish that fact or the absence of a carried payload.
    """

    def __init__(self, config):
        self.config = config

    @staticmethod
    def valid_pose(robot):
        return (
            robot is not None
            and robot.health == RobotHealth.ONLINE
            and not robot.fault
            and not robot.health_detail
            and robot.pose is not None
            and all(
                math.isfinite(v) for v in (robot.pose.x, robot.pose.y, robot.pose.yaw)
            )
        )

    def proves_outside(self, lease: Lease, robot: RobotSnapshot | None) -> bool:
        if not self.valid_pose(robot) or robot.robot_id != lease.robot_id:
            return False
        x, y = robot.pose.x, robot.pose.y
        dock = self.config.docks.get(lease.resource_id)
        if dock is not None:
            margin = 2 * dock.robot_radius + dock.arrival_tolerance
            return all(
                math.hypot(x - center.x, y - center.y) > margin
                for center in (dock.staging_pose, dock.charging_pose)
            )
        bounds = self.config.traffic_bounds.get(lease.resource_id)
        if bounds is None:
            bounds = self.config.resource_bounds.get(lease.resource_id)
        if bounds is None:
            return False
        margin = max(
            (
                2 * dock.robot_radius + dock.arrival_tolerance
                for dock in self.config.docks.values()
            ),
            default=math.inf,
        )
        left, bottom, right, top = bounds
        # Rectangle distance, expanded by configured footprint and arrival bound.
        distance = math.hypot(max(left - x, 0, x - right), max(bottom - y, 0, y - top))
        return distance > margin

    def dock_state(self, robot, resource, *, contact=None):
        """Reconstruct charging only with current pose, contact, and new authority."""
        dock = self.config.docks[resource.resource_id]
        lease = resource.lease
        if (
            self.valid_pose(robot)
            and robot.mode == RobotMode.CHARGING
            and robot.payload_state == "EMPTY"
            and not robot.mission_id
            and contact is True
            and not resource.reconciliation_required
            and lease is not None
            and lease.robot_id == robot.robot_id
            and math.hypot(
                robot.pose.x - dock.charging_pose.x, robot.pose.y - dock.charging_pose.y
            )
            <= dock.arrival_tolerance
        ):
            return RobotMode.CHARGING
        return RobotMode.RECOVERY_REQUIRED

    def plan(
        self,
        active: Sequence[MissionRecord],
        robots: Mapping[str, RobotSnapshot],
        resources: Sequence[ResourceSnapshot],
    ) -> tuple[RecoveryDecision, ...]:
        decisions = []
        closed = {
            MissionState.COMPLETED,
            MissionState.FAILED,
            MissionState.CANCELLED,
            MissionState.RECOVERY_REQUIRED,
        }
        for record in sorted(
            active, key=lambda item: (item.created_at, item.request.mission_id)
        ):
            mission_id = record.request.mission_id
            state, robot_id, ownership = (
                record.state,
                record.assigned_robot_id,
                record.payload_ownership,
            )
            reason = "existing terminal state"
            if state not in closed:
                robot = robots.get(robot_id)
                claims = [
                    value for value in robots.values() if value.mission_id == mission_id
                ]
                if robot_id is None and len(claims) == 1:
                    robot = claims[0]
                    robot_id = robot.robot_id
                uncertain_payload = ownership != PayloadOwnership.NOT_PICKED_UP
                uncertain_payload |= any(
                    value.payload_state != "EMPTY" for value in claims
                )
                safe_robot = (
                    self.valid_pose(robot)
                    and robot.mode == RobotMode.AVAILABLE
                    and not robot.mission_id
                    and robot.payload_state == "EMPTY"
                )
                safe_resources = all(
                    not resource.reconciliation_required and resource.lease is None
                    for resource in resources
                    if any(
                        lease is not None
                        and (
                            lease.mission_id == mission_id or lease.robot_id == robot_id
                        )
                        for lease in (resource.lease, resource.former_lease)
                    )
                )
                schedulable = state in (
                    MissionState.RECEIVED,
                    MissionState.QUEUED,
                    MissionState.ASSIGNING,
                    MissionState.REASSIGNING,
                )
                safe = not uncertain_payload and not claims and safe_resources
                if robot_id is not None or not schedulable:
                    safe &= safe_robot
                if safe:
                    state, robot_id, reason = (
                        MissionState.QUEUED,
                        None,
                        "fresh empty robot and reconciled resources",
                    )
                else:
                    state, reason = (
                        MissionState.RECOVERY_REQUIRED,
                        "uncertain restart execution, payload, or resource evidence",
                    )
                    # Preserve stronger durable ownership and former robot identity.
                    if ownership == PayloadOwnership.NOT_PICKED_UP:
                        if robot is not None and robot.payload_state == "LOADED":
                            ownership = PayloadOwnership.PICKED_UP
                        elif not safe_robot or uncertain_payload:
                            ownership = PayloadOwnership.UNKNOWN
            decisions.append(
                RecoveryDecision(mission_id, state, robot_id, ownership, reason)
            )
        return tuple(decisions)
