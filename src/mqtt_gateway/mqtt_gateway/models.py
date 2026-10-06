# SPDX-License-Identifier: Apache-2.0
"""Canonical mission identity shared with the gateway adapter."""

from dataclasses import dataclass


@dataclass(frozen=True)
class MissionPayload:
    mission_id: str
    robot_id: str | None
    pickup: str
    dropoff: str
    part: str
