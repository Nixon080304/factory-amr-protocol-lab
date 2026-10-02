# SPDX-License-Identifier: Apache-2.0
"""Track process-local mission identity and the latest state."""

from copy import deepcopy
from typing import Literal

from .models import MissionPayload


class MissionConflictError(ValueError):
    error_code = "MISSION_ID_CONFLICT"

    def __init__(self, mission_id: str):
        self.mission_id = mission_id
        super().__init__(f"Mission {mission_id} already has different values")


class MissionRegistry:
    def __init__(self):
        self._missions: dict[str, MissionPayload] = {}
        self._states: dict[str, dict[str, object]] = {}

    def register(self, mission: MissionPayload) -> Literal["new", "duplicate"]:
        existing = self._missions.get(mission.mission_id)
        if existing is not None:
            if existing != mission:
                raise MissionConflictError(mission.mission_id)
            return "duplicate"
        self._missions[mission.mission_id] = mission
        return "new"

    def update_state(self, mission_id: str, state: dict[str, object]) -> None:
        if mission_id not in self._missions:
            raise KeyError(mission_id)
        self._states[mission_id] = deepcopy(state)

    def state_for(self, mission_id: str) -> dict[str, object] | None:
        return deepcopy(self._states.get(mission_id))
