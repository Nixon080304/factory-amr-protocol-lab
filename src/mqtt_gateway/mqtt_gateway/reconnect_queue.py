# SPDX-License-Identifier: Apache-2.0
"""Keep bounded state events and the latest telemetry snapshot."""

from collections import deque
from copy import deepcopy


class ReconnectQueue:
    def __init__(self):
        self._states: deque[dict[str, object]] = deque(maxlen=100)
        self._telemetry: dict[str, object] | None = None

    def push_state(self, payload: dict[str, object]) -> None:
        self._states.append(deepcopy(payload))

    def set_telemetry(self, payload: dict[str, object]) -> None:
        self._telemetry = deepcopy(payload)

    def drain_states(self) -> list[dict[str, object]]:
        states = list(self._states)
        self._states.clear()
        return states

    def pop_telemetry(self) -> dict[str, object] | None:
        telemetry = self._telemetry
        self._telemetry = None
        return telemetry
