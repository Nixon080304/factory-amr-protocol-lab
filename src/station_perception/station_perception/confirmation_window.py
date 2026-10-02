# SPDX-License-Identifier: Apache-2.0
"""Mission-owned, simulation-time station confirmation."""

from collections import deque
import math


class ConfirmationWindow:
    """Confirm five distinct expected-station image stamps within 1.5 seconds.

    The coordinator creates/resets this window on entry to station verification.
    observe() reports confirmation only for the current observation; it does not
    latch permission for a later PLC transfer. The caller must check freshness
    against its simulation clock before transfer. Wrong stations, duplicate or
    backward stamps, and invalid stamps discard the sequence. reset() also clears
    the timestamp watermark, allowing a new mission or simulation-clock epoch.
    """

    def __init__(self, expected_station_id: str):
        self.expected_station_id = expected_station_id
        self.reset()

    def reset(self) -> None:
        self._stamps = deque(maxlen=5)
        self._last_stamp = None

    def observe(self, station_id: str, stamp_seconds: float) -> bool:
        if not math.isfinite(stamp_seconds) or stamp_seconds < 0:
            self._stamps.clear()
            return False
        if self._last_stamp is not None and stamp_seconds <= self._last_stamp:
            self._stamps.clear()
            return False
        self._last_stamp = stamp_seconds
        if station_id != self.expected_station_id:
            self._stamps.clear()
            return False
        while self._stamps and stamp_seconds - self._stamps[0] > 1.5:
            self._stamps.popleft()
        self._stamps.append(stamp_seconds)
        return len(self._stamps) == 5
