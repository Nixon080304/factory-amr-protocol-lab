# SPDX-License-Identifier: Apache-2.0
"""Pure one-part lifecycle driven only by validated confirmed PLC completions."""

from dataclasses import dataclass
from enum import Enum
import json
import re


class PayloadState(str, Enum):
    AT_ASSEMBLY = "AT_ASSEMBLY"
    IN_TRANSIT = "IN_TRANSIT"
    AT_INSPECTION = "AT_INSPECTION"


@dataclass(frozen=True)
class PayloadTransition:
    previous_state: PayloadState
    state: PayloadState
    applied: bool
    reason: str
    mission_id: str = ""
    transfer_kind: str = ""
    cycle_counter: int | None = None


class PayloadStateMachine:
    """Accept exactly one load/unload pair; no implicit per-mission resets."""

    def __init__(self):
        self.state = PayloadState.AT_ASSEMBLY
        self.mission_id = ""
        self._completed = set()

    def _rejected(self, reason):
        return PayloadTransition(self.state, self.state, False, reason, self.mission_id)

    def apply_transfer(
        self, mission_id, transfer_kind, cycle_counter
    ) -> PayloadTransition:
        if (
            not isinstance(mission_id, str)
            or re.fullmatch(r"[A-Za-z0-9_-]{1,64}", mission_id) is None
            or transfer_kind not in ("LOADING", "UNLOADING")
            or type(cycle_counter) is not int
            or not 0 <= cycle_counter <= 65535
        ):
            return self._rejected("invalid_transfer")
        key = (mission_id, transfer_kind, cycle_counter)
        if key in self._completed:
            return self._rejected("duplicate")
        if self.mission_id and mission_id != self.mission_id:
            return self._rejected("different_mission")
        if transfer_kind == "LOADING" and self.state == PayloadState.AT_ASSEMBLY:
            target = PayloadState.IN_TRANSIT
        elif transfer_kind == "UNLOADING" and self.state == PayloadState.IN_TRANSIT:
            target = PayloadState.AT_INSPECTION
        else:
            return self._rejected("invalid_sequence")
        previous = self.state
        self.state = target
        self.mission_id = mission_id
        self._completed.add(key)
        return PayloadTransition(
            previous, target, True, "applied", mission_id, transfer_kind, cycle_counter
        )

    def apply_protocol_event(
        self, mission_id, protocol, event, outcome, detail
    ) -> PayloadTransition:
        if protocol != "MODBUS" or outcome not in ("SUCCEEDED", "FAILED"):
            return self._rejected("not_successful_modbus")
        expected = {
            "modbus_pickup_finished": ("assembly", "LOADING"),
            "modbus_dropoff_finished": ("inspection", "UNLOADING"),
        }.get(event)
        if expected is None:
            return self._rejected("not_transfer_finish")
        try:
            fields = json.loads(detail)
        except (ValueError, TypeError, RecursionError):
            return self._rejected("invalid_detail")
        if (
            not isinstance(fields, dict)
            or (fields.get("station_id"), fields.get("transfer_kind")) != expected
        ):
            return self._rejected("invalid_detail")
        if outcome == "FAILED" and fields.get("transfer_outcome") != "COMPLETED":
            return self._rejected("not_confirmed_modbus")
        return self.apply_transfer(
            mission_id, fields["transfer_kind"], fields.get("cycle_counter")
        )
