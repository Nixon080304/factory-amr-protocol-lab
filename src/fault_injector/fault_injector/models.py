"""Typed requests; fault effects start only at the matching boundary."""

from dataclasses import dataclass
import math
import re

FAULT_NAMES = frozenset(
    (
        "mqtt_duplicate",
        "mqtt_conflict",
        "mqtt_disconnect",
        "modbus_delay",
        "modbus_timeout",
        "modbus_stale_completion",
        "plc_fault",
        "wrong_marker",
        "nav_reject_once",
        "nav_reject_twice",
        "qos_mismatch",
    )
)


@dataclass(frozen=True)
class FaultRequest:
    name: str
    mission_id: str
    station: str | None = None
    activation_point: str = "transfer_start"
    duration: float = 1.0
    one_shot: bool = True
    fault_code: int = 73
    robot_id: str | None = None

    def __post_init__(self):
        if self.name not in FAULT_NAMES:
            raise ValueError("unknown fault name")
        if not isinstance(self.mission_id, str) or not re.fullmatch(
            r"[A-Za-z0-9_-]{1,64}", self.mission_id
        ):
            raise ValueError("invalid mission ID")
        if self.station not in (None, "assembly", "inspection"):
            raise ValueError("invalid station")
        if self.robot_id is not None and (
            not isinstance(self.robot_id, str)
            or not re.fullmatch(r"[A-Za-z0-9_-]{1,64}", self.robot_id)
        ):
            raise ValueError("invalid robot ID")
        if self.name == "mqtt_disconnect" and (
            self.robot_id is not None or self.station is not None
        ):
            raise ValueError(
                "mqtt_disconnect affects the shared broker transport and requires global scope"
            )
        if self.name == "wrong_marker" and self.robot_id is not None:
            raise ValueError(
                "wrong_marker moves a shared station entity and cannot use robot scope"
            )
        if self.name == "wrong_marker" and self.station == "inspection":
            raise ValueError("wrong_marker supports the assembly station only")
        if not isinstance(self.activation_point, str) or not self.activation_point:
            raise ValueError("activation point must be nonempty")
        if type(self.duration) not in (float, int):
            raise ValueError("duration must be finite and positive")
        try:
            duration = float(self.duration)
        except OverflowError as error:
            raise ValueError("duration must be finite and positive") from error
        if not math.isfinite(duration) or duration <= 0:
            raise ValueError("duration must be finite and positive")
        object.__setattr__(self, "duration", duration)
        if type(self.one_shot) is not bool:
            raise ValueError("one_shot must be boolean")
        if type(self.fault_code) is not int or not 1 <= self.fault_code <= 65535:
            raise ValueError("fault_code must be a nonzero uint16")
