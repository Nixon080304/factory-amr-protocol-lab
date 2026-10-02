# SPDX-License-Identifier: Apache-2.0
"""The ProtocolEvent fields plus an observer-local sequence number."""

from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any, Mapping, Optional


@dataclass(frozen=True)
class ProtocolEventRecord:
    stamp: datetime
    mission_id: str
    protocol: str
    direction: str
    event: str
    outcome: str
    latency_ms: float
    detail: str
    sequence: Optional[int] = None

    def __post_init__(self) -> None:
        if self.stamp.tzinfo is None or self.stamp.utcoffset() is None:
            raise ValueError("stamp must have a timezone")
        if self.sequence is not None and (
            isinstance(self.sequence, bool)
            or not isinstance(self.sequence, int)
            or self.sequence < 0
        ):
            raise ValueError("sequence must be a non-negative integer or None")
        object.__setattr__(self, "stamp", self.stamp.astimezone(timezone.utc))

    def to_dict(self) -> dict[str, Any]:
        return {
            "stamp": self.stamp.isoformat(),
            "mission_id": self.mission_id,
            "protocol": self.protocol,
            "direction": self.direction,
            "event": self.event,
            "outcome": self.outcome,
            "latency_ms": self.latency_ms,
            "detail": self.detail,
            "sequence": self.sequence,
        }

    @classmethod
    def from_dict(cls, value: Mapping[str, Any]) -> "ProtocolEventRecord":
        fields = dict(value)
        fields["stamp"] = datetime.fromisoformat(fields["stamp"])
        return cls(**fields)
