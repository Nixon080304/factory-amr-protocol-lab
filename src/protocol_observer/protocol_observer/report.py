# SPDX-License-Identifier: Apache-2.0
"""Deterministic mission summaries from explicit event pairs.

See README.md for the event contract consumed by the future ROS adapters.
No wall-clock calls or latency_ms estimates are used for durations.
"""

from dataclasses import dataclass
from typing import Optional, Sequence

from .models import ProtocolEventRecord


PHASE_LABELS = {
    "mission": "End-to-end mission duration",
    "mqtt_acceptance": "MQTT acceptance latency",
    "navigation_pickup": "Navigation duration: pickup",
    "navigation_dropoff": "Navigation duration: dropoff",
    "perception_pickup": "Perception confirmation: pickup",
    "perception_dropoff": "Perception confirmation: dropoff",
    "modbus_pickup": "Modbus handshake: pickup",
    "modbus_dropoff": "Modbus handshake: dropoff",
}


def _duration_ms(phase: str, events: Sequence[ProtocolEventRecord]) -> Optional[float]:
    start = None
    total = 0.0
    pairs = 0
    for event in events:
        if event.event == f"{phase}_started":
            if start is not None:
                return None
            start = event.stamp
        elif event.event == f"{phase}_finished":
            if start is None:
                return None
            elapsed = (event.stamp - start).total_seconds() * 1000
            if elapsed < 0:
                return None
            total += elapsed
            pairs += 1
            start = None
    return total if pairs and start is None else None


def _markdown_text(value: str) -> str:
    """Keep externally supplied values in one Markdown table cell."""
    return (value.replace("&", "&amp;").replace("<", "&lt;")
            .replace(">", "&gt;").replace("|", "&#124;")
            .replace("\r", " ").replace("\n", " "))


@dataclass(frozen=True)
class MissionReport:
    mission_id: str
    events: tuple[ProtocolEventRecord, ...]
    durations_ms: dict[str, Optional[float]]
    retry_count: int
    failure_count: int
    final_outcome: str

    @classmethod
    def from_events(cls, mission_id: str, events: Sequence[ProtocolEventRecord]) -> "MissionReport":
        if not mission_id or not mission_id.strip():
            raise ValueError("mission_id must not be blank")
        ordered = tuple(sorted(
            (event for event in events if event.mission_id == mission_id),
            key=lambda event: (event.stamp, event.sequence if event.sequence is not None else 0),
        ))
        terminal = [event for event in ordered if event.event == "mission_finished"]
        return cls(
            mission_id=mission_id,
            events=ordered,
            durations_ms={phase: _duration_ms(phase, ordered) for phase in PHASE_LABELS},
            retry_count=sum(event.event == "retry" for event in ordered),
            failure_count=sum(event.outcome == "FAILED" for event in ordered),
            final_outcome=terminal[-1].outcome if terminal and terminal[-1].outcome else "not available",
        )

    def to_markdown(self) -> str:
        rows = [
            "# Mission report", "", f"Mission ID: {_markdown_text(self.mission_id)}", "",
            "| Metric | Value |", "| --- | --- |",
        ]
        for phase, label in PHASE_LABELS.items():
            duration = self.durations_ms[phase]
            value = "not available" if duration is None else f"{duration:.3f} ms"
            rows.append(f"| {label} | {value} |")
        rows.extend([
            f"| Retry events | {self.retry_count} |",
            f"| Failure events | {self.failure_count} |",
            f"| Final outcome | {_markdown_text(self.final_outcome)} |",
        ])
        return "\n".join(rows) + "\n"
