# SPDX-License-Identifier: Apache-2.0
"""Deterministic mission summaries from explicit event pairs.

See README.md for the event contract consumed by the future ROS adapters.
No wall-clock calls or latency_ms estimates are used for durations.
"""

from dataclasses import dataclass
import json
from pathlib import Path
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


def compare_scenario(
    scenario: str,
    expected: dict,
    output: Path,
    *,
    command_exit_code: int,
    timed_out: bool = False,
    interrupted: int = 0,
) -> dict:
    """Compare independently recorded behavior and correlated trace predicates."""
    failures, assertions = [], []
    actual, records = {}, []
    try:
        actual = json.loads((output / "actual.json").read_text())
        records = [
            json.loads(line)
            for line in (output / "protocol_events.jsonl").read_text().splitlines()
        ]
    except (OSError, ValueError) as error:
        failures.append(f"Evidence unavailable: {error}")
    for key in ("final_state", "error_code", "source", "action_executions"):
        if key in expected and (key not in actual or actual[key] != expected[key]):
            failures.append(
                f"{key}: expected {expected[key]!r}, actual {actual.get(key)!r}"
            )
    mission_id = actual.get("mission_id")
    if not mission_id:
        failures.append("Evidence must identify the mission or experiment")
    correlated = (
        [record for record in records if record.get("mission_id") == mission_id]
        if mission_id
        else []
    )
    for predicate in expected["trace"]:
        matches = [
            record
            for record in correlated
            if all(
                record.get(key) == predicate[key]
                for key in ("event", "protocol", "outcome")
                if key in predicate
            )
            and predicate.get("detail_contains", "") in record.get("detail", "")
        ]
        passed = len(matches) >= predicate.get("min", 1) and len(
            matches
        ) <= predicate.get("max", float("inf"))
        assertions.append(
            {"predicate": predicate, "count": len(matches), "passed": passed}
        )
        if not passed:
            failures.append(f"Trace predicate {predicate}: count={len(matches)}")
    if command_exit_code:
        failures.append(f"Scenario command exited {command_exit_code}")
    if timed_out:
        failures.append("Scenario deadline exceeded")
    if interrupted:
        failures.append(f"Scenario interrupted by signal {interrupted}")
    return dict(
        scenario=scenario,
        expected=expected,
        actual=actual,
        trace_assertions=assertions,
        command_exit_code=command_exit_code,
        timed_out=timed_out,
        interrupted=interrupted,
        matched=not failures,
        failures=failures,
    )


def scenario_matrix_markdown(outcomes: Sequence[dict]) -> str:
    """Render measured matrix rows without inventing missing actual outcomes."""
    rows = [
        "# Scenario report",
        "",
        "| Scenario | Source | Expected | Actual | Trace checks | Match |",
        "| --- | --- | --- | --- | --- | --- |",
    ]
    for result in outcomes:
        actual = result["actual"]
        checks = result["trace_assertions"]
        expected_state = result["expected"]["final_state"]
        actual_state = actual.get("final_state", "not available")
        if result["expected"].get("error_code"):
            expected_state += "/" + result["expected"]["error_code"]
        if actual.get("error_code"):
            actual_state += "/" + actual["error_code"]
        cells = [
            result["scenario"],
            actual.get("source", "not available"),
            expected_state,
            actual_state,
            f"{sum(check['passed'] for check in checks)}/{len(checks)}",
            str(result["matched"]),
        ]
        rows.append(
            "| " + " | ".join(_markdown_text(str(value)) for value in cells) + " |"
        )
    rows.extend(
        [
            "",
            f"Unexpected outcomes: {sum(not result['matched'] for result in outcomes)}",
            "",
        ]
    )
    return "\n".join(rows)


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
    return (
        value.replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace("|", "&#124;")
        .replace("\r", " ")
        .replace("\n", " ")
    )


@dataclass(frozen=True)
class MissionReport:
    mission_id: str
    events: tuple[ProtocolEventRecord, ...]
    durations_ms: dict[str, Optional[float]]
    retry_count: int
    failure_count: int
    final_outcome: str

    @classmethod
    def from_events(
        cls, mission_id: str, events: Sequence[ProtocolEventRecord]
    ) -> "MissionReport":
        if not mission_id or not mission_id.strip():
            raise ValueError("mission_id must not be blank")
        ordered = tuple(
            sorted(
                (event for event in events if event.mission_id == mission_id),
                key=lambda event: (
                    event.stamp,
                    event.sequence if event.sequence is not None else 0,
                ),
            )
        )
        terminal = [event for event in ordered if event.event == "mission_finished"]
        return cls(
            mission_id=mission_id,
            events=ordered,
            durations_ms={
                phase: _duration_ms(phase, ordered) for phase in PHASE_LABELS
            },
            retry_count=sum(event.event == "retry" for event in ordered),
            failure_count=sum(event.outcome == "FAILED" for event in ordered),
            final_outcome=terminal[-1].outcome
            if terminal and terminal[-1].outcome
            else "not available",
        )

    def to_markdown(self) -> str:
        rows = [
            "# Mission report",
            "",
            f"Mission ID: {_markdown_text(self.mission_id)}",
            "",
            "| Metric | Value |",
            "| --- | --- |",
        ]
        for phase, label in PHASE_LABELS.items():
            duration = self.durations_ms[phase]
            value = "not available" if duration is None else f"{duration:.3f} ms"
            rows.append(f"| {label} | {value} |")
        rows.extend(
            [
                f"| Retry events | {self.retry_count} |",
                f"| Failure events | {self.failure_count} |",
                f"| Final outcome | {_markdown_text(self.final_outcome)} |",
            ]
        )
        return "\n".join(rows) + "\n"
