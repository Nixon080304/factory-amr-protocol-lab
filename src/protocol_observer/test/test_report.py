from datetime import datetime, timedelta, timezone
from dataclasses import replace

import pytest

from protocol_observer.models import ProtocolEventRecord
from protocol_observer.report import MissionReport


def event(seconds, name, outcome="", mission_id="M-001", sequence=0, robot_id="amr_01"):
    return ProtocolEventRecord(
        datetime(2026, 10, 2, tzinfo=timezone.utc) + timedelta(seconds=seconds),
        mission_id,
        "DDS",
        "internal",
        name,
        outcome,
        999999.0,
        "",
        sequence,
        robot_id=robot_id,
    )


def test_successful_report_uses_timestamps_and_filters_other_missions():
    events = [
        event(0, "mission_started"),
        event(0, "mqtt_acceptance_started"),
        event(0.25, "mqtt_acceptance_finished"),
        event(1, "navigation_pickup_started"),
        event(5, "navigation_pickup_finished"),
        event(5, "perception_pickup_started"),
        event(6.5, "perception_pickup_finished"),
        event(6.5, "modbus_pickup_started"),
        event(7, "modbus_pickup_finished"),
        event(7, "navigation_dropoff_started"),
        event(10, "navigation_dropoff_finished"),
        event(10, "perception_dropoff_started"),
        event(11, "perception_dropoff_finished"),
        event(11, "modbus_dropoff_started"),
        event(12, "modbus_dropoff_finished"),
        event(12, "mission_finished", "COMPLETED"),
        event(13, "mission_finished", "FAILED", mission_id="other"),
        event(8, "retry", mission_id=""),
    ]
    report = MissionReport.from_events("M-001", list(reversed(events)))
    assert report.durations_ms == {
        "mission": 12000.0,
        "mqtt_acceptance": 250.0,
        "navigation_pickup": 4000.0,
        "navigation_dropoff": 3000.0,
        "perception_pickup": 1500.0,
        "perception_dropoff": 1000.0,
        "modbus_pickup": 500.0,
        "modbus_dropoff": 1000.0,
    }
    assert report.final_outcome == "COMPLETED"
    assert report.retry_count == 0
    assert report.failure_count == 0
    assert (
        report.to_markdown() == MissionReport.from_events("M-001", events).to_markdown()
    )
    assert "12000.000 ms" in report.to_markdown()


def test_timestamp_then_sequence_order_controls_ties():
    report = MissionReport.from_events(
        "M-001",
        [
            event(2, "mission_finished", "COMPLETED", sequence=3),
            event(2, "mission_finished", "FAILED", sequence=2),
            event(2, "mission_started", sequence=1),
            event(1, "retry", sequence=9),
        ],
    )
    assert [item.sequence for item in report.events] == [9, 1, 2, 3]
    assert report.final_outcome == "COMPLETED"


def test_phase_finish_from_other_robot_cannot_complete_first_robot_phase():
    report = MissionReport.from_events(
        "M-001",
        [
            replace(event(1, "navigation_pickup_started"), robot_id="amr_01"),
            replace(event(3, "navigation_pickup_finished"), robot_id="amr_02"),
        ],
    )
    assert report.durations_ms["navigation_pickup"] is None


def test_interleaved_robot_phases_keep_independent_start_stamps():
    report = MissionReport.from_events(
        "M-001",
        [
            replace(event(1, "navigation_pickup_started"), robot_id="amr_01"),
            replace(event(2, "navigation_pickup_started"), robot_id="amr_02"),
            replace(event(4, "navigation_pickup_finished"), robot_id="amr_01"),
            replace(event(6, "navigation_pickup_finished"), robot_id="amr_02"),
        ],
    )
    assert report.durations_ms["navigation_pickup"] == 7000.0


def test_failed_report_counts_retries_and_retains_missing_phases():
    report = MissionReport.from_events(
        "M-001",
        [
            event(0, "mission_started"),
            event(1, "navigation_pickup_started"),
            event(3, "navigation_pickup_finished", "FAILED"),
            event(3, "retry"),
            event(4, "retry"),
            event(5, "mission_finished", "FAILED"),
        ],
    )
    assert report.final_outcome == "FAILED"
    assert report.retry_count == 2
    assert report.failure_count == 2
    assert report.durations_ms["mission"] == 5000.0
    assert report.durations_ms["navigation_pickup"] == 2000.0
    assert report.durations_ms["mqtt_acceptance"] is None
    assert "not available" in report.to_markdown()


@pytest.mark.parametrize(
    "events",
    [[], [event(0, "mission_started")], [event(2, "mission_finished", "FAILED")]],
)
def test_incomplete_or_absent_mission_has_no_invented_duration(events):
    report = MissionReport.from_events("M-001", events)
    assert report.durations_ms["mission"] is None
    assert "not available" in report.to_markdown()


def test_repeated_phase_sums_only_when_all_attempts_are_paired():
    events = [
        event(0, "modbus_pickup_started"),
        event(1, "modbus_pickup_finished"),
        event(2, "modbus_pickup_started"),
        event(4, "modbus_pickup_finished"),
    ]
    assert (
        MissionReport.from_events("M-001", events).durations_ms["modbus_pickup"]
        == 3000.0
    )
    assert (
        MissionReport.from_events("M-001", events[:-1]).durations_ms["modbus_pickup"]
        is None
    )
    assert (
        MissionReport.from_events("M-001", list(events[1:])).durations_ms[
            "modbus_pickup"
        ]
        is None
    )


def test_blank_mission_cannot_group_uncorrelated_events():
    with pytest.raises(ValueError, match="mission_id"):
        MissionReport.from_events("", [event(0, "mission_started", mission_id="")])


def test_fleet_report_serializes_robot_phase_and_transfer_evidence():
    report = MissionReport.from_events(
        "M-001",
        [
            replace(event(1, "navigation_pickup_started"), robot_id="amr_01"),
            replace(event(2, "navigation_pickup_started"), robot_id="amr_02"),
            replace(event(4, "navigation_pickup_finished"), robot_id="amr_01"),
            replace(event(6, "navigation_pickup_finished"), robot_id="amr_02"),
            replace(
                event(7, "modbus_pickup_finished", "SUCCEEDED"),
                robot_id="amr_02",
                protocol="MODBUS",
                detail='{"station_id":"assembly","transfer_kind":"LOADING","cycle_counter":5}',
            ),
        ],
    )
    serialized = report.to_dict()
    assert serialized["robot_durations_ms"]["amr_01"]["navigation_pickup"] == 3000.0
    assert serialized["robot_durations_ms"]["amr_02"]["navigation_pickup"] == 4000.0
    assert serialized["durations_ms"]["navigation_pickup"] == 7000.0
    assert serialized["events"][-1]["robot_id"] == "amr_02"
    assert "| amr_02 | assembly | LOADING | 5 |" in report.to_markdown()


def test_unidentified_legacy_robot_phase_cannot_create_fleet_duration():
    report = MissionReport.from_events(
        "M-001",
        [
            event(1, "navigation_pickup_started", robot_id=""),
            event(4, "navigation_pickup_finished", robot_id=""),
        ],
    )
    assert report.durations_ms["navigation_pickup"] is None
