import json
from datetime import datetime, timezone

import pytest

from protocol_observer.models import ProtocolEventRecord
from protocol_observer.trace_writer import TraceWriter


def test_append_preserves_records_and_assigns_sequence_across_reopen(tmp_path):
    path = tmp_path / "trace.jsonl"
    event = ProtocolEventRecord(
        stamp=datetime(2026, 10, 2, tzinfo=timezone.utc),
        mission_id="M-001", protocol="MQTT", direction="inbound",
        event="mqtt_acceptance_started", outcome="", latency_ms=999.0,
        detail="Unicode payload: pièce\nsecond line", sequence=800,
    )
    TraceWriter(path).append(event)
    first_line = path.read_text()
    TraceWriter(path).append(event)
    assert path.read_text().startswith(first_line)
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    assert [row["sequence"] for row in rows] == [1, 2]
    assert rows[0]["stamp"] == "2026-10-02T00:00:00+00:00"
    restored = ProtocolEventRecord.from_dict(rows[0])
    assert restored.detail == "Unicode payload: pièce\nsecond line"
    assert restored.mission_id == "M-001"
    assert restored.latency_ms == 999.0
    assert restored.stamp == event.stamp
    assert restored.sequence == 1
    assert event.sequence == 800


def test_naive_timestamp_is_rejected_before_writing(tmp_path):
    with pytest.raises(ValueError, match="timezone"):
        event = ProtocolEventRecord(datetime(2026, 10, 2), "M", "DDS", "internal", "retry", "", 0, "")
        TraceWriter(tmp_path / "trace.jsonl").append(event)
    assert not (tmp_path / "trace.jsonl").exists()


def test_unterminated_existing_record_is_rejected_without_modification(tmp_path):
    path = tmp_path / "trace.jsonl"
    path.write_text('{"stamp":"2026-10-02T00:00:00+00:00","mission_id":"M",'
                    '"protocol":"DDS","direction":"internal","event":"retry",'
                    '"outcome":"","latency_ms":0,"detail":"","sequence":1}')
    original = path.read_bytes()
    with pytest.raises(ValueError, match="newline"):
        TraceWriter(path)
    assert path.read_bytes() == original


@pytest.mark.parametrize("sequence", [-2, 1.5, True, "1"])
def test_invalid_sequence_is_rejected_by_constructor_and_from_dict(sequence):
    fields = {
        "stamp": datetime(2026, 10, 2, tzinfo=timezone.utc),
        "mission_id": "M", "protocol": "DDS", "direction": "internal",
        "event": "retry", "outcome": "", "latency_ms": 0,
        "detail": "", "sequence": sequence,
    }
    with pytest.raises(ValueError, match="sequence"):
        ProtocolEventRecord(**fields)

    fields["stamp"] = fields["stamp"].isoformat()
    with pytest.raises(ValueError, match="sequence"):
        ProtocolEventRecord.from_dict(fields)


@pytest.mark.parametrize("sequence", [None, 0, 1, 42])
def test_valid_sequence_values(sequence):
    event = ProtocolEventRecord(
        datetime(2026, 10, 2, tzinfo=timezone.utc), "M", "DDS", "internal",
        "retry", "", 0, "", sequence,
    )
    assert event.sequence == sequence


def test_reopening_invalid_sequence_rejects_trace_without_modification(tmp_path):
    path = tmp_path / "trace.jsonl"
    path.write_text(
        '{"stamp":"2026-10-02T00:00:00+00:00","mission_id":"M",'
        '"protocol":"DDS","direction":"internal","event":"retry",'
        '"outcome":"","latency_ms":0,"detail":"","sequence":1.5}\n'
    )
    original = path.read_bytes()
    with pytest.raises(ValueError, match="sequence"):
        TraceWriter(path)
    assert path.read_bytes() == original
