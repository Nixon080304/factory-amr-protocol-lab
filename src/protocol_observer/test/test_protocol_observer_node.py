"""Observer actual DDS event conversion and isolated failures."""

import json
import time
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from factory_interfaces.msg import ProtocolEvent
from protocol_observer.node import ProtocolObserverNode


def test_transport_boundary_preserves_both_robot_ids(tmp_path, monkeypatch):
    from types import SimpleNamespace
    from rclpy.expand_topic_name import expand_topic_name
    from rclpy.node import Node

    endpoints = []
    namespace = "/amr_01"
    output = tmp_path / "amr_01"
    monkeypatch.setattr(Node, "__init__", lambda self, *args, **kwargs: None)
    monkeypatch.setattr(Node, "set_parameters", lambda self, values: [])
    monkeypatch.setattr(
        Node,
        "declare_parameter",
        lambda self, name, value: SimpleNamespace(value=str(output)),
    )

    def subscription(self, kind, topic, callback, qos, **kwargs):
        endpoints.append(expand_topic_name(topic, "protocol_observer", namespace))
        return SimpleNamespace()

    monkeypatch.setattr(Node, "create_subscription", subscription)
    for robot_id in ("amr_01", "amr_02"):
        namespace = f"/{robot_id}"
        output = tmp_path / robot_id
        node = ProtocolObserverNode(namespace=namespace)
        for assigned_robot in ("amr_01", "amr_02"):
            message = ProtocolEvent(
                mission_id=f"mission_{assigned_robot}",
                robot_id=assigned_robot,
                protocol="ROS",
                event="mission_finished",
                outcome="COMPLETED",
            )
            message.stamp.sec = 42
            node._observe(message)
        rows = [
            json.loads(line)
            for line in (output / "protocol_events.jsonl").read_text().splitlines()
        ]
        assert [(row["mission_id"], row.get("robot_id")) for row in rows] == [
            ("mission_amr_01", "amr_01"),
            ("mission_amr_02", "amr_02"),
        ]
        reports = [path.read_text() for path in output.glob("mission_*.md")]
        assert any("Robot IDs: amr_01" in report for report in reports)
        assert any("Robot IDs: amr_02" in report for report in reports)
    # Protocol events are deliberately fleet-wide, even under a ROS namespace.
    assert endpoints == ["/factory/protocol_events", "/factory/protocol_events"]


def test_observer_keeps_robot_local_history_and_one_fleet_report(tmp_path, monkeypatch):
    from types import SimpleNamespace
    from rclpy.node import Node

    monkeypatch.setattr(Node, "__init__", lambda *args, **kwargs: None)
    monkeypatch.setattr(Node, "set_parameters", lambda *args: [])
    monkeypatch.setattr(
        Node, "declare_parameter", lambda *args: SimpleNamespace(value=str(tmp_path))
    )
    monkeypatch.setattr(Node, "create_subscription", lambda *args, **kwargs: None)
    node = ProtocolObserverNode()
    for robot, name, seconds in [
        ("amr_01", "navigation_pickup_started", 1),
        ("amr_02", "navigation_pickup_started", 2),
        ("amr_02", "navigation_pickup_finished", 6),
        ("amr_02", "mission_finished", 7),
        ("amr_01", "navigation_pickup_finished", 4),
    ]:
        message = ProtocolEvent(
            mission_id="M-1",
            robot_id=robot,
            protocol="ROS",
            event=name,
            outcome="COMPLETED" if name == "mission_finished" else "",
        )
        message.stamp.sec = seconds
        node._observe(message)
    assert set(node.robot_records) == {("M-1", "amr_01"), ("M-1", "amr_02")}
    assert len(node.records["M-1"]) == 5
    reports = list(tmp_path.glob("mission_*.json"))
    assert len(reports) == 1
    report = json.loads(reports[0].read_text())
    assert report["durations_ms"]["navigation_pickup"] == 7000.0
    assert report["robot_durations_ms"]["amr_01"]["navigation_pickup"] == 3000.0
    assert report["robot_durations_ms"]["amr_02"]["navigation_pickup"] == 4000.0


@pytest.mark.parametrize("namespace", ["/amr_01", "/amr_02"])
def test_dds_trace_report_late_phase_and_write_failure(tmp_path, namespace):
    context = Context()
    rclpy.init(context=context, domain_id=79)
    node = ProtocolObserverNode(
        context=context,
        namespace=namespace,
        parameter_overrides=[Parameter("output_dir", value=str(tmp_path))],
    )
    peer = rclpy.create_node("observer_peer", context=context)
    publisher = peer.create_publisher(ProtocolEvent, "/factory/protocol_events", 100)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    executor.add_node(peer)

    def publish(name, seconds, outcome="", nanoseconds=0, protocol="ROS", detail=""):
        message = ProtocolEvent(
            mission_id="M-001",
            robot_id=namespace.removeprefix("/"),
            protocol=protocol,
            event=name,
            outcome=outcome,
            detail=detail,
        )
        message.stamp.sec = seconds
        message.stamp.nanosec = nanoseconds
        publisher.publish(message)
        for _ in range(20):
            executor.spin_once(timeout_sec=0.01)

    try:
        deadline = time.monotonic() + 2
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
        publish("mission_started", 10)
        publish("modbus_pickup_started", 11)
        publish("mission_finished", 13, "COMPLETED")
        publish("modbus_pickup_finished", 12, "SUCCEEDED")
        records = [
            json.loads(line)
            for line in (tmp_path / "protocol_events.jsonl").read_text().splitlines()
        ]
        assert [r["sequence"] for r in records] == [1, 2, 3, 4]
        assert [r["robot_id"] for r in records] == [namespace.removeprefix("/")] * 4
        assert records[-1]["stamp"].startswith("1970-01-01T00:00:12")
        report = (
            tmp_path
            / "mission_02acf5bdbbaa226dc29a07a86e64c1b2d630385779533ed70d193a8c258149dc.md"
        )
        assert "1000.000 ms" in report.read_text()
        assert "Mission ID: M-001" in report.read_text()
        assert not (tmp_path / "M-001.md").exists()
        publish("invalid_stamp", 14, nanoseconds=1_000_000_000)
        assert node.failure_count == 1
        assert len((tmp_path / "protocol_events.jsonl").read_text().splitlines()) == 4
        node.writer.path = tmp_path
        publish("retry", 14)
        assert node.failure_count == 2
        node.writer.path = tmp_path / "protocol_events.jsonl"
        publish("mission_finished", 15, "FAILED")
        physical = {
            "station_id": "assembly",
            "transfer_kind": "LOADING",
            "cycle_counter": 1,
            "transfer_outcome": "COMPLETED",
            "error_code": "PLC_TIMEOUT_TRANSFER_COMPLETED",
            "message": "robot coil cleanup acknowledgment lost",
        }
        publish(
            "modbus_pickup_finished",
            14,
            "FAILED",
            protocol="MODBUS",
            detail=json.dumps(physical),
        )
        latest = json.loads(
            (tmp_path / "protocol_events.jsonl").read_text().splitlines()[-1]
        )
        assert latest["outcome"] == "FAILED"
        assert json.loads(latest["detail"]) == physical
        rendered = report.read_text()
        assert "| Final outcome | FAILED |" in rendered
        assert (
            "| assembly | LOADING | 1 | COMPLETED | FAILED | PLC_TIMEOUT_TRANSFER_COMPLETED |"
            in rendered
        )
        assert "robot coil cleanup acknowledgment lost" in rendered
    finally:
        executor.shutdown()
        node.destroy_node()
        peer.destroy_node()
        context.shutdown()
