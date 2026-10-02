"""Actual DDS event conversion and isolated observer failures."""
import json
import time
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from factory_interfaces.msg import ProtocolEvent
from protocol_observer.node import ProtocolObserverNode


def test_dds_trace_report_late_phase_and_write_failure(tmp_path):
    context = Context()
    rclpy.init(context=context, domain_id=79)
    node = ProtocolObserverNode(context=context, parameter_overrides=[Parameter('output_dir', value=str(tmp_path))])
    peer = rclpy.create_node('observer_peer', context=context)
    publisher = peer.create_publisher(ProtocolEvent, '/factory/protocol_events', 100)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    executor.add_node(peer)
    def publish(name, seconds, outcome='', nanoseconds=0):
        message = ProtocolEvent(mission_id='M-001', protocol='ROS', event=name, outcome=outcome)
        message.stamp.sec = seconds
        message.stamp.nanosec = nanoseconds
        publisher.publish(message)
        for _ in range(20):
            executor.spin_once(timeout_sec=0.01)
    try:
        deadline = time.monotonic() + 2
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
        publish('mission_started', 10)
        publish('modbus_pickup_started', 11)
        publish('mission_finished', 13, 'COMPLETED')
        publish('modbus_pickup_finished', 12, 'SUCCEEDED')
        records = [json.loads(line) for line in (tmp_path / 'protocol_events.jsonl').read_text().splitlines()]
        assert [r['sequence'] for r in records] == [1, 2, 3, 4]
        assert records[-1]['stamp'].startswith('1970-01-01T00:00:12')
        report = tmp_path / 'mission_02acf5bdbbaa226dc29a07a86e64c1b2d630385779533ed70d193a8c258149dc.md'
        assert '1000.000 ms' in report.read_text()
        assert 'Mission ID: M-001' in report.read_text()
        assert not (tmp_path / 'M-001.md').exists()
        publish('invalid_stamp', 14, nanoseconds=1_000_000_000)
        assert node.failure_count == 1
        assert len((tmp_path / 'protocol_events.jsonl').read_text().splitlines()) == 4
        node.writer.path = tmp_path
        publish('retry', 14)
        assert node.failure_count == 2
    finally:
        executor.shutdown()
        node.destroy_node()
        peer.destroy_node()
        context.shutdown()
