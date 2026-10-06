"""Best-effort observer side effects; DDS delivery remains reliable."""

from dataclasses import replace
from datetime import datetime, timezone
import hashlib
import math
from pathlib import Path
import re

import rclpy
from rclpy.impl.implementation_singleton import rclpy_implementation
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from factory_interfaces.msg import ProtocolEvent
from .models import ProtocolEventRecord
from .trace_writer import TraceWriter
from .report import MissionReport


class ProtocolObserverNode(Node):
    def __init__(self, **kwargs):
        super().__init__("protocol_observer", **kwargs)
        self.set_parameters([Parameter("use_sim_time", value=True)])
        self.output = Path(
            self.declare_parameter("output_dir", "artifacts/traces").value
        )
        self.failure_count = 0
        self.writer = None
        self.records = {}
        self.terminal = set()
        try:
            self.output.mkdir(parents=True, exist_ok=True)
            self.writer = TraceWriter(self.output / "protocol_events.jsonl")
        except Exception as error:
            self._failure("", error)
        self.protocol_group = MutuallyExclusiveCallbackGroup()
        self.subscription = self.create_subscription(
            ProtocolEvent,
            "/factory/protocol_events",
            self._observe,
            QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE),
            callback_group=self.protocol_group,
        )

    def _failure(self, mission_id, error):
        self.failure_count += 1
        self.get_logger().error(f"Mission {mission_id}: observer failure: {error}")

    def _observe(self, message):
        try:
            if message.stamp.sec < 0 or message.stamp.nanosec >= 1_000_000_000:
                raise ValueError(
                    "event stamp must be a nonnegative normalized ROS time"
                )
            if not math.isfinite(message.latency_ms) or message.latency_ms < 0:
                raise ValueError("event latency_ms must be finite and nonnegative")
            record = ProtocolEventRecord(
                stamp=datetime.fromtimestamp(
                    message.stamp.sec + message.stamp.nanosec / 1e9, timezone.utc
                ),
                mission_id=message.mission_id,
                robot_id=message.robot_id,
                protocol=message.protocol,
                direction=message.direction,
                event=message.event,
                outcome=message.outcome,
                latency_ms=message.latency_ms,
                detail=message.detail,
            )
            if self.writer is None:
                raise OSError("trace writer unavailable")
            self.writer.append(record)
            record = replace(record, sequence=self.writer._sequence)
            if not re.fullmatch(r"[A-Za-z0-9_-]{1,64}", message.mission_id):
                return
            records = self.records.setdefault(message.mission_id, [])
            records.append(record)
            if message.event == "mission_finished":
                self.terminal.add(message.mission_id)
            # Other publishers may deliver a prior phase finish after the result.
            if message.mission_id in self.terminal:
                report = MissionReport.from_events(message.mission_id, records)
                digest = hashlib.sha256(message.mission_id.encode("utf-8")).hexdigest()
                (self.output / f"mission_{digest}.md").write_text(
                    report.to_markdown(), encoding="utf-8"
                )
        except Exception as error:
            self._failure(message.mission_id, error)


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = SingleThreadedExecutor()
    try:
        node = ProtocolObserverNode()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except rclpy_implementation.RCLError as error:
        # Humble can race signal shutdown while creating its next wait set.
        if rclpy.ok() or not any(
            message in str(error)
            for message in ("context is invalid", "context is not valid")
        ):
            raise
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
