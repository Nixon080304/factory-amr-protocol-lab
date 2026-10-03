#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Real DDS lifecycle seam for the actual visualizer, with owned finite children."""

import importlib.util
from importlib.machinery import SourceFileLoader
import inspect
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import traceback

ROOT = Path(__file__).resolve().parents[2]


def identity(pid):
    fields = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
    return dict(
        pid=pid,
        parent=int(fields[1]),
        group=int(fields[2]),
        start_ticks=fields[19],
        state=fields[0],
    )


def load_visualizer():
    loader = SourceFileLoader(
        "actual_visualizer_lifecycle",
        str(ROOT / "src/factory_bringup/scripts/factory_visualization"),
    )
    spec = importlib.util.spec_from_loader(loader.name, loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


def child_probe(output, case, arm_fd):
    from factory_interfaces.msg import StationDetection
    from rclpy.executors import Executor

    module = load_visualizer()
    sys.argv = [
        "factory_visualization",
        "--ros-args",
        "-p",
        "stations_file:=" + str(ROOT / "src/factory_bringup/config/stations.yaml"),
        "-p",
        "use_sim_time:=true",
    ]
    descriptor = os.open(
        output / "events.jsonl", os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600
    )
    owner = identity(os.getpid())
    metadata = json.loads((output / "ownership.json").read_text())
    parent_owner = json.loads(os.environ["VISUALIZER_PROBE_PARENT"])
    state = dict(context=None, take=None, armed=False, signal_count=0, in_native=False)
    prior_handlers = {
        number: signal.getsignal(number) for number in (signal.SIGINT, signal.SIGTERM)
    }

    def context_ok():
        context = state["context"]
        return (
            context.handle.ok()
            if context is not None and context.handle is not None
            else None
        )

    def emit(kind, **values):
        row = dict(
            kind=kind,
            monotonic=time.monotonic(),
            pid=os.getpid(),
            thread=threading.get_ident(),
            context_ok=context_ok(),
            **values,
        )
        os.write(descriptor, (json.dumps(row) + "\n").encode())

    def verify_owner():
        current, parent = identity(owner["pid"]), identity(parent_owner["pid"])
        assert (
            current["start_ticks"] == owner["start_ticks"]
            and current["group"] == owner["group"]
        )
        assert current["parent"] == parent["pid"]
        assert (
            parent["start_ticks"] == parent_owner["start_ticks"]
            and parent["group"] == parent_owner["group"]
        )
        chain = [current]
        while (
            chain[-1]["pid"] != metadata["supervisor"]["pid"]
            and chain[-1]["parent"] > 1
        ):
            chain.append(identity(chain[-1]["parent"]))
        assert chain[-1]["pid"] == metadata["supervisor"]["pid"]
        assert chain[-1]["start_ticks"] == metadata["supervisor"]["start_ticks"]
        assert chain[-1]["group"] == metadata["supervisor"]["group"]
        return chain

    saved_node_init = module.FactoryVisualization.__init__
    saved_callback = module.FactoryVisualization.detection
    saved_constructor = StationDetection.__init__
    saved_take = Executor._take_subscription
    lines, first_line = inspect.getsourcelines(saved_take)
    statement = "msg_info = sub.handle.take_message(sub.msg_type, sub.raw)"
    positions = [
        first_line + index
        for index, line in enumerate(lines)
        if line.strip() == statement
    ]
    assert len(positions) == 1
    call_line = positions[0]
    assert lines[call_line + 1 - first_line].strip() == "if msg_info is not None:"

    def observed_init(self, *args, **kwargs):
        result = saved_node_init(self, *args, **kwargs)
        state["context"] = self.context
        emit("node_created", identity=owner, ancestry=verify_owner())
        return result

    def arm():
        if not state["armed"]:
            try:
                state["armed"] = os.read(arm_fd, 16) == b"ARM"
            except BlockingIOError:
                pass
        return state["armed"]

    def actual_signal(number=signal.SIGINT, repetitions=1):
        for _ in range(repetitions):
            ancestry = verify_owner()
            state["signal_count"] += 1
            emit(
                "actual_signal",
                signal=int(number),
                ancestry=ancestry,
                take=state["take"],
            )
            os.kill(os.getpid(), number)

    def observed_callback(self, sample):
        emit("callback_enter", station=sample.station_id, marker_id=sample.marker_id)
        if case.startswith("callback_error") and arm():
            if case.endswith("after_stop"):
                actual_signal()
            emit("unrelated_callback_error")
            raise RuntimeError("unrelated visualization callback defect")
        result = saved_callback(self, sample)
        emit(
            "callback_completed", station=sample.station_id, marker_id=sample.marker_id
        )
        return result

    def observed_take(self, subscription, *args, **kwargs):
        previous = state["take"]
        state["take"] = dict(
            topic=subscription.topic,
            raw=subscription.raw,
            target=subscription.msg_type is StationDetection,
            thread=threading.get_ident(),
        )
        try:
            return saved_take(self, subscription, *args, **kwargs)
        finally:
            state["take"] = previous

    def trace(frame, event, argument):
        if frame.f_code is saved_take.__code__:
            if event == "line" and frame.f_lineno == call_line:
                state["in_native"] = True
                emit("native_call_boundary_before", take=state["take"], line=call_line)
            elif event == "line" and frame.f_lineno == call_line + 1:
                state["in_native"] = False
                emit(
                    "native_call_boundary_return",
                    take=state["take"],
                    line=call_line + 1,
                )
            elif event == "exception" and frame.f_lineno == call_line:
                state["in_native"] = False
                emit(
                    "native_call_boundary_exception",
                    take=state["take"],
                    line=call_line,
                    exception_type=argument[0].__name__,
                    exception_text=str(argument[1]),
                )
        if event == "exception" and frame.f_code is constructor.__code__:
            emit(
                "constructor_exception",
                exception_type=argument[0].__name__,
                exception_text=str(argument[1]),
            )
        return trace

    def constructor(self, *args, **kwargs):
        take = state["take"]
        eligible = bool(
            state["in_native"]
            and take
            and take["target"]
            and take["topic"] == "/factory/station_detection"
            and take["raw"] is False
            and take["thread"] == threading.get_ident()
        )
        result = saved_constructor(self, *args, **kwargs)
        if eligible and arm():
            StationDetection.__init__ = saved_constructor
            assert any(
                frame.f_code is saved_take.__code__ and line == call_line
                for frame, line in traceback.walk_stack(None)
            )
            emit("saved_constructor_completed", original_calls=1, take=take)
            if case.startswith("constructor_sig"):
                actual_signal(
                    signal.SIGINT
                    if case != "constructor_sigterm_repeated"
                    else signal.SIGTERM,
                    3 if case.endswith("repeated") else 1,
                )
            elif case == "constructor_error_after_stop":
                actual_signal()
            if case.startswith("constructor_error"):
                emit("unrelated_constructor_error")
                # A real converter naturally masks this unrelated error. Never
                # manufacture the target native RuntimeError in Python.
                raise ValueError("unrelated generated constructor defect")
            emit("constructor_returned_after_signal", take=take)
        return result

    module.FactoryVisualization.__init__ = observed_init
    module.FactoryVisualization.detection = observed_callback
    Executor._take_subscription = observed_take
    if case.startswith("constructor_"):
        StationDetection.__init__ = constructor
    os.set_blocking(arm_fd, False)
    sys.settrace(trace)
    try:
        module.main()
    finally:
        error = sys.exc_info()[1]
        emit(
            "main_exit",
            exception_type=type(error).__name__ if error else None,
            exception_text=str(error) if error else None,
            full_exception="".join(
                traceback.format_exception(type(error), error, error.__traceback__)
            )
            if error
            else None,
            exception_cause=repr(error.__cause__) if error else None,
            exception_context=repr(error.__context__) if error else None,
            actual_signals=state["signal_count"],
            constructor_restored=StationDetection.__init__ is saved_constructor,
            handlers_restored=all(
                signal.getsignal(number) is handler
                for number, handler in prior_handlers.items()
            ),
        )
        sys.settrace(None)
        StationDetection.__init__ = saved_constructor
        Executor._take_subscription = saved_take
        os.close(descriptor)


def parent_probe(output, case):
    from factory_interfaces.msg import ProtocolEvent, StationDetection
    from geometry_msgs.msg import PoseStamped
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from rosgraph_msgs.msg import Clock
    from visualization_msgs.msg import Marker

    os.environ["FACTORY_SCENARIO_OUTPUT"] = str(output)
    domain = int(os.environ["FACTORY_SCENARIO_DOMAIN"])
    rclpy.init(domain_id=domain)
    node = rclpy.create_node("owned_visualizer_lifecycle_probe")
    protocol = node.create_publisher(ProtocolEvent, "/factory/protocol_events", 100)
    detection = node.create_publisher(StationDetection, "/factory/station_detection", 5)
    clock = node.create_publisher(Clock, "/clock", 10)
    goals, markers = [], []
    subscriptions = [
        node.create_subscription(
            PoseStamped,
            "/factory/navigation_goal",
            goals.append,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        ),
        node.create_subscription(
            Marker, "/factory/station_markers", markers.append, 10
        ),
    ]

    def traffic():
        tick = Clock()
        tick.clock.sec = 10
        clock.publish(tick)
        event = ProtocolEvent(
            mission_id="M-lifecycle", protocol="ROS", event="navigation_pickup_started"
        )
        event.stamp.sec = 10
        protocol.publish(event)
        sample = StationDetection(station_id="assembly", marker_id=10, confidence=1.0)
        sample.header.stamp.sec = 10
        detection.publish(sample)
        rclpy.spin_once(node, timeout_sec=0.025)
        time.sleep(0.025)

    read_arm, write_arm = os.pipe()
    log = (output / "visualizer.log").open("w")
    child = subprocess.Popen(
        [sys.executable, __file__, "child", str(output), case, str(read_arm)],
        env={
            **os.environ,
            "ROS_LOCALHOST_ONLY": "1",
            "VISUALIZER_PROBE_PARENT": json.dumps(identity(os.getpid())),
        },
        stdout=log,
        stderr=subprocess.STDOUT,
        pass_fds=(read_arm,),
        start_new_session=True,
    )
    os.close(read_arm)
    owner = identity(child.pid)
    registry = output / "resources.json"
    resources = (
        json.loads(registry.read_text())
        if registry.exists()
        else dict(containers=[], compose_projects=[], ports=[], groups=[])
    )
    resources.setdefault("groups", []).append(
        dict(pid=child.pid, start_ticks=owner["start_ticks"])
    )
    temporary = registry.with_suffix(".tmp")
    temporary.write_text(json.dumps(resources) + "\n")
    temporary.replace(registry)
    signals, readiness, failure, exit_before_cleanup = [], None, None, None

    def owned_signal(number):
        current = identity(child.pid)
        assert (
            current["start_ticks"] == owner["start_ticks"]
            and current["group"] == owner["group"]
        )
        assert current["parent"] == os.getpid() and current["state"] != "Z"
        signals.append(
            dict(signal=int(number), monotonic=time.monotonic(), identity=current)
        )
        child.send_signal(number)

    try:
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            assert child.poll() is None, "visualizer exited before normal DDS readiness"
            traffic()
            if len(goals) >= 20 and len(markers) >= 20:
                break
        else:
            raise AssertionError("20-second DDS readiness bound expired")
        readiness = dict(
            goals=len(goals),
            markers=len(markers),
            goal_readers=node.count_subscribers("/factory/navigation_goal"),
            marker_readers=node.count_subscribers("/factory/station_markers"),
        )
        assert all(
            goal.header.frame_id == "map"
            and goal.header.stamp.sec == 10
            and goal.pose.position.x == -3.0
            and goal.pose.position.y == 0.8
            and abs(goal.pose.orientation.z - 0.7071067811865475) < 1e-12
            and abs(goal.pose.orientation.w - 0.7071067811865476) < 1e-12
            for goal in goals
        )
        assert all(
            marker.header.frame_id == "map"
            and marker.header.stamp.sec == 10
            and marker.text == "assembly: camera marker 10"
            and marker.id == 10
            and marker.pose.position.x == -3.0
            and marker.pose.position.y == 0.8
            and marker.lifetime.sec == 1
            and marker.lifetime.nanosec == 500000000
            for marker in markers
        )
        if case.startswith("idle_"):
            quiet_until = time.monotonic() + 0.2
            while time.monotonic() < quiet_until:
                rclpy.spin_once(node, timeout_sec=0.025)
            owned_signal(signal.SIGINT if case == "idle_sigint" else signal.SIGTERM)
        else:
            os.write(write_arm, b"ARM")
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline and child.poll() is None:
            if case.startswith("idle_"):
                rclpy.spin_once(node, timeout_sec=0.025)
            else:
                traffic()
        exit_before_cleanup = child.poll()
    except (AssertionError, OSError) as error:
        failure = repr(error)
    finally:
        if child.poll() is None:
            owned_signal(signal.SIGTERM)
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                owned_signal(signal.SIGKILL)
                child.wait(timeout=5)
        else:
            child.wait()
        os.close(write_arm)
        log.close()
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
    receipt = dict(
        case=case,
        domain_id=domain,
        child=owner,
        readiness=readiness,
        guard_failure=failure,
        external_signals=signals,
        exit_before_cleanup=exit_before_cleanup,
        actual_child_exit=child.returncode,
        process_absent=not Path(f"/proc/{child.pid}").exists(),
        source_blob=subprocess.check_output(
            ["git", "hash-object", "src/factory_bringup/scripts/factory_visualization"],
            text=True,
        ).strip(),
    )
    (output / "probe.json").write_text(json.dumps(receipt, indent=2) + "\n")
    print(json.dumps(receipt), flush=True)
    return 1 if failure else 0


if __name__ == "__main__":
    if sys.argv[1] == "child":
        child_probe(Path(sys.argv[2]), sys.argv[3], int(sys.argv[4]))
    else:
        raise SystemExit(
            parent_probe(Path(sys.argv[2]), os.environ["VISUALIZER_PROBE_CASE"])
        )
