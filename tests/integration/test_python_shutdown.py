"""Normal ROS signals stop each production Python entry point cleanly."""

import importlib
import os
from pathlib import Path
import signal
import select
import socket
import subprocess
import sys
import time
import uuid

import pytest
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.impl.implementation_singleton import rclpy_implementation


ENTRYPOINTS = [
    ("mqtt_gateway", "MqttGatewayNode"),
    ("modbus_gateway", "ModbusGatewayNode"),
    ("protocol_observer", "ProtocolObserverNode"),
    ("fault_injector", "FaultInjectorNode"),
    ("payload_simulator", "PayloadSimulatorNode"),
    ("station_perception", "StationDetectorNode"),
]


@pytest.fixture
def entrypoint_args(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", "117")
    # Hold an owned, non-listening broker port. A shutdown check must never
    # publish availability or telemetry to somebody else's localhost broker.
    with socket.socket() as broker:
        broker.bind(("127.0.0.1", 0))
        yield [
            "-p",
            f"broker_port:={broker.getsockname()[1]}",
            "-p",
            f"output_dir:={tmp_path}",
        ]


@pytest.mark.parametrize("package,node_class", ENTRYPOINTS)
@pytest.mark.parametrize("stop_signal", [signal.SIGINT, signal.SIGTERM])
def test_actual_entrypoint_normal_signal_exit(
    package, node_class, stop_signal, tmp_path, entrypoint_args
):
    child_name = "owned_shutdown_child_" + uuid.uuid4().hex
    read_ready, write_ready = os.pipe()
    code = f"""import os
from {package}.node import main
import rclpy
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
def announce(operation):
    announced = False
    def ready(*args, **kwargs):
        nonlocal announced
        if not announced:
            os.write({write_ready}, b'1')
            announced = True
        return operation(*args, **kwargs)
    return ready
SingleThreadedExecutor.spin_once = announce(SingleThreadedExecutor.spin_once)
MultiThreadedExecutor.spin_once = announce(MultiThreadedExecutor.spin_once)
main()
"""
    process = subprocess.Popen(
        [
            sys.executable,
            "-c",
            code,
            "--ros-args",
            "-r",
            "__node:=" + child_name,
            *entrypoint_args,
        ],
        env={**os.environ, "ROS_DOMAIN_ID": "117", "ROS_LOCALHOST_ONLY": "1"},
        cwd=tmp_path,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        pass_fds=(write_ready,),
    )
    os.close(write_ready)
    try:
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            if (
                select.select([read_ready], [], [], 0.05)[0]
                and os.read(read_ready, 1) == b"1"
            ):
                break
            assert process.poll() is None, process.communicate(timeout=2)[0]
        else:
            pytest.fail("production child never reached executor spin")
        process.send_signal(stop_signal)
        output, _ = process.communicate(timeout=10)
        assert process.returncode == 0, output
        assert "Traceback" not in output, output
        assert not Path(f"/proc/{process.pid}").exists()
    finally:
        if process.poll() is None:
            process.kill()
            process.communicate(timeout=5)
        os.close(read_ready)


@pytest.mark.parametrize("package,node_class", ENTRYPOINTS)
@pytest.mark.parametrize(
    "error_type,message,shutdown",
    [
        (RuntimeError, "unexpected callback failure", False),
        (RuntimeError, "unexpected callback failure", True),
        (rclpy_implementation.RCLError, "context is not valid", False),
        (rclpy_implementation.RCLError, "unrelated RCL failure", True),
    ],
)
def test_entrypoint_preserves_unexpected_callback_error(
    monkeypatch, package, node_class, error_type, message, shutdown, entrypoint_args
):
    module = importlib.import_module(package + ".node")

    def failed_spin(executor):
        if shutdown:
            rclpy.try_shutdown()
        raise error_type(message)

    # The owned executor is the slow boundary; node and context remain real.
    executor_type = getattr(module, "SingleThreadedExecutor", None) or getattr(
        module, "MultiThreadedExecutor", None
    )
    if executor_type is None:
        monkeypatch.setattr(module.rclpy, "spin", lambda node: failed_spin(None))
    else:
        monkeypatch.setattr(executor_type, "spin", failed_spin)
    with pytest.raises(error_type, match=message):
        module.main(args=["--ros-args", *entrypoint_args])
    assert not rclpy.ok()


@pytest.mark.parametrize("package,node_class", ENTRYPOINTS)
def test_entrypoint_handles_context_shutdown_waitset_race(
    monkeypatch, package, node_class, entrypoint_args
):
    module = importlib.import_module(package + ".node")
    executor_type = (
        getattr(module, "SingleThreadedExecutor", None) or module.MultiThreadedExecutor
    )

    def stopped(executor):
        rclpy.try_shutdown()
        raise rclpy_implementation.RCLError(
            "failed to initialize wait set: the given context is not valid"
        )

    monkeypatch.setattr(executor_type, "spin", stopped)
    module.main(args=["--ros-args", *entrypoint_args])
    assert not rclpy.ok()


@pytest.mark.parametrize("queued_error", [False, True])
def test_modbus_entrypoint_drains_owned_worker_before_node_destruction(
    monkeypatch, queued_error, entrypoint_args
):
    import threading
    from modbus_gateway import node as module

    destroyed, completed = [], []
    release = threading.Event()
    original_destroy = module.ModbusGatewayNode.destroy_node

    def destroy(node):
        assert completed == ["callback finished"]
        destroyed.append(True)
        return original_destroy(node)

    def stopped(executor):
        def callback():
            completed.append("callback finished")
            if queued_error:
                raise RuntimeError("queued callback failed")

        # Saturate the owned Humble pool, then queue an actual ROS timer handler.
        for _ in range(executor._executor._max_workers):
            executor._executor.submit(lambda: release.wait(timeout=2))
        node = executor.get_nodes()[0]
        node.create_timer(0.001, callback, clock=rclpy.clock.Clock())
        deadline = time.monotonic() + 1
        while not executor._futures and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
        assert executor._futures and not completed
        threading.Timer(0.1, release.set).start()
        rclpy.try_shutdown()
        raise ExternalShutdownException()

    monkeypatch.setattr(module.ModbusGatewayNode, "destroy_node", destroy)
    monkeypatch.setattr(module.MultiThreadedExecutor, "spin", stopped)
    try:
        if queued_error:
            with pytest.raises(RuntimeError, match="queued callback failed"):
                module.main(args=["--ros-args", *entrypoint_args])
        else:
            module.main(args=["--ros-args", *entrypoint_args])
    finally:
        release.set()
        rclpy.try_shutdown()
    assert destroyed == [True]
