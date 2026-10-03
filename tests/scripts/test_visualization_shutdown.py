"""Expected ROS shutdown and the production visualization dependency contract."""

from importlib.machinery import SourceFileLoader
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET
import signal

import pytest

from rclpy.executors import ExternalShutdownException


ROOT = Path(__file__).resolve().parents[2]


def test_visualization_handles_expected_external_shutdown(monkeypatch):
    loader = SourceFileLoader(
        "factory_visualization_shutdown_test",
        str(ROOT / "src/factory_bringup/scripts/factory_visualization"),
    )
    spec = importlib.util.spec_from_loader(loader.name, loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    calls = []

    class Node:
        def destroy_node(self):
            calls.append("destroy")

    def stopped(node, **kwargs):
        raise ExternalShutdownException()

    monkeypatch.setattr(module, "FactoryVisualization", Node)
    monkeypatch.setattr(module.rclpy, "init", lambda **kwargs: None)
    monkeypatch.setattr(module.rclpy, "spin", stopped)
    monkeypatch.setattr(module.rclpy, "spin_once", stopped)
    monkeypatch.setattr(module.rclpy, "ok", lambda: True)
    monkeypatch.setattr(module.rclpy, "shutdown", lambda: calls.append("shutdown"))
    module.main()
    assert calls == ["destroy", "shutdown"]


@pytest.mark.parametrize("failure_point", ["init", "node"])
def test_visualization_restores_signal_handlers_after_initialization_failure(
    monkeypatch, failure_point
):
    loader = SourceFileLoader(
        "factory_visualization_init_failure",
        str(ROOT / "src/factory_bringup/scripts/factory_visualization"),
    )
    spec = importlib.util.spec_from_loader(loader.name, loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    original = {
        number: signal.getsignal(number) for number in (signal.SIGINT, signal.SIGTERM)
    }
    received = []

    def previous_handler(number, frame):
        received.append(number)

    def failed(*args, **kwargs):
        raise ValueError("visualizer initialization defect")

    try:
        for number in original:
            signal.signal(number, previous_handler)
        monkeypatch.setattr(
            module.rclpy,
            "init",
            failed if failure_point == "init" else lambda **kwargs: None,
        )
        monkeypatch.setattr(module, "FactoryVisualization", failed)
        monkeypatch.setattr(module.rclpy, "ok", lambda: False)
        with pytest.raises(ValueError, match="visualizer initialization defect"):
            module.main()
        # Exercise the restored handlers, not only their identity or source text.
        signal.raise_signal(signal.SIGINT)
        signal.raise_signal(signal.SIGTERM)
        assert received == [signal.SIGINT, signal.SIGTERM]
    finally:
        for number, handler in original.items():
            signal.signal(number, handler)


def test_geometry_messages_are_declared_for_production_visualization():
    package = ET.parse(ROOT / "src/factory_bringup/package.xml").getroot()
    assert "geometry_msgs" in {
        dependency.text for dependency in package.findall("exec_depend")
    }
