"""Expected ROS shutdown and the production visualization dependency contract."""
from importlib.machinery import SourceFileLoader
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

from rclpy.executors import ExternalShutdownException


ROOT = Path(__file__).resolve().parents[2]


def test_visualization_handles_expected_external_shutdown(monkeypatch):
    loader = SourceFileLoader("factory_visualization_shutdown_test",
                              str(ROOT / "src/factory_bringup/scripts/factory_visualization"))
    spec = importlib.util.spec_from_loader(loader.name, loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    calls = []

    class Node:
        def destroy_node(self):
            calls.append("destroy")

    def stopped(node):
        raise ExternalShutdownException()

    monkeypatch.setattr(module, "FactoryVisualization", Node)
    monkeypatch.setattr(module.rclpy, "init", lambda: None)
    monkeypatch.setattr(module.rclpy, "spin", stopped)
    monkeypatch.setattr(module.rclpy, "ok", lambda: True)
    monkeypatch.setattr(module.rclpy, "shutdown", lambda: calls.append("shutdown"))
    module.main()
    assert calls == ["destroy", "shutdown"]


def test_geometry_messages_are_declared_for_production_visualization():
    package = ET.parse(ROOT / "src/factory_bringup/package.xml").getroot()
    assert "geometry_msgs" in {dependency.text for dependency in package.findall("exec_depend")}
