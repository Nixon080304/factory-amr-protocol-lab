# SPDX-License-Identifier: Apache-2.0
"""Compose the real simulation and reviewed protocol adapters."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def generate_launch_description():
    bringup = Path(get_package_share_directory("factory_bringup"))
    stations_file = bringup / "config/stations.yaml"
    stations = yaml.safe_load(stations_file.read_text())["stations"]
    poses = {f"stations.{name}.pose": [float(values[field]) for field in ("x", "y", "yaw")]
             for name, values in stations.items()}
    system = yaml.safe_load((bringup / "config/factory_system.yaml").read_text())
    launches = []
    for package, filename, arguments in [
        ("factory_simulation", "simulation.launch.py", {"gui": LaunchConfiguration("gui")}),
        ("factory_bringup", "navigation.launch.py", {"rviz": LaunchConfiguration("rviz")}),
    ]:
        share = Path(get_package_share_directory(package))
        launches.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share / "launch" / filename)),
                                                launch_arguments=arguments.items()))
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("broker_port", default_value="1883"),
        DeclareLaunchArgument("plc_port", default_value="1502"),
        DeclareLaunchArgument("output_dir", default_value="artifacts/traces"),
        *launches,
        *[Node(package=package, executable=executable, name=name, output="screen",
               parameters=[system["/**"]["ros__parameters"],
                           system.get(name, {}).get("ros__parameters", {}), parameters])
          for package, executable, name, parameters in [
              ("mission_coordinator", "mission_coordinator", "mission_coordinator", poses),
              ("mqtt_gateway", "mqtt_gateway", "mqtt_gateway",
               {"broker_port": ParameterValue(LaunchConfiguration("broker_port"), value_type=int)}),
              ("modbus_gateway", "modbus_gateway", "modbus_gateway",
               {"plc_port": ParameterValue(LaunchConfiguration("plc_port"), value_type=int)}),
              ("protocol_observer", "protocol_observer", "protocol_observer",
               {"output_dir": ParameterValue(LaunchConfiguration("output_dir"), value_type=str)}),
              ("station_perception", "station_detector", "station_detector", {"stations_file": str(stations_file)}),
              ("payload_simulator", "payload_simulator", "payload_simulator", {}),
              ("fault_injector", "fault_injector", "fault_injector",
               {"fault_owners": ["mqtt_gateway", "modbus_gateway", "mission_coordinator", "simulation", "qos_experiment"]}),
              ("factory_bringup", "factory_visualization", "factory_visualization", {"stations_file": str(stations_file)}),
          ]],
    ])
