# SPDX-License-Identifier: Apache-2.0
"""Build installed robot assets and launch actions for any robot identity."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch_ros.actions import Node
import xacro


def robot_description(namespace: str = "", frame_prefix: str = "") -> str:
    package = Path(get_package_share_directory("factory_simulation"))
    return xacro.process_file(
        str(package / "urdf/factory_amr.urdf.xacro"),
        mappings={"namespace": namespace, "frame_prefix": frame_prefix},
    ).toxml()


def simulator_actions(world, gui):
    """One server owns the world; robot creation is a separate repeated action."""
    package = Path(get_package_share_directory("factory_simulation"))
    model_path = os.pathsep.join(
        [
            str(package / "models"),
            os.environ.get("GAZEBO_MODEL_PATH", ""),
            str(Path(get_package_share_directory("turtlebot3_description")).parent),
        ]
    )
    return [
        SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path),
        SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", ""),
        ExecuteProcess(
            cmd=[
                "gzserver",
                world,
                "--seed",
                "42",
                "-s",
                "libgazebo_ros_init.so",
                "-s",
                "libgazebo_ros_factory.so",
            ],
            output="screen",
            sigterm_timeout="5",
            sigkill_timeout="5",
        ),
        ExecuteProcess(
            cmd=["gzclient"],
            condition=IfCondition(gui),
            output="screen",
            sigterm_timeout="5",
            sigkill_timeout="5",
        ),
    ]


def robot_actions(namespace, frame_prefix, entity_name, x, y, yaw):
    description_topic = (
        namespace.rstrip("/") + "/robot_description"
        if namespace
        else "robot_description"
    )
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            namespace=namespace,
            parameters=[
                {
                    "robot_description": robot_description(namespace, frame_prefix),
                    "use_sim_time": True,
                }
            ],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
            output="screen",
        ),
        # Gazebo's spawn service is global. The description source and entity are unique.
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            namespace=namespace,
            arguments=[
                "-entity",
                entity_name,
                "-topic",
                description_topic,
                "-robot_namespace",
                namespace or "/",
                "-x",
                x,
                "-y",
                y,
                "-z",
                "0.01",
                "-Y",
                yaw,
                "-timeout",
                "30",
            ],
            output="screen",
        ),
    ]
