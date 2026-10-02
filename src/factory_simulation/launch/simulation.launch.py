# SPDX-License-Identifier: Apache-2.0
"""Launch only the simulation and its robot TF tree."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    package = Path(get_package_share_directory("factory_simulation"))
    description = xacro.process_file(str(package / "urdf/factory_amr.urdf.xacro")).toxml()
    model_path = str(package / "models") + os.pathsep + os.environ.get("GAZEBO_MODEL_PATH", "")
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("world", default_value=str(package / "worlds/factory_floor.world")),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="-3.0"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path),
        # Local models and lighting avoid Gazebo's online model database dependency.
        SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", ""),
        ExecuteProcess(
            cmd=["gzserver", LaunchConfiguration("world"), "--seed", "42",
                 "-s", "libgazebo_ros_init.so", "-s", "libgazebo_ros_factory.so"],
            output="screen", sigterm_timeout="5", sigkill_timeout="5"),
        ExecuteProcess(cmd=["gzclient"], condition=IfCondition(LaunchConfiguration("gui")),
                       output="screen", sigterm_timeout="5", sigkill_timeout="5"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": description, "use_sim_time": True}], output="screen"),
        Node(package="gazebo_ros", executable="spawn_entity.py",
             arguments=["-entity", "factory_amr", "-topic", "robot_description",
                        "-x", LaunchConfiguration("x"), "-y", LaunchConfiguration("y"),
                        "-z", "0.01", "-Y", LaunchConfiguration("yaw"), "-timeout", "30"],
             output="screen"),
    ])
