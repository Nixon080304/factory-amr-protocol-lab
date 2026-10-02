# SPDX-License-Identifier: Apache-2.0
"""Prove bounded real Nav2 motion, AMCL accuracy, and independent world pose."""

import math
import os
from pathlib import Path
import socket
import time
import unittest

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import GetEntityState
from geometry_msgs.msg import PoseWithCovarianceStamped
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid, Path as NavPath
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


@pytest.mark.launch_test
def generate_test_description():
    domain = int(os.environ.get("FACTORY_NAV_TEST_DOMAIN_ID", "73"))
    if not 1 <= domain <= 232:
        raise ValueError("FACTORY_NAV_TEST_DOMAIN_ID must be an isolated domain from 1 to 232")
    os.environ["ROS_DOMAIN_ID"] = str(domain)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    with socket.socket() as reserved:
        reserved.bind(("127.0.0.1", 0))
        port = reserved.getsockname()[1]
    os.environ["GAZEBO_MASTER_URI"] = f"http://127.0.0.1:{port}"
    launches = []
    for package, filename in [("factory_simulation", "simulation.launch.py"),
                              ("factory_bringup", "navigation.launch.py")]:
        share = Path(get_package_share_directory(package))
        launches.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch" / filename)),
            launch_arguments={"gui": "false", "rviz": "false"}.items()))
    return LaunchDescription([*launches, launch_testing.actions.ReadyToTest()])


def yaw(quaternion):
    return math.atan2(2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
                      1 - 2 * (quaternion.y ** 2 + quaternion.z ** 2))


def angle_error(first, second):
    return abs(math.atan2(math.sin(first - second), math.cos(first - second)))


class TestNavigation(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("factory_navigation_probe",
                                   parameter_overrides=[Parameter("use_sim_time", value=True)])

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin_until(self, predicate, timeout, message):
        deadline = min(self.deadline, time.monotonic() + timeout)
        while not predicate() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertTrue(predicate(), message)

    def entity_pose(self, client):
        request = GetEntityState.Request()
        request.name = "factory_amr"
        request.reference_frame = "world"
        future = client.call_async(request)
        self.spin_until(future.done, 5, "Gazebo world pose did not arrive")
        self.assertTrue(future.result().success)
        return future.result().state.pose

    def test_navigate_to_pose_moves_real_robot_and_meets_tolerances(self):
        started = time.monotonic()
        self.deadline = started + 150
        buffer = Buffer(node=self.node)
        listener = TransformListener(buffer, self.node)
        maps, plans, localization = [], [], []
        subscriptions = [
            self.node.create_subscription(OccupancyGrid, "/map", maps.append,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
            self.node.create_subscription(NavPath, "/plan", plans.append, 10),
            self.node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", localization.append, 10),
        ]
        initial = self.node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        action = ActionClient(self.node, NavigateToPose, "/navigate_to_pose")
        entity = self.node.create_client(GetEntityState, "/gazebo/get_entity_state")
        clients, handle = [], None
        try:
            for name in ("map_server", "amcl", "controller_server", "planner_server", "smoother_server",
                         "behavior_server", "bt_navigator", "waypoint_follower", "velocity_smoother"):
                client = self.node.create_client(GetState, f"/{name}/get_state")
                clients.append(client)
                self.spin_until(client.service_is_ready, 40, f"{name} lifecycle service unavailable")
                def active():
                    future = client.call_async(GetState.Request())
                    self.spin_until(future.done, 3, f"{name} state request timed out")
                    return future.result().current_state.id == State.PRIMARY_STATE_ACTIVE
                self.spin_until(active, 35, f"{name} did not activate")
            self.spin_until(lambda: maps and self.node.get_clock().now().nanoseconds > 0,
                            10, "map or simulation clock missing")
            self.assertEqual((maps[-1].info.width, maps[-1].info.height), (248, 168))
            self.spin_until(entity.service_is_ready, 5, "Gazebo pose service missing")
            before = self.entity_pose(entity)
            self.assertLess(math.hypot(before.position.x, before.position.y + 3), 0.03)
            pose = PoseWithCovarianceStamped()
            pose.header.frame_id = "map"
            pose.pose.pose.position.y = -3.0
            pose.pose.pose.orientation.w = 1.0
            pose.pose.covariance[0] = pose.pose.covariance[7] = 0.0025
            pose.pose.covariance[35] = 0.0025
            self.spin_until(lambda: initial.get_subscription_count() > 0, 5, "AMCL initial pose subscriber missing")
            pose.header.stamp = self.node.get_clock().now().to_msg()
            initial.publish(pose)
            self.spin_until(lambda: localization and buffer.can_transform("map", "base_footprint", Time()),
                            10, "AMCL map-to-base localization missing")
            self.assertEqual(self.node.count_publishers("/odom"), 1)
            self.assertEqual(self.node.count_publishers("/cmd_vel"), 1,
                             "only the velocity smoother may publish robot commands")
            for name in ("local_costmap", "global_costmap"):
                client = self.node.create_client(ClearEntireCostmap, f"/{name}/clear_entirely_{name}")
                clients.append(client)
                self.spin_until(client.service_is_ready, 5, f"{name} clear service unavailable")
                future = client.call_async(ClearEntireCostmap.Request())
                self.spin_until(future.done, 5, f"{name} clear failed")
                self.assertIsNotNone(future.result())
            self.spin_until(action.server_is_ready, 5, "NavigateToPose unavailable")
            target_x, target_y, target_yaw = -1.5, -3.0, 0.4
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = "map"
            goal.pose.header.stamp = self.node.get_clock().now().to_msg()
            goal.pose.pose.position.x, goal.pose.pose.position.y = target_x, target_y
            goal.pose.pose.orientation.z = math.sin(target_yaw / 2)
            goal.pose.pose.orientation.w = math.cos(target_yaw / 2)
            future = action.send_goal_async(goal)
            self.spin_until(future.done, 5, "goal acceptance timed out")
            handle = future.result()
            self.assertTrue(handle.accepted)
            result = handle.get_result_async()
            self.spin_until(result.done, 100, "navigation exceeded bounded goal timeout")
            self.assertEqual(result.result().status, GoalStatus.STATUS_SUCCEEDED)
            self.assertTrue(plans and len(plans[-1].poses) > 1, "real planner output missing")
            # Allow the smoother to stop before measuring independent world state.
            settle = min(self.deadline, time.monotonic() + 1)
            while time.monotonic() < settle:
                rclpy.spin_once(self.node, timeout_sec=0.05)
            after = self.entity_pose(entity)
            transform = buffer.lookup_transform("map", "base_footprint", Time()).transform
            for name, x, y, heading in [("Gazebo", after.position.x, after.position.y, yaw(after.orientation)),
                                       ("AMCL/TF", transform.translation.x, transform.translation.y,
                                        yaw(transform.rotation))]:
                position_error = math.hypot(x - target_x, y - target_y)
                heading_error = angle_error(heading, target_yaw)
                print(f"{name} final=({x:.6f}, {y:.6f}, {heading:.6f}); "
                      f"position_error={position_error:.6f} m; yaw_error={heading_error:.6f} rad")
                self.assertLess(position_error, 0.25, name)
                self.assertLess(heading_error, 0.25, name)
            displacement = math.hypot(after.position.x - before.position.x, after.position.y - before.position.y)
            self.assertGreater(displacement, 1.0, "successful action must physically move the robot")
            print(f"NavigateToPose SUCCEEDED; physical motion={displacement:.6f} m; "
                  f"elapsed={time.monotonic() - started:.2f} s; /cmd_vel publishers=1")
        finally:
            if handle is not None:
                cancel = handle.cancel_goal_async()
                deadline = time.monotonic() + 2
                while not cancel.done() and time.monotonic() < deadline:
                    rclpy.spin_once(self.node, timeout_sec=0.05)
            action.destroy()
            listener.unregister()
            self.node.destroy_publisher(initial)
            self.node.destroy_client(entity)
            for client in clients:
                self.node.destroy_client(client)
            for subscription in subscriptions:
                self.node.destroy_subscription(subscription)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
