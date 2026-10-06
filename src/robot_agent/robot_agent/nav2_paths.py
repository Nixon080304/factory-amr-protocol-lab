# SPDX-License-Identifier: Apache-2.0
"""Nonblocking robot-local ComputePathToPose action transport."""

import math

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose


def pose_message(pose, frame, stamp):
    message = PoseStamped()
    message.header.frame_id, message.header.stamp = frame, stamp
    message.pose.position.x, message.pose.position.y = float(pose[0]), float(pose[1])
    message.pose.orientation.z = math.sin(pose[2] / 2)
    message.pose.orientation.w = math.cos(pose[2] / 2)
    return message


def path_distance(path, start, goal, frame):
    if path.header.frame_id != frame or not path.poses:
        raise ValueError("empty or foreign path")
    points = []
    for item in path.poses:
        p = item.pose.position
        if item.header.frame_id != frame or not all(
            math.isfinite(v) for v in (p.x, p.y, p.z)
        ):
            raise ValueError("invalid path pose or frame")
        points.append((p.x, p.y))
    # Nav2 may snap start/goal to a nearby map cell, but a truncated route is not success.
    endpoints = [(points[-1], goal)]
    if start is not None:
        endpoints.append((points[0], start))
    if any(
        math.hypot(p[0] - expected[0], p[1] - expected[1]) > 0.25
        for p, expected in endpoints
    ):
        raise ValueError("path does not reach requested endpoints")
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:]))


class Nav2Paths:
    def __init__(self, client, stamp):
        self.client, self.stamp = client, stamp

    def compute(self, start, goal, frame, callback):
        if not self.client.server_is_ready():
            callback(None, "Nav2 path unavailable")
            return lambda: None
        active, handle, result_future = True, None, None
        stamp = self.stamp()
        request = ComputePathToPose.Goal(
            start=pose_message(start, frame, stamp)
            if start is not None
            else PoseStamped(),
            goal=pose_message(goal, frame, stamp),
            use_start=start is not None,
        )

        def completed(future):
            nonlocal active
            if not active:
                return
            active = False
            try:
                outcome = future.result()
                if outcome.status != GoalStatus.STATUS_SUCCEEDED:
                    raise ValueError("Nav2 path failed")
                length = path_distance(outcome.result.path, start, goal, frame)
            except Exception as error:
                callback(None, str(error))
                return
            callback(length, "")

        def accepted(future):
            nonlocal active, handle, result_future
            if not active:
                return
            try:
                handle = future.result()
                if not handle.accepted:
                    raise ValueError("Nav2 path rejected")
                result_future = handle.get_result_async()
                result_future.add_done_callback(completed)
            except Exception as error:
                active = False
                callback(None, str(error))

        goal_future = self.client.send_goal_async(request)
        goal_future.add_done_callback(accepted)

        def abandon():
            nonlocal active
            if not active:
                return
            active = False
            # rclpy ActionClient removes cancelled futures from its native pending
            # maps via its own done callbacks. No executor-thread wait is needed.
            if not goal_future.done():
                goal_future.cancel()
            if result_future is not None and not result_future.done():
                result_future.cancel()
            if handle is not None and handle.accepted:
                cancel = handle.cancel_goal_async()
                cancel.cancel()

        return abandon
