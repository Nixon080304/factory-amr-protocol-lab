"""Exercise production Nav2 goals/results; replace only the external wire."""

import importlib
from pathlib import Path
import sys
from types import SimpleNamespace

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path as RosPath
import pytest
from rclpy.task import Future

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def api():
    try:
        return importlib.import_module("robot_agent.nav2_paths")
    except ModuleNotFoundError:
        pytest.fail("Nav2 production path adapter is missing")


class Client:
    def __init__(self, ready=True):
        self.ready = ready
        self.goals = []
        self.future = Future()

    def server_is_ready(self):
        return self.ready

    def send_goal_async(self, goal):
        self.goals.append(goal)
        return self.future


class Handle:
    accepted = True

    def __init__(self):
        self.future = Future()
        self.cancelled = False

    def get_result_async(self):
        return self.future

    def cancel_goal_async(self):
        self.cancelled = True
        return Future()


def path(frame="floor/cart/map", points=((0, 0), (0, 4), (3, 4))):
    message = RosPath()
    message.header.frame_id = frame
    for x, y in points:
        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.pose.position.x, pose.pose.position.y = float(x), float(y)
        pose.pose.orientation.w = 1.0
        message.poses.append(pose)
    return message


def result(message, status=GoalStatus.STATUS_SUCCEEDED):
    return SimpleNamespace(status=status, result=ComputePathToPose.Result(path=message))


def test_nav2_path_distance_and_goals_use_same_prefixed_map():
    client = Client()
    adapter = api().Nav2Paths(client, lambda: Time(sec=42))
    results = []
    adapter.compute(
        (0, 0, 0), (3, 4, 1), "floor/cart/map", lambda *args: results.append(args)
    )
    goal = client.goals[0]
    assert goal.use_start and goal.planner_id == ""
    assert goal.start.header.frame_id == goal.goal.header.frame_id == "floor/cart/map"
    assert goal.start.header.stamp.sec == goal.goal.header.stamp.sec == 42
    assert goal.start.pose.position.x == 0 and goal.goal.pose.position.x == 3
    handle = Handle()
    client.future.set_result(handle)
    handle.future.set_result(result(path()))
    assert results == [(7.0, "")]


def test_nav2_current_start_uses_server_localization_and_validates_only_goal():
    client, results = Client(), []
    api().Nav2Paths(client, Time).compute(
        None, (3, 4, 0), "floor/cart/map", lambda *args: results.append(args)
    )
    assert client.goals[0].use_start is False
    handle = Handle()
    client.future.set_result(handle)
    handle.future.set_result(result(path(points=((1, 0), (1, 4), (3, 4)))))
    assert results == [(6.0, "")]


@pytest.mark.parametrize(
    "failure",
    [
        "unavailable",
        "rejected",
        "aborted",
        "empty",
        "foreign",
        "nonfinite",
        "truncated",
    ],
)
def test_nav2_failure_never_returns_euclidean_distance(failure):
    client = Client(ready=failure != "unavailable")
    results = []
    api().Nav2Paths(client, Time).compute(
        (0, 0, 0), (3, 4, 0), "floor/cart/map", lambda *args: results.append(args)
    )
    if failure != "unavailable":
        handle = Handle()
        if failure == "rejected":
            handle.accepted = False
        client.future.set_result(handle)
        if handle.accepted:
            message = path()
            if failure == "empty":
                message.poses = []
            if failure == "foreign":
                message.poses[1].header.frame_id = "other/map"
            if failure == "nonfinite":
                message.poses[1].pose.position.x = float("nan")
            if failure == "truncated":
                message.poses.pop()
            handle.future.set_result(
                result(
                    message,
                    GoalStatus.STATUS_ABORTED
                    if failure == "aborted"
                    else GoalStatus.STATUS_SUCCEEDED,
                )
            )
    assert len(results) == 1 and results[0][0] is None and results[0][1]


@pytest.mark.parametrize("accepted", [False, True])
def test_abandon_cancels_pending_futures_and_ignores_late_reply(accepted):
    client, handle, results = Client(), Handle(), []
    cancel = (
        api()
        .Nav2Paths(client, Time)
        .compute(
            (0, 0, 0), (3, 4, 0), "floor/cart/map", lambda *args: results.append(args)
        )
    )
    if accepted:
        client.future.set_result(handle)
    cancel()
    assert (handle.future if accepted else client.future).cancelled()
    assert handle.cancelled is accepted
    assert results == []


@pytest.mark.parametrize("accepted", [False, True])
def test_production_abandon_removes_actual_rclpy_pending_maps(accepted):
    # Replacing only the native DDS handle preserves rclpy request bookkeeping.
    from rclpy.action import ActionClient
    from rclpy.action.client import ClientGoalHandle
    from unique_identifier_msgs.msg import UUID

    class Wire:
        sequence = 0

        def send_goal_request(self, request):
            self.sequence += 1
            return self.sequence

        send_result_request = send_goal_request
        send_cancel_request = send_goal_request

    client = ActionClient.__new__(ActionClient)
    client._action_type = ComputePathToPose
    client._client_handle = Wire()
    client._pending_goal_requests = {}
    client._pending_result_requests = {}
    client._pending_cancel_requests = {}
    client._goal_sequence_number_to_goal_id = {}
    client._result_sequence_number_to_goal_id = {}
    client._feedback_callbacks = {}
    client._futures = []
    client._generate_random_uuid = lambda: UUID(uuid=[1] * 16)
    client.server_is_ready = lambda: True
    for _ in range(10):
        cancel = (
            api()
            .Nav2Paths(client, Time)
            .compute(None, (3, 4, 0), "floor/cart/map", lambda *_: None)
        )
        assert len(client._pending_goal_requests) == 1
        if accepted:
            handle = ClientGoalHandle(
                client,
                UUID(uuid=[1] * 16),
                ComputePathToPose.Impl.SendGoalService.Response(accepted=True),
            )
            next(iter(client._pending_goal_requests.values())).set_result(handle)
            assert len(client._pending_result_requests) == 1
        cancel()
        assert (
            not client._pending_goal_requests
            and not client._goal_sequence_number_to_goal_id
        )
        assert (
            not client._pending_result_requests and not client._pending_cancel_requests
        )
        assert not client._result_sequence_number_to_goal_id
        assert not client._futures
