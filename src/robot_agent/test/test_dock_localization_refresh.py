"""Verify post-navigation physical evidence through production ROS boundaries."""

from types import SimpleNamespace

from geometry_msgs.msg import PoseWithCovarianceStamped
import pytest
from rclpy.task import Future
from std_srvs.srv import Empty

from test_node import api, runtime, sensors
from test_docking import Wire


class RefreshClient:
    def __init__(self, ready=True):
        self.ready, self.requests, self.removed = ready, [], []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        assert isinstance(request, Empty.Request)
        future = Future()
        self.requests.append(future)
        return future

    def remove_pending_request(self, future):
        self.removed.append(future)


def rig(ready=True):
    host, runtime_, _, wall = runtime()
    sensors(host)
    host.sim_sec = 2
    resource = Wire()
    client = RefreshClient(ready)
    client.navigation_results = []

    def send_goal(goal):
        accepted, terminal = Future(), Future()
        client.navigation_results.append(terminal)
        accepted.set_result(
            SimpleNamespace(
                accepted=True,
                get_result_async=lambda: terminal,
                cancel_goal_async=lambda: Future(),
            )
        )
        return accepted

    # Keep production service registration and request path; substitute DDS only.
    def create_client(service, name):
        assert service is Empty
        assert host.resolve(name) == "/cart_1/request_nomotion_update"
        return client

    host.create_client = create_client
    transport = api().DockTransport(
        host,
        SimpleNamespace(server_is_ready=lambda: True, send_goal_async=send_goal),
        {},
    )
    transport.resource = resource.resource
    results = []
    controller = api().DockingController(
        runtime_.adapter,
        transport,
        "charger",
        (2, 0, 0),
        (3, 0, 0),
        clock=lambda: wall[0],
    )
    assert controller.start(80, lambda _: None, results.append)
    return (
        host,
        runtime_,
        wall,
        resource,
        client,
        client.navigation_results[0],
        results,
        controller,
    )


def pose(host, stamp, yaw=0.0, x=2.0):
    import math

    message = PoseWithCovarianceStamped()
    message.header.frame_id, message.header.stamp.sec = "floor/cart_1/map", stamp
    message.pose.pose.position.x = x
    message.pose.pose.orientation.z = math.sin(yaw / 2)
    message.pose.pose.orientation.w = math.cos(yaw / 2)
    host.subscriptions[host.resolve("amcl_pose")](message)


def test_terminal_nav2_refreshes_retained_final_turn_before_exact_arrival():
    host, runtime_, _, resource, client, terminal, _, controller = rig()
    pose(host, 2, yaw=0.242)
    terminal.set_result(SimpleNamespace(status=4))
    assert len(client.requests) == 1
    assert controller.state == "STAGING" and not resource.requests
    client.requests[0].set_result(Empty.Response())
    controller.tick()
    assert controller.state == "STAGING" and not resource.requests
    pose(host, 3, yaw=0.149)
    controller.tick()
    assert controller.state == "WAITING_FOR_LEASE"
    assert resource.requests[-1][0] == "acquire"
    assert runtime_.adapter.pose[2] == pytest.approx(0.149)


@pytest.mark.parametrize(
    "failure",
    [
        "missing",
        "exception",
        "invalid_response",
        "timeout",
        "old_stamp",
        "wrong_yaw",
        "wrong_position",
    ],
)
def test_refresh_failure_never_grants_arrival(failure):
    host, runtime_, wall, resource, client, terminal, results, controller = rig(
        failure != "missing"
    )
    pose(host, 2, yaw=0.242)
    terminal.set_result(SimpleNamespace(status=4))
    if failure == "exception":
        client.requests[0].set_exception(RuntimeError("AMCL unavailable"))
    elif failure == "invalid_response":
        client.requests[0].set_result(None)
    elif failure in ("old_stamp", "wrong_yaw", "wrong_position"):
        client.requests[0].set_result(Empty.Response())
        pose(
            host,
            2 if failure == "old_stamp" else 3,
            yaw=0.201 if failure == "wrong_yaw" else 0,
            x=2.151 if failure == "wrong_position" else 2.0,
        )
    wall[0] += 1.0
    controller.tick()
    assert not resource.requests and not controller.active
    assert runtime_.adapter.mode == "RECOVERY_REQUIRED"
    assert len(results) == 1 and not results[0].success
    if failure == "timeout":
        assert client.removed == client.requests
        client.requests[0].set_result(Empty.Response())
        pose(host, 3)
        controller.tick()
        assert not resource.requests and len(results) == 1


def test_retained_in_tolerance_pose_still_needs_post_result_observation():
    host, _, _, resource, client, terminal, _, controller = rig()
    pose(host, 2)
    terminal.set_result(SimpleNamespace(status=4))
    assert not resource.requests and controller.state == "STAGING"
    assert len(client.requests) == 1
    client.requests[0].set_result(Empty.Response())
    controller.tick()
    assert not resource.requests
    pose(host, 3)
    controller.tick()
    assert resource.requests[-1][0] == "acquire"


def test_shutdown_removes_pending_refresh_and_ignores_late_response():
    host, runtime_, _, resource, client, terminal, results, controller = rig()
    pose(host, 2, yaw=0.242)
    terminal.set_result(SimpleNamespace(status=4))
    controller.shutdown()
    assert client.removed == client.requests
    client.requests[0].set_result(Empty.Response())
    pose(host, 3)
    controller.tick()
    assert not resource.requests and len(results) == 1
    assert runtime_.adapter.mode == "RECOVERY_REQUIRED"


def test_localization_before_acknowledgement_waits_without_request_storm():
    host, _, _, resource, client, terminal, _, controller = rig()
    pose(host, 2, yaw=0.242)
    terminal.set_result(SimpleNamespace(status=4))
    pose(host, 3)
    for _ in range(100):
        controller.tick()
    assert len(client.requests) == 1 and not resource.requests
    client.requests[0].set_result(Empty.Response())
    assert controller.state == "WAITING_FOR_LEASE"


@pytest.mark.parametrize(
    "failure",
    ["stale_odom", "invalid_odom", "invalid_localization", "stale_after_movement"],
)
def test_unhealthy_evidence_during_refresh_fails_closed(failure):
    host, runtime_, wall, resource, client, terminal, results, controller = rig()
    agent = runtime_.adapter
    pose(host, 2, yaw=0.242)
    terminal.set_result(SimpleNamespace(status=4))
    if failure == "stale_odom":
        wall[0] += 3.0
    elif failure == "invalid_odom":
        agent.odometry(2, float("nan"), 0, agent.frame_prefix + "odom")
    elif failure == "invalid_localization":
        agent.localization(3, float("nan"), 0, 0, agent.frame_prefix + "map")
    else:
        wall[0] += 3.0
        agent.odometry(2, 0.6, 0, agent.frame_prefix + "odom")
    controller.tick()
    assert len(results) == 1 and results[0].error_code == "ROBOT_UNAVAILABLE"
    assert agent.mode == "RECOVERY_REQUIRED" and not resource.requests
    assert client.removed == client.requests


def test_cancel_refresh_fences_old_reply_from_clearance_and_new_session():
    host, runtime_, _, resource, client, terminal, results, controller = rig()
    pose(host, 2, yaw=0.242)
    terminal.set_result(SimpleNamespace(status=4))
    old = client.requests[0]
    controller.cancel()
    assert client.removed == [old] and controller.state == "CLEARING"
    client.navigation_results[1].set_result(SimpleNamespace(status=4))
    pose(host, 3, x=0.0)
    old.set_result(Empty.Response())
    controller.tick()
    assert not results and not resource.requests
    client.requests[1].set_result(Empty.Response())
    assert len(results) == 1 and results[0].error_code == "CANCELLED"
    assert runtime_.adapter.mode == "AVAILABLE"
    assert controller.start(80, lambda _: None, results.append)
    client.navigation_results[2].set_result(SimpleNamespace(status=4))
    pose(host, 4)
    # A repeated old generation callback cannot acknowledge the new request.
    old.set_result(Empty.Response())
    controller.tick()
    assert not resource.requests and len(results) == 1
    client.requests[2].set_result(Empty.Response())
    assert controller.state == "WAITING_FOR_LEASE"


def test_delayed_pre_request_sample_cannot_prove_arrival_but_equal_epoch_can():
    host, _, _, resource, client, terminal, _, controller = rig()
    pose(host, 2, yaw=0.242)
    host.sim_sec = 4
    terminal.set_result(SimpleNamespace(status=4))
    client.requests[0].set_result(Empty.Response())
    # This sample is received after the result and newer than the retained pose,
    # but its source time precedes the no-motion request.
    pose(host, 3)
    controller.tick()
    assert not resource.requests and controller.state == "STAGING"
    # A paused ROS clock can produce evidence exactly at the request epoch.
    pose(host, 4)
    controller.tick()
    assert controller.state == "WAITING_FOR_LEASE"
