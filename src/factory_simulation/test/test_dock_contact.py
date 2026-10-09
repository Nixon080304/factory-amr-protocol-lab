"""The simulation-only contact sensor consumes independent Gazebo evidence."""

import importlib
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import GetEntityState
from rclpy.task import Future

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


class Host:
    def __init__(self):
        self.messages, self.timers = [], []
        self.logs = []

    def get_logger(self):
        return SimpleNamespace(info=self.logs.append)

    def create_publisher(self, kind, name, qos):
        assert name == "factory/dock_contact"
        return SimpleNamespace(
            publish=lambda message: self.messages.append(message.data)
        )

    def create_timer(self, period, callback, **kwargs):
        self.timers.append(callback)
        return callback


def sensor():
    try:
        module = importlib.import_module("factory_simulation.dock_contact")
    except ModuleNotFoundError:
        pytest.fail("production Gazebo dock contact producer is missing")
    host, wall, futures, requests = Host(), [100.0], [], []

    def query(request):
        requests.append(request)
        future = Future()
        futures.append(future)
        return future

    client = SimpleNamespace(service_is_ready=lambda: True, call_async=query)
    runtime = module.DockContactRuntime(
        host, client, "configured_cart", (4, -2, 0), clock=lambda: wall[0]
    )
    return runtime, host, wall, client, futures, requests


def reply(pose=(4, -2, 0), success=True):
    value = Pose()
    value.position.x, value.position.y, value.position.z = map(float, pose)
    value.orientation.w = 1.0
    # gazebo_ros_state3.9 sets header.frame_id but leaves both identity strings
    # at their actual ROS defaults, even for a named world-frame request.
    response = GetEntityState.Response(success=success)
    response.header.frame_id = "world"
    response.state.pose = value
    return response


@pytest.mark.parametrize(
    "reason", ["xy", "yaw", "z", "identity", "quaternion", "service", "stale"]
)
def test_contact_transition_diagnostics_classify_exact_loss_reason(reason):
    import json
    import math

    runtime, host, wall, _, futures, _ = sensor()
    runtime.tick()
    futures[-1].set_result(reply())
    runtime.tick()
    value = reply()
    if reason == "xy":
        value.state.pose.position.x = 4.151
    elif reason == "yaw":
        value.state.pose.orientation.z = math.sin(0.201 / 2)
        value.state.pose.orientation.w = math.cos(0.201 / 2)
    elif reason == "z":
        value.state.pose.position.z = 0.151
    elif reason == "identity":
        value.header.frame_id = "wrong"
    elif reason == "quaternion":
        value.state.pose.orientation.w = 0.9
    if reason == "stale":
        wall[0] += 0.5
        runtime.tick()
    elif reason == "service":
        futures[-1].set_exception(RuntimeError("simulator response lost"))
    else:
        wall[0] += 0.02
        futures[-1].set_result(value)
    assert host.messages[-1] is False
    assert host.logs, "contact transitions need structured diagnostics"
    diagnostic = json.loads(host.logs[-1])
    assert diagnostic["event"] == "dock_contact_transition"
    assert diagnostic["confirmed"] is False and diagnostic["reason"] == reason
    assert diagnostic["entity_name"] == "configured_cart"
    assert diagnostic["thresholds"] == {
        "xy_m": 0.15,
        "yaw_rad": 0.2,
        "z_m": 0.15,
        "quaternion_norm_error": 0.01,
        "stale_sec": 0.5,
    }
    assert diagnostic["request_age_sec"] == pytest.approx(
        0.5 if reason == "stale" else 0.0 if reason == "service" else 0.02
    )
    if reason == "yaw":
        assert diagnostic["measurements"]["yaw_rad"] == pytest.approx(0.201)
        assert diagnostic["receipt_age_sec"] == 0.0
    elif reason == "xy":
        assert diagnostic["measurements"]["xy_m"] == pytest.approx(0.151)
    elif reason == "z":
        assert diagnostic["measurements"]["z_m"] == pytest.approx(0.151)
    elif reason == "quaternion":
        assert diagnostic["measurements"]["quaternion_norm_squared"] == pytest.approx(
            0.81
        )


def test_contact_diagnostics_emit_only_boolean_transitions_not_periodic_samples():
    import json

    runtime, host, _, _, futures, _ = sensor()
    assert len(host.logs) == 1
    for _ in range(20):
        runtime.tick()
        futures[-1].set_result(reply())
    assert [json.loads(value)["confirmed"] for value in host.logs] == [False, True]
    for _ in range(20):
        runtime.tick()
        futures[-1].set_result(reply((4.2, -2, 0)))
    assert [json.loads(value)["confirmed"] for value in host.logs] == [
        False,
        True,
        False,
    ]


def test_nonfinite_contact_diagnostics_remain_valid_json_and_fail_closed():
    import json

    runtime, host, _, _, futures, _ = sensor()
    runtime.tick()
    futures[-1].set_result(reply())
    runtime.tick()
    value = reply()
    value.state.pose.position.x = float("nan")
    futures[-1].set_result(value)
    assert host.messages[-1] is False
    assert host.logs
    diagnostic = json.loads(host.logs[-1])
    assert diagnostic["reason"] == "xy"
    assert diagnostic["measurements"]["xy_m"] is None


@pytest.mark.parametrize(
    "identity",
    [
        "wrong_entity",
        "wrong_reference",
        "wrong_header",
        "empty_header",
        "missing_name",
        "missing_reference",
        "missing_header",
    ],
)
def test_mismatched_or_malformed_response_identity_never_confirms_contact(identity):
    runtime, host, _, _, futures, _ = sensor()
    runtime.tick()
    value = reply()
    if identity == "wrong_entity":
        value.state.name = "another_robot"
    elif identity == "wrong_reference":
        value.state.reference_frame = "another_frame"
    elif identity == "wrong_header":
        value.header.frame_id = "another_frame"
    elif identity == "empty_header":
        value.header.frame_id = ""
    elif identity == "missing_header":
        value = SimpleNamespace(success=True, state=value.state)
    else:
        fields = dict(pose=value.state.pose, name="", reference_frame="")
        fields.pop("name" if identity == "missing_name" else "reference_frame")
        value = SimpleNamespace(
            success=True, header=value.header, state=SimpleNamespace(**fields)
        )
    futures[-1].set_result(value)
    assert host.messages[-1] is False


@pytest.mark.parametrize("explicit_identity", [False, True])
def test_current_exact_world_request_accepts_real_upstream_empty_or_matching_identity(
    explicit_identity,
):
    runtime, host, _, _, futures, requests = sensor()
    runtime.tick()
    assert (
        requests[-1].name == "configured_cart"
        and requests[-1].reference_frame == "world"
    )
    value = reply()
    if explicit_identity:
        value.state.name, value.state.reference_frame = "configured_cart", "world"
    futures[-1].set_result(value)
    assert host.messages[-1] is True


def test_upstream_empty_identity_requires_current_exact_pending_query():
    runtime, _, _, _, _, _ = sensor()
    assert not runtime._at_contact(reply())


def test_gazebo_world_pose_confirms_contact_and_physical_loss_clears_it():
    runtime, host, _, _, futures, requests = sensor()
    runtime.tick()
    assert not host.messages[-1]
    assert (
        requests[-1].name == "configured_cart"
        and requests[-1].reference_frame == "world"
    )
    futures[-1].set_result(reply())
    assert host.messages[-1] is True
    runtime.tick()
    futures[-1].set_result(reply((3.5, -2, 0)))
    assert host.messages[-1] is False


def test_contact_freshness_is_bounded_from_request_not_delayed_delivery():
    runtime, host, wall, _, futures, _ = sensor()
    runtime.tick()
    wall[0] += 0.4
    futures[-1].set_result(reply())
    assert host.messages[-1] is True
    wall[0] += 0.11
    runtime.tick()
    assert host.messages[-1] is False


def test_shutdown_clears_contact_and_fences_late_simulator_callback():
    runtime, host, _, _, futures, _ = sensor()
    runtime.tick()
    futures[-1].set_result(reply())
    runtime.tick()
    pending = futures[-1]
    runtime.shutdown()
    assert host.messages[-1] is False
    pending.set_result(reply())
    assert host.messages[-1] is False


def test_shutdown_after_ros_context_invalidation_never_publishes():
    runtime, host, _, _, futures, _ = sensor()
    host.context = SimpleNamespace(ok=lambda: False)
    runtime.tick()
    before = list(host.messages)
    runtime.shutdown()
    assert host.messages == before
    assert runtime._pending is None
    futures[-1].set_result(reply())
    assert host.messages == before


def test_timed_out_world_queries_remove_client_pending_requests_before_retry():
    runtime, host, wall, client, futures, _ = sensor()
    removed = []
    client.remove_pending_request = removed.append
    runtime.tick()
    old = futures[-1]
    wall[0] += 0.6
    runtime.tick()
    assert removed == [old]
    assert len(futures) == 2 and host.messages[-1] is False
    runtime.shutdown()
    assert removed == futures


@pytest.mark.parametrize(
    "failure", ["absent", "nan", "wrong_yaw", "exception", "stale", "unavailable"]
)
def test_missing_invalid_or_stale_simulator_evidence_publishes_false(failure):
    runtime, host, wall, client, futures, _ = sensor()
    runtime.tick()
    futures[-1].set_result(reply())
    assert host.messages[-1] is True
    runtime.tick()
    pending = futures[-1]
    if failure == "stale":
        wall[0] += 0.6
        runtime.tick()
        assert host.messages[-1] is False
        pending.set_result(reply())
        assert host.messages[-1] is False  # Late evidence cannot resurrect contact.
    elif failure == "unavailable":
        pending.set_result(reply())
        client.service_is_ready = lambda: False
        runtime.tick()
    elif failure == "exception":
        pending.set_exception(RuntimeError("Gazebo disconnected"))
    else:
        value = reply(success=failure != "absent")
        if failure == "nan":
            value.state.pose.position.x = float("nan")
        if failure == "wrong_yaw":
            value.state.pose.orientation.z, value.state.pose.orientation.w = 1.0, 0.0
        pending.set_result(value)
    assert host.messages[-1] is False


@pytest.mark.parametrize(
    "shutdown, interrupted, handled",
    [(True, True, True), (False, True, False), (True, False, False)],
)
def test_main_handles_only_shutdown_interrupted_response_conversion(
    monkeypatch, shutdown, interrupted, handled
):
    """A SIGINT during generated response conversion must not make exit fail."""
    from types import SimpleNamespace
    import factory_simulation.dock_contact as contact

    live = [True]
    destroyed = []
    node = SimpleNamespace(
        context=SimpleNamespace(ok=lambda: live[0]),
        destroy_node=lambda: destroyed.append(True),
    )
    monkeypatch.setattr(contact, "DockContactNode", lambda: node)
    monkeypatch.setattr(contact.rclpy, "init", lambda: None)
    monkeypatch.setattr(contact.rclpy, "try_shutdown", lambda: None)

    def spin(_):
        live[0] = not shutdown
        error = SystemError("response conversion interrupted")
        if interrupted:
            raise error from KeyboardInterrupt()
        raise error

    monkeypatch.setattr(contact.rclpy, "spin", spin)
    if handled:
        contact.main()
    else:
        with pytest.raises(SystemError, match="response conversion interrupted"):
            contact.main()
    assert destroyed == [True]


@pytest.mark.parametrize(
    "shutdown,message,handled",
    [
        (
            True,
            "failed to initialize wait set: the given context is not valid, either "
            "rcl_init() was not called or rcl_shutdown() was called., "
            "at ./src/rcl/wait.c:130",
            True,
        ),
        (
            False,
            "failed to initialize wait set: the given context is not valid, either "
            "rcl_init() was not called or rcl_shutdown() was called., "
            "at ./src/rcl/wait.c:130",
            False,
        ),
        (True, "failed to initialize wait set: allocation failure", False),
    ],
)
def test_main_handles_only_exact_inactive_context_wait_set_error(
    monkeypatch, shutdown, message, handled
):
    """An active or unrelated native error must remain a failed process."""
    import factory_simulation.dock_contact as contact
    from rclpy.impl.implementation_singleton import rclpy_implementation

    live = [True]
    destroyed, shutdown_calls = [], []
    node = SimpleNamespace(
        context=SimpleNamespace(ok=lambda: live[0]),
        destroy_node=lambda: destroyed.append(True),
    )
    monkeypatch.setattr(contact, "DockContactNode", lambda: node)
    monkeypatch.setattr(contact.rclpy, "init", lambda: None)
    monkeypatch.setattr(
        contact.rclpy, "try_shutdown", lambda: shutdown_calls.append(True)
    )

    def spin(_):
        live[0] = not shutdown
        raise rclpy_implementation.RCLError(message)

    monkeypatch.setattr(contact.rclpy, "spin", spin)
    if handled:
        contact.main()
    else:
        with pytest.raises(rclpy_implementation.RCLError) as caught:
            contact.main()
        assert str(caught.value) == message
    assert destroyed == [True] and shutdown_calls == [True]
