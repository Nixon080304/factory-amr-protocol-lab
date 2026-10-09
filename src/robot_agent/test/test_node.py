"""Run production ROS registrations/callbacks without replacing robot policy."""

import importlib
import json
from pathlib import Path
import sys
from types import SimpleNamespace

from builtin_interfaces.msg import Time
from factory_interfaces.msg import ProtocolEvent, RobotState
from factory_interfaces.srv import EstimateMissionCost
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import pytest
from rclpy.clock import ClockType
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from test_agent_adapter import Paths


def api():
    try:
        return importlib.import_module("robot_agent.node")
    except ModuleNotFoundError:
        pytest.fail("robot agent ROS boundary is missing")


@pytest.mark.parametrize(
    "shutdown, message, handled",
    [
        (
            True,
            "Unable to convert call argument to Python object (compile in debug mode for details)",
            True,
        ),
        (
            False,
            "Unable to convert call argument to Python object (compile in debug mode for details)",
            False,
        ),
        (True, "unrelated robot agent failure", False),
    ],
)
def test_main_preserves_errors_except_exact_shutdown_message_conversion(
    monkeypatch, shutdown, message, handled
):
    module = api()
    live, cleanup = [True], []

    def spin():
        live[0] = not shutdown
        raise RuntimeError(message)

    node = SimpleNamespace(destroy_node=lambda: cleanup.append("node"))
    executor = SimpleNamespace(
        add_node=lambda n: None,
        spin=spin,
        shutdown=lambda: cleanup.append("executor"),
    )
    monkeypatch.setattr(module, "RobotAgentNode", lambda: node)
    monkeypatch.setattr(module, "SingleThreadedExecutor", lambda: executor)
    monkeypatch.setattr(module.rclpy, "init", lambda **kwargs: None)
    monkeypatch.setattr(module.rclpy, "ok", lambda: live[0])
    monkeypatch.setattr(module.rclpy, "try_shutdown", lambda: cleanup.append("context"))
    if handled:
        module.main()
    else:
        with pytest.raises(RuntimeError, match=message.split(" (")[0]):
            module.main()
    assert cleanup == ["executor", "node", "context"]


def test_real_dock_shutdown_after_context_shutdown_does_not_publish_dead_goal():
    import time

    from factory_interfaces.action import DockRobot
    from geometry_msgs.msg import PoseStamped
    from rclpy.action import ActionClient
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from test_docking import rig

    context = Context()
    context.init()
    host = Node("dock_shutdown", namespace="/cart_1", context=context)
    executor = SingleThreadedExecutor(context=context)
    client, docking, server_destroyed = None, None, False
    try:
        agent, _, wire, *_ = rig()
        docking = api().DockRuntime(
            host, agent, wire, "charger", (2, -3, 0), (3, -3, 0)
        )
        client = ActionClient(host, DockRobot, "factory/dock_robot")
        executor.add_node(host)

        def pose(x):
            message = PoseStamped()
            message.header.frame_id = "floor/cart_1/map"
            message.pose.position.x, message.pose.position.y = float(x), -3.0
            message.pose.orientation.w = 1.0
            return message

        assert client.wait_for_server(timeout_sec=2.0)
        reply = client.send_goal_async(
            DockRobot.Goal(
                dock_id="charger",
                staging_pose=pose(2),
                charging_pose=pose(3),
                target_percent=80.0,
            )
        )
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and (
            not reply.done() or docking._handle is None
        ):
            executor.spin_once(timeout_sec=0.01)
        assert reply.done() and reply.result().accepted and docking.controller.active
        context.try_shutdown()
        docking.shutdown()
        server_destroyed = True
        assert not docking.controller.active and agent.mode == "RECOVERY_REQUIRED"
    finally:
        if docking is not None and not server_destroyed:
            docking.server.destroy()
        if client is not None:
            client.destroy()
        executor.shutdown()
        host.destroy_node()
        context.try_shutdown()


@pytest.mark.parametrize(
    "shutdown, message, handled",
    [
        (
            True,
            "Failed get goal status array: feedback publisher is invalid, at ./src/rcl_action/action_server.c:919",
            True,
        ),
        (
            False,
            "Failed get goal status array: feedback publisher is invalid, at ./src/rcl_action/action_server.c:919",
            False,
        ),
        (True, "unrelated action server failure", False),
    ],
)
def test_dock_shutdown_preserves_errors_except_exact_inactive_goal_status(
    shutdown, message, handled
):
    from rclpy.impl.implementation_singleton import rclpy_implementation
    from test_docking import rig

    host = Host("/cart_1")
    host.context = SimpleNamespace(ok=lambda: not shutdown)
    agent, _, wire, *_ = rig()
    docking = api().DockRuntime(
        host,
        agent,
        wire,
        "charger",
        (2, -3, 0),
        (3, -3, 0),
        server_factory=lambda *args, **kwargs: SimpleNamespace(destroy=lambda: None),
    )

    def execute():
        raise rclpy_implementation.RCLError(message)

    handle = SimpleNamespace(
        request=SimpleNamespace(target_percent=80),
        publish_feedback=lambda _: None,
        execute=execute,
    )
    docking.accepted(handle)
    if handled:
        docking.shutdown()
    else:
        with pytest.raises(rclpy_implementation.RCLError, match=message.split(":")[0]):
            docking.shutdown()
    assert agent.mode == "RECOVERY_REQUIRED" and not docking.controller.active


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class Host:
    def __init__(self, namespace):
        self.namespace = namespace
        self.publishers, self.subscriptions, self.services, self.timers = {}, {}, {}, []
        self.sim_sec = 42
        self.subscription_qos = {}

    def resolve(self, name):
        return name if name.startswith("/") else self.namespace + "/" + name

    def create_publisher(self, message_type, name, qos):
        publisher = Publisher()
        self.publishers[self.resolve(name)] = (message_type, publisher)
        return publisher

    def create_subscription(self, message_type, name, callback, qos):
        self.subscriptions[self.resolve(name)] = callback
        self.subscription_qos[self.resolve(name)] = qos
        return object()

    def create_service(self, message_type, name, callback, **kwargs):
        self.services[self.resolve(name)] = callback
        return object()

    def create_client(self, *args, **kwargs):
        return object()

    def create_timer(self, period, callback, clock):
        self.timers.append((period, callback, clock.clock_type))
        return object()

    def get_clock(self):
        return SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=self.sim_sec))
        )


def runtime(namespace="/cart_1"):
    host, wall, paths = Host(namespace), [100.0], Paths()
    config = api().AgentConfig(
        "cart_1", "floor/cart_1/", {"pick": (-3, 1, 0), "drop": (3, 1, 0)}, 80
    )
    agent = api().AgentRuntime(host, config, paths, clock=lambda: wall[0])
    return host, agent, paths, wall


def test_initial_idle_amcl_pose_uses_retained_reliable_observation():
    from rclpy.qos import DurabilityPolicy, ReliabilityPolicy

    host, _, _, _ = runtime()
    qos = host.subscription_qos[host.resolve("amcl_pose")]
    assert qos.durability == DurabilityPolicy.TRANSIENT_LOCAL
    assert qos.reliability == ReliabilityPolicy.RELIABLE
    assert qos.depth == 1


@pytest.mark.parametrize(
    "value, want, invalid",
    [
        (None, None, False),
        ([3.5, -3.4, 3.141592653589793], (3.5, -3.4, 3.141592653589793), False),
        ([3.5, -3.4], None, True),
        ([3.5, float("nan"), 0.0], None, True),
    ],
)
def test_optional_exit_parameter_is_typed_empty_or_finite_pose(value, want, invalid):
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.parameter import Parameter

    context = Context()
    context.init()
    host = None
    try:
        overrides = [] if value is None else [Parameter("dock.exit_pose", value=value)]
        host = Node(
            "exit_configuration", context=context, parameter_overrides=overrides
        )
        if invalid:
            with pytest.raises(ValueError, match="exit"):
                api().optional_exit_pose(host)
        else:
            assert api().optional_exit_pose(host) == want
            parameter = host.get_parameter("dock.exit_pose")
            assert parameter.type_ is Parameter.Type.DOUBLE_ARRAY
            assert parameter.value == ([] if value is None else value)
            assert host.describe_parameter("dock.exit_pose").read_only
    finally:
        if host is not None:
            host.destroy_node()
        context.try_shutdown()


def sensors(host):
    odom = Odometry()
    odom.header.frame_id = "floor/cart_1/odom"
    odom.header.stamp.sec = 1
    odom.child_frame_id = "floor/cart_1/base_footprint"
    odom.pose.pose.orientation.w = 1.0
    host.subscriptions[host.resolve("odom")](odom)
    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = "floor/cart_1/map"
    pose.header.stamp.sec = 1
    pose.pose.pose.orientation.w = 1.0
    host.subscriptions[host.resolve("amcl_pose")](pose)
    host.subscriptions["/factory/payload_state"](
        String(
            data=json.dumps(
                {
                    "robot_id": "cart_1",
                    "mission_id": "",
                    "state": "AT_ASSEMBLY",
                    "transfer_kind": "",
                    "cycle_counter": None,
                }
            )
        )
    )


@pytest.mark.parametrize("namespace", ["/cart_1", "/warehouse/cart_2"])
def test_namespace_topics_steady_cadence_and_ros_stamps_under_paused_clock(namespace):
    host, runtime_, _, wall = runtime(namespace)
    assert host.publishers[namespace + "/factory/robot_state"][0] is RobotState
    assert host.publishers[namespace + "/battery_state"][0] is BatteryState
    assert namespace + "/factory/estimate_mission_cost" in host.services
    assert (0.5, ClockType.STEADY_TIME) in [(p, kind) for p, _, kind in host.timers]
    sensors(host)
    heartbeat = next(callback for period, callback, _ in host.timers if period == 0.5)
    for _ in range(4):
        heartbeat()
        wall[0] += 0.5
    states = host.publishers[namespace + "/factory/robot_state"][1].messages
    batteries = host.publishers[namespace + "/battery_state"][1].messages
    assert len(states) == len(batteries) == 4
    assert {s.stamp.sec for s in states} == {42}
    assert {b.header.stamp.sec for b in batteries} == {42}
    assert states[0].mode == "AVAILABLE" and states[0].frame_id == "floor/cart_1/map"
    assert batteries[0].percentage == pytest.approx(states[0].battery_percent / 100)
    wall[0] += 3
    heartbeat()
    assert states[-1].mode == "UNHEALTHY" and "stale" in states[-1].health_detail
    assert batteries[-1].percentage < batteries[0].percentage


def test_malformed_sensor_and_foreign_event_do_not_create_pose_or_payload():
    host, runtime_, _, _ = runtime()
    sensors(host)
    bad = PoseWithCovarianceStamped()
    bad.header.frame_id = "floor/cart_1/map"
    bad.pose.pose.position.x = float("nan")
    host.subscriptions[host.resolve("amcl_pose")](bad)
    host.subscriptions["/factory/protocol_events"](
        ProtocolEvent(
            robot_id="foreign",
            mission_id="m1",
            protocol="ROS",
            event="state_changed",
            detail="LOADING",
        )
    )
    runtime_.heartbeat()
    state = host.publishers[host.resolve("factory/robot_state")][1].messages[-1]
    assert state.mode == "UNHEALTHY" and not state.frame_id
    assert state.payload_state == "EMPTY" and not state.mission_id


def test_cost_service_returns_immediately_while_paths_populate_fresh_cache():
    host, runtime_, paths, _ = runtime()
    sensors(host)
    callback = host.services[host.resolve("factory/estimate_mission_cost")]
    response = callback(
        EstimateMissionCost.Request(
            mission_id="m1", pickup_station="pick", dropoff_station="drop", part="motor"
        ),
        EstimateMissionCost.Response(),
    )
    assert isinstance(response, EstimateMissionCost.Response)
    assert not response.feasible and "pending" in response.reason
    paths.finish(0, 7)
    paths.finish(1, 8)
    response = callback(
        EstimateMissionCost.Request(
            mission_id="m1", pickup_station="pick", dropoff_station="drop", part="motor"
        ),
        EstimateMissionCost.Response(),
    )
    assert response.feasible and response.path_cost == 7
    assert response.predicted_final_battery == 76.5
    assert len(paths.calls) == 2


def test_cost_timeout_is_immediate_infeasible_without_pending_service_coroutines():
    host, runtime_, paths, wall = runtime()
    sensors(host)
    callback = host.services[host.resolve("factory/estimate_mission_cost")]
    request = EstimateMissionCost.Request(
        mission_id="m1", pickup_station="pick", dropoff_station="drop", part="motor"
    )
    first = callback(request, EstimateMissionCost.Response())
    second = callback(request, EstimateMissionCost.Response())
    assert not first.feasible and not second.feasible
    assert len(paths.calls) == 1
    wall[0] += 0.8
    next(t for p, t, _ in host.timers if p == 0.02)()
    third = callback(request, EstimateMissionCost.Response())
    assert not third.feasible and "timeout" in third.reason
    paths.finish(0, 1)
    assert sorted(paths.cancelled) == [0]
    assert len(paths.calls) == 1


def test_dock_runtime_registers_action_and_requires_real_contact_before_charge(
    monkeypatch,
):
    from factory_interfaces.action import DockRobot
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Bool
    from test_docking import Wire

    module = api()
    assert hasattr(module, "DockRuntime"), "production DockRobot server seam is missing"
    host, runtime_, _, wall = runtime("/warehouse/cart_1")
    sensors(host)
    registrations = []

    def action_server(node, action, name, execute, **callbacks):
        registrations.append((host.resolve(name), action, execute, callbacks))
        return SimpleNamespace(destroy=lambda: None)

    wire = Wire()
    docking = module.DockRuntime(
        host,
        runtime_.adapter,
        wire,
        "charger",
        (2, 0, 0),
        (3, 0, 0),
        server_factory=action_server,
    )
    assert registrations[0][:2] == ("/warehouse/cart_1/factory/dock_robot", DockRobot)
    assert "/warehouse/cart_1/factory/dock_contact" in host.subscriptions

    def pose(x):
        message = PoseStamped()
        message.header.frame_id = "floor/cart_1/map"
        message.pose.position.x = float(x)
        message.pose.orientation.w = 1.0
        return message

    goal = DockRobot.Goal(
        dock_id="charger",
        staging_pose=pose(2),
        charging_pose=pose(3),
        target_percent=80.0,
    )
    callbacks = registrations[0][3]
    assert callbacks["goal_callback"](goal) == module.GoalResponse.ACCEPT
    assert callbacks["goal_callback"](goal) == module.GoalResponse.REJECT
    feedback, status, finished = [], [], []
    handle = SimpleNamespace(
        request=goal,
        publish_feedback=feedback.append,
        execute=lambda: finished.append(True),
        is_cancel_requested=False,
        succeed=lambda: status.append("succeeded"),
        abort=lambda: status.append("aborted"),
        canceled=lambda: status.append("cancelled"),
    )
    callbacks["handle_accepted_callback"](handle)
    wire.moves[0][2](True, "")
    runtime_.adapter.localization(2, 2, 0, 0, "floor/cart_1/map")
    docking.controller.tick()
    wire.reply(granted=True, lease_id="real-lease", lease_ttl_sec=10.0)
    wire.moves[1][2](True, "")
    runtime_.adapter.localization(3, 3, 0, 0, "floor/cart_1/map")
    docking.controller.tick()
    runtime_.heartbeat()
    assert runtime_.adapter.mode == "DOCKING"
    assert (
        host.publishers[host.resolve("battery_state")][1]
        .messages[-1]
        .power_supply_status
        == BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
    )
    runtime_.adapter.energy.battery_percent = 79.95
    host.subscriptions[host.resolve("factory/dock_contact")](Bool(data=True))
    assert runtime_.adapter.mode == "CHARGING"
    runtime_.heartbeat()
    assert (
        host.publishers[host.resolve("battery_state")][1]
        .messages[-1]
        .power_supply_status
        == BatteryState.POWER_SUPPLY_STATUS_CHARGING
    )
    wall[0] += 0.1
    docking.controller.tick()
    wire.moves[2][2](True, "")
    runtime_.adapter.localization(4, 0, 0, 0, "floor/cart_1/map")
    docking.controller.tick()
    wire.reply(released=True)
    assert finished == [True]
    reply = registrations[0][2](handle)
    assert reply.success and status == ["succeeded"]


def test_nav2_dock_transport_cancels_goal_accepted_after_cancel():
    from rclpy.task import Future

    module = api()
    assert hasattr(module, "DockTransport"), "production docking transport is missing"
    host = Host("/cart_1")
    goal_reply, result_reply, cancel_reply = Future(), Future(), Future()
    goals, results, cancellations = [], [], []

    def send(goal):
        goals.append(goal)
        return goal_reply

    client = SimpleNamespace(server_is_ready=lambda: True, send_goal_async=send)
    transport = module.DockTransport(host, client, {})
    cancel = transport.navigate(
        (2, 3, 0), "floor/cart_1/map", lambda *reply: results.append(reply)
    )
    cancel()

    def cancel_goal():
        cancellations.append(True)
        return cancel_reply

    goal_reply.set_result(
        SimpleNamespace(
            accepted=True,
            cancel_goal_async=cancel_goal,
            get_result_async=lambda: result_reply,
        )
    )
    assert cancellations == [True] and results == []
    result_reply.set_result(SimpleNamespace(status=5))
    assert results == [(False, "navigation cancelled", True)]
    assert goals[0].pose.header.frame_id == "floor/cart_1/map"


@pytest.mark.parametrize("phase", ["STAGING", "ENTERING", "EXITING"])
@pytest.mark.parametrize("cancel_requested", [False, True])
def test_dock_navigation_result_exception_is_unknown_motion(phase, cancel_requested):
    from rclpy.task import Future
    from test_docking import rig, start

    agent, _, wire, outcomes, _, controller = r = rig()
    pending, cancellations = [], []

    def cancel_goal():
        cancellations.append(True)
        return Future()

    def send(goal):
        accepted, result = Future(), Future()
        pending.append(result)
        accepted.set_result(
            SimpleNamespace(
                accepted=True,
                cancel_goal_async=cancel_goal,
                get_result_async=lambda: result,
            )
        )
        return accepted

    transport = api().DockTransport(
        Host("/cart_1"),
        SimpleNamespace(server_is_ready=lambda: True, send_goal_async=send),
        {},
    )
    wire.navigate = transport.navigate
    start(r)

    def terminal_at(pose):
        stamp = agent._pose_stamp + 1
        agent.odometry(stamp, 0, 0, agent.frame_prefix + "odom")
        pending[-1].set_result(SimpleNamespace(status=4))
        agent.localization(stamp, *pose, agent.frame_prefix + "map")
        controller.tick()

    if phase != "STAGING":
        terminal_at(controller.staging)
        wire.reply(granted=True, lease_id="lease-1", lease_ttl_sec=10.0)
    if phase == "EXITING":
        terminal_at(controller.charging)
        controller.contact(True)
        controller.cancel()
    elif cancel_requested:
        controller.cancel()
    before = len(pending)
    pending[-1].set_exception(RuntimeError("result transport lost"))
    assert agent.mode == "RECOVERY_REQUIRED"
    assert controller.state == "RECOVERY_REQUIRED" and not controller.active
    assert len(pending) == before  # Unknown old motion never permits another goal.
    assert len(outcomes) == 1 and outcomes[0].error_code == "NAVIGATION_UNKNOWN"
    assert cancellations == [True]  # Best effort stop, never terminal proof.
    assert not any(operation == "release" for operation, *_ in wire.requests)


def test_immediate_navigation_result_exception_still_requests_best_effort_stop():
    from rclpy.task import Future
    from test_docking import rig, start

    r = rig()
    agent, _, wire, outcomes, _, _ = r
    accepted, result, cancellations = Future(), Future(), []
    result.set_exception(RuntimeError("immediate result failure"))
    accepted.set_result(
        SimpleNamespace(
            accepted=True,
            get_result_async=lambda: result,
            cancel_goal_async=lambda: (cancellations.append(True), Future())[1],
        )
    )
    wire.navigate = (
        api()
        .DockTransport(
            Host("/cart_1"),
            SimpleNamespace(
                server_is_ready=lambda: True, send_goal_async=lambda _: accepted
            ),
            {},
        )
        .navigate
    )
    start(r)
    assert cancellations == [True]
    assert (
        agent.mode == "RECOVERY_REQUIRED"
        and outcomes[0].error_code == "NAVIGATION_UNKNOWN"
    )


def test_missing_cancel_wait_service_is_not_ownership_clearance():
    from robot_agent.docking import DockKey

    replies = []
    host = Host("/cart_1")
    module = api()
    transport = module.DockTransport(
        host, None, {"cancel_wait": SimpleNamespace(service_is_ready=lambda: False)}
    )
    transport.resource(
        "cancel_wait", DockKey("cart_1", "dock-1", "charger"), replies.append
    )
    assert replies[0].reconciliation_required is True


def test_actual_demo_parameters_construct_namespaced_agent_with_unprefixed_frames(
    monkeypatch,
):
    from launch import LaunchContext
    from launch_ros.actions import Node as LaunchNode
    from launch_ros.utilities import evaluate_parameters

    root = Path(__file__).resolve().parents[2]
    spec = importlib.util.spec_from_file_location(
        "demo_constructor", root / "factory_bringup/launch/demo.launch.py"
    )
    demo = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(demo)
    monkeypatch.setattr(
        demo, "get_package_share_directory", lambda name: str(root / name)
    )
    description = demo.generate_launch_description()
    launch_node = next(
        n
        for n in description.entities
        if isinstance(n, LaunchNode) and n.node_package == "robot_agent"
    )
    context = LaunchContext()
    launch_node._perform_substitutions(context)
    parameters = evaluate_parameters(context, launch_node._Node__parameters)[0]
    module = api()
    for name in ("publishers", "subscriptions", "services", "timers"):
        monkeypatch.setattr(module.Node, name, None)
    monkeypatch.setattr(
        module.Node,
        "__init__",
        lambda self, *args, **kwargs: Host.__init__(self, kwargs["namespace"]),
    )
    for name in (
        "create_publisher",
        "create_subscription",
        "create_service",
        "create_client",
        "create_timer",
        "get_clock",
    ):
        monkeypatch.setattr(module.Node, name, getattr(Host, name))
    monkeypatch.setattr(module.Node, "get_namespace", lambda self: self.namespace)
    monkeypatch.setattr(module.Node, "resolve", Host.resolve, raising=False)
    monkeypatch.setattr(
        module.Node,
        "declare_parameter",
        lambda self, name, default, *args: SimpleNamespace(
            value=parameters.get(
                name, None if default is module.Parameter.Type.DOUBLE_ARRAY else default
            )
        ),
    )
    monkeypatch.setattr(
        module.Node,
        "set_parameters",
        lambda self, values: [SimpleNamespace(successful=True)],
    )
    monkeypatch.setattr(module.Node, "set_descriptor", lambda self, *args: None)
    monkeypatch.setattr(module, "ActionClient", lambda *args: object())
    monkeypatch.setattr(
        module,
        "ActionServer",
        lambda *args, **kwargs: SimpleNamespace(destroy=lambda: None),
    )
    original = module.DockRuntime
    monkeypatch.setattr(
        module,
        "DockRuntime",
        lambda *args, **kwargs: original(
            *args, **kwargs, server_factory=module.ActionServer
        ),
    )
    node = module.RobotAgentNode(namespace=launch_node.expanded_node_namespace)
    assert node.runtime.adapter.frame_prefix == ""
    assert node.namespace + "/factory/robot_state" in node.publishers
    parameters.pop("legacy_unprefixed_frames", None)
    with pytest.raises(ValueError, match="namespaced robots require frame_prefix"):
        module.RobotAgentNode(namespace=node.namespace)


@pytest.mark.parametrize("legacy", [False, True])
def test_real_legacy_nav2_clients_use_root_without_cross_namespace_discovery(legacy):
    import time
    import rclpy
    from rclpy.action import ActionServer
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from nav2_msgs.action import ComputePathToPose, NavigateToPose
    from std_srvs.srv import Empty

    context = Context()
    rclpy.init(context=context, domain_id=78)
    executor = SingleThreadedExecutor(context=context)
    agent, peer = None, None
    servers, requests = [], []
    try:
        peer = rclpy.create_node("root_v1_nav2", context=context)

        def execute(handle, action):
            requests.append(action.__name__)
            handle.succeed()
            return action.Result()

        for action, name in (
            (ComputePathToPose, "/compute_path_to_pose"),
            (NavigateToPose, "/navigate_to_pose"),
        ):
            servers.append(
                ActionServer(
                    peer,
                    action,
                    name,
                    lambda handle, action=action: execute(handle, action),
                )
            )
        peer.create_service(
            Empty, "/request_nomotion_update", lambda request, response: response
        )
        agent = api().RobotAgentNode(
            namespace="/cart",
            context=context,
            parameter_overrides=[
                Parameter("robot_id", value="cart"),
                Parameter("frame_prefix", value="" if legacy else "cart/"),
                Parameter("legacy_unprefixed_frames", value=legacy),
            ],
        )
        executor.add_node(agent)
        executor.add_node(peer)
        until = time.monotonic() + 2
        while time.monotonic() < until:
            executor.spin_once(timeout_sec=0.02)
        assert agent.path_client.server_is_ready() is legacy
        assert agent.navigation_client.server_is_ready() is legacy
        localization = agent.docking.controller.transport.localization_client
        assert localization.service_is_ready() is legacy
        if legacy:
            for client, action in (
                (agent.path_client, ComputePathToPose),
                (agent.navigation_client, NavigateToPose),
            ):
                future = client.send_goal_async(action.Goal())
                until = time.monotonic() + 2
                while not future.done() and time.monotonic() < until:
                    executor.spin_once(timeout_sec=0.02)
                assert future.done() and future.result().accepted
                result = future.result().get_result_async()
                while not result.done() and time.monotonic() < until:
                    executor.spin_once(timeout_sec=0.02)
                assert result.done() and result.result().status == 4
            assert requests == ["ComputePathToPose", "NavigateToPose"]
            refresh = localization.call_async(Empty.Request())
            until = time.monotonic() + 2
            while not refresh.done() and time.monotonic() < until:
                executor.spin_once(timeout_sec=0.02)
            assert refresh.done() and refresh.result() is not None
        else:
            assert requests == []
    finally:
        for server in servers:
            server.destroy()
        executor.shutdown()
        if agent is not None:
            agent.destroy_node()
        if peer is not None:
            peer.destroy_node()
        context.shutdown()


@pytest.mark.parametrize(
    "scenario", ["success", "rejected", "failed", "timeout", "missing_pose"]
)
def test_real_ros_agent_cost_with_robot_local_nav2_action(scenario):
    # Exercise the real executor/service/action chain; never hide DDS denial.
    import rclpy
    from rclpy.action import ActionServer, GoalResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.clock import Clock
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from nav2_msgs.action import ComputePathToPose
    from nav_msgs.msg import Path as RosPath
    from geometry_msgs.msg import PoseStamped
    from rclpy.qos import DurabilityPolicy, QoSProfile
    import time

    context = Context()
    context.init()
    executor = SingleThreadedExecutor(context=context)
    nodes, server = [], None
    try:
        agent = api().RobotAgentNode(
            namespace="/cart_1",
            context=context,
            parameter_overrides=[
                Parameter("robot_id", value="cart_1"),
                Parameter("frame_prefix", value="floor/cart_1/"),
                Parameter("use_sim_time", value=True),
            ],
        )
        nodes.append(agent)
        peer = rclpy.create_node(
            "planner_and_sensors", namespace="/cart_1", context=context
        )
        nodes.append(peer)
        goals = []

        def execute(handle):
            goals.append(handle.request)
            message = RosPath()
            message.header.frame_id = "floor/cart_1/map"
            start = handle.request.start if handle.request.use_start else PoseStamped()
            start.header.frame_id = message.header.frame_id
            start.pose.orientation.w = 1.0
            message.poses = [start, handle.request.goal]
            if scenario == "failed" or handle in preempted:
                handle.abort()
            else:
                handle.succeed()
            return ComputePathToPose.Result(path=message)

        # A never-completed action execute callback is deferred rather than blocking.
        handles = []
        waiting, preempted = [], []

        def accepted(handle):
            # Model Humble's single current goal rather than independent goals.
            preempted.extend(previous for previous in handles if previous.is_active)
            handles.append(handle)
            waiting.append(handle)

        def run_planner():
            if scenario == "timeout":
                return
            for handle in tuple(waiting):
                waiting.remove(handle)
                handle.execute()

        server = ActionServer(
            peer,
            ComputePathToPose,
            "compute_path_to_pose",
            execute,
            goal_callback=lambda _: (
                GoalResponse.REJECT if scenario == "rejected" else GoalResponse.ACCEPT
            ),
            handle_accepted_callback=accepted,
            callback_group=ReentrantCallbackGroup(),
        )
        peer.create_timer(
            0.04, run_planner, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )
        odom_pub = peer.create_publisher(Odometry, "odom", 10)
        pose_pub = peer.create_publisher(
            PoseWithCovarianceStamped,
            "amcl_pose",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        payload_pub = peer.create_publisher(
            String,
            "/factory/payload_state",
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        sequence = [1]

        def publish():
            odom = Odometry()
            odom.header.frame_id, odom.child_frame_id = (
                "floor/cart_1/odom",
                "floor/cart_1/base_footprint",
            )
            odom.header.stamp.sec = sequence[0]
            odom.pose.pose.orientation.w = 1.0
            odom_pub.publish(odom)
            if scenario != "missing_pose":
                pose = PoseWithCovarianceStamped()
                pose.header.frame_id, pose.header.stamp.sec = (
                    "floor/cart_1/map",
                    sequence[0],
                )
                pose.pose.pose.orientation.w = 1.0
                pose_pub.publish(pose)
            sequence[0] += 1
            payload_pub.publish(
                String(
                    data=json.dumps(
                        {
                            "robot_id": "cart_1",
                            "mission_id": "",
                            "state": "AT_ASSEMBLY",
                            "transfer_kind": "",
                            "cycle_counter": None,
                        }
                    )
                )
            )

        peer.create_timer(0.05, publish, clock=Clock(clock_type=ClockType.STEADY_TIME))
        client = peer.create_client(
            EstimateMissionCost, "factory/estimate_mission_cost"
        )
        for node in nodes:
            executor.add_node(node)
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline and (
            not client.service_is_ready()
            or not agent.path_client.server_is_ready()
            or agent.runtime.adapter._odom_receipt is None
            or agent.runtime.adapter.payload_state != "EMPTY"
            or (scenario != "missing_pose" and agent.runtime.adapter.pose is None)
        ):
            executor.spin_once(timeout_sec=0.02)
        assert client.service_is_ready() and agent.path_client.server_is_ready()
        started = time.monotonic()
        future = client.call_async(
            EstimateMissionCost.Request(
                mission_id="m1",
                pickup_station="assembly",
                dropoff_station="inspection",
                part="motor",
            )
        )
        while not future.done() and time.monotonic() - started < 1:
            executor.spin_once(timeout_sec=0.01)
        assert future.done() and time.monotonic() - started < 0.25
        response = future.result()
        assert not response.feasible
        if scenario != "missing_pose":
            assert "pending" in response.reason
            deadline = time.monotonic() + 1.0
            while "pending" in response.reason and time.monotonic() < deadline:
                # FleetAdapter also retries an explicit infeasible candidate.
                for _ in range(5):
                    executor.spin_once(timeout_sec=0.02)
                future = client.call_async(
                    EstimateMissionCost.Request(
                        mission_id="m1",
                        pickup_station="assembly",
                        dropoff_station="inspection",
                        part="motor",
                    )
                )
                while not future.done() and time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=0.01)
                assert future.done()
                response = future.result()
        assert response.feasible is (scenario == "success")
        if scenario == "success":
            assert preempted == []
            assert len(goals) == 2 and {g.goal.header.frame_id for g in goals} == {
                "floor/cart_1/map"
            }
            assert response.path_cost > 3
        else:
            assert response.reason
    finally:
        executor.shutdown()
        if server is not None:
            server.destroy()
        for node in reversed(nodes):
            node.destroy_node()
        context.try_shutdown()


@pytest.mark.parametrize("namespace", ["/cart_1", "/warehouse/cart_2"])
def test_real_ros_agent_paused_clock_battery_and_namespace(namespace):
    # This mandatory external DDS test never skips a restricted environment.
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    import time

    context = Context()
    context.init()
    agent = None
    executor = SingleThreadedExecutor(context=context)
    try:
        agent = api().RobotAgentNode(
            namespace=namespace,
            context=context,
            parameter_overrides=[
                Parameter("robot_id", value="cart_1"),
                Parameter("frame_prefix", value="floor/cart_1/"),
                Parameter("use_sim_time", value=True),
            ],
        )
        observer = rclpy.create_node("agent_observer", context=context)
        states, batteries = [], []
        observer.create_subscription(
            RobotState, namespace + "/factory/robot_state", states.append, 10
        )
        observer.create_subscription(
            BatteryState, namespace + "/battery_state", batteries.append, 10
        )
        executor.add_node(agent)
        executor.add_node(observer)
        deadline = time.monotonic() + 2.2
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        assert len(states) >= 3 and len(batteries) >= 3
        assert {s.stamp.sec for s in states} == {0}
        assert all(s.mode == "UNHEALTHY" for s in states)
        observer.destroy_node()
    finally:
        executor.shutdown()
        if agent is not None:
            agent.destroy_node()
        context.try_shutdown()


def test_real_ros_dock_action_is_namespaced_and_rejects_unhealthy_goal():
    import rclpy
    from factory_interfaces.action import DockRobot
    from rclpy.action import ActionClient
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    import time

    context = Context()
    context.init()
    executor = SingleThreadedExecutor(context=context)
    nodes, client = [], None
    try:
        agent = api().RobotAgentNode(
            namespace="/warehouse/cart_1",
            context=context,
            parameter_overrides=[
                Parameter("robot_id", value="cart_1"),
                Parameter("frame_prefix", value="floor/cart_1/"),
            ],
        )
        nodes.append(agent)
        peer = rclpy.create_node("dock_observer", context=context)
        nodes.append(peer)
        for node in nodes:
            executor.add_node(node)
        client = ActionClient(peer, DockRobot, "/warehouse/cart_1/factory/dock_robot")
        deadline = time.monotonic() + 5
        while not client.server_is_ready() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert client.server_is_ready()
        future = client.send_goal_async(
            DockRobot.Goal(dock_id="dock_01", target_percent=80.0)
        )
        while not future.done() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert future.done() and not future.result().accepted
        assert not agent.docking.controller.active
    finally:
        executor.shutdown()
        if client is not None:
            client.destroy()
        for node in reversed(nodes):
            node.destroy_node()
        context.try_shutdown()
