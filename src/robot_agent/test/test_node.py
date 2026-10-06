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

    def resolve(self, name):
        return name if name.startswith("/") else self.namespace + "/" + name

    def create_publisher(self, message_type, name, qos):
        publisher = Publisher()
        self.publishers[self.resolve(name)] = (message_type, publisher)
        return publisher

    def create_subscription(self, message_type, name, callback, qos):
        self.subscriptions[self.resolve(name)] = callback
        return object()

    def create_service(self, message_type, name, callback, **kwargs):
        self.services[self.resolve(name)] = callback
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
    assert len(paths.calls) == 2
    wall[0] += 0.8
    next(t for p, t, _ in host.timers if p == 0.02)()
    third = callback(request, EstimateMissionCost.Response())
    assert not third.feasible and "timeout" in third.reason
    paths.finish(0, 1)
    paths.finish(1, 1)
    assert sorted(paths.cancelled) == [0, 1]


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
            if scenario == "failed":
                handle.abort()
            else:
                handle.succeed()
            return ComputePathToPose.Result(path=message)

        # A never-completed action execute callback is deferred rather than blocking.
        handles = []

        def accepted(handle):
            handles.append(handle)
            if scenario != "timeout":
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
        odom_pub = peer.create_publisher(Odometry, "odom", 10)
        pose_pub = peer.create_publisher(PoseWithCovarianceStamped, "amcl_pose", 10)
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
