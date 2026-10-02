"""MQTT adapter real ROS exchanges; only the broker transport is replaced."""
import json
import threading
import time

import pytest
import rclpy
from rclpy.action import ActionClient, ActionServer, GoalResponse
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from factory_interfaces.action import ExecuteFactoryMission
from factory_interfaces.msg import ProtocolEvent
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from mqtt_gateway.node import MqttGatewayNode


class Broker:
    def __init__(self):
        self.published = []
    def set_handlers(self, message, connection):
        self.message, self.connection = message, connection
    def start(self):
        self.connection(True)
    def publish(self, topic, payload, qos=0, retain=False):
        self.published.append((topic, json.loads(payload) if payload.startswith('{') else payload, qos, retain))
        return True
    def close(self):
        pass


@pytest.fixture
def rig(request):
    context = Context()
    rclpy.init(context=context, domain_id=77)
    broker = Broker()
    node = MqttGatewayNode(mqtt_client=broker, context=context)
    peer = rclpy.create_node('mqtt_action_peer', context=context)
    peer.declare_parameter('execute_delay_sec', 0.0)
    executed = []
    busy = [False]
    def execute(handle):
        executed.append(handle.request)
        handle.publish_feedback(ExecuteFactoryMission.Feedback(state='LOADING', station='assembly', detail='confirmed'))
        time.sleep(peer.get_parameter('execute_delay_sec').value)
        handle.succeed()
        return ExecuteFactoryMission.Result(success=True, final_state='COMPLETED')
    server = None if getattr(request, 'param', True) is False else ActionServer(
        peer, ExecuteFactoryMission, '/factory/execute_mission', execute,
        goal_callback=lambda goal: GoalResponse.REJECT if busy[0] else GoalResponse.ACCEPT)
    executor = (SingleThreadedExecutor(context=context) if getattr(request, 'param', '') == 'result-transport'
                else MultiThreadedExecutor(num_threads=3, context=context))
    executor.add_node(node)
    executor.add_node(peer)
    def wait(predicate):
        deadline = time.monotonic() + 4
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert predicate()
    yield broker, node, executed, busy, wait, peer
    if server is not None:
        server.destroy()
    executor.shutdown()
    node.destroy_node()
    peer.destroy_node()
    context.shutdown()


def request(broker, mission_id='M-001', **changes):
    mission = dict(mission_id=mission_id, robot_id='amr_01', pickup='assembly', dropoff='inspection', part='motor')
    mission.update(changes)
    thread = threading.Thread(target=lambda: broker.message(json.dumps(mission).encode()))
    thread.start()
    thread.join()


def statuses(broker):
    return [payload for topic, payload, _, _ in broker.published if topic.endswith('/status')]


def test_real_action_conversion_status_schema_and_duplicate(rig):
    broker, node, executed, _, wait, _ = rig
    request(broker)
    wait(lambda: any(s['state'] == 'COMPLETED' for s in statuses(broker)))
    assert len(executed) == 1
    assert executed[0].pickup_station == 'assembly'
    assert executed[0].dropoff_station == 'inspection'
    assert all(set(s) == {'mission_id', 'robot_id', 'state', 'station', 'timestamp', 'sequence', 'detail', 'error_code'} for s in statuses(broker))
    request(broker)
    count = len(statuses(broker))
    wait(lambda: len(statuses(broker)) > count)
    assert len(executed) == 1


def test_inflight_duplicate_never_overlaps_acceptance_phase(rig):
    broker, node, executed, _, wait, peer = rig
    events = []
    subscription = peer.create_subscription(ProtocolEvent, '/factory/protocol_events', events.append, 100)
    wait(lambda: node.events.get_subscription_count() > 0)
    request(broker, 'duplicate')
    request(broker, 'duplicate')
    wait(lambda: any(e.event == 'mqtt_acceptance_finished' for e in events) and any(s['state'] == 'COMPLETED' for s in statuses(broker)))
    phases = [e.event for e in events if e.event.startswith('mqtt_acceptance_')]
    assert phases == ['mqtt_acceptance_started', 'mqtt_acceptance_finished']
    assert len(executed) == 1
    peer.destroy_subscription(subscription)


@pytest.mark.parametrize('changes', [{'part': 'gear'}, {'pickup': 'inspection'}, {'dropoff': 'assembly'}, {'robot_id': 'amr_02'}])
def test_known_id_conflicts_precede_configuration_validation(rig, changes):
    broker, node, executed, _, wait, _ = rig
    request(broker)
    wait(lambda: any(s['state'] == 'COMPLETED' for s in statuses(broker)))
    saved = node.registry.state_for('M-001')
    request(broker, **changes)
    wait(lambda: any(s['error_code'] == 'MISSION_ID_CONFLICT' for s in statuses(broker)))
    assert node.registry.state_for('M-001') == saved
    assert len(executed) == 1
    request(broker, 'unknown', **changes)
    wait(lambda: any(s['mission_id'] == 'unknown' and s['error_code'] == 'INVALID_MISSION' for s in statuses(broker)))
    assert len(executed) == 1
    broker.message(json.dumps(dict(mission_id='M-001', robot_id='amr_01', pickup='assembly', dropoff='inspection', part=12)).encode())
    wait(lambda: any(s['mission_id'] == 'invalid' and s['error_code'] == 'INVALID_MISSION' for s in statuses(broker)))
    assert node.registry.state_for('M-001') == saved


def test_validation_busy_and_bounded_reconnect_replay(rig):
    broker, node, executed, busy, wait, _ = rig
    busy[0] = True
    request(broker, 'busy')
    wait(lambda: any(s['error_code'] == 'ROBOT_BUSY' for s in statuses(broker)))
    assert [s['state'] for s in statuses(broker)] == ['FAILED']
    broker.message(b'{invalid')
    wait(lambda: any(s['error_code'] == 'INVALID_MISSION' for s in statuses(broker)))
    assert not executed
    broker.connection(False)
    wait(lambda: not node.connected)
    for index in range(105):
        node.publish_status('busy', 'RECOVERING', detail=str(index))
    before = len(statuses(broker))
    broker.connection(True)
    wait(lambda: len(statuses(broker)) == before + 100)
    assert [s['detail'] for s in statuses(broker)[-100:]] == [str(i) for i in range(5, 105)]


def test_real_pose_odometry_conversion_and_latest_telemetry(rig):
    broker, node, _, _, wait, peer = rig
    clock = peer.create_publisher(Clock, '/clock', 10)
    pose_publisher = peer.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', 10)
    odom_publisher = peer.create_publisher(Odometry, '/odom', 10)
    wait(lambda: pose_publisher.get_subscription_count() > 0)
    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = 'map'
    pose.pose.pose.orientation.w = 1.0
    pose.pose.pose.position.x = 3.0
    odom = Odometry()
    odom.header.frame_id = 'odom'
    odom.pose.pose.orientation.w = 1.0
    odom.twist.twist.linear.x = 0.2
    odom.twist.twist.angular.z = 0.1
    clock.publish(Clock(clock=rclpy.time.Time(seconds=100).to_msg()))
    pose_publisher.publish(pose)
    odom_publisher.publish(odom)
    telemetry = lambda: [p for t, p, _, _ in broker.published if t.endswith('/telemetry')]
    wait(lambda: bool(telemetry()))
    sample = telemetry()[-1]
    assert sample['robot_id'] == 'amr_01'
    assert sample['frame_id'] == 'map' and sample['x'] == 3.0
    assert sample['linear_velocity'] == 0.2 and sample['angular_velocity'] == 0.1
    assert sample['timestamp'].startswith('1970-01-01T00:01:40')
    broker.connection(False)
    wait(lambda: not node.connected)
    pose.pose.pose.position.x = 4.0
    pose_publisher.publish(pose)
    wait(lambda: node.reconnect._telemetry is not None and node.reconnect._telemetry['x'] == 4.0)
    pose.pose.pose.position.x = 5.0
    pose_publisher.publish(pose)
    wait(lambda: node.reconnect._telemetry['x'] == 5.0)
    count = len(telemetry())
    broker.connection(True)
    wait(lambda: len(telemetry()) > count)
    assert telemetry()[count]['x'] == 5.0


def test_goal_acceptance_preserves_feedback_received_before_response(rig):
    broker, node, _, _, wait, peer = rig
    peer.set_parameters([rclpy.parameter.Parameter('execute_delay_sec', value=0.15)])
    request(broker)
    wait(lambda: any(s['state'] == 'LOADING' for s in statuses(broker)) and node.current_mission_id == 'M-001')
    assert node.current_state == 'LOADING'
    assert [s['state'] for s in statuses(broker)][:2] == ['RECEIVED', 'LOADING']
    wait(lambda: any(s['state'] == 'COMPLETED' for s in statuses(broker)))


@pytest.mark.parametrize('rig', ['result-transport'], indirect=True)
def test_feedback_before_acceptance_waits_for_received_then_preserves_order(rig):
    broker, node, _, _, wait, peer = rig
    wait(lambda: node.connected and node.action.server_is_ready())
    node.timer.cancel()
    request(broker, 'early-feedback')
    node._drain()
    # Poll the real ActionClient feedback while acceptance consumption is paused.
    wait(lambda: statuses(broker) or bool(node.awaiting_acceptance.get('early-feedback')))
    assert statuses(broker) == []
    assert node.registry.state_for('early-feedback') is None
    wait(lambda: not node.completions.empty())
    node._drain()
    assert [s['state'] for s in statuses(broker)][:2] == ['RECEIVED', 'LOADING']
    assert node.current_state == 'LOADING'
    node.timer.reset()
    wait(lambda: any(s['state'] == 'COMPLETED' for s in statuses(broker)))


@pytest.mark.parametrize('rig', [False], indirect=True)
def test_unavailable_server_does_not_publish_received(rig):
    broker, node, executed, _, wait, peer = rig
    events = []
    subscription = peer.create_subscription(ProtocolEvent, '/factory/protocol_events', events.append, 100)
    wait(lambda: node.events.get_subscription_count() > 0)
    request(broker, 'unavailable')
    wait(lambda: 'unavailable' in node.pending and bool(events))
    assert statuses(broker) == []
    assert 'unavailable' in node.pending
    assert not executed
    assert [event.event for event in events] == ['mqtt_acceptance_started']
    peer.destroy_subscription(subscription)


@pytest.mark.parametrize('rig', ['result-transport'], indirect=True)
def test_destroyed_result_client_finishes_acceptance_exactly_once(rig):
    broker, node, _, _, wait, peer = rig
    events = []
    subscription = peer.create_subscription(ProtocolEvent, '/factory/protocol_events', events.append, 100)
    wait(lambda: node.events.get_subscription_count() > 0 and node.action.server_is_ready())
    # Hold adapter response consumption, not the real ROS goal exchange.
    node.timer.cancel()
    request(broker, 'result-transport')
    node._drain()
    wait(lambda: not node.completions.empty())
    node.action.destroy()
    node._drain()
    # Restore an owned real client for normal node teardown and executor polling.
    node.action = ActionClient(node, ExecuteFactoryMission, '/factory/execute_mission',
                               callback_group=node.protocol_group)
    node.events.publish(ProtocolEvent(event='test_delivery_barrier'))
    wait(lambda: any(e.event == 'test_delivery_barrier' for e in events))
    wait(lambda: any(s['error_code'] == 'MISSION_TRANSPORT_ERROR' for s in statuses(broker)))
    assert [e.outcome for e in events if e.event == 'mqtt_acceptance_finished'] == ['SUCCEEDED']
    assert statuses(broker)[0]['state'] == 'RECEIVED'
    assert statuses(broker)[-1]['state'] == 'FAILED'
    peer.destroy_subscription(subscription)
