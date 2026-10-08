"""Read-only dashboard contracts against real sockets and the fleet journal."""

import http.client
import importlib
import json
from pathlib import Path
import queue
import socket
import sqlite3
import subprocess
import sys
import threading
import time

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from fleet_manager.adapter import FleetAdapter
from fleet_manager.config import Pose2D, load_fleet_config
from fleet_manager.journal import MissionJournal, MissionState
from fleet_manager.models import MissionRequest, RobotSnapshot
from fleet_manager.resources import LeaseRequest


def api():
    try:
        return importlib.import_module("fleet_manager.web")
    except ModuleNotFoundError:
        pytest.fail("read-only fleet dashboard is not implemented")


def snapshot():
    return {
        "robots": [{"robot_id": "amr_01", "mode": "AVAILABLE"}],
        "missions": [],
        "resources": [],
        "dock_queue": [],
        "events": [],
        "updated_at": time.time(),
    }


def wait_for(predicate, timeout=3):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    assert predicate(), "condition did not become true within its bound"


@pytest.fixture
def dashboard():
    dashboard = api().FleetDashboard(port=0, heartbeat=0.1)
    dashboard.hub.publish(snapshot())
    dashboard.start()
    yield dashboard
    dashboard.stop()


def request(dashboard, path="/api/snapshot", method="GET"):
    connection = http.client.HTTPConnection(*dashboard.address, timeout=2)
    connection.request(method, path)
    response = connection.getresponse()
    body = response.read()
    connection.close()
    return response, body


@pytest.mark.parametrize(
    "host",
    ["0.0.0.0", "192.0.2.1", "127.0.0.2", "::", "example.com", "127.0.0.1\r\nX: bad"],
)
def test_rejects_nonapproved_bind_before_creating_socket(host):
    with pytest.raises(ValueError):
        api().FleetDashboard(host=host, port=0)


@pytest.mark.parametrize("host", ["localhost", "127.0.0.1"])
def test_loopback_ephemeral_binding_and_restart_same_port(host):
    server = api().FleetDashboard(host=host, port=0)
    server.start()
    address = server.address
    try:
        assert address[0] == "127.0.0.1" and address[1] > 0
        assert request(server)[0].status == 200
    finally:
        server.stop()
    replacement = api().FleetDashboard(port=address[1])
    replacement.start()
    try:
        assert request(replacement)[0].status == 200
    finally:
        replacement.stop()


def test_snapshot_schema_is_detached_finite_bounded_plain_json(dashboard):
    value = snapshot()
    value["robots"][0].update(
        battery_percent=float("nan"), health_detail="x" * 10000 + "\ud800"
    )
    value["missions"] = [{"mission_id": f"m{i}"} for i in range(200)]
    value["unknown"] = object()
    dashboard.hub.publish(value)
    value["robots"][0]["robot_id"] = "changed"
    response, body = request(dashboard)
    result = json.loads(body)
    assert response.status == 200
    assert response.getheader("Content-Type") == "application/json; charset=utf-8"
    assert response.getheader("Cache-Control") == "no-store"
    assert {
        "robots",
        "missions",
        "resources",
        "dock_queue",
        "updated_at",
        "sequence",
        "session_id",
        "events",
    } <= result.keys()
    assert result["robots"][0]["robot_id"] == "amr_01"
    assert result["robots"][0]["battery_percent"] is None
    assert len(result["robots"][0]["health_detail"]) <= 512
    assert len(result["missions"]) <= 100
    assert b"NaN" not in body and len(body) < 512000


@pytest.mark.parametrize(
    "path",
    [
        "/missing",
        "/static/",
        "/../setup.py",
        "/%2e%2e/setup.py",
        "/%252e%252e/setup.py",
        "/styles.css/../setup.py",
        "/%2fetc/passwd",
        "/app.js%00",
        "//app.js",
        "/api/snapshot?ignored=1",
    ],
)
def test_unknown_and_traversal_paths_fail_closed(dashboard, path):
    assert request(dashboard, path)[0].status == 404


@pytest.mark.parametrize(
    "method",
    ["HEAD", "POST", "PUT", "DELETE", "PATCH", "OPTIONS", "TRACE", "CONNECT", "CUSTOM"],
)
def test_every_non_get_method_is_405_with_allow(dashboard, method):
    response, body = request(dashboard, method=method)
    assert response.status == 405
    assert response.getheader("Allow") == "GET"
    if method == "HEAD":
        assert body == b""


@pytest.mark.parametrize(
    "path,mime",
    [
        ("/", "text/html; charset=utf-8"),
        ("/styles.css", "text/css; charset=utf-8"),
        ("/app.js", "text/javascript; charset=utf-8"),
        ("/favicon.svg", "image/svg+xml; charset=utf-8"),
    ],
)
def test_static_assets_mime_and_browser_security_headers(dashboard, path, mime):
    response, body = request(dashboard, path)
    assert response.status == 200 and body
    assert response.getheader("Content-Type") == mime
    assert response.getheader("X-Content-Type-Options") == "nosniff"
    assert response.getheader("Cache-Control") == "no-store"
    assert "default-src 'self'" in response.getheader("Content-Security-Policy")


def test_static_symlink_does_not_expose_arbitrary_file(tmp_path):
    secret = tmp_path / "secret"
    secret.write_text("do not expose")
    root = tmp_path / "static"
    root.mkdir()
    (root / "app.js").symlink_to(secret)
    dashboard = api().FleetDashboard(port=0, static_dir=root)
    dashboard.start()
    try:
        assert request(dashboard, "/app.js")[0].status == 404
    finally:
        dashboard.stop()


def test_sse_framing_heartbeat_injection_and_disconnect_cleanup(dashboard):
    connection = http.client.HTTPConnection(*dashboard.address, timeout=2)
    connection.request("GET", "/api/events")
    response = connection.getresponse()
    assert response.status == 200
    assert response.getheader("Content-Type") == "text/event-stream; charset=utf-8"
    assert response.readline().startswith(b"id: ")
    assert response.readline() == b"event: snapshot\n"
    initial = json.loads(response.readline().removeprefix(b"data: "))
    assert response.readline() == b"\n"
    assert response.readline() == b": heartbeat\n"
    assert response.readline() == b"\n"
    value = snapshot()
    value["robots"][0]["health_detail"] = "bad\r\nevent: injected\ndata: <script>"
    dashboard.hub.publish(value)
    assert response.readline().startswith(b"id: ")
    assert response.readline() == b"event: snapshot\n"
    update = json.loads(response.readline().removeprefix(b"data: "))
    assert update["sequence"] > initial["sequence"]
    assert update["robots"][0]["health_detail"].startswith("bad")
    assert response.readline() == b"\n"
    response.close()
    connection.close()
    wait_for(lambda: dashboard.hub.client_count == 0)


def test_slow_subscriber_eviction_and_client_cap_never_block_publisher():
    hub = api().SnapshotHub(queue_size=2, max_clients=2)
    first, second = hub.subscribe(), hub.subscribe()
    assert hub.subscribe() is None
    # Full queued snapshots evict rather than accumulating history or waiting.
    started = time.monotonic()
    for _ in range(30):
        hub.publish(snapshot())
    assert time.monotonic() - started < 0.5
    assert first.closed.is_set() and second.closed.is_set()
    assert first.queue.qsize() <= 2 and hub.client_count == 0
    hub.unsubscribe(first)


def test_sse_client_cap_returns_503(dashboard):
    clients = [dashboard.hub.subscribe() for _ in range(8)]
    try:
        response, _ = request(dashboard, "/api/events")
        assert response.status == 503
    finally:
        for client in clients:
            dashboard.hub.unsubscribe(client)


def test_bounded_shutdown_with_idle_request_and_live_sse(dashboard):
    baseline = {thread.ident for thread in threading.enumerate()}
    idle = socket.create_connection(dashboard.address, timeout=2)
    stream = http.client.HTTPConnection(*dashboard.address, timeout=2)
    stream.request("GET", "/api/events")
    response = stream.getresponse()
    assert response.readline().startswith(b"id:")
    started = time.monotonic()
    dashboard.stop()
    assert time.monotonic() - started < 2
    idle.close()
    response.close()
    stream.close()
    wait_for(
        lambda: (
            not [
                thread
                for thread in threading.enumerate()
                if thread.ident not in baseline
            ]
        )
    )
    with pytest.raises(OSError):
        socket.create_connection(dashboard.address, timeout=0.1)
    dashboard.stop()


def test_idle_connections_have_bounded_thread_count(dashboard):
    sockets = []
    try:
        for _ in range(40):
            sockets.append(socket.create_connection(dashboard.address, timeout=2))
        time.sleep(0.1)
        assert dashboard.request_count <= 24
    finally:
        for connection in sockets:
            connection.close()


def rig(tmp_path):
    config = load_fleet_config(
        Path(__file__).resolve().parents[2] / "factory_bringup/config/fleet.yaml"
    )
    path = tmp_path / "fleet.sqlite3"
    journal = MissionJournal(path)
    adapter = FleetAdapter(config, journal, None, clock=lambda: 100)
    adapter.registry.observe(
        RobotSnapshot("amr_01", "CHARGING", Pose2D(4, -2, 0), 26, "EMPTY"), 100
    )
    adapter.registry.observe(
        RobotSnapshot("amr_02", "EXECUTING", Pose2D(0, 0, 0), 72, "LOADED", "m1"), 100
    )
    adapter.resources.acquire(LeaseRequest("amr_01", "charge1", "dock_01"), 100)
    journal.register(
        MissionRequest("m1", "assembly", "inspection", "motor"), "hash", 100
    )
    journal.transition("m1", MissionState.QUEUED, MissionState.COMPLETED, {}, 101)
    return path, journal, adapter


def test_observer_captures_all_robots_resources_and_persistent_history_without_writes(
    tmp_path,
):
    path, journal, adapter = rig(tmp_path)
    total_changes = journal._connection.total_changes
    dashboard = api().FleetDashboard(port=0, journal_path=path, heartbeat=0.1)
    dashboard.capture(adapter)
    dashboard.start()
    try:
        wait_for(lambda: json.loads(request(dashboard)[1])["missions"])
        result = json.loads(request(dashboard)[1])
        assert [robot["robot_id"] for robot in result["robots"]] == ["amr_01", "amr_02"]
        assert result["robots"][0]["current_resource"] == "dock_01"
        assert result["robots"][1]["payload_state"] == "LOADED"
        assert (
            next(
                resource
                for resource in result["resources"]
                if resource["resource_id"] == "dock_01"
            )["owner"]
            == "amr_01"
        )
        assert result["missions"][0]["state"] == "COMPLETED"
        assert result["events"][-1]["state"] == "COMPLETED"
        assert journal._connection.total_changes == total_changes
        with sqlite3.connect(path) as read:
            assert (
                read.execute("SELECT COUNT(*) FROM mission_events").fetchone()[0] == 2
            )
    finally:
        dashboard.stop()
        journal.close()


def test_capture_does_not_expire_resources_or_wait_on_reader_or_resource_lock(tmp_path):
    path, journal, adapter = rig(tmp_path)
    adapter.clock = lambda: 500
    dashboard = api().FleetDashboard(port=0, journal_path=path)
    dashboard.capture(adapter)
    assert "dock_01" in adapter.resources._leases
    adapter.resources._lock.acquire()
    try:
        started = time.monotonic()
        assert dashboard.capture(adapter) is False
        assert time.monotonic() - started < 0.1
    finally:
        adapter.resources._lock.release()
        journal.close()


def test_hub_concurrent_publish_read_is_atomic():
    hub = api().SnapshotHub()
    failures = queue.Queue()

    def writer():
        for number in range(150):
            value = snapshot()
            value["robots"] = [{"robot_id": str(number)}] * 10
            hub.publish(value)

    def reader():
        for _ in range(150):
            value = json.loads(hub.snapshot_bytes())
            if len({robot["robot_id"] for robot in value["robots"]}) > 1:
                failures.put(value)

    threads = [threading.Thread(target=writer), threading.Thread(target=reader)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(3)
        assert not thread.is_alive()
    assert failures.empty()


def test_protocol_observations_are_bounded_and_never_written_to_journal(tmp_path):
    from types import SimpleNamespace

    path, journal, adapter = rig(tmp_path)
    before = journal._connection.total_changes
    dashboard = api().FleetDashboard(port=0, journal_path=path)
    dashboard.capture(adapter)
    for number in range(120):
        dashboard.observe_event(
            SimpleNamespace(
                mission_id="m1",
                robot_id="amr_02",
                protocol="Modbus",
                event=f"transfer-{number}",
                outcome="ok",
                detail="<script>alert(1)</script>",
            )
        )
    dashboard.start()
    try:
        wait_for(
            lambda: any(
                event.get("event") == "transfer-119"
                for event in json.loads(request(dashboard)[1])["events"]
            )
        )
        result = json.loads(request(dashboard)[1])
        assert len(result["events"]) <= 50
        assert journal._connection.total_changes == before
    finally:
        dashboard.stop()
        journal.close()


@pytest.mark.parametrize("enabled", [False, True])
def test_real_node_owns_optional_dashboard_and_preserves_shutdown(tmp_path, enabled):
    import rclpy
    from rclpy.context import Context
    from rclpy.parameter import Parameter
    from fleet_manager.node import FleetManagerNode

    api()
    context = Context()
    rclpy.init(context=context, domain_id=85)
    node = None
    try:
        node = FleetManagerNode(
            context=context,
            parameter_overrides=[
                Parameter(
                    "fleet_file",
                    value=str(
                        Path(__file__).resolve().parents[2]
                        / "factory_bringup/config/fleet.yaml"
                    ),
                ),
                Parameter("journal_path", value=str(tmp_path / "node.sqlite3")),
                Parameter("dashboard_enabled", value=enabled),
                Parameter("dashboard_port", value=0),
            ],
        )
        assert (node.dashboard is not None) is enabled
        if enabled:
            node._tick()
            wait_for(lambda: len(json.loads(request(node.dashboard)[1])["robots"]) == 2)
            address = node.dashboard.address
        node.destroy_node()
        node = None
        if enabled:
            with pytest.raises(OSError):
                socket.create_connection(address, timeout=0.1)
    finally:
        if node is not None:
            node.destroy_node()
        context.shutdown()


def test_dashboard_bind_failure_does_not_interrupt_fleet_startup(tmp_path):
    import rclpy
    from rclpy.context import Context
    from rclpy.parameter import Parameter
    from fleet_manager.node import FleetManagerNode

    api()
    context = Context()
    rclpy.init(context=context, domain_id=86)
    node = None
    try:
        node = FleetManagerNode(
            context=context,
            parameter_overrides=[
                Parameter(
                    "fleet_file",
                    value=str(
                        Path(__file__).resolve().parents[2]
                        / "factory_bringup/config/fleet.yaml"
                    ),
                ),
                Parameter("journal_path", value=str(tmp_path / "node.sqlite3")),
                Parameter("dashboard_enabled", value=True),
                Parameter("dashboard_host", value="0.0.0.0"),
            ],
        )
        assert node.dashboard is None
        assert node.adapter.state == "RECONCILING"
        assert len(node.actions) == 2
        node._tick()
    finally:
        if node is not None:
            node.destroy_node()
        context.shutdown()


def test_large_bounded_snapshot_retains_schema_metadata(dashboard):
    value = snapshot()
    value["robots"] = [
        {
            "robot_id": f"amr_{number}",
            "mode": "AVAILABLE",
            "health": "ONLINE",
            "battery_percent": 50,
            "mission_id": None,
            "payload_state": "EMPTY",
            "health_detail": "",
            "fault": "",
            "pose": {"x": 0, "y": 0, "yaw": 0},
        }
        for number in range(10)
    ]
    value["resources"] = [
        {
            "resource_id": f"resource-{number}",
            "kind": "station",
            "owner": None,
            "waiters": [],
            "reconciliation_required": False,
        }
        for number in range(100)
    ]
    value["missions"] = [
        {
            "mission_id": f"m{number}",
            "state": "COMPLETED",
            "part": "motor",
            "pickup_station": "assembly",
            "dropoff_station": "inspection",
            "assigned_robot_id": "amr_01",
            "payload_ownership": "DELIVERED",
            "updated_at": 100,
        }
        for number in range(100)
    ]
    value["events"] = [
        {
            "event_id": str(number),
            "mission_id": "m1",
            "state": "COMPLETED",
            "timestamp": 100,
            "sequence": number,
            "detail": "x" * 256,
        }
        for number in range(50)
    ]
    dashboard.hub.publish(value)
    result = json.loads(request(dashboard)[1])
    assert result["updated_at"] == value["updated_at"]
    assert len(result["events"]) == 50 and result["events"][-1]["event_id"] == "49"


@pytest.mark.parametrize("failure", ["threading.Thread.start", "threading.Thread"])
def test_failed_thread_start_releases_listener_with_bounded_cleanup(failure):
    api()
    code = """
from unittest.mock import patch
import socket
from fleet_manager.web import FleetDashboard
server = FleetDashboard(port=0)
with patch(FAILURE, side_effect=RuntimeError('thread unavailable')):
    try:
        server.start()
    except RuntimeError:
        pass
    else:
        raise AssertionError('thread failure swallowed')
try:
    socket.create_connection(server.address, timeout=0.1)
except OSError:
    pass
else:
    raise AssertionError('listener survived failed start')
"""
    code = code.replace("FAILURE", repr(failure))
    process = subprocess.run(
        [sys.executable, "-c", code], capture_output=True, text=True, timeout=3
    )
    assert process.returncode == 0, process.stderr


def test_observer_lock_contention_never_blocks_executor_capture(tmp_path):
    path, journal, adapter = rig(tmp_path)
    dashboard = api().FleetDashboard(port=0, journal_path=path)
    dashboard._capture_lock.acquire()
    try:
        started = time.monotonic()
        assert dashboard.capture(adapter) is False
        assert time.monotonic() - started < 0.1
    finally:
        dashboard._capture_lock.release()
        journal.close()


def test_recent_terminal_history_does_not_hide_older_active_work(tmp_path):
    path, journal, adapter = rig(tmp_path)
    journal.register(
        MissionRequest("old-active", "assembly", "inspection", "motor"), "old-hash", 100
    )
    for number in range(120):
        mission_id = f"completed-{number}"
        journal.register(
            MissionRequest(mission_id, "assembly", "inspection", "motor"),
            mission_id,
            100,
        )
        journal.transition(
            mission_id, MissionState.QUEUED, MissionState.COMPLETED, {}, 100
        )
    dashboard = api().FleetDashboard(port=0, journal_path=path)
    dashboard.capture(adapter)
    dashboard.start()
    try:
        wait_for(lambda: len(json.loads(request(dashboard)[1])["missions"]) == 100)
        missions = json.loads(request(dashboard)[1])["missions"]
        assert any(mission["mission_id"] == "old-active" for mission in missions)
        assert any(mission["mission_id"] == "completed-119" for mission in missions)
    finally:
        dashboard.stop()
        journal.close()


def test_dashboard_stop_error_does_not_prevent_fleet_node_cleanup(tmp_path):
    from types import SimpleNamespace
    import rclpy
    from rclpy.context import Context
    from rclpy.parameter import Parameter
    from fleet_manager.node import FleetManagerNode

    context = Context()
    rclpy.init(context=context, domain_id=87)
    node = FleetManagerNode(
        context=context,
        parameter_overrides=[
            Parameter(
                "fleet_file",
                value=str(
                    Path(__file__).resolve().parents[2]
                    / "factory_bringup/config/fleet.yaml"
                ),
            ),
            Parameter("journal_path", value=str(tmp_path / "node.sqlite3")),
        ],
    )

    def failed_stop():
        raise OSError("observer cleanup failure")

    node.dashboard = SimpleNamespace(stop=failed_stop)
    try:
        node.destroy_node()
        with pytest.raises(sqlite3.ProgrammingError):
            node.journal.get("m1")
        assert node._journal_lock.closed
    finally:
        node.dashboard = None
        if not node._journal_lock.closed:
            node.destroy_node()
        context.shutdown()


def test_new_mission_event_remains_visible_after_full_protocol_event_window(tmp_path):
    from types import SimpleNamespace

    path, journal, adapter = rig(tmp_path)
    dashboard = api().FleetDashboard(port=0, journal_path=path)
    dashboard.capture(adapter)
    for number in range(50):
        dashboard.observe_event(SimpleNamespace(event=f"protocol-{number}"))
    dashboard.start()
    try:
        wait_for(lambda: len(json.loads(request(dashboard)[1])["events"]) == 50)
        journal.register(
            MissionRequest("new-mission", "assembly", "inspection", "motor"), "new", 100
        )
        wait_for(
            lambda: any(
                event.get("mission_id") == "new-mission"
                for event in json.loads(request(dashboard)[1])["events"]
            )
        )
        assert len(json.loads(request(dashboard)[1])["events"]) <= 50
    finally:
        dashboard.stop()
        journal.close()


def test_observer_resource_collection_stops_iteration_at_its_bound():
    from fleet_manager.config import ResourceConfig
    from fleet_manager.resources import ResourceManager

    class BoundedRead(dict):
        def values(self):
            for number, value in enumerate(super().values()):
                if number >= 100:
                    raise AssertionError(
                        "observer walks beyond its bounded resource window"
                    )
                yield value

    resources = ResourceManager(
        [ResourceConfig(f"zone-{number}", "traffic_zone", 1) for number in range(150)]
    )
    resources._resources = BoundedRead(resources._resources)
    projected = resources.observer_snapshot()
    assert len(projected) == 100
    assert projected[0].resource_id == "zone-0"
    assert projected[-1].resource_id == "zone-99"
