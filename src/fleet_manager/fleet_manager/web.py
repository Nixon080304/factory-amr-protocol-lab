# SPDX-License-Identifier: Apache-2.0
"""Bounded loopback observer. HTTP threads never access ROS or its SQLite writer.

The executor offers detached immutable telemetry without waiting. One observer
thread reads a separate query-only WAL connection and publishes cached JSON.
Each SSE client gets a bounded queue; a full queue evicts that client. Reconnect
always starts from a snapshot, so no event replay buffer is needed.
"""

from collections import deque
from dataclasses import asdict, dataclass, field
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from itertools import islice
import math
from pathlib import Path
import queue
import secrets
import select
import socket
import sqlite3
import threading
import time


def _plain(value, budget, depth=0):
    """Copy only bounded plain data, including broken Unicode and nonfinite floats."""
    if budget[0] <= 0 or depth > 6:
        return None
    budget[0] -= 1
    if value is None or type(value) is bool:
        return value
    if isinstance(value, str):
        return value[:256].encode("utf-8", "replace").decode("utf-8")
    if type(value) in (float, int):
        try:
            return value if math.isfinite(value) else None
        except OverflowError:
            return None
    if type(value) is dict:
        return {
            key[:64]: _plain(item, budget, depth + 1)
            for key, item in islice(value.items(), 32)
            if type(key) is str and key.isascii()
        }
    if type(value) in (list, tuple):
        return [_plain(item, budget, depth + 1) for item in value[:100]]
    return None


def _empty():
    return {
        "robots": [],
        "missions": [],
        "resources": [],
        "dock_queue": [],
        "events": [],
        "updated_at": None,
        "fleet_state": "STARTING",
    }


@dataclass(eq=False)
class _Subscriber:
    queue: queue.Queue
    closed: threading.Event = field(default_factory=threading.Event)


class SnapshotHub:
    """Publish immutable wire bytes; never retain unbounded history or clients."""

    def __init__(self, *, queue_size=8, max_clients=8):
        if not 1 <= queue_size <= 64 or not 1 <= max_clients <= 32:
            raise ValueError("invalid SSE bounds")
        self._queue_size, self._max_clients = queue_size, max_clients
        self._clients = set()
        self._lock = threading.Lock()
        self._sequence = 0
        self._session = secrets.token_hex(16)
        self._bytes = b""
        self.publish(_empty())

    @property
    def client_count(self):
        with self._lock:
            return len(self._clients)

    def publish(self, snapshot):
        value = {
            key: _plain(snapshot.get(key, default), [2000])
            for key, default in _empty().items()
        }
        value["events"] = (
            value["events"][-50:] if isinstance(value["events"], list) else []
        )
        with self._lock:
            self._sequence += 1
            value.update(sequence=self._sequence, session_id=self._session)
            body = json.dumps(
                value, ensure_ascii=True, allow_nan=False, separators=(",", ":")
            ).encode("utf-8")
            # Keep the wire payload bounded as well as each scalar/container.
            while len(body) > 512000:
                largest = max(
                    (
                        key
                        for key in ("missions", "events", "resources", "robots")
                        if isinstance(value[key], list) and value[key]
                    ),
                    key=lambda key: len(json.dumps(value[key])),
                    default=None,
                )
                if largest is None:
                    break
                value[largest] = value[largest][: len(value[largest]) // 2]
                body = json.dumps(
                    value, ensure_ascii=True, allow_nan=False, separators=(",", ":")
                ).encode("utf-8")
            self._bytes = body
            for client in tuple(self._clients):
                try:
                    client.queue.put_nowait((self._sequence, self._bytes))
                except queue.Full:
                    client.closed.set()
                    self._clients.discard(client)

    def snapshot_bytes(self):
        with self._lock:
            return self._bytes

    def subscribe(self):
        with self._lock:
            if len(self._clients) >= self._max_clients:
                return None
            client = _Subscriber(queue.Queue(self._queue_size))
            client.queue.put_nowait((self._sequence, self._bytes))
            self._clients.add(client)
            return client

    def unsubscribe(self, client):
        with self._lock:
            self._clients.discard(client)
            client.closed.set()

    def close(self):
        with self._lock:
            for client in self._clients:
                client.closed.set()
            self._clients.clear()


class _HTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = False
    block_on_close = True

    def __init__(self, address, dashboard):
        self.dashboard = dashboard
        self.slots = threading.BoundedSemaphore(24)
        self.sockets = set()
        self.socket_lock = threading.Lock()
        super().__init__(address, _Handler)

    def get_request(self):
        connection, address = super().get_request()
        connection.settimeout(0.5)
        return connection, address

    def process_request(self, connection, address):
        if not self.slots.acquire(blocking=False):
            try:
                connection.sendall(
                    b"HTTP/1.0 503 Service Unavailable\r\nContent-Length: 0\r\nConnection: close\r\n\r\n"
                )
            except OSError:
                pass
            self.shutdown_request(connection)
            return
        with self.socket_lock:
            self.sockets.add(connection)
        try:
            super().process_request(connection, address)
        except BaseException:
            with self.socket_lock:
                self.sockets.discard(connection)
            self.slots.release()
            self.shutdown_request(connection)
            raise

    def process_request_thread(self, connection, address):
        try:
            super().process_request_thread(connection, address)
        finally:
            with self.socket_lock:
                self.sockets.discard(connection)
            self.slots.release()

    def close_connections(self):
        with self.socket_lock:
            for connection in tuple(self.sockets):
                try:
                    connection.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass


class _Handler(BaseHTTPRequestHandler):
    # HTTP/1.0 close-delimited SSE is valid, including indefinite event streams.
    protocol_version = "HTTP/1.0"

    def log_message(self, *_):
        pass

    def __getattr__(self, name):
        if name.startswith("do_"):
            return self._method_not_allowed
        raise AttributeError(name)

    def _method_not_allowed(self):
        self._reply(405, b"", "text/plain; charset=utf-8", allow="GET")

    def _headers(self, status, mime, length=None, allow=None):
        self.send_response(status)
        self.send_header("Content-Type", mime)
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header(
            "Content-Security-Policy",
            "default-src 'self'; script-src 'self'; style-src 'self'; connect-src 'self'; img-src 'self'; frame-ancestors 'none'; base-uri 'none'; form-action 'none'",
        )
        self.send_header("Connection", "close")
        if length is not None:
            self.send_header("Content-Length", str(length))
        if allow:
            self.send_header("Allow", allow)
        self.end_headers()

    def _reply(self, status, body, mime, allow=None):
        self._headers(status, mime, len(body), allow)
        if self.command != "HEAD":
            self.wfile.write(body)

    def do_GET(self):
        try:
            dashboard = self.server.dashboard
            raw_path = self.requestline.split()[1]
            if raw_path != self.path:
                self._reply(404, b"", "text/plain; charset=utf-8")
            elif self.path == "/api/snapshot":
                self._reply(
                    200,
                    dashboard.hub.snapshot_bytes(),
                    "application/json; charset=utf-8",
                )
            elif self.path == "/api/events":
                self._events(dashboard)
            elif self.path in dashboard.assets:
                body, mime = dashboard.assets[self.path]
                self._reply(200, body, mime)
            else:
                self._reply(404, b"", "text/plain; charset=utf-8")
        except (OSError, ValueError):
            pass  # Disconnection is local to this HTTP observer.

    def _events(self, dashboard):
        client = dashboard.hub.subscribe()
        if client is None:
            self._reply(503, b"", "text/plain; charset=utf-8")
            return
        try:
            self._headers(200, "text/event-stream; charset=utf-8")
            heartbeat_at = time.monotonic() + dashboard.heartbeat
            while not dashboard.stopping.is_set() and not client.closed.is_set():
                readable, _, _ = select.select([self.connection], [], [], 0)
                if readable and not self.connection.recv(1, socket.MSG_PEEK):
                    break
                try:
                    sequence, body = client.queue.get(
                        timeout=min(0.1, dashboard.heartbeat)
                    )
                    self.wfile.write(
                        b"id: "
                        + str(sequence).encode("ascii")
                        + b"\nevent: snapshot\ndata: "
                        + body
                        + b"\n\n"
                    )
                    self.wfile.flush()
                except queue.Empty:
                    pass
                if time.monotonic() >= heartbeat_at:
                    self.wfile.write(b": heartbeat\n\n")
                    self.wfile.flush()
                    heartbeat_at = time.monotonic() + dashboard.heartbeat
        finally:
            dashboard.hub.unsubscribe(client)


class FleetDashboard:
    """Own HTTP/observer lifecycle. Start/stop never touches the fleet writer."""

    def __init__(
        self,
        *,
        host="127.0.0.1",
        port=8080,
        journal_path=None,
        heartbeat=5.0,
        static_dir=None,
    ):
        if host not in ("localhost", "127.0.0.1"):
            raise ValueError("dashboard host must be localhost or 127.0.0.1")
        if type(port) is not int or not 0 <= port <= 65535:
            raise ValueError("dashboard port must be an integer from 0 to 65535")
        if not math.isfinite(heartbeat) or heartbeat <= 0:
            raise ValueError("heartbeat must be positive and finite")
        self.address = ("127.0.0.1", port)
        self.journal_path = (
            Path(journal_path).resolve() if journal_path is not None else None
        )
        self.heartbeat = heartbeat
        self.hub = SnapshotHub()
        self.stopping = threading.Event()
        self._capture_lock = threading.Lock()
        self._capture = None
        self._events = deque(maxlen=50)
        self._event_number = 0
        self._server = self._thread = self._observer = None
        root = (
            Path(static_dir)
            if static_dir is not None
            else Path(__file__).with_name("static")
        )
        self.assets = {}
        for route, name, mime in (
            ("/", "index.html", "text/html"),
            ("/styles.css", "styles.css", "text/css"),
            ("/app.js", "app.js", "text/javascript"),
            ("/favicon.svg", "favicon.svg", "image/svg+xml"),
        ):
            path = root / name
            # Fixed allowlist, no path interpolation, and no symlink following.
            if root.is_symlink() or path.is_symlink() or not path.is_file():
                continue
            if path.stat().st_size <= 256000:
                self.assets[route] = (path.read_bytes(), mime + "; charset=utf-8")

    @property
    def request_count(self):
        server = self._server
        if server is None:
            return 0
        with server.socket_lock:
            return len(server.sockets)

    def capture(self, adapter):
        """Called on the executor: bounded frozen values, no SQLite or waiting."""
        resources = adapter.resources.observer_snapshot()
        if resources is None or not self._capture_lock.acquire(blocking=False):
            return False
        try:
            now = adapter.clock()
            self._capture = (
                tuple(
                    adapter.registry.get(robot.robot_id, now)
                    for robot in adapter.config.robots
                ),
                resources,
                adapter.core.charging_snapshot(),
                adapter.state,
                now,
                time.time(),
            )
            return True
        finally:
            self._capture_lock.release()

    def observe_event(self, message):
        """Consume bounded protocol fields without retaining a ROS message."""
        if not self._capture_lock.acquire(blocking=False):
            return False
        try:
            self._event_number += 1
            event = {
                field: getattr(message, field, "")[:256]
                for field in (
                    "mission_id",
                    "robot_id",
                    "protocol",
                    "event",
                    "outcome",
                    "detail",
                )
            }
            event.update(
                event_id=f"protocol-{self._event_number}",
                timestamp=time.time(),
                observed_at=time.time(),
            )
            self._events.append(event)
            return True
        finally:
            self._capture_lock.release()

    def _read_journal(self, connection):
        # Query only bounded scalar fields. Large result/detail JSON is never read.
        columns = (
            "SELECT substr(mission_id,1,256) AS mission_id,state,"
            "substr(assigned_robot_id,1,256) AS assigned_robot_id,payload_ownership,"
            "substr(pickup_station,1,256) AS pickup_station,"
            "substr(dropoff_station,1,256) AS dropoff_station,"
            "substr(part,1,256) AS part,updated_at FROM missions "
        )
        # Use the existing active index; terminal history cannot hide active work.
        missions = [
            dict(row)
            for row in connection.execute(
                columns + "WHERE state NOT IN ('COMPLETED','FAILED','CANCELLED') "
                "ORDER BY created_at,mission_id LIMIT 100"
            )
        ]
        missions.extend(
            dict(row)
            for row in connection.execute(
                columns + "WHERE state IN ('COMPLETED','FAILED','CANCELLED') "
                "ORDER BY rowid DESC LIMIT ?",
                (100 - len(missions),),
            )
        )
        events = [
            dict(row)
            for row in connection.execute(
                "SELECT sequence,substr(mission_id,1,256) AS mission_id,state,timestamp FROM mission_events ORDER BY sequence DESC LIMIT 50"
            )
        ][::-1]
        for event in events:
            event["event_id"] = f"mission-{event['sequence']}"
        return missions, events

    def _observe(self):
        connection = None
        observed_events = {}
        try:
            if self.journal_path is not None:
                connection = sqlite3.connect(
                    self.journal_path.as_uri() + "?mode=ro", uri=True, timeout=0
                )
                connection.row_factory = sqlite3.Row
                connection.execute("PRAGMA query_only=ON")
                # Even a large journal scan cannot delay shutdown indefinitely.
                connection.set_progress_handler(
                    lambda: int(self.stopping.is_set()), 1000
                )
            while not self.stopping.is_set():
                with self._capture_lock:
                    capture, protocol_events = self._capture, tuple(self._events)
                if capture is not None:
                    robots, resources, charging, state, now, updated_at = capture
                    value = _empty()
                    value.update(fleet_state=state, updated_at=updated_at)
                    owners = {}
                    for resource in resources:
                        expired = (
                            resource.lease is not None
                            and resource.lease.expires_at <= now
                        )
                        owner = resource.lease or resource.former_lease
                        if owner:
                            owners.setdefault(owner.robot_id, []).append(
                                resource.resource_id
                            )
                        value["resources"].append(
                            {
                                "resource_id": resource.resource_id,
                                "kind": resource.kind,
                                "owner": owner.robot_id if owner else None,
                                "waiters": [
                                    waiter.robot_id for waiter in resource.waiters
                                ],
                                "reconciliation_required": resource.reconciliation_required
                                or expired,
                            }
                        )
                    for robot in robots:
                        item = asdict(robot)
                        item["current_resource"] = (
                            ", ".join(owners.get(robot.robot_id, ())) or None
                        )
                        value["robots"].append(item)
                    value["dock_queue"] = [asdict(charge) for charge in charging]
                    if connection is not None:
                        try:
                            connection.execute("BEGIN")
                            value["missions"], value["events"] = self._read_journal(
                                connection
                            )
                        except sqlite3.Error:
                            value["fleet_state"] = "OBSERVER_STORAGE_UNAVAILABLE"
                        finally:
                            connection.rollback()
                    # Journal times are monotonic; protocol receipts use wall time.
                    # Order the merged window by when this observer sees each event.
                    next_observed = {}
                    for event in value["events"]:
                        event_id = event["event_id"]
                        event["observed_at"] = observed_events.get(
                            event_id, time.time()
                        )
                        next_observed[event_id] = event["observed_at"]
                    observed_events = next_observed
                    value["events"] = sorted(
                        value["events"] + list(protocol_events),
                        key=lambda event: event["observed_at"],
                    )[-50:]
                    self.hub.publish(value)
                self.stopping.wait(0.25)
        except (sqlite3.Error, OSError):
            # Observer failure is never a fleet failure. Cache ages in the browser.
            pass
        finally:
            if connection is not None:
                connection.close()

    def start(self):
        if self._server is not None:
            return
        self.stopping.clear()
        server = _HTTPServer(self.address, self)
        self._server = server
        self.address = server.server_address
        try:
            self._thread = threading.Thread(
                target=server.serve_forever,
                kwargs={"poll_interval": 0.05},
                name="fleet-dashboard-http",
            )
            self._observer = threading.Thread(
                target=self._observe, name="fleet-dashboard-observer"
            )
            self._thread.start()
            self._observer.start()
        except BaseException:
            self.stop()
            raise

    def stop(self):
        self.stopping.set()
        self.hub.close()
        server = self._server
        if server is not None:
            # shutdown() waits for serve_forever; a failed Thread.start() never
            # enters that loop and must close the listener directly.
            if self._thread is not None and self._thread.is_alive():
                server.shutdown()
            server.close_connections()
            server.server_close()
            self._server = None
        for thread in (self._thread, self._observer):
            if thread is not None and thread.ident is not None:
                thread.join(timeout=1)
