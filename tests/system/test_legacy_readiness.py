# SPDX-License-Identifier: Apache-2.0
"""Legacy missions wait for their own fleet manager, never observer discovery."""

import importlib
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import sys
import threading

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))


@pytest.mark.parametrize("missing", ["cost", "action", "health", None])
def test_legacy_readiness_uses_owned_manager_snapshot(tmp_path, missing):
    spec = importlib.util.find_spec("legacy_readiness")
    assert spec is not None, "legacy manager-view readiness is missing"
    ready = importlib.import_module("legacy_readiness").manager_ready
    robot = dict(
        robot_id="amr_01",
        cost_service_ready=missing != "cost",
        mission_action_ready=missing != "action",
        health="OFFLINE" if missing == "health" else "ONLINE",
        mode="AVAILABLE",
        payload_state="EMPTY",
    )
    paths = []

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            paths.append(self.path)
            body = json.dumps(dict(fleet_state="RUNNING", robots=[robot])).encode()
            self.send_response(200)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *args):
            pass

    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    thread = threading.Thread(target=server.serve_forever)
    thread.start()
    try:
        evidence = tmp_path / "startup-snapshot.json"
        assert ready(server.server_port, evidence) is (missing is None)
        assert paths == ["/api/snapshot"]
        assert evidence.exists() is (missing is None)
        if missing is None:
            assert json.loads(evidence.read_text())["robots"] == [robot]
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)
        assert not thread.is_alive()
