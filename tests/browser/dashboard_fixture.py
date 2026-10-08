"""Real production observer/server fixture controlled only through stdin."""

from dataclasses import replace
import json
from pathlib import Path
import sys
import tempfile

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src/fleet_manager"))

from fleet_manager.adapter import FleetAdapter
from fleet_manager.config import Pose2D, load_fleet_config
from fleet_manager.journal import MissionJournal, MissionState
from fleet_manager.models import MissionRequest, RobotSnapshot
from fleet_manager.resources import Lease, LeaseRequest


def main():
    try:
        from fleet_manager.web import FleetDashboard
    except ModuleNotFoundError:
        raise AssertionError("read-only fleet dashboard is not implemented") from None
    with tempfile.TemporaryDirectory(prefix="fleet-dashboard-journal-") as directory:
        path = Path(directory) / "missions.sqlite3"
        journal = MissionJournal(path)
        config = load_fleet_config(
            Path(__file__).resolve().parents[2]
            / "src/factory_bringup/config/fleet.yaml"
        )
        adapter = FleetAdapter(config, journal, None, clock=lambda: 100)
        adapter.registry.observe(
            RobotSnapshot("amr_01", "CHARGING", Pose2D(4, -2, 0), 26, "EMPTY"), 100
        )
        adapter.registry.observe(
            RobotSnapshot(
                "amr_02", "EXECUTING", Pose2D(0.5, 0, 0), 72, "LOADED", "delivery-042"
            ),
            100,
        )
        adapter.resources.acquire(LeaseRequest("amr_01", "charging-01", "dock_01"), 100)
        adapter.resources.acquire(
            LeaseRequest("amr_02", "delivery-042", "central_aisle"), 100
        )
        adapter.resources.acquire(
            LeaseRequest("amr_01", "next-aisle", "central_aisle"), 100
        )
        journal.register(
            MissionRequest("delivery-042", "assembly", "inspection", "motor"),
            "hash",
            100,
        )
        journal.transition(
            "delivery-042", MissionState.QUEUED, MissionState.ASSIGNING, {}, 100
        )
        journal.assign("delivery-042", "amr_02", 100)
        journal.transition(
            "delivery-042",
            MissionState.ASSIGNED,
            MissionState.EXECUTING,
            {"payload_ownership": "PICKED_UP"},
            100,
        )
        server = FleetDashboard(port=0, journal_path=path, heartbeat=0.2)
        server.capture(adapter)
        server.start()
        port = server.address[1]
        print(
            json.dumps({"ready": True, "url": f"http://127.0.0.1:{port}"}), flush=True
        )
        try:
            for line in sys.stdin:
                command = json.loads(line)
                if command["command"] == "update":
                    adapter.registry.observe(
                        replace(
                            adapter.registry.get("amr_02", 100),
                            mode="WAITING_FOR_RESOURCE",
                            health_detail="Waiting for inspection station",
                        ),
                        100,
                    )
                    server.capture(adapter)
                elif command["command"] == "stop":
                    server.stop()
                elif command["command"] == "restart":
                    server = FleetDashboard(port=port, journal_path=path, heartbeat=0.2)
                    adapter.registry.observe(
                        replace(
                            adapter.registry.get("amr_01", 100), battery_percent=35
                        ),
                        100,
                    )
                    server.capture(adapter)
                    server.start()
                elif command["command"] == "quarantine":
                    adapter.resources.expire(200)
                    adapter.resources.quarantine_evidence(
                        Lease(
                            "amr_01", "former-aisle", "central_aisle", "old-aisle", 90
                        )
                    )
                    server.capture(adapter)
                elif command["command"] == "dense":
                    with server._capture_lock:
                        server._capture = None
                    snapshot = json.loads(server.hub.snapshot_bytes())
                    snapshot["updated_at"] = __import__("time").time()
                    snapshot["resources"] = [
                        {
                            "resource_id": f"zone-{number}",
                            "kind": "traffic_zone",
                            "owner": None,
                            "waiters": [f"amr-{waiter}" for waiter in range(15)],
                        }
                        for number in range(100)
                    ]
                    server.hub.publish(snapshot)
                elif command["command"] == "truncated":
                    snapshot = json.loads(server.hub.snapshot_bytes())
                    snapshot["resources"] *= 2
                    server.hub.publish(snapshot)
                elif command["command"] == "quit":
                    break
                print(json.dumps({"done": command["command"]}), flush=True)
        finally:
            server.stop()
            journal.close()


if __name__ == "__main__":
    main()
