# SPDX-License-Identifier: Apache-2.0
"""Read the owned legacy demo manager's actual dispatch readiness."""

import json
from urllib.request import urlopen


def manager_ready(port, evidence):
    from fleet_driver import manager_dispatch_ready

    try:
        with urlopen(f"http://127.0.0.1:{port}/api/snapshot", timeout=0.5) as response:
            snapshot = json.load(response)
    except OSError:
        return False
    robots = snapshot["robots"]
    ready = (
        snapshot["fleet_state"] == "RUNNING"
        and manager_dispatch_ready(robots, {"amr_01"})
        and all(
            robot["health"] == "ONLINE"
            and robot["mode"] == "AVAILABLE"
            and robot["payload_state"] == "EMPTY"
            for robot in robots
        )
    )
    if ready:
        evidence.write_text(json.dumps(snapshot, indent=2, allow_nan=False) + "\n")
    return ready
