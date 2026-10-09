#!/usr/bin/python3
# SPDX-License-Identifier: Apache-2.0
"""Controlled external CDP peer for real helper ownership tests, not rendering."""

import json
import os
from pathlib import Path
import subprocess
import sys
import signal

mode = next(
    arg.removeprefix("--peer=") for arg in sys.argv if arg.startswith("--peer=")
)
profile = next(
    arg.removeprefix("--user-data-dir=")
    for arg in sys.argv
    if arg.startswith("--user-data-dir=")
)
if mode == "ignore-term-reject":
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
children = [subprocess.Popen(["/bin/cat"], stdin=subprocess.PIPE) for _ in range(2)]
Path(os.environ["CDP_PEER_RECEIPT"]).write_text(
    json.dumps(
        {
            "pid": os.getpid(),
            "group": os.getpgrp(),
            "children": [child.pid for child in children],
            "profile": profile,
            "arguments": sys.argv[1:],
        }
    )
)
print("controlled peer startup diagnostic", file=sys.stderr, flush=True)
if mode == "stderr-reject":
    print("x" * 8000 + "DIAGNOSTIC-TAIL", file=sys.stderr, flush=True)
if mode == "cleanup-failure-reject":
    Path(profile).parent.chmod(0o500)
if mode == "pipe-close":
    os.close(3)
    os.close(4)
    import time

    time.sleep(30)
buffer = b""
while True:
    chunk = os.read(3, 65536)
    if not chunk:
        break
    buffer += chunk
    while b"\0" in buffer:
        raw, buffer = buffer.split(b"\0", 1)
        message = json.loads(raw)
        method = message["method"]
        if mode == "timeout":
            continue
        if (
            mode
            in (
                "target-reject",
                "stderr-reject",
                "cleanup-failure-reject",
                "ignore-term-reject",
            )
            and method == "Target.createTarget"
        ) or (mode == "attach-reject" and method == "Target.attachToTarget"):
            response = {
                "id": message["id"],
                "error": {"code": -32000, "message": f"controlled {method} rejection"},
            }
            response["private_payload"] = "NO-RAW-CDP-PAYLOAD"
        else:
            result = {
                "Target.createTarget": {"targetId": "controlled-target"},
                "Target.attachToTarget": {"sessionId": "controlled-session"},
                "Browser.getVersion": {"product": "ControlledCDPPeer/1"},
                "Runtime.evaluate": {"result": {"value": 42}},
            }.get(method, {})
            response = {"id": message["id"], "result": result}
        os.write(4, json.dumps(response).encode() + b"\0")
        if method == "Browser.close":
            for child in children:
                child.stdin.close()
                child.wait(timeout=2)
            sys.exit(0)
