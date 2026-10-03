# SPDX-License-Identifier: Apache-2.0
"""The real demo cleanup must finish before its real caller escalates it."""

import ast
import ctypes
import json
import math
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import time
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[2]


def identity(pid):
    fields = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
    return dict(
        pid=pid,
        parent=int(fields[1]),
        group=int(fields[2]),
        start_ticks=fields[19],
        state=fields[0],
    )


def caller_stop_block():
    path = ROOT / "tests/system/test_successful_mission.py"
    source = path.read_text()
    function = next(
        node
        for node in ast.parse(source).body
        if isinstance(node, ast.FunctionDef)
        and node.name == "test_successful_factory_mission"
    )
    candidates = [
        node.finalbody[0]
        for node in ast.walk(function)
        if isinstance(node, ast.Try)
        and node.finalbody
        and isinstance(node.finalbody[0], ast.If)
        and ast.unparse(node.finalbody[0].test) == "process.poll() is None"
    ]
    assert len(candidates) == 1, "original caller stop block missing or ambiguous"
    return compile(ast.Module(body=candidates, type_ignores=[]), str(path), "exec")


def observe(mode, directory):
    """Run one finite observation in its own subreaping Python process."""
    assert mode in ("caller", "caller-early", "success", "early", "guard")
    assert ctypes.CDLL(None, use_errno=True).prctl(36, 1, 0, 0, 0) == 0
    source = (ROOT / "scripts/run_demo.sh").read_text()
    cleanup = (
        "cleanup() {"
        + source.split("cleanup() {", 1)[1].split("\ntrap cleanup EXIT", 1)[0]
    )
    directory.mkdir()
    events_path = directory / "events.jsonl"
    marker = directory / "compose-complete"
    ready_path = directory / "ready.json"
    compose = directory / "compose-fixture.py"
    compose.write_text(
        "#!/usr/bin/env python3\n"
        "import json, os, pathlib, sys, time\n"
        "assert sys.argv[1:] == ['down', '--timeout', '5'], sys.argv\n"
        f"events=pathlib.Path({str(events_path)!r})\n"
        "def emit(kind):\n"
        " with events.open('a') as handle:\n"
        "  handle.write(json.dumps(dict(kind=kind,monotonic=time.monotonic(),wall_time=time.time(),pid=os.getpid(),parent=os.getppid()))+'\\n')\n"
        "emit('compose_enter')\n"
        "time.sleep(5)\n"
        f"pathlib.Path({str(marker)!r}).write_text('actual five-second Compose fixture completed\\n')\n"
        "emit('compose_complete')\n"
    )
    compose.chmod(0o700)
    q = shlex.quote
    groups = 2 if mode in ("early", "caller-early") else 1
    actor_ready = [directory / f"actor-{index}-ready" for index in range(groups)]
    spawn = []
    for index, path in enumerate(actor_ready):
        command = f"trap '' INT TERM; : > {q(str(path))}; exec sleep 300"
        spawn.append(f"setsid bash -c {q(command)} &\nactor_{index}=$!")
    # Bash builtin logging adds no subprocess to each grace-period iteration.
    # Epoch timestamps stay labeled separately from the Python monotonic clock.
    logging = f"""
events_file={q(str(events_path))}
kill() {{
    if [[ $1 != -0 ]]; then
        printf '{{"kind":"wrapper_signal","wall_time":%s,"signal":"%s","group":"%s"}}\n' "$EPOCHREALTIME" "$1" "$3" >> "$events_file"
    fi
    builtin kill "$@"
}}
wait() {{
    builtin wait "$@"
    fixture_wait_status=$?
    printf '{{"kind":"wrapper_wait_exit","wall_time":%s,"pid":%s,"status":%s}}\n' "$EPOCHREALTIME" "$1" "$fixture_wait_status" >> "$events_file"
    return "$fixture_wait_status"
}}
"""
    script = (
        "set -u\n"
        + "\n".join(spawn)
        + "\n"
        + "\n".join(
            f"while [[ ! -f {q(str(path))} ]]; do sleep 0.01; done"
            for path in actor_ready
        )
        + (
            f"\n: > {q(str(directory / 'guard-boundary'))}\n"
            + "while :; do sleep 0.05; done\n"
            if mode == "guard"
            else ""
        )
        + f"\nlaunch_pid=$actor_0; readiness_pid={'$actor_1' if groups == 2 else chr(39) + chr(39)}\n"
        + "owns_project=true\ncompose=("
        + q(str(compose))
        + ")\n"
        + logging
        + cleanup
        + "\ntrap cleanup EXIT\ntrap 'exit 130' INT\ntrap 'exit 143' TERM\n"
        + f'printf \'{{"wrapper":%s,"actors":[%s{",%s" if groups == 2 else ""}]}}\\n\' "$$" "$actor_0" '
        + ('"$actor_1" ' if groups == 2 else "")
        + f"> {q(str(ready_path))}\n"
        + "while :; do sleep 0.05; done\n"
    )
    log = (directory / "wrapper.log").open("w")
    process = subprocess.Popen(
        ["bash", "-c", script],
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    owner = identity(process.pid)
    actors = []
    parent_events = []
    guard = None
    started = stopped = None
    actor_statuses = {}
    owned = {owner["pid"]: owner}

    def emit(kind, **values):
        parent_events.append(
            dict(kind=kind, monotonic=time.monotonic(), wall_time=time.time(), **values)
        )

    def verify(record, expected_parent=None):
        current = identity(record["pid"])
        assert (
            current["start_ticks"] == record["start_ticks"]
            and current["group"] == record["group"]
        )
        assert current["state"] != "Z"
        if expected_parent is not None:
            assert current["parent"] == expected_parent
        return current

    def capture_owned():
        # This process is a dedicated subreaper with only this fixture's
        # wrapper/descendants. Capture before readiness and before/after stop.
        for record in descendants(os.getpid()):
            previous = owned.get(record["pid"])
            if previous is not None:
                assert record["start_ticks"] == previous["start_ticks"]
                assert record["group"] == previous["group"]
            else:
                assert record["parent"] == os.getpid() or record["parent"] in owned
                assert record["group"] != os.getpgrp()
                owned[record["pid"]] = record

    saved_send = process.send_signal
    saved_wait = process.wait

    def send(number):
        emit(
            "caller_wrapper_signal",
            signal=int(number),
            identity=verify(owner, os.getpid()),
        )
        return saved_send(number)

    def kill_group(group, number):
        assert group == owner["group"]
        emit(
            "caller_group_signal",
            signal=int(number),
            identity=verify(owner, os.getpid()),
        )
        return os.killpg(group, number)

    def wait(*args, **kwargs):
        emit("caller_wait_enter", timeout=kwargs.get("timeout"))
        try:
            result = saved_wait(*args, **kwargs)
        except subprocess.TimeoutExpired:
            emit("caller_wait_timeout", timeout=kwargs.get("timeout"))
            raise
        emit("caller_wait_exit", actual_status=result)
        return result

    process.send_signal, process.wait = send, wait
    try:
        if mode == "guard":
            deadline = time.monotonic() + 5
            while not (directory / "guard-release").exists():
                capture_owned()
                assert process.poll() is None, "guard wrapper exited before fault"
                assert time.monotonic() < deadline, "outer safety readiness expired"
                time.sleep(0.01)
            emit("intentional_pre_readiness_fault")
            raise AssertionError("named pre-aggregate readiness fault")
        deadline = time.monotonic() + 5
        while not ready_path.exists() and time.monotonic() < deadline:
            capture_owned()
            assert process.poll() is None, "fixture wrapper exited before readiness"
            time.sleep(0.01)
        assert ready_path.exists(), "owned group readiness bound expired"
        ready = json.loads(ready_path.read_text())
        assert ready["wrapper"] == owner["pid"]
        actors = [identity(pid) for pid in ready["actors"]]
        capture_owned()
        assert len(actors) == groups
        verify(owner, os.getpid())
        for actor in actors:
            assert actor["group"] == actor["pid"]
            verify(actor, owner["pid"])
        emit("verified_ready", wrapper=owner, actors=actors)
        started = time.monotonic()
        if mode in ("caller", "caller-early"):
            exec(
                caller_stop_block(),
                {
                    "process": process,
                    "subprocess": subprocess,
                    "os": SimpleNamespace(killpg=kill_group),
                    "signal": signal,
                },
            )
        else:
            send(signal.SIGINT)
            wait(timeout=55)
        stopped = time.monotonic()
    except (AssertionError, OSError, subprocess.TimeoutExpired) as failure:
        guard = repr(failure)
    finally:
        capture_owned()
        if process.poll() is None:
            kill_group(owner["group"], signal.SIGKILL)
            saved_wait(timeout=5)
        process.send_signal, process.wait = saved_send, saved_wait
        capture_owned()
        for actor in owned.values():
            if actor["pid"] == owner["pid"]:
                continue
            try:
                current = identity(actor["pid"])
            except FileNotFoundError:
                continue
            assert (
                current["start_ticks"] == actor["start_ticks"]
                and current["group"] == actor["group"]
            )
            assert current["parent"] == os.getpid() or current["parent"] in owned
            if current["state"] != "Z":
                emit(
                    "owned_actor_reclamation",
                    signal=int(signal.SIGKILL),
                    identity=current,
                )
                # Kill only this exact captured identity. Wrapper-group kill
                # above already handles its group; isolated actors are owned.
                os.kill(current["pid"], signal.SIGKILL)
        # Reap only descendants adopted by this dedicated subreaper. They are
        # created by this one fixture, never unrelated host processes.
        end = time.monotonic() + 5
        while time.monotonic() < end:
            try:
                pid, status = os.waitpid(-1, os.WNOHANG)
            except ChildProcessError:
                break
            if not pid:
                time.sleep(0.01)
            else:
                assert pid in owned, ("uncaptured adopted descendant", pid)
                actor_statuses[str(pid)] = os.waitstatus_to_exitcode(status)
                emit(
                    "adopted_descendant_reaped",
                    pid=pid,
                    actual_status=os.waitstatus_to_exitcode(status),
                )
        else:
            guard = guard or "owned adopted descendant reclamation bound expired"
        log.close()
    wrapper_events = (
        [json.loads(line) for line in events_path.read_text().splitlines()]
        if events_path.exists()
        else []
    )
    term = next(
        (
            row
            for row in parent_events
            if row["kind"] == "caller_group_signal" and row["signal"] == signal.SIGTERM
        ),
        None,
    )
    caller_timeouts = [
        row for row in parent_events if row["kind"] == "caller_wait_timeout"
    ]
    complete = marker.exists()
    clean = all(
        not Path(f"/proc/{record['pid']}").exists() for record in owned.values()
    )
    nominal = groups * 15 + 5
    receipt = dict(
        mode=mode,
        groups=groups,
        guard=guard,
        wrapper=owner,
        actors=actors,
        owned_descendants=list(owned.values()),
        wrapper_status=process.returncode,
        parent_events=parent_events,
        wrapper_events=wrapper_events,
        actor_statuses=actor_statuses,
        marker_complete=complete,
        cleanup_verified=clean,
        elapsed=stopped - started
        if stopped is not None and started is not None
        else None,
        nominal_grace_seconds=nominal,
        measured_overhead=stopped - started - nominal
        if stopped is not None and started is not None and complete
        else None,
        causal_red=bool(
            mode in ("caller", "caller-early")
            and not guard
            and clean
            and term
            and caller_timeouts
            and process.returncode == -signal.SIGTERM
            and not complete
        ),
        source_demo_blob=subprocess.check_output(
            ["git", "hash-object", "scripts/run_demo.sh"], cwd=ROOT, text=True
        ).strip(),
        source_caller_blob=subprocess.check_output(
            ["git", "hash-object", "tests/system/test_successful_mission.py"],
            cwd=ROOT,
            text=True,
        ).strip(),
    )
    (directory / "receipt.json").write_text(json.dumps(receipt, indent=2) + "\n")
    print(json.dumps(receipt), flush=True)
    return 1 if guard or not clean else 0


def descendants(pid):
    records = []
    try:
        children = Path(f"/proc/{pid}/task/{pid}/children").read_text().split()
    except FileNotFoundError:
        return records
    for child in children:
        try:
            record = identity(int(child))
        except FileNotFoundError:
            continue
        records.append(record)
        records.extend(descendants(record["pid"]))
    return records


def guard_safety(directory):
    """Independent outer owner protects the intentionally broken inner guard."""
    assert ctypes.CDLL(None, use_errno=True).prctl(36, 1, 0, 0, 0) == 0
    directory.mkdir()
    sentinel = subprocess.Popen(["sleep", "300"], start_new_session=True)
    sentinel_owner = identity(sentinel.pid)
    inner = subprocess.Popen(
        [sys.executable, __file__, "--observe", "guard", str(directory / "inner")],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        start_new_session=True,
    )
    inner_owner = identity(inner.pid)
    captured = {}
    actual_statuses = {}
    guard = None
    inner_clean = sentinel_untouched = False
    try:
        deadline = time.monotonic() + 5
        while not (directory / "inner/guard-boundary").exists():
            assert inner.poll() is None, "inner exited before owned safety readiness"
            for record in descendants(inner.pid):
                captured[record["pid"]] = record
            assert time.monotonic() < deadline, "guard safety readiness expired"
            time.sleep(0.01)
        for record in descendants(inner.pid):
            captured[record["pid"]] = record
        actors = [
            record
            for record in captured.values()
            if record["group"] == record["pid"] and record["parent"] != inner.pid
        ]
        assert len(actors) == 1, ("owned setsid actor missing or ambiguous", captured)
        (directory / "inner/guard-release").touch()
        stdout, stderr = inner.communicate(timeout=15)
        assert inner.returncode == 1, (inner.returncode, stdout, stderr)
        receipt = json.loads((directory / "inner/receipt.json").read_text())
        assert (
            receipt["guard"] == "AssertionError('named pre-aggregate readiness fault')"
        ), receipt
        inner_clean = all(not Path(f"/proc/{row['pid']}").exists() for row in actors)
        current = identity(sentinel.pid)
        assert all(
            current[key] == sentinel_owner[key]
            for key in ("pid", "start_ticks", "group", "parent")
        )
        sentinel_untouched = sentinel.poll() is None
    except (AssertionError, OSError, subprocess.TimeoutExpired) as failure:
        guard = repr(failure)
    finally:
        # Capture again before reclaim: only descendants of this inner owner,
        # plus already captured actors adopted by this dedicated outer owner.
        for record in descendants(inner.pid):
            captured[record["pid"]] = record
        for record in (inner_owner, *captured.values(), sentinel_owner):
            try:
                current = identity(record["pid"])
            except FileNotFoundError:
                continue
            assert current["start_ticks"] == record["start_ticks"]
            assert current["group"] == record["group"]
            assert current["parent"] in (record["parent"], os.getpid())
            assert current["group"] != os.getpgrp()
            if current["state"] != "Z":
                os.kill(current["pid"], signal.SIGKILL)
        if inner.poll() is None:
            inner.wait(timeout=5)
        sentinel.wait(timeout=5)
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            try:
                pid, status = os.waitpid(-1, os.WNOHANG)
            except ChildProcessError:
                break
            if pid:
                actual_statuses[str(pid)] = os.waitstatus_to_exitcode(status)
            else:
                time.sleep(0.01)
        else:
            guard = guard or "outer owned reclamation bound expired"
    result = dict(
        guard=guard,
        captured=list(captured.values()),
        inner_status=inner.returncode,
        inner_clean=inner_clean,
        sentinel=sentinel_owner,
        sentinel_untouched=sentinel_untouched,
        sentinel_cleanup_status=sentinel.returncode,
        adopted_statuses=actual_statuses,
        safety_cleanup=all(
            not Path(f"/proc/{row['pid']}").exists()
            for row in (inner_owner, *captured.values(), sentinel_owner)
        ),
    )
    (directory / "guard-safety.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result), flush=True)
    return 1 if guard or not result["safety_cleanup"] else 0


def observation(tmp_path, mode):
    output = tmp_path / "observation"
    result = subprocess.run(
        [sys.executable, __file__, "--observe", mode, str(output)],
        capture_output=True,
        text=True,
        timeout=70,
    )
    assert (output / "receipt.json").exists(), result.stdout + result.stderr
    receipt = json.loads((output / "receipt.json").read_text())
    assert result.returncode == 0 and receipt["guard"] is None, (receipt, result.stderr)
    assert receipt["cleanup_verified"], receipt
    return receipt


@pytest.mark.parametrize("mode", ["caller", "caller-early"])
def test_real_caller_allows_actual_cleanup_to_complete(tmp_path, mode):
    receipt = observation(tmp_path, mode)
    assert receipt["marker_complete"], receipt
    assert receipt["wrapper_status"] == 130, receipt


def test_pre_readiness_guard_reclaims_only_owned_descendants(tmp_path):
    output = tmp_path / "guard-safety"
    result = subprocess.run(
        [sys.executable, __file__, "--guard-safety", str(output)],
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert (output / "guard-safety.json").exists(), result.stdout + result.stderr
    receipt = json.loads((output / "guard-safety.json").read_text())
    assert result.returncode == 0 and receipt["guard"] is None, receipt
    assert receipt["safety_cleanup"] and receipt["sentinel_untouched"], receipt
    assert receipt["inner_clean"], receipt


@pytest.mark.parametrize("mode", ["success", "early"])
def test_uninterrupted_source_cleanup_budget(tmp_path, mode):
    receipt = observation(tmp_path, mode)
    assert receipt["marker_complete"] and receipt["wrapper_status"] == 130, receipt
    assert receipt["elapsed"] >= receipt["nominal_grace_seconds"], receipt
    assert (
        math.isfinite(receipt["measured_overhead"])
        and receipt["measured_overhead"] >= 0
    )
    completed = [
        row for row in receipt["wrapper_events"] if row["kind"] == "compose_complete"
    ]
    entered = [
        row for row in receipt["wrapper_events"] if row["kind"] == "compose_enter"
    ]
    assert len(entered) == len(completed) == 1
    assert completed[0]["monotonic"] - entered[0]["monotonic"] >= 5


if __name__ == "__main__":
    if sys.argv[1] == "--guard-safety":
        raise SystemExit(guard_safety(Path(sys.argv[2])))
    assert sys.argv[1] == "--observe"
    raise SystemExit(observe(sys.argv[2], Path(sys.argv[3])))
