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


def save_json(path, value):
    temporary = path.with_suffix(path.suffix + ".pending")
    temporary.write_text(json.dumps(value))
    temporary.replace(path)


def identity(pid):
    fields = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
    return dict(
        pid=pid,
        parent=int(fields[1]),
        group=int(fields[2]),
        session=int(fields[3]),
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
    cleanup_errors = []
    startup = True

    def emit(kind, **values):
        row = dict(
            kind=kind, monotonic=time.monotonic(), wall_time=time.time(), **values
        )
        parent_events.append(row)
        with (directory / "parent-events.jsonl").open("a") as handle:
            handle.write(json.dumps(row) + "\n")

    emit("initial_owner", identity=owner)

    def verify(record, expected_parent=None):
        current = identity(record["pid"])
        assert (
            current["start_ticks"] == record["start_ticks"]
            and current["group"] == record["group"]
            and current["session"] == record["session"]
        )
        assert current["state"] != "Z"
        if expected_parent is not None:
            assert current["parent"] == expected_parent
        return current

    def capture_owned():
        # This process is a dedicated subreaper with only this fixture's
        # wrapper/descendants. Capture before readiness and before/after stop.
        failures = []
        for record in descendants(os.getpid()):
            emit("ownership_observation", identity=record)
            try:
                assert record["parent"] == os.getpid() or record["parent"] in owned
                if record["parent"] != os.getpid():
                    parent = identity(record["parent"])
                    assert parent["start_ticks"] == owned[parent["pid"]]["start_ticks"]
                assert record["group"] != os.getpgrp()
                previous = owned.get(record["pid"])
                if previous is not None:
                    assert record["start_ticks"] == previous["start_ticks"]
                    if (record["group"], record["session"]) != (
                        previous["group"],
                        previous["session"],
                    ):
                        assert startup and previous["parent"] == owner["pid"]
                        assert (
                            previous["group"] == previous["session"] == owner["group"]
                        )
                        assert record["group"] == record["session"] == record["pid"]
                        assert record["parent"] in (owner["pid"], os.getpid())
                        emit("verified_startup_setsid", before=previous, after=record)
                        owned[record["pid"]] = record
                else:
                    owned[record["pid"]] = record
            except (AssertionError, OSError) as failure:
                failures.append(repr(failure))
        if failures:
            raise AssertionError(failures)

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
        startup = False
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
        try:
            capture_owned()
        except (AssertionError, OSError) as failure:
            cleanup_errors.append(repr(failure))
        if process.poll() is None:
            try:
                kill_group(owner["group"], signal.SIGKILL)
                saved_wait(timeout=5)
            except (AssertionError, OSError, subprocess.TimeoutExpired) as failure:
                cleanup_errors.append(repr(failure))
        process.send_signal, process.wait = saved_send, saved_wait
        try:
            capture_owned()
        except (AssertionError, OSError) as failure:
            cleanup_errors.append(repr(failure))
        for actor in list(owned.values()):
            if actor["pid"] == owner["pid"]:
                continue
            try:
                current = identity(actor["pid"])
            except FileNotFoundError:
                continue
            try:
                assert current["start_ticks"] == actor["start_ticks"]
                assert current["group"] == actor["group"]
                assert current["session"] == actor["session"]
                assert current["parent"] == os.getpid() or current["parent"] in owned
                if current["parent"] != os.getpid():
                    parent = identity(current["parent"])
                    assert parent["start_ticks"] == owned[parent["pid"]]["start_ticks"]
                if current["state"] != "Z":
                    emit(
                        "owned_actor_reclamation",
                        signal=int(signal.SIGKILL),
                        identity=current,
                    )
                    os.kill(current["pid"], signal.SIGKILL)
            except (AssertionError, OSError) as failure:
                cleanup_errors.append(repr(failure))
        # Reap only descendants adopted by this dedicated subreaper. They are
        # created by this one fixture, never unrelated host processes.
        end = time.monotonic() + 5
        while time.monotonic() < end:
            try:
                pid, status = os.waitpid(-1, os.WNOHANG)
            except ChildProcessError:
                break
            except OSError as failure:
                cleanup_errors.append(repr(failure))
                break
            if not pid:
                time.sleep(0.01)
            else:
                if pid not in owned:
                    cleanup_errors.append(f"uncaptured adopted descendant {pid}")
                actor_statuses[str(pid)] = os.waitstatus_to_exitcode(status)
                emit(
                    "adopted_descendant_reaped",
                    pid=pid,
                    actual_status=os.waitstatus_to_exitcode(status),
                )
        else:
            guard = guard or "owned adopted descendant reclamation bound expired"
        log.close()
        if cleanup_errors:
            guard = guard or repr(cleanup_errors)
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
        cleanup_errors=cleanup_errors,
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
        except (FileNotFoundError, ProcessLookupError):
            # A stat file opened before exit can report ESRCH when read after
            # the parent reaps the process, rather than ENOENT during open.
            continue
        records.append(record)
        records.extend(descendants(record["pid"]))
    return records


def verify_wrapper_startup_setsid(previous, record, inner):
    """Validate a wrapper observed before Popen completes its real setsid."""
    assert previous["pid"] == record["pid"]
    assert previous["start_ticks"] == record["start_ticks"]
    assert previous["parent"] == inner["pid"]
    assert (
        previous["group"]
        == previous["session"]
        == inner["group"]
        == inner["session"]
        == inner["pid"]
    )
    assert record["group"] == record["session"] == record["pid"]
    assert record["parent"] in (inner["pid"], os.getpid())


def test_descendants_handles_exit_after_stat_open(monkeypatch):
    process = subprocess.Popen(["sleep", "300"], start_new_session=True)
    stat_path = Path(f"/proc/{process.pid}/stat")
    original_open = Path.open
    observed = []

    def open_then_reap(path, *args, **kwargs):
        handle = original_open(path, *args, **kwargs)
        if path == stat_path:
            observed.append(process.pid)
            process.kill()
            process.wait(timeout=5)
        return handle

    monkeypatch.setattr(Path, "open", open_then_reap)
    try:
        records = descendants(os.getpid())
        assert observed == [process.pid]
        assert all(record["pid"] != process.pid for record in records)
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(timeout=5)


@pytest.mark.parametrize(
    "invalid_field", [None, "pid", "start_ticks", "group", "session", "parent"]
)
def test_wrapper_startup_setsid_preserves_exact_owned_identity(invalid_field):
    script = (
        "import json, os, runpy\n"
        f"identity = runpy.run_path({str(Path(__file__).resolve())!r})['identity']\n"
        "inner = identity(os.getpid())\n"
        "pid = os.fork()\n"
        "if pid == 0:\n"
        " before = identity(os.getpid())\n"
        " os.setsid()\n"
        " after = identity(os.getpid())\n"
        " print(json.dumps(dict(inner=inner, before=before, after=after)), flush=True)\n"
        " os._exit(0)\n"
        "os.waitpid(pid, 0)\n"
    )
    result = subprocess.run(
        [sys.executable, "-c", script],
        start_new_session=True,
        capture_output=True,
        text=True,
        check=True,
        timeout=5,
    )
    transition = json.loads(result.stdout)
    before, after, inner = (transition[key] for key in ("before", "after", "inner"))
    if invalid_field is None:
        verify_wrapper_startup_setsid(before, after, inner)
    else:
        invalid_values = dict(
            pid=after["pid"] + 1,
            start_ticks=str(int(after["start_ticks"]) + 1),
            group=inner["pid"],
            session=inner["pid"],
            parent=0,
        )
        invalid = dict(after, **{invalid_field: invalid_values[invalid_field]})
        with pytest.raises(AssertionError):
            verify_wrapper_startup_setsid(before, invalid, inner)


def setsid_adapter(directory, arguments):
    """Pause before the real kernel transition; execute the original argv."""
    before = identity(os.getpid())
    save_json(directory / "adapter-before.json", before)
    deadline = time.monotonic() + 20
    while not (directory / "release-setsid").exists():
        assert time.monotonic() < deadline, "setsid release readiness expired"
        time.sleep(0.01)
    os.setsid()
    after = identity(os.getpid())
    save_json(directory / "adapter-after.json", after)
    os.execvp(arguments[0], arguments)


def controlled_inner(directory, fault):
    """Observe the actual capture before/after real setsid, without data edits."""
    os.environ["PATH"] = str(directory / "bin") + os.pathsep + os.environ["PATH"]
    function = next(
        node
        for node in ast.parse(Path(__file__).read_text()).body
        if isinstance(node, ast.FunctionDef) and node.name == "observe"
    )
    outer_try = next(node for node in function.body if isinstance(node, ast.Try))
    final_capture_line = next(
        node.lineno
        for node in ast.walk(outer_try.finalbody[0])
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "capture_owned"
    )
    captured = False
    refreshed = False
    injected = False

    def observer(frame, event, argument):
        nonlocal captured, refreshed, injected
        if (
            event != "return"
            or frame.f_code.co_name != "capture_owned"
            or frame.f_code.co_filename != str(Path(__file__).resolve())
        ):
            return observer
        caller = frame.f_back
        assert caller.f_code is observe.__code__
        before_path = directory / "adapter-before.json"
        if not before_path.exists():
            return observer
        before = json.loads(before_path.read_text())
        record = frame.f_locals["owned"].get(before["pid"])
        if record is None:
            return observer
        if not captured:
            actual = identity(before["pid"])
            assert (
                record["start_ticks"] == actual["start_ticks"] == before["start_ticks"]
            )
            assert (
                record["group"] == record["session"] == caller.f_locals["owner"]["pid"]
            )
            assert actual["parent"] == caller.f_locals["owner"]["pid"]
            save_json(
                directory / "original-captured.json",
                dict(captured=record, actual=actual, wrapper=caller.f_locals["owner"]),
            )
            captured = True
            deadline = time.monotonic() + 20
            while not (directory / "adapter-after.json").exists():
                assert time.monotonic() < deadline, "actual setsid readiness expired"
                time.sleep(0.01)
        elif not refreshed and record["group"] == record["session"] == before["pid"]:
            save_json(
                directory / "refresh-observed.json",
                dict(actual=identity(before["pid"]), captured=record),
            )
            refreshed = True
        if fault and caller.f_lineno == final_capture_line and not injected:
            injected = True
            (directory / "named-observation-fault").touch()
            raise AssertionError("named capture observation fault")
        return observer

    sys.settrace(observer)
    try:
        return observe("guard" if fault else "caller", directory / "inner")
    finally:
        sys.settrace(None)


def guard_safety(directory, controlled=False, fault=False):
    """Independent owner persists evidence before checks and always reclaims."""
    assert ctypes.CDLL(None, use_errno=True).prctl(36, 1, 0, 0, 0) == 0
    directory.mkdir()
    progress = directory / "safety-events.jsonl"
    captured = {}
    actual_statuses = {}
    failures = []

    def emit(kind, **values):
        with progress.open("a") as handle:
            handle.write(
                json.dumps(dict(kind=kind, monotonic=time.monotonic(), **values)) + "\n"
            )

    emit("outer_identity", identity=identity(os.getpid()))
    if controlled:
        (directory / "bin").mkdir()
        shim = directory / "bin/setsid"
        shim.write_text(
            "#!"
            + sys.executable
            + "\nimport os, sys\nos.execv(sys.executable, [sys.executable, "
            + repr(str(Path(__file__).resolve()))
            + ", '--setsid-adapter', "
            + repr(str(directory))
            + "] + sys.argv[1:])\n"
        )
        shim.chmod(0o700)
    sentinel = subprocess.Popen(["sleep", "300"], start_new_session=True)
    sentinel_owner = identity(sentinel.pid)
    emit("sentinel_identity", identity=sentinel_owner)
    arguments = (
        [
            sys.executable,
            __file__,
            "--controlled-inner",
            str(directory),
            str(int(fault)),
        ]
        if controlled
        else [sys.executable, __file__, "--observe", "guard", str(directory / "inner")]
    )
    inner = subprocess.Popen(
        arguments,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        start_new_session=True,
    )
    inner_owner = identity(inner.pid)
    emit("inner_identity", identity=inner_owner)
    captured[inner.pid] = inner_owner
    captured[sentinel.pid] = sentinel_owner
    initial_parents = {inner.pid: os.getpid(), sentinel.pid: os.getpid()}
    startup = True
    inner_clean = sentinel_untouched = False

    def census():
        for record in descendants(os.getpid()):
            emit("ownership_observation", identity=record)
            try:
                assert record["parent"] == os.getpid() or record["parent"] in captured
                if record["parent"] != os.getpid():
                    assert (
                        identity(record["parent"])["start_ticks"]
                        == captured[record["parent"]]["start_ticks"]
                    )
                previous = captured.get(record["pid"])
                if previous:
                    assert previous["start_ticks"] == record["start_ticks"]
                    if (previous["group"], previous["session"]) != (
                        record["group"],
                        record["session"],
                    ):
                        assert startup and record["pid"] not in (
                            inner.pid,
                            sentinel.pid,
                        )
                        if initial_parents[record["pid"]] == inner.pid:
                            verify_wrapper_startup_setsid(previous, record, inner_owner)
                        else:
                            wrapper_pid = initial_parents[record["pid"]]
                            assert wrapper_pid in captured
                            assert initial_parents[wrapper_pid] == inner.pid
                            assert (
                                previous["group"] == previous["session"] == wrapper_pid
                            )
                            assert record["group"] == record["session"] == record["pid"]
                            assert record["parent"] in (wrapper_pid, os.getpid())
                            if controlled:
                                transition = json.loads(
                                    (directory / "adapter-after.json").read_text()
                                )
                                assert all(
                                    transition[key] == record[key]
                                    for key in (
                                        "pid",
                                        "start_ticks",
                                        "group",
                                        "session",
                                    )
                                )
                        emit("verified_startup_setsid", before=previous, after=record)
                assert record["group"] != os.getpgrp()
                captured[record["pid"]] = record
                initial_parents.setdefault(record["pid"], record["parent"])
            except (AssertionError, OSError) as failure:
                failures.append(repr(failure))

    def current_owned(record):
        current = identity(record["pid"])
        assert all(
            current[key] == record[key]
            for key in ("pid", "start_ticks", "group", "session")
        )
        assert current["parent"] == os.getpid() or current["parent"] in captured
        if current["parent"] != os.getpid():
            assert (
                identity(current["parent"])["start_ticks"]
                == captured[current["parent"]]["start_ticks"]
            )
        assert current["group"] != os.getpgrp()
        return current

    try:
        deadline = time.monotonic() + 20
        boundary = directory / (
            "original-captured.json" if controlled else "inner/guard-boundary"
        )
        while not boundary.exists():
            census()
            assert not failures, failures
            assert inner.poll() is None, "inner exited before safety readiness"
            assert time.monotonic() < deadline, "safety readiness expired"
            time.sleep(0.01)
        census()
        if controlled:
            before = json.loads((directory / "adapter-before.json").read_text())
            assert before["group"] == before["session"] == before["parent"]
            (directory / "release-setsid").touch()
            deadline = time.monotonic() + 20
            while not (directory / "adapter-after.json").exists():
                assert time.monotonic() < deadline, (
                    "kernel transition readiness expired"
                )
                time.sleep(0.01)
            after = json.loads((directory / "adapter-after.json").read_text())
            assert (
                after["pid"] == before["pid"]
                and after["start_ticks"] == before["start_ticks"]
            )
            assert after["group"] == after["session"] == after["pid"]
            census()
        startup = False
        if not controlled or fault:
            (directory / "inner/guard-release").touch()
        stdout, stderr = inner.communicate(
            timeout=45 if controlled and not fault else 10
        )
        emit("inner_wait", actual_status=inner.returncode, stdout=stdout, stderr=stderr)
        receipt = json.loads((directory / "inner/receipt.json").read_text())
        assert inner.returncode == (1 if not controlled or fault else 0), receipt
        if not controlled or fault:
            assert (
                receipt["guard"]
                == "AssertionError('named pre-aggregate readiness fault')"
            ), receipt
        if fault:
            assert receipt["cleanup_errors"] == [
                "AssertionError('named capture observation fault')"
            ], receipt
        else:
            assert not receipt["cleanup_errors"], receipt
        if controlled:
            assert (directory / "refresh-observed.json").exists()
            assert any(
                row["kind"] == "verified_startup_setsid"
                for row in receipt["parent_events"]
            ), receipt
        inner_clean = receipt["cleanup_verified"] and all(
            not Path(f"/proc/{row['pid']}").exists()
            for row in captured.values()
            if row["pid"] != sentinel.pid
        )
        current = current_owned(sentinel_owner)
        sentinel_untouched = sentinel.poll() is None and current["state"] != "Z"
        emit("sentinel_before_safety", identity=current, untouched=sentinel_untouched)
    except (AssertionError, OSError, subprocess.TimeoutExpired) as failure:
        failures.append(repr(failure))
    finally:
        # No capture/validation/reap assertion may skip another valid target.
        deadline = time.monotonic() + 5
        try:
            census()
        except (AssertionError, OSError) as failure:
            failures.append(repr(failure))
        # Stop established spawning parents first. A child created concurrently
        # with the stop is captured/reclaimed in the post-stop bounded loop.
        producers = [
            record
            for record in captured.values()
            if record["pid"] == inner.pid or initial_parents[record["pid"]] == inner.pid
        ]
        for record in producers:
            try:
                current = current_owned(record)
                if current["state"] != "Z":
                    emit(
                        "owned_producer_stop",
                        identity=current,
                        signal=int(signal.SIGKILL),
                    )
                    os.kill(current["pid"], signal.SIGKILL)
            except FileNotFoundError:
                continue
            except (AssertionError, OSError) as failure:
                failures.append(repr(failure))
        # Persist the sentinel snapshot even if acceptance failed. That does
        # not turn an unreached sentinel assertion into a passing predicate.
        try:
            emit(
                "sentinel_cleanup_snapshot",
                identity=current_owned(sentinel_owner),
                acceptance_reached=sentinel_untouched,
            )
        except (AssertionError, OSError) as failure:
            failures.append(repr(failure))
        while time.monotonic() < deadline:
            try:
                census()
            except (AssertionError, OSError) as failure:
                failures.append(repr(failure))
            for record in list(captured.values()):
                try:
                    current = current_owned(record)
                    if current["state"] != "Z":
                        emit(
                            "owned_safety_signal",
                            identity=current,
                            signal=int(signal.SIGKILL),
                        )
                        os.kill(current["pid"], signal.SIGKILL)
                except FileNotFoundError:
                    continue
                except (AssertionError, OSError) as failure:
                    failures.append(repr(failure))
            while True:
                try:
                    pid, status = os.waitpid(-1, os.WNOHANG)
                except ChildProcessError:
                    break
                except OSError as failure:
                    failures.append(repr(failure))
                    break
                if not pid:
                    break
                actual_statuses[str(pid)] = os.waitstatus_to_exitcode(status)
                emit(
                    "adopted_wait",
                    pid=pid,
                    actual_status=actual_statuses[str(pid)],
                    identity_known=pid in captured,
                )
                for process in (inner, sentinel):
                    if process.pid == pid:
                        process.returncode = actual_statuses[str(pid)]
                if pid not in captured:
                    failures.append(f"unproven adopted identity {pid}")
            if all(
                not Path(f"/proc/{record['pid']}").exists()
                for record in captured.values()
            ):
                break
            time.sleep(0.01)
        else:
            failures.append("outer owned reclamation bound expired")
    result = dict(
        guard=repr(failures) if failures else None,
        captured=list(captured.values()),
        inner_status=inner.returncode,
        inner_clean=inner_clean,
        sentinel=sentinel_owner,
        sentinel_untouched=sentinel_untouched,
        sentinel_cleanup_status=sentinel.returncode,
        adopted_statuses=actual_statuses,
        safety_cleanup=all(
            not Path(f"/proc/{row['pid']}").exists() for row in captured.values()
        ),
    )
    (directory / "guard-safety.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result), flush=True)
    return 1 if failures or not result["safety_cleanup"] else 0


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


@pytest.mark.parametrize("fault", [False, True], ids=["actual-setsid", "capture-error"])
def test_actual_setsid_cleanup_retains_capture_errors(tmp_path, fault):
    output = tmp_path / "transition-safety"
    result = subprocess.run(
        [sys.executable, __file__, "--transition-safety", str(output), str(int(fault))],
        capture_output=True,
        text=True,
        timeout=70,
    )
    assert (output / "guard-safety.json").exists(), result.stdout + result.stderr
    receipt = json.loads((output / "guard-safety.json").read_text())
    assert result.returncode == 0 and receipt["guard"] is None, receipt
    assert receipt["inner_clean"] and receipt["safety_cleanup"], receipt
    assert receipt["sentinel_untouched"], receipt
    assert receipt["inner_status"] == (1 if fault else 0), receipt
    before = json.loads((output / "adapter-before.json").read_text())
    after = json.loads((output / "adapter-after.json").read_text())
    captured = json.loads((output / "original-captured.json").read_text())
    refreshed = json.loads((output / "refresh-observed.json").read_text())
    assert captured["captured"]["group"] == captured["wrapper"]["pid"]
    assert captured["captured"]["session"] == captured["wrapper"]["pid"]
    assert (
        before["pid"] == after["pid"] and before["start_ticks"] == after["start_ticks"]
    )
    assert after["group"] == after["session"] == after["pid"]
    assert (
        refreshed["captured"]["group"]
        == refreshed["captured"]["session"]
        == after["pid"]
    )
    assert refreshed["captured"]["start_ticks"] == before["start_ticks"]
    assert (output / "named-observation-fault").exists() == fault


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
    if sys.argv[1] == "--setsid-adapter":
        setsid_adapter(Path(sys.argv[2]), sys.argv[3:])
    if sys.argv[1] == "--controlled-inner":
        raise SystemExit(controlled_inner(Path(sys.argv[2]), bool(int(sys.argv[3]))))
    if sys.argv[1] == "--transition-safety":
        raise SystemExit(
            guard_safety(
                Path(sys.argv[2]), controlled=True, fault=bool(int(sys.argv[3]))
            )
        )
    if sys.argv[1] == "--guard-safety":
        raise SystemExit(guard_safety(Path(sys.argv[2])))
    assert sys.argv[1] == "--observe"
    raise SystemExit(observe(sys.argv[2], Path(sys.argv[3])))
