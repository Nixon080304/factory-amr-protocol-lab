"""Exercise the public runner with a transport-independent command boundary."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import pytest

ROOT = Path(__file__).resolve().parents[2]
RUNNER = ROOT / 'scripts/run_scenario.sh'


@pytest.fixture
def command(tmp_path):
    path = tmp_path / 'fake.py'
    path.write_text('''#!/usr/bin/env python3
import json, os, pathlib, signal, subprocess, sys, time
scenario, output = sys.argv[1:]
output = pathlib.Path(output)
(output / "started").write_text("started")
mode = os.environ.get("FAKE_MODE", "success")
if mode == "exit": sys.exit(17)
if mode == "orphan":
    child = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    (output / "child.pid").write_text(str(child.pid))
    sys.exit(17)
if mode == "detached":
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    child = subprocess.Popen([sys.executable, "-c", "import signal, time; signal.signal(signal.SIGTERM, signal.SIG_IGN); time.sleep(60)"],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    (output / "child.pid").write_text(str(child.pid))
    start = pathlib.Path(f"/proc/{child.pid}/stat").read_text().rsplit(")", 1)[1].split()[19]
    (output / "resources.json").write_text(json.dumps(dict(groups=[dict(pid=child.pid, start_ticks=start)],
                                                          containers=[], compose_projects=[])))
    time.sleep(60)
if mode == "hang":
    child = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"])
    (output / "child.pid").write_text(str(child.pid))
    (output / "runner.pid").write_text(str(os.getppid()))
    time.sleep(60)
if mode == "foreign_hint":
    (output / "resources.json").write_text(json.dumps(dict(groups=[dict(
        pid=int(os.environ["FOREIGN_PID"]), start_ticks=os.environ["FOREIGN_START"])],
        containers=[], compose_projects=[])))
actual = dict(final_state="FAILED" if mode == "wrong_state" else "COMPLETED",
    error_code=None, source="real_gazebo", mission_id="M-test", action_executions=1)
(output / "actual.json").write_text(json.dumps(actual))
events = [dict(mission_id="M-test", protocol="ROS", event="mission_started", outcome=""),
    dict(mission_id="M-test", protocol="ROS", event="mission_finished", outcome="COMPLETED"),
    dict(mission_id="M-test", protocol="MODBUS", event="modbus_pickup_finished", outcome="SUCCEEDED"),
    dict(mission_id="M-test", protocol="MODBUS", event="modbus_dropoff_finished", outcome="SUCCEEDED")]
if mode == "missing_trace": events.pop()
(output / "protocol_events.jsonl").write_text("".join(json.dumps(event)+"\\n" for event in events))
''')
    path.chmod(0o755)
    return path


def environment(tmp_path, command, mode='success'):
    return {**os.environ, 'FACTORY_SCENARIO_COMMAND': str(command),
            'FACTORY_REPORT_ROOT': str(tmp_path / 'reports'), 'FAKE_MODE': mode,
            'FACTORY_SCENARIO_TIMEOUT': '0.4'}


def run(tmp_path, command, mode='success', scenario='success'):
    assert RUNNER.exists(), 'public scenario runner missing'
    return subprocess.run([str(RUNNER), scenario], cwd=ROOT,
                          env=environment(tmp_path, command, mode), capture_output=True, text=True, timeout=10)


def outcomes(tmp_path):
    return [json.loads(path.read_text()) for path in (tmp_path / 'reports').glob('*/*/outcome.json')]


def test_unknown_name_fails_before_command_starts(tmp_path, command):
    result = run(tmp_path, command, scenario='unknown')
    assert result.returncode == 2
    assert not list(tmp_path.rglob('started'))


def test_success_compares_actual_state_and_trace_in_isolated_directories(tmp_path, command):
    first, second = run(tmp_path, command), run(tmp_path, command)
    assert first.returncode == second.returncode == 0, first.stderr + second.stderr
    reports = outcomes(tmp_path)
    assert len(reports) == 2 and all(row['matched'] for row in reports)
    assert all(row['actual']['final_state'] == row['expected']['final_state'] == 'COMPLETED' for row in reports)
    assert all(row['trace_assertions'] for row in reports)


@pytest.mark.parametrize('mode', ['wrong_state', 'missing_trace'])
def test_state_or_trace_mismatch_returns_nonzero(tmp_path, command, mode):
    result = run(tmp_path, command, mode)
    assert result.returncode == 1
    assert not outcomes(tmp_path)[0]['matched']
    assert outcomes(tmp_path)[0]['failures']


def test_command_exit_code_is_propagated(tmp_path, command):
    result = run(tmp_path, command, 'exit')
    assert result.returncode == 17
    assert outcomes(tmp_path)[0]['command_exit_code'] == 17


def wait_stopped(pid):
    end = time.monotonic() + 3
    while time.monotonic() < end:
        path = Path(f'/proc/{pid}/stat')
        if not path.exists() or path.read_text().split()[2] == 'Z':
            return
        time.sleep(0.02)
    pytest.fail(f'owned child {pid} survived cleanup')


def test_true_timeout_stops_owned_child_and_records_timeout(tmp_path, command):
    started = time.monotonic()
    result = run(tmp_path, command, 'hang')
    assert result.returncode == 124
    assert time.monotonic() - started < 5
    assert outcomes(tmp_path)[0]['timed_out']
    wait_stopped(int(next(tmp_path.rglob('child.pid')).read_text()))


def test_interruption_cleans_owned_child_and_preserves_signal_exit(tmp_path, command):
    assert RUNNER.exists(), 'public scenario runner missing'
    env = environment(tmp_path, command, 'hang')
    env['FACTORY_SCENARIO_TIMEOUT'] = '60'
    process = subprocess.Popen([str(RUNNER), 'success'], cwd=ROOT, env=env,
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    end = time.monotonic() + 5
    while not list(tmp_path.rglob('child.pid')) and time.monotonic() < end:
        time.sleep(0.02)
    assert list(tmp_path.rglob('child.pid')), 'fake command never started'
    process.send_signal(signal.SIGTERM)
    process.communicate(timeout=10)
    assert process.returncode == 143
    assert outcomes(tmp_path)[0]['interrupted'] == 15
    wait_stopped(int(next(tmp_path.rglob('child.pid')).read_text()))


def test_matrix_reports_every_case_despite_unexpected_outcomes(tmp_path, command):
    runner = ROOT / 'scripts/run_all_scenarios.sh'
    assert runner.exists(), 'public matrix runner missing'
    result = subprocess.run([str(runner)], cwd=ROOT, env=environment(tmp_path, command),
                            capture_output=True, text=True, timeout=15)
    assert result.returncode == 1
    summary = json.loads(next((tmp_path / 'reports').glob('*/summary.json')).read_text())
    assert summary['complete'] and len(summary['outcomes']) == 12
    assert summary['unexpected_outcomes'] == 11
    assert len({row['output_dir'] for row in summary['outcomes']}) == 12
    assert 'Unexpected outcomes: 11' in next((tmp_path / 'reports').glob('*/summary.md')).read_text()


def test_failed_command_cannot_leave_owned_child_running(tmp_path, command):
    result = run(tmp_path, command, 'orphan')
    assert result.returncode == 17
    wait_stopped(int(next(tmp_path.rglob('child.pid')).read_text()))


def test_matrix_interruption_stops_active_case_and_writes_partial_report(tmp_path, command):
    env = environment(tmp_path, command, 'hang')
    env['FACTORY_SCENARIO_TIMEOUT'] = '60'
    process = subprocess.Popen([str(ROOT / 'scripts/run_all_scenarios.sh')], cwd=ROOT, env=env,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    end = time.monotonic() + 5
    while not list(tmp_path.rglob('runner.pid')) and time.monotonic() < end:
        time.sleep(0.02)
    assert list(tmp_path.rglob('runner.pid'))
    runner_pid = int(next(tmp_path.rglob('runner.pid')).read_text())
    try:
        process.send_signal(signal.SIGTERM)
        process.wait(timeout=10)
        assert process.returncode == 143
        summary = json.loads(next((tmp_path / 'reports').glob('*/summary.json')).read_text())
        assert not summary['complete'] and len(summary['outcomes']) == 1
        wait_stopped(int(next(tmp_path.rglob('child.pid')).read_text()))
    finally:
        # Clean the exact runner from the RED regression, even if the matrix
        # process does not yet forward its interruption.
        try:
            os.kill(runner_pid, signal.SIGTERM)
        except ProcessLookupError:
            pass


@pytest.mark.parametrize('interrupt', [False, True])
def test_detached_owned_group_stops_after_timeout_or_interruption(tmp_path, command, interrupt):
    env = environment(tmp_path, command, 'detached')
    if interrupt:
        env['FACTORY_SCENARIO_TIMEOUT'] = '60'
    process = subprocess.Popen([str(RUNNER), 'success'], cwd=ROOT, env=env,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    end = time.monotonic() + 5
    while not list(tmp_path.rglob('child.pid')) and time.monotonic() < end:
        time.sleep(0.02)
    assert list(tmp_path.rglob('child.pid'))
    pid = int(next(tmp_path.rglob('child.pid')).read_text())
    try:
        if interrupt:
            process.send_signal(signal.SIGTERM)
        process.wait(timeout=10)
        assert process.returncode == (143 if interrupt else 124)
        wait_stopped(pid)
    finally:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass


@pytest.mark.parametrize('stale', [False, True])
def test_foreign_process_hints_never_authorize_group_cleanup(tmp_path, command, stale):
    sentinel = subprocess.Popen(['sleep', '30'], start_new_session=True)
    try:
        start = Path(f'/proc/{sentinel.pid}/stat').read_text().rsplit(')', 1)[1].split()[19]
        env = {**environment(tmp_path, command, 'foreign_hint'), 'FOREIGN_PID': str(sentinel.pid),
               'FOREIGN_START': str(int(start) + 1) if stale else start}
        result = subprocess.run([str(RUNNER), 'success'], cwd=ROOT, env=env,
                                capture_output=True, text=True, timeout=10)
        assert result.returncode == 0, result.stderr
        assert sentinel.poll() is None
        assert outcomes(tmp_path)[0]['cleanup_verified']
    finally:
        sentinel.terminate()
        sentinel.wait(timeout=5)
