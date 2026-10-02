#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail
project_root=$(cd -- "${BASH_SOURCE[0]%/*}/.." && pwd)
export PYTHONNOUSERSITE=1
export PYTHONPATH="$project_root/src/protocol_observer${PYTHONPATH:+:$PYTHONPATH}"
exec python3 - "$project_root" "$@" <<'PY'
import json
import ctypes
import fcntl
import math
import os
from pathlib import Path
import re
import random
import signal
import socket
import subprocess
import sys
import tempfile
import time
import uuid
from protocol_observer.report import compare_scenario, scenario_matrix_markdown

root = Path(sys.argv[1])
expected = json.loads((root / 'tests/scenarios/expected_outcomes.yaml').read_text())
if len(sys.argv) != 3 or sys.argv[2] not in expected:
    print('Usage: scripts/run_scenario.sh <' + '|'.join(expected) + '>', file=sys.stderr)
    sys.exit(2)
scenario = sys.argv[2]
try:
    timeout = float(os.environ.get('FACTORY_SCENARIO_TIMEOUT', '300'))
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError()
    run_id = os.environ.get('FACTORY_RUN_ID', time.strftime('%Y%m%dT%H%M%S') + '-' + uuid.uuid4().hex[:12])
    if not re.fullmatch(r'[a-zA-Z0-9_-]+', run_id):
        raise ValueError()
except ValueError:
    print('Use a positive finite timeout and a safe FACTORY_RUN_ID', file=sys.stderr)
    sys.exit(2)
output = Path(os.environ.get('FACTORY_REPORT_ROOT', root / 'reports')).resolve() / run_id / scenario
output.mkdir(parents=True, exist_ok=False)
override = os.environ.get('FACTORY_SCENARIO_COMMAND')
command = [override, scenario, str(output)] if override else ['bash', '-c',
    'set -e; source .venv/bin/activate; source /opt/ros/humble/setup.bash; '
    'source install/setup.bash; exec python3 tests/scenarios/run_case.py "$@"',
    'factory-scenario', scenario, str(output)]
interrupted = 0
def interrupt(signum, frame):
    global interrupted
    interrupted = signum
signal.signal(signal.SIGINT, interrupt)
signal.signal(signal.SIGTERM, interrupt)
timed_out = False
started = time.monotonic()
# Linux subreaping keeps separately launched sessions owned by this supervisor
# if the case driver dies before its finally block. It also permits reaping.
if ctypes.CDLL(None, use_errno=True).prctl(36, 1, 0, 0, 0) != 0:
    raise OSError(ctypes.get_errno(), 'Cannot own adopted scenario descendants')
owned = {}
def process_identity(pid):
    try:
        fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
        return dict(pid=pid, parent=int(fields[1]), group=int(fields[2]), start_ticks=fields[19], state=fields[0])
    except (FileNotFoundError, ProcessLookupError):
        return None
def capture_descendants():
    entries = [process_identity(int(path.name)) for path in Path('/proc').iterdir() if path.name.isdigit()]
    entries = [entry for entry in entries if entry]
    parents = {os.getpid()}
    while True:
        descendants = [entry for entry in entries if entry['parent'] in parents and entry['pid'] not in parents]
        if not descendants:
            break
        for entry in descendants:
            parents.add(entry['pid'])
            owned[entry['pid']] = entry
def live_owned():
    live = []
    for identity in owned.values():
        current = process_identity(identity['pid'])
        if current and current['start_ticks'] == identity['start_ticks'] and current['group'] == identity['group']:
            if current['state'] != 'Z':
                live.append(current)
    return live
def reap_adopted():
    while True:
        try:
            pid, _ = os.waitpid(-1, os.WNOHANG)
            if not pid:
                return
        except ChildProcessError:
            return
command_startup_error = None
lease = None
ownership = None
def write_lease_metadata(metadata):
    # This descriptor already holds the exclusive flock; never truncate first.
    lease.seek(0)
    lease.truncate()
    lease.write(json.dumps(metadata) + '\n')
    lease.flush()
    os.fsync(lease.fileno())
def allocate_domain():
    global lease, ownership
    used = set()
    for path in list(output.parent.glob('*/actual.json')) + list(output.parent.glob('*/ownership.json')):
        domain = json.loads(path.read_text()).get('domain_id')
        if domain is not None:
            used.add(domain)
    domains = [domain for domain in range(100, 221) if domain not in used]
    random.shuffle(domains)
    for domain in domains:
        candidate = open(Path(tempfile.gettempdir()) / f'factory-amr-scenario-domain-{domain}.lock', 'a+')
        try:
            fcntl.flock(candidate, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            candidate.close()
            continue
        candidate.seek(0)
        content = candidate.read()
        try:
            metadata = json.loads(content) if content else {}
        except ValueError:
            candidate.close()
            continue
        if not isinstance(metadata, dict) or metadata.get('quarantined'):
            candidate.close()
            continue
        lease = candidate
        ownership = dict(domain_id=domain, supervisor=process_identity(os.getpid()),
                         output_dir=str(output), scenario=scenario)
        # Fail closed if finalization cannot verify cleanup. No automatic reclaim.
        write_lease_metadata({**ownership, 'quarantined': True,
                              'cleanup_failures': ['Cleanup verification pending']})
        os.environ['FACTORY_SCENARIO_DOMAIN'] = os.environ['ROS_DOMAIN_ID'] = str(domain)
        (output / 'ownership.json').write_text(json.dumps(ownership) + '\n')
        return
    raise OSError('No isolated scenario DDS domain available; locked or quarantined domains require verification')
def execute_command(log):
    global timed_out, command_startup_error
    try:
        process = subprocess.Popen(command, cwd=root, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    except OSError as error:
        command_startup_error = dict(errno=error.errno, message=str(error))
        return 1
    while process.poll() is None and not interrupted and time.monotonic() - started < timeout:
        capture_descendants()
        time.sleep(0.05)
    timed_out = process.poll() is None and not interrupted
    if process.poll() is None:
        # The case driver unwinds its explicitly owned launch groups and services.
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=30 if not override else 2)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)
    # Adopted descendants retain ownership even after the command leader exits.
    # Every group below has a live member with its recorded Linux start time.
    capture_descendants()
    for group in {entry['group'] for entry in live_owned()}:
        try:
            os.killpg(group, signal.SIGTERM)
        except ProcessLookupError:
            pass
    cleanup_deadline = time.monotonic() + 2
    while live_owned() and time.monotonic() < cleanup_deadline:
        capture_descendants()
        reap_adopted()
        time.sleep(0.02)
    for group in {entry['group'] for entry in live_owned()}:
        try:
            os.killpg(group, signal.SIGKILL)
        except ProcessLookupError:
            pass
    cleanup_deadline = time.monotonic() + 2
    while live_owned() and time.monotonic() < cleanup_deadline:
        reap_adopted()
        time.sleep(0.02)
    reap_adopted()
    capture_descendants()
    return process.returncode
with (output / 'command.log').open('w') as log:
    try:
        allocate_domain()
    except OSError as error:
        command_startup_error = dict(errno=error.errno, message=str(error), stage='domain_allocation')
        command_exit_code = 1
    else:
        command_exit_code = execute_command(log)
cleanup_failures = []
if live_owned():
    cleanup_failures.append('Owned descendants survived explicit cleanup')
resources = json.loads((output / 'resources.json').read_text()) if (output / 'resources.json').exists() else {}
for project in resources.get('compose_projects', []):
    if not re.fullmatch(r'factory-system-[a-f0-9]{32}', project):
        cleanup_failures.append('Invalid owned Compose identity')
        continue
    try:
        subprocess.run(['docker', 'compose', '-f', str(root / 'docker/compose.yaml'), '-p', project,
                        'down', '--timeout', '3'], check=True, capture_output=True, timeout=10)
        remaining = subprocess.check_output(['docker', 'compose', '-f', str(root / 'docker/compose.yaml'),
            '-p', project, 'ps', '-aq'], text=True, timeout=5)
        if remaining.strip():
            cleanup_failures.append('Owned Compose containers survived cleanup')
    except (subprocess.SubprocessError, OSError) as error:
        cleanup_failures.append(f'Owned Compose cleanup failed: {error}')
for container in resources.get('containers', []):
    name, owner = container.get('name', ''), container.get('owner', '')
    if not re.fullmatch(r'factory-fault-[a-f0-9]{32}', name) or not re.fullmatch(r'[a-f0-9]{32}', owner):
        cleanup_failures.append('Invalid owned broker identity')
        continue
    try:
        found = subprocess.run(['docker', 'inspect', '--format',
            '{{.Id}} {{index .Config.Labels "factory-amr.scenario-owner"}}', name],
            capture_output=True, text=True, timeout=5)
        if found.returncode == 0:
            fields = found.stdout.split()
            if len(fields) != 2 or fields[1] != owner or not re.fullmatch(r'[a-f0-9]{64}', fields[0]):
                cleanup_failures.append('Owned broker name now has a different ownership label')
                continue
            container_id = fields[0]
            subprocess.run(['docker', 'rm', '-f', container_id], check=True, capture_output=True, timeout=10)
            if subprocess.run(['docker', 'inspect', container_id], capture_output=True, timeout=5).returncode == 0:
                cleanup_failures.append('Owned broker survived cleanup')
            remaining = subprocess.check_output(['docker', 'ps', '-aq', '--filter',
                'id=' + container_id], text=True, timeout=5)
            if remaining.strip():
                cleanup_failures.append('Owned broker identity remains after cleanup')
        else:
            remaining = subprocess.check_output(['docker', 'ps', '-aq', '--filter',
                'name=^/' + name + '$'], text=True, timeout=5)
            if remaining.strip():
                cleanup_failures.append('Owned broker exists but cannot be inspected')
    except (subprocess.SubprocessError, OSError) as error:
        cleanup_failures.append(f'Owned broker cleanup failed: {error}')
for port in resources.get('ports', []):
    with socket.socket() as connection:
        connection.settimeout(0.2)
        if connection.connect_ex(('127.0.0.1', port)) == 0:
            cleanup_failures.append(f'Owned port {port} remains open')
if lease is not None:
    write_lease_metadata({**ownership, 'quarantined': bool(cleanup_failures),
                          'cleanup_failures': cleanup_failures, 'resources': resources,
                          'owned_processes': list(owned.values())})
    lease.close()
result = compare_scenario(scenario, expected[scenario], output, command_exit_code=command_exit_code,
                          timed_out=timed_out, interrupted=interrupted)
if command_startup_error:
    result['command_startup_error'] = command_startup_error
    result['failures'].append('Command startup failed: ' + command_startup_error['message'])
    result['matched'] = False
result['elapsed_sec'] = time.monotonic() - started
result['output_dir'] = str(output)
result['cleanup_verified'] = not cleanup_failures
result['failures'].extend(cleanup_failures)
result['matched'] = result['matched'] and not cleanup_failures
(output / 'outcome.json').write_text(json.dumps(result, indent=2) + '\n')
(output / 'outcome.md').write_text(scenario_matrix_markdown([result]))
print(json.dumps(dict(scenario=scenario, actual=result['actual'].get('final_state'),
    expected=result['expected']['final_state'], matched=result['matched'], failures=result['failures'],
    output_dir=str(output))))
sys.exit(128 + interrupted if interrupted else 124 if timed_out else
         (command_exit_code if command_exit_code > 0 else 128 - command_exit_code)
         if command_exit_code else 0 if result['matched'] else 1)
PY
