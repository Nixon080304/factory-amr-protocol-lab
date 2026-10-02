#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail
project_root=$(cd -- "${BASH_SOURCE[0]%/*}/.." && pwd)
export PYTHONNOUSERSITE=1
export PYTHONPATH="$project_root/src/protocol_observer${PYTHONPATH:+:$PYTHONPATH}"
exec python3 - "$project_root" <<'PY'
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid
from protocol_observer.report import scenario_matrix_markdown
root = Path(sys.argv[1])
expected = json.loads((root / 'tests/scenarios/expected_outcomes.yaml').read_text())
run_id = time.strftime('%Y%m%dT%H%M%S') + '-' + uuid.uuid4().hex[:12]
environment = {**os.environ, 'FACTORY_RUN_ID': run_id}
output = Path(os.environ.get('FACTORY_REPORT_ROOT', root / 'reports')).resolve() / run_id
outcomes = []
interrupted = 0
process = None
def interrupt(signum, frame):
    global interrupted
    interrupted = signum
    if process is not None and process.poll() is None:
        process.send_signal(signum)
signal.signal(signal.SIGINT, interrupt)
signal.signal(signal.SIGTERM, interrupt)
for scenario in expected:
    if interrupted:
        break
    process = subprocess.Popen([str(root / 'scripts/run_scenario.sh'), scenario], env=environment, cwd=root)
    returncode = process.wait()
    path = output / scenario / 'outcome.json'
    if not path.exists():
        raise SystemExit(f'{scenario} failed without an outcome: exit {returncode}')
    outcomes.append(json.loads(path.read_text()))
    if interrupted or returncode in (130, 143):
        break
summary = dict(run_id=run_id, outcomes=outcomes, unexpected_outcomes=sum(not row['matched'] for row in outcomes),
               complete=len(outcomes) == len(expected))
(output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
(output / 'summary.md').write_text(scenario_matrix_markdown(outcomes))
print(f'Matrix report: {output}; unexpected outcomes: {summary["unexpected_outcomes"]}')
sys.exit(128 + interrupted if interrupted else 0 if summary['complete'] and not summary['unexpected_outcomes'] else 1)
PY
