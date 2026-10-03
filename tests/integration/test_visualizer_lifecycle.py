# SPDX-License-Identifier: Apache-2.0
"""Actual visualizer signals defer interruption without swallowing real errors."""

import json
import os
from pathlib import Path
import subprocess
import uuid

import pytest

ROOT = Path(__file__).resolve().parents[2]
NATIVE_ERROR = "Unable to convert call argument to Python object (compile in debug mode for details)"


def probe(tmp_path, case):
    run_id = "visualizer-lifecycle-" + uuid.uuid4().hex
    environment = {
        **os.environ,
        "FACTORY_RUN_ID": run_id,
        "FACTORY_REPORT_ROOT": str(tmp_path),
        "FACTORY_SCENARIO_COMMAND": str(
            Path(__file__).with_name("visualizer_lifecycle_probe.py")
        ),
        "FACTORY_SCENARIO_TIMEOUT": "60",
        "VISUALIZER_PROBE_CASE": case,
        "ROS_LOCALHOST_ONLY": "1",
        "PYTHONNOUSERSITE": "1",
    }
    result = subprocess.run(
        [str(ROOT / "scripts/run_scenario.sh"), "success"],
        cwd=ROOT,
        env=environment,
        capture_output=True,
        text=True,
        timeout=75,
    )
    output = tmp_path / run_id / "success"
    outcome = json.loads((output / "outcome.json").read_text())
    assert outcome["cleanup_verified"], outcome
    assert not outcome["timed_out"], outcome
    assert result.returncode == 1 and not outcome["matched"], (
        "no-mission probe must not masquerade as a mission"
    )
    receipt = json.loads((output / "probe.json").read_text())
    assert receipt["guard_failure"] is None, (
        receipt,
        (output / "command.log").read_text(),
    )
    assert receipt["process_absent"]
    assert receipt["readiness"]["goals"] >= 20 and receipt["readiness"]["markers"] >= 20
    assert (
        receipt["readiness"]["goal_readers"]
        == receipt["readiness"]["marker_readers"]
        == 1
    )
    rows = [
        json.loads(line) for line in (output / "events.jsonl").read_text().splitlines()
    ]
    return receipt, rows, (output / "visualizer.log").read_text()


@pytest.mark.parametrize(
    "case,signal_count",
    [
        ("constructor_sigint", 1),
        ("constructor_sigint_repeated", 3),
        ("constructor_sigterm_repeated", 3),
    ],
)
def test_constructor_sigint_finishes_real_take_before_shutdown(
    tmp_path, case, signal_count
):
    receipt, rows, log = probe(tmp_path, case)
    assert receipt["exit_before_cleanup"] == receipt["actual_child_exit"] == 0, (
        receipt,
        log,
    )
    injected = [
        index for index, row in enumerate(rows) if row["kind"] == "actual_signal"
    ]
    assert len(injected) == signal_count
    first = injected[0]
    constructor = rows[first - 1]
    assert (
        constructor["kind"] == "saved_constructor_completed"
        and constructor["original_calls"] == 1
    )
    assert constructor["take"]["target"] and constructor["take"]["raw"] is False
    assert constructor["take"]["thread"] == constructor["thread"]
    assert all(row["context_ok"] for row in rows[first : injected[-1] + 1])
    after = rows[injected[-1] + 1 :]
    returned = next(
        index
        for index, row in enumerate(after)
        if row["kind"] == "constructor_returned_after_signal"
    )
    taken = next(
        index
        for index, row in enumerate(after)
        if row["kind"] == "native_call_boundary_return"
    )
    completed = next(
        index for index, row in enumerate(after) if row["kind"] == "callback_completed"
    )
    assert returned < taken < completed
    assert all(after[index]["context_ok"] for index in (returned, taken, completed))
    assert not any(row["kind"] == "native_call_boundary_exception" for row in after)
    assert rows[-1]["kind"] == "main_exit" and rows[-1]["exception_type"] is None
    assert (
        rows[-1]["context_ok"] is False
        and rows[-1]["constructor_restored"]
        and rows[-1]["handlers_restored"]
    )
    assert not receipt["external_signals"], "GREEN must not require TERM/KILL cleanup"


@pytest.mark.parametrize("case", ["idle_sigint", "idle_sigterm"])
def test_idle_signal_stops_actual_visualizer(tmp_path, case):
    receipt, rows, log = probe(tmp_path, case)
    assert receipt["exit_before_cleanup"] == receipt["actual_child_exit"] == 0, (
        receipt,
        log,
    )
    assert len(receipt["external_signals"]) == 1
    assert rows[-1]["exception_type"] is None and rows[-1]["handlers_restored"]


@pytest.mark.parametrize(
    "case",
    [
        "callback_error",
        "callback_error_after_stop",
        "constructor_error",
        "constructor_error_after_stop",
    ],
)
def test_unrelated_visualizer_errors_propagate_even_after_stop(tmp_path, case):
    receipt, rows, log = probe(tmp_path, case)
    assert receipt["exit_before_cleanup"] == receipt["actual_child_exit"] == 1, (
        receipt,
        log,
    )
    expected_kind = (
        "unrelated_constructor_error"
        if case.startswith("constructor")
        else "unrelated_callback_error"
    )
    assert any(row["kind"] == expected_kind for row in rows)
    assert (
        rows[-1]["exception_type"] == "RuntimeError" and rows[-1]["handlers_restored"]
    )
    expected_text = (
        NATIVE_ERROR
        if case.startswith("constructor")
        else "unrelated visualization callback defect"
    )
    assert rows[-1]["exception_text"] == expected_text
    assert "Traceback" in log and not receipt["external_signals"]
    if case.endswith("after_stop"):
        assert rows[-1]["actual_signals"] == 1
