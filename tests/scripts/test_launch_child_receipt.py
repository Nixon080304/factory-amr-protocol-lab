"""Normal child exits are independent of wrapper interruption and PID absence."""

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
spec = spec_from_file_location(
    "successful_mission_receipt", ROOT / "tests/system/test_successful_mission.py"
)
module = module_from_spec(spec)
spec.loader.exec_module(module)


def test_clean_children_do_not_require_zero_interrupted_wrapper_status():
    receipt = module.launch_child_receipt(
        "[INFO] [gzclient-2]: process started with pid [12]\n"
        "[INFO] [gateway-3]: process started with pid [13]\n"
        "[INFO] [gzclient-2]: process has finished cleanly [pid 12]\n"
        "[INFO] [gateway-3]: process has finished cleanly [pid 13]\n",
        130,
    )
    assert receipt["wrapper_exit_code"] == 130
    assert receipt["all_children_clean"]
    assert receipt["children"] == [
        {"name": "gzclient-2", "pid": 12, "exit_code": 0},
        {"name": "gateway-3", "pid": 13, "exit_code": 0},
    ]


@pytest.mark.parametrize(
    "exit_line,status",
    [
        (
            "[ERROR] [gzclient-2]: process has died [pid 12, exit code -11, cmd 'gzclient'].",
            -11,
        ),
        (
            "[ERROR] [gzclient-2]: process has died [pid 12, exit code 1, cmd 'gzclient'].",
            1,
        ),
        ("", None),
    ],
)
def test_crashed_or_unrecorded_child_prevents_clean_receipt(exit_line, status):
    receipt = module.launch_child_receipt(
        "[INFO] [gzclient-2]: process started with pid [12]\n" + exit_line, 0
    )
    assert not receipt["all_children_clean"]
    assert receipt["children"][0]["exit_code"] == status
