# SPDX-License-Identifier: Apache-2.0
"""The visible part travels on each conveyor before its final placement."""

import pytest

from payload_simulator.node import animation_steps


@pytest.mark.parametrize(
    "kind, x, expected_y, final",
    [
        ("LOADING", -3.0, [2.0, 1.95, 1.9, 1.85, 1.8], (0.0, 0.0, -2.0)),
        ("UNLOADING", 3.0, [1.8, 1.85, 1.9, 1.95, 2.0], (3.0, 2.0, 0.65)),
    ],
)
def test_part_travels_in_bounded_steps_before_final_placement(
    kind, x, expected_y, final
):
    steps = list(animation_steps(kind))
    parts = [target.position for name, target in steps if name == "factory_part"]
    visible = parts[:-1]
    assert [position.y for position in visible] == pytest.approx(expected_y)
    assert all(position.x == x and position.z == 0.65 for position in visible)
    assert (parts[-1].x, parts[-1].y, parts[-1].z) == final
    belts = [target.position for name, target in steps if name != "factory_part"]
    assert len(belts) == 4
    assert belts[-1].y == 2.0
    assert all(position.x == x and position.z == 0.603 for position in belts)
