# SPDX-License-Identifier: Apache-2.0
"""The visible part travels on each conveyor before its final placement."""

import pytest

from payload_simulator.node import animation_steps


@pytest.mark.parametrize(
    "kind, x, expected_y, final, final_frame",
    [
        (
            "LOADING",
            -3.0,
            [2.0, 1.95, 1.9, 1.85, 1.8],
            (-0.03, 0.0, 0.28),
            "factory_amr",
        ),
        (
            "UNLOADING",
            3.0,
            [1.8, 1.85, 1.9, 1.95, 2.0],
            (3.0, 2.0, 0.65),
            "world",
        ),
    ],
)
def test_part_travels_in_bounded_steps_before_final_placement(
    kind, x, expected_y, final, final_frame
):
    steps = list(animation_steps(kind))
    assert all(len(step) == 3 for step in steps), "every pose needs a reference frame"
    parts = [
        (target.position, reference)
        for name, target, reference in steps
        if name == "factory_part"
    ]
    visible = parts[:-1]
    assert [position.y for position, _ in visible] == pytest.approx(expected_y)
    assert all(
        position.x == x and position.z == 0.65 and reference == "world"
        for position, reference in visible
    )
    final_position, reference = parts[-1]
    assert (final_position.x, final_position.y, final_position.z) == final
    assert reference == final_frame
    belts = [
        target.position for name, target, reference in steps if name != "factory_part"
    ]
    assert len(belts) == 4
    assert belts[-1].y == 2.0
    assert all(position.x == x and position.z == 0.603 for position in belts)
