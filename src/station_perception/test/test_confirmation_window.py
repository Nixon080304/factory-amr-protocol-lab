"""Require fresh, distinct, consecutive observations for the expected station."""

from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from station_perception.confirmation_window import ConfirmationWindow


def test_five_matching_images_within_inclusive_window_confirm():
    window = ConfirmationWindow("assembly")
    assert [window.observe("assembly", t) for t in [1, 1.3, 1.6, 1.9, 2.5]] == [False] * 4 + [True]


def test_old_observations_expire():
    window = ConfirmationWindow("assembly")
    assert not any(window.observe("assembly", t) for t in [0, 0.4, 0.8, 1.2, 1.5001])
    assert window.observe("assembly", 1.6)
    assert not window.observe("assembly", 4)


def test_wrong_station_breaks_sequence():
    window = ConfirmationWindow("assembly")
    for t in [0, 0.1, 0.2, 0.3]:
        assert not window.observe("assembly", t)
    assert not window.observe("inspection", 0.4)
    assert not window.observe("assembly", 0.5)


def test_alternating_stations_never_confirm():
    window = ConfirmationWindow("assembly")
    assert not any(window.observe(station, i * 0.1)
                   for i, station in enumerate(["assembly", "inspection"] * 8))


def test_reset_discards_confirmation():
    window = ConfirmationWindow("inspection")
    for t in [1, 1.1, 1.2, 1.3, 1.4]:
        result = window.observe("inspection", t)
    assert result
    window.reset()
    assert not window.observe("inspection", 1.5)


@pytest.mark.parametrize("bad_stamp", [1.3, 1.0, float("nan"), float("inf"), -1.0])
def test_duplicate_backward_or_invalid_stamp_cannot_complete_window(bad_stamp):
    window = ConfirmationWindow("assembly")
    for t in [1, 1.1, 1.2, 1.3]:
        assert not window.observe("assembly", t)
    assert not window.observe("assembly", bad_stamp)
    assert not window.observe("assembly", 1.4)
