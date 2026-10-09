# SPDX-License-Identifier: Apache-2.0
"""Catch unleased restart domains and release leaks on failed fixture setup."""

import importlib.util
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tests/system"))
fleet_isolation = importlib.import_module("fleet_isolation")


@pytest.fixture
def restart_module():
    spec = importlib.util.spec_from_file_location(
        "restart_domain_contract",
        ROOT / "src/factory_bringup/test/test_restart_scenarios.py",
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def controlled_pool(monkeypatch, tmp_path):
    monkeypatch.setattr(fleet_isolation, "domain_choices", lambda: [20, 21])
    monkeypatch.setattr(
        fleet_isolation, "Path", lambda value: tmp_path / Path(value).name
    )


def reserve_fixture(module):
    fixture = getattr(module, "isolated_restart_domain", None)
    assert fixture is not None, "restart cases must reserve the shared locked pool"
    return fixture.__wrapped__()


def test_restart_fixture_excludes_shared_owner_and_releases_lock(
    restart_module, controlled_pool
):
    reservation = reserve_fixture(restart_module)
    first = next(reservation)
    other = fleet_isolation.Domain()
    try:
        assert first == 20
        assert other.id == 21, (
            "a concurrent fleet runner must not reuse the case domain"
        )
        reservation.close()
        replacement = fleet_isolation.Domain()
        try:
            assert replacement.id == 20, (
                "completed cases must release their exact lease"
            )
        finally:
            replacement.close()
    finally:
        reservation.close()
        other.close()


def test_restart_fixture_releases_lock_when_case_raises(
    restart_module, controlled_pool
):
    reservation = reserve_fixture(restart_module)
    assert next(reservation) == 20
    with pytest.raises(RuntimeError, match="case setup failed"):
        reservation.throw(RuntimeError("case setup failed"))
    replacement = fleet_isolation.Domain()
    try:
        assert replacement.id == 20
    finally:
        replacement.close()


def test_restart_case_initializes_dds_in_its_reserved_domain(
    restart_module, tmp_path, monkeypatch
):
    import inspect
    import rclpy

    case = restart_module.test_actual_kill_restart_failure_matrix
    assert "isolated_restart_domain" in inspect.signature(case).parameters, (
        "the actual restart matrix must consume its reserved domain"
    )

    class InitializationCaptured(Exception):
        pass

    observed = []

    def capture_init(*, context, domain_id):
        # Stop before creating DDS nodes or subprocesses; the real matrix below
        # independently exercises initialization, heartbeats and restart calls.
        observed.append(domain_id)
        raise InitializationCaptured

    monkeypatch.setattr(rclpy, "init", capture_init)
    with pytest.raises(InitializationCaptured):
        case(tmp_path, *restart_module.MATRIX[0], isolated_restart_domain=21)
    assert observed == [21], "fixed or inherited domains must not replace the lease"
