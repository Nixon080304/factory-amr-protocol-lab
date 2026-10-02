"""Exercise the cycle contract independently of the TCP transport."""
import pytest

from plc_simulator.station_cycle import StationCycle


@pytest.mark.parametrize("unit_id", [1, 2])
def test_valid_sequence_completes_once_and_cleared_request_resets(unit_id):
    station = StationCycle(unit_id, cycle_delay=0.2)
    station.write_register(1, 42)
    station.write_coil(1, True, now=0)
    station.write_coil(2, True, now=0)
    assert station.coils == [False, True, True, False, False]
    station.advance(0.199)
    assert station.registers == [unit_id, 42, 0, 0]
    station.advance(0.2)
    assert station.coils[3] is True
    assert station.registers[2] == 1
    station.advance(10)
    assert station.registers[2] == 1
    station.write_coil(2, False, now=10)
    station.write_coil(1, False, now=10)
    assert station.coils == [True, False, False, False, False]


def test_invalid_request_without_robot_present_cannot_start_later():
    station = StationCycle(1)
    station.write_register(1, 9)
    station.write_coil(2, True, now=0)
    station.write_coil(1, True, now=0)
    station.advance(20)
    assert station.registers[2] == 0
    assert station.coils[3] is False


def test_request_requires_part_code_written_before_presence():
    station = StationCycle(1)
    station.write_coil(1, True, now=0)
    station.write_register(1, 9)
    station.write_coil(2, True, now=0)
    station.advance(20)
    assert station.registers[2] == 0


def test_station_fault_prevents_cycle_and_preserves_fault_code():
    station = StationCycle(2, fault_code=73)
    station.write_register(1, 9)
    station.write_coil(1, True, now=0)
    station.write_coil(2, True, now=0)
    station.advance(20)
    assert station.coils[4] is True
    assert station.registers == [2, 9, 0, 73]


def test_stale_completion_never_increments_counter():
    station = StationCycle(1, stale_completion=True)
    assert station.coils[3] is True
    station.write_register(1, 9)
    station.write_coil(1, True, now=0)
    station.write_coil(2, True, now=0)
    station.advance(20)
    assert station.coils[3] is True
    assert station.registers[2] == 0
    station.write_coil(2, False, now=20)
    assert station.coils[3] is False


def test_timeout_never_completes_and_reset_cancels_pending_cycle():
    station = StationCycle(1, cycle_delay=0.5, timeout=True)
    station.write_register(1, 9)
    station.write_coil(1, True, now=0)
    station.write_coil(2, True, now=0)
    station.advance(20)
    assert station.coils[3] is False
    station.write_coil(2, False, now=20)
    station.advance(100)
    assert station.registers[2] == 0


def test_counter_wraps_and_two_cycles_increment_twice():
    station = StationCycle(1, cycle_delay=0)
    station.registers[2] = 65535
    for expected in [0, 1]:
        station.write_register(1, 9)
        station.write_coil(1, True, now=0)
        station.write_coil(2, True, now=0)
        station.advance(0)
        assert station.registers[2] == expected
        station.write_coil(2, False, now=0)
        station.write_coil(1, False, now=0)
