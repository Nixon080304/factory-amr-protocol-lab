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


def test_owned_cycle_cannot_be_cleared_or_completed_by_wrong_robot():
    station = StationCycle(1, cycle_delay=0)
    assert station.claim("amr_01", "M-1", "motor")
    assert not station.claim("amr_02", "M-2", "motor")
    station.write_register(1, 1)
    station.write_coil(1, True, now=0)
    station.write_coil(2, True, now=0)
    assert not station.complete("amr_02", "M-1", now=1)
    assert station.registers[2] == 0
    assert not station.release("amr_02", "M-1", "motor")
    assert station.complete("amr_01", "M-1", now=1)
    assert station.status()["robot_id"] == "amr_01"
    assert station.status()["mission_id"] == "M-1"
    assert station.status()["cycle_counter"] == 1
    station.write_coil(2, False, now=1)
    station.write_coil(1, False, now=1)
    assert station.release("amr_01", "M-1", "motor")
    assert station.status()["robot_id"] == ""


def test_control_channel_carries_typed_identity_without_changing_raw_map():
    import asyncio
    import json
    import struct
    from plc_simulator.server import PlcServer

    async def command(server, operation, robot="amr_01", mission="M-1"):
        reader = asyncio.StreamReader()
        fields = dict(
            operation=operation,
            unit_id=1,
            robot_id=robot,
            mission_id=mission,
            part="motor",
        )
        encoded = json.dumps(fields).encode()
        reader.feed_data(b"\xff" + struct.pack("!H", len(encoded)) + encoded)
        reader.feed_eof()
        output = bytearray()

        class Writer:
            def write(self, data):
                output.extend(data)

            async def drain(self):
                pass

        await server._control_request(reader, Writer())
        assert output[:1] == b"\xff"
        return json.loads(output[3:])

    async def scenario():
        server = PlcServer()
        assert (await command(server, "claim"))["accepted"]
        assert not (await command(server, "claim", "amr_02", "M-2"))["accepted"]
        assert not (await command(server, "release", "amr_02"))["accepted"]
        assert server.stations[1].status()["robot_id"] == "amr_01"
        assert (await command(server, "status"))["robot_id"] == "amr_01"
        assert (await command(server, "release"))["accepted"]
        assert server.stations[1].registers == [1, 0, 0, 0]
        assert len(server.stations[1].coils) == 5

    asyncio.run(scenario())


def test_closed_control_frame_rejects_without_unhandled_exception():
    import asyncio
    from plc_simulator.server import PlcServer

    async def scenario():
        reader = asyncio.StreamReader()
        reader.feed_eof()
        output = bytearray()

        class Writer:
            def write(self, data):
                output.extend(data)

            async def drain(self):
                pass

        await PlcServer()._control_request(reader, Writer())
        assert output == b"\x00"

    asyncio.run(scenario())
