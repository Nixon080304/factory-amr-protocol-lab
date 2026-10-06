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
    from types import SimpleNamespace
    from plc_simulator.server import PlcServer, OwnershipModbusServer

    session = {"token": "", "sequence": 0}

    async def command(server, operation, robot="amr_01", mission="M-1"):
        reader = asyncio.StreamReader()
        fields = dict(
            operation=operation,
            unit_id=1,
            robot_id=robot,
            mission_id=mission,
            part="motor",
        )
        if operation == "claim":
            fields["endpoint"] = ["127.0.0.1", 20001]
        else:
            fields.update(session=session["token"], sequence=session["sequence"] + 1)
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
        result = json.loads(output[3:])
        if result["accepted"]:
            if operation == "claim":
                session["token"] = result["session"]
            else:
                session["sequence"] += 1
        return result

    async def scenario():
        server = PlcServer()
        tcp = OwnershipModbusServer(server)
        handler = tcp.callback_new_connection()
        handler.transport = SimpleNamespace(
            get_extra_info=lambda name: ("127.0.0.1", 20001)
        )
        handler.callback_connected()
        assert (await command(server, "claim"))["accepted"]
        assert not (await command(server, "claim", "amr_02", "M-2"))["accepted"]
        assert not (await command(server, "release", "amr_02"))["accepted"]
        assert server.stations[1].status()["robot_id"] == "amr_01"
        assert (await command(server, "status"))["robot_id"] == "amr_01"
        assert (await command(server, "release"))["accepted"]
        assert server.stations[1].registers == [1, 0, 0, 0]
        assert len(server.stations[1].coils) == 5
        handler.callback_disconnected(None)

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


@pytest.mark.parametrize("claimed", [False, True])
def test_review_unbound_raw_mutation_cannot_execute_or_clear_owned_station(claimed):
    import asyncio
    from pymodbus.constants import ExcCodes
    from plc_simulator.server import PlcServer

    async def scenario():
        server = PlcServer(cycle_delay=0)
        station = server.stations[1]
        if claimed:
            assert station.claim("amr_01", "M-1", "motor")
            station.write_register(1, 1)
            station.write_coil(1, True, now=0)
            station.write_coil(2, True, now=0)
        snapshot = (tuple(station.coils), tuple(station.registers))
        device = server._device(1)
        result = await device.action(
            15 if claimed else 6,
            0,
            1,
            2 if claimed else 1,
            [0, 0, 0, 0],
            [False, False] if claimed else [1],
        )
        assert result == ExcCodes.ILLEGAL_FUNCTION
        assert (tuple(station.coils), tuple(station.registers)) == snapshot
        if claimed:
            await device.action(1, 0, 0, 5, [0], None)
            assert station.registers[2] == 0

    asyncio.run(scenario())


def test_review_session_authorization_is_bound_to_actual_modbus_connection():
    import asyncio
    import json
    import struct
    from types import SimpleNamespace
    from pymodbus.pdu.bit_message import (
        ReadCoilsRequest,
        WriteSingleCoilRequest,
        WriteMultipleCoilsRequest,
    )
    from pymodbus.pdu.register_message import WriteSingleRegisterRequest
    from plc_simulator.server import PlcServer, OwnershipModbusServer

    async def exchange(server, fields):
        reader = asyncio.StreamReader()
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
        return json.loads(output[3:])

    async def scenario():
        now = [0.0]
        server = PlcServer(cycle_delay=0, clock=lambda: now[0])
        tcp = OwnershipModbusServer(server)
        connections = []
        for port in (20101, 20102):
            handler = tcp.callback_new_connection()
            handler.transport = SimpleNamespace(
                get_extra_info=lambda name, port=port: ("127.0.0.1", port)
            )
            handler.callback_connected()
            connections.append(handler)
        first, foreign = connections

        async def raw(handler, pdu):
            result = []
            handler.last_pdu = pdu
            handler.server_send = lambda response, address: result.append(response)
            await handler.handle_request()
            return result[-1]

        base = dict(unit_id=1, robot_id="amr_01", mission_id="M-1", part="motor")
        assert (
            await raw(
                first,
                WriteSingleRegisterRequest(
                    dev_id=1, transaction_id=1, address=1, registers=[1]
                ),
            )
        ).isError()
        # A complete packet received before claim must not borrow authorization
        # from a later claim while its event-loop dispatch is delayed.
        delayed = []
        actual_loop = first.loop
        first.loop = SimpleNamespace(
            call_soon=lambda callback: delayed.append(callback)
        )
        unclaimed = WriteSingleRegisterRequest(
            dev_id=1, transaction_id=100, address=1, registers=[99]
        )
        packet = first.framer.buildFrame(unclaimed)
        assert first.callback_data(packet) == len(packet)
        first.loop = actual_loop
        claimed = await exchange(
            server, dict(base, operation="claim", endpoint=["127.0.0.1", 20101])
        )
        assert claimed["accepted"] and claimed["session"]
        token = claimed["session"]
        pending_responses = []
        first.server_send = lambda response, address: pending_responses.append(response)
        delayed.pop()()
        for _ in range(20):
            if pending_responses:
                break
            await asyncio.sleep(0.001)
        assert pending_responses[0].isError()
        assert server.stations[1].registers[1] == 0
        assert not (
            await exchange(
                server, dict(base, operation="claim", endpoint=["127.0.0.1", 20102])
            )
        )["accepted"]
        assert (
            await raw(
                foreign,
                WriteSingleRegisterRequest(
                    dev_id=1, transaction_id=1, address=1, registers=[1]
                ),
            )
        ).isError()
        server.response_delay = 0.01
        responses = []
        first.server_send = lambda response, address: responses.append(response)
        first.last_pdu = WriteSingleRegisterRequest(
            dev_id=1, transaction_id=2, address=1, registers=[1]
        )
        pending_write = asyncio.create_task(first.handle_request())
        await asyncio.sleep(0)
        first.last_pdu = ReadCoilsRequest(
            dev_id=1, transaction_id=9, address=0, count=5
        )
        await asyncio.gather(pending_write, first.handle_request())
        assert {response.transaction_id for response in responses} == {2, 9}
        server.response_delay = 0
        for pdu in [
            WriteSingleCoilRequest(dev_id=1, transaction_id=3, address=1, bits=[True]),
            WriteSingleCoilRequest(dev_id=1, transaction_id=4, address=2, bits=[True]),
            ReadCoilsRequest(dev_id=1, transaction_id=5, address=0, count=5),
        ]:
            assert not (await raw(first, pdu)).isError()
        assert server.stations[1].last_completion["robot_id"] == "amr_01"
        assert (
            await raw(
                first, ReadCoilsRequest(dev_id=1, transaction_id=5, address=0, count=5)
            )
        ).isError()
        cleanup = WriteMultipleCoilsRequest(
            dev_id=1, transaction_id=6, address=1, bits=[False, False]
        )
        assert (await raw(foreign, cleanup)).isError()
        assert server.stations[1].coils[1:3] == [True, True]
        assert not (await raw(first, cleanup)).isError()
        responses = []
        first.server_send = lambda response, address: responses.append(response)
        for transaction in (10, 11):
            pdu = ReadCoilsRequest(
                dev_id=1, transaction_id=transaction, address=0, count=5
            )
            encoded = first.framer.buildFrame(pdu)
            assert first.callback_data(encoded) == len(encoded)
        for _ in range(20):
            if len(responses) == 2:
                break
            await asyncio.sleep(0.001)
        assert {response.transaction_id for response in responses} == {10, 11}
        fault = dict(
            base,
            operation="fault",
            session=token,
            sequence=1,
            name="plc_fault",
            duration=2.0,
            fault_code=73,
        )
        assert (await exchange(server, fault))["accepted"]
        assert server.stations[1].coils[4]
        reset = dict(fault, name="", duration=0.0, fault_code=0, sequence=2)
        assert not (await exchange(server, dict(reset, robot_id="amr_02")))["accepted"]
        assert server.stations[1].coils[4]
        assert (await exchange(server, reset))["accepted"]
        assert not server.stations[1].coils[4]
        status = dict(base, operation="status", session=token, sequence=3)
        assert not (await exchange(server, dict(status, robot_id="amr_02")))["accepted"]
        assert not (await exchange(server, dict(status, session="foreign")))["accepted"]
        assert (await exchange(server, status))["accepted"]
        assert not (await exchange(server, status))["accepted"]
        assert (
            await exchange(
                server, dict(base, operation="release", session=token, sequence=4)
            )
        )["accepted"]
        assert (
            await raw(
                first,
                WriteSingleRegisterRequest(
                    dev_id=1, transaction_id=7, address=1, registers=[1]
                ),
            )
        ).isError()
        claimed = await exchange(
            server, dict(base, operation="claim", endpoint=["127.0.0.1", 20101])
        )
        assert claimed["session"] != token
        # Release/reclaim on the same connection cannot authorize replay of a
        # transaction from the prior station session.
        assert (
            await raw(
                first,
                WriteSingleRegisterRequest(
                    dev_id=1, transaction_id=2, address=1, registers=[99]
                ),
            )
        ).isError()
        assert server.stations[1].registers[1] == 1
        now[0] = 31.0
        assert (
            await raw(
                first,
                WriteSingleRegisterRequest(
                    dev_id=1, transaction_id=8, address=1, registers=[2]
                ),
            )
        ).isError()
        assert not (
            await exchange(
                server,
                dict(base, operation="release", session=claimed["session"], sequence=1),
            )
        )["accepted"]
        first.callback_disconnected(None)
        assert 1 not in server._sessions
        assert ("127.0.0.1", 20101) not in server._connections
        foreign.callback_disconnected(None)

    asyncio.run(scenario())


def test_review_explicit_v1_mode_preserves_raw_handshake():
    import asyncio
    from plc_simulator.server import PlcServer

    async def scenario():
        server = PlcServer(cycle_delay=0, ownership_enabled=False)
        device = server._device(1)
        for function, address, values in (
            (6, 1, [1]),
            (5, 1, [True]),
            (5, 2, [True]),
        ):
            assert (
                await device.action(function, 0, address, 1, [0, 0, 0, 0], values)
                is None
            )
        assert await device.action(1, 0, 0, 5, [0], None) is None
        assert server.stations[1].registers[2] == 1
        assert server.stations[1].coils[3]
        assert await device.action(15, 0, 1, 2, [0], [False, False]) is None
        assert server.stations[1].coils[1:4] == [False, False, False]

    asyncio.run(scenario())
