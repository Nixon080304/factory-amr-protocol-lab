"""Use a real TCP listener, including transport loss and PLC failure modes."""

import asyncio
from contextlib import asynccontextmanager
from pathlib import Path
import sys

import pytest
from pymodbus.client import AsyncModbusTcpClient

ROOT = Path(__file__).resolve().parents[2]
for package in ("modbus_gateway", "plc_simulator"):
    sys.path.insert(0, str(ROOT / "src" / package))

from modbus_gateway.station_client import StationClient, NetworkFailure
from plc_simulator.server import PlcServer


@asynccontextmanager
async def station_server(**options):
    server = PlcServer(port=0, **options)
    await server.start()
    port = server.port
    try:
        yield server
    finally:
        await server.stop()
        with pytest.raises(OSError):
            await asyncio.wait_for(asyncio.open_connection("127.0.0.1", port), 0.5)


async def read_station(port, unit):
    client = AsyncModbusTcpClient("127.0.0.1", port=port, retries=0, timeout=0.5)
    try:
        assert await client.connect()
        coils = await client.read_coils(0, count=5, device_id=unit)
        registers = await client.read_holding_registers(0, count=4, device_id=unit)
        assert not coils.isError() and not registers.isError()
        return coils.bits[:5], registers.registers
    finally:
        client.close()


@pytest.mark.parametrize("unit", [1, 2])
def test_successful_transfer_uses_exact_map_and_clears_robot_coils(unit):
    async def scenario():
        async with station_server(cycle_delay=0.12) as server:
            result = await StationClient(port=server.port).transfer(unit, 42)
            assert (result.success, result.error_code, result.cycle_counter) == (
                True,
                "",
                1,
            )
            coils, registers = await read_station(server.port, unit)
            assert coils == [True, False, False, False, False]
            assert registers == [unit, 42, 1, 0]
            result = await StationClient(port=server.port).transfer(unit, 43)
            assert result.cycle_counter == 2

    asyncio.run(scenario())


def test_fault_reports_register_fault_code_and_clears_robot_coils():
    async def scenario():
        async with station_server(fault_code=73) as server:
            result = await StationClient(port=server.port).transfer(2, 42)
            assert not result.success and result.error_code == "PLC_FAULT"
            assert "73" in result.message
            coils, _ = await read_station(server.port, 2)
            assert coils[1:3] == [False, False]

    asyncio.run(scenario())


def test_stale_completion_requires_counter_change_and_cleans_request():
    async def scenario():
        async with station_server(stale_completion=True) as server:
            result = await StationClient(
                port=server.port, transfer_timeout=0.25
            ).transfer(1, 42)
            assert not result.success and result.error_code == "STALE_PLC_STATE"
            coils, registers = await read_station(server.port, 1)
            assert coils[1:4] == [False, False, False]
            assert registers[2] == 0

    asyncio.run(scenario())


def test_cycle_timeout_is_bounded_and_cleans_request():
    async def scenario():
        async with station_server(timeout=True) as server:
            started = asyncio.get_running_loop().time()
            result = await StationClient(
                port=server.port, transfer_timeout=0.25
            ).transfer(1, 42)
            assert not result.success and result.error_code == "PLC_TIMEOUT"
            assert asyncio.get_running_loop().time() - started < 1
            coils, registers = await read_station(server.port, 1)
            assert coils[1:3] == [False, False]
            assert registers[2] == 0

    asyncio.run(scenario())


def test_connection_timeout_uses_real_closed_localhost_port():
    async def scenario():
        async with station_server() as server:
            port = server.port
        started = asyncio.get_running_loop().time()
        result = await StationClient(port=port, response_timeout=0.1).transfer(1, 42)
        assert not result.success and result.error_code == "PLC_TIMEOUT"
        assert result.outcome == "NOT_REQUESTED"
        elapsed = asyncio.get_running_loop().time() - started
        assert 3.4 <= elapsed < 5

    asyncio.run(scenario())


def test_ambiguous_request_response_does_not_replay_transfer(monkeypatch):
    sent_requests = []
    lost_acknowledgments = []

    def trace(sending, pdu):
        if (
            sending
            and pdu.function_code == 5
            and pdu.address == 2
            and pdu.bits == [True]
        ):
            sent_requests.append(pdu.transaction_id)
        return pdu

    def transport(*args, **kwargs):
        return AsyncModbusTcpClient(*args, trace_pdu=trace, **kwargs)

    monkeypatch.setattr("modbus_gateway.station_client.AsyncModbusTcpClient", transport)

    class ObservedClient(StationClient):
        async def _network(self, operation, **kwargs):
            try:
                return await super()._network(operation, **kwargs)
            except NetworkFailure:
                if kwargs.get("retry") is False:
                    lost_acknowledgments.append(True)
                raise

    async def scenario():
        async with station_server(
            cycle_delay=0.02, request_response_delay=0.5
        ) as server:
            result = await ObservedClient(
                port=server.port, response_timeout=0.2
            ).transfer(1, 42)
            assert result.success and result.cycle_counter == 1
            assert lost_acknowledgments == [True]
            assert len(sent_requests) == 1
            assert server.stations[1].registers[2] == 1
            coils, _ = await read_station(server.port, 1)
            assert coils[1:3] == [False, False]

    asyncio.run(scenario())


def test_fault_during_cycle_clears_already_written_robot_coils():
    async def scenario():
        async with station_server(timeout=True) as server:
            task = asyncio.create_task(StationClient(port=server.port).transfer(1, 42))

            async def inject():
                while not server.stations[1].coils[2]:
                    await asyncio.sleep(0.01)
                assert server.stations[1].coils[1] is True
                server.stations[1].coils[4] = True
                server.stations[1].registers[3] = 91

            await asyncio.wait_for(inject(), 1)
            result = await asyncio.wait_for(task, 1)
            assert not result.success and result.error_code == "PLC_FAULT"
            assert "91" in result.message
            coils, registers = await read_station(server.port, 1)
            assert coils[1:3] == [False, False]
            assert registers[2] == 0

    asyncio.run(scenario())


def test_nonresponsive_server_returns_bounded_response_timeout():
    async def scenario():
        async with station_server(response_delay=0.4) as server:
            started = asyncio.get_running_loop().time()
            result = await StationClient(
                port=server.port, response_timeout=0.2
            ).transfer(1, 42)
            assert not result.success and result.error_code == "PLC_TIMEOUT"
            assert asyncio.get_running_loop().time() - started < 6
            # The initial state read never succeeded; no request reached the PLC.
            assert server.stations[1].registers[2] == 0
            assert server.stations[1].coils[1:3] == [False, False]

    asyncio.run(scenario())


def test_completed_cycle_survives_lost_cleanup_acknowledgement(monkeypatch):
    class LostCleanupAck(AsyncModbusTcpClient):
        async def write_coils(self, *args, **kwargs):
            response = await super().write_coils(*args, **kwargs)
            assert not response.isError()
            raise OSError("cleanup applied but acknowledgment lost")

    monkeypatch.setattr(
        "modbus_gateway.station_client.AsyncModbusTcpClient", LostCleanupAck
    )

    async def scenario():
        async with station_server(cycle_delay=0.01) as server:
            result = await StationClient(port=server.port).transfer(1, 42)
            assert not result.success and result.error_code == "PLC_TIMEOUT"
            assert result.cycle_counter == 1
            assert result.outcome == "COMPLETED"
            coils, registers = await read_station(server.port, 1)
            assert registers[2] == 1
            assert coils[1:3] == [False, False]

    asyncio.run(scenario())


@pytest.mark.parametrize("observation_error", [False, True])
def test_post_request_timeout_is_unknown_and_never_replays(
    monkeypatch, observation_error
):
    requests = []

    class CountRequests(AsyncModbusTcpClient):
        async def write_coil(self, address, value, **kwargs):
            if address == 2 and value:
                requests.append((address, value))
            return await super().write_coil(address, value, **kwargs)

        async def read_coils(self, *args, **kwargs):
            if observation_error and requests:
                raise RuntimeError("unexpected post-request observation failure")
            return await super().read_coils(*args, **kwargs)

    monkeypatch.setattr(
        "modbus_gateway.station_client.AsyncModbusTcpClient", CountRequests
    )

    async def scenario():
        async with station_server(timeout=True) as server:
            result = await StationClient(
                port=server.port, transfer_timeout=0.25
            ).transfer(1, 42)
            assert not result.success and result.error_code == "PLC_TIMEOUT"
            assert result.outcome == "UNKNOWN"
            assert requests == [(2, True)]
            coils, registers = await read_station(server.port, 1)
            assert coils[1:3] == [False, False]
            assert registers[2] == 0

    asyncio.run(scenario())
