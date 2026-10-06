import asyncio

import pytest

from modbus_gateway.station_client import StationClient, NetworkFailure


def test_network_retries_initial_attempt_plus_three_with_exact_delays(monkeypatch):
    delays = []
    attempts = []

    async def sleep(delay):
        delays.append(delay)

    async def operation():
        attempts.append(1)
        if len(attempts) < 4:
            raise OSError("network unavailable")
        return 42

    monkeypatch.setattr(asyncio, "sleep", sleep)
    assert asyncio.run(StationClient()._network(operation)) == 42
    assert len(attempts) == 4
    assert delays == [0.5, 1.0, 2.0]


def test_retry_hook_reports_only_actual_retries(monkeypatch):
    observed = []
    attempts = []

    async def sleep(delay):
        pass

    async def operation():
        attempts.append(1)
        if len(attempts) < 3:
            raise OSError("retry hook fixture")
        return 42

    monkeypatch.setattr(asyncio, "sleep", sleep)
    client = StationClient(
        on_retry=lambda attempt, delay: observed.append((attempt, delay))
    )
    assert asyncio.run(client._network(operation)) == 42
    assert observed == [(1, 0.5), (2, 1.0)]


def test_network_exhaustion_stops_after_four_attempts(monkeypatch):
    delays = []
    attempts = []

    async def sleep(delay):
        delays.append(delay)

    async def operation():
        attempts.append(1)
        raise OSError("network unavailable")

    monkeypatch.setattr(asyncio, "sleep", sleep)
    with pytest.raises(NetworkFailure):
        asyncio.run(StationClient()._network(operation))
    assert len(attempts) == 4 and delays == [0.5, 1.0, 2.0]


def test_ambiguous_non_idempotent_operation_is_never_retried(monkeypatch):
    attempts = []

    async def operation():
        attempts.append(1)
        raise OSError("reply lost after write")

    with pytest.raises(NetworkFailure):
        asyncio.run(StationClient()._network(operation, retry=False))
    assert len(attempts) == 1


def test_nonresponsive_operation_is_bounded():
    async def scenario():
        async def operation():
            await asyncio.Event().wait()

        started = asyncio.get_running_loop().time()
        with pytest.raises(NetworkFailure, match="response timeout"):
            await StationClient(response_timeout=0.02)._network(operation, retry=False)
        assert asyncio.get_running_loop().time() - started < 0.2

    asyncio.run(scenario())


@pytest.mark.parametrize(
    "unit,part", [(0, 1), (3, 1), (True, 1), (1, -1), (1, 65536), (1, True), (1, "42")]
)
def test_invalid_unit_or_part_never_reaches_network(unit, part):
    with pytest.raises(ValueError):
        asyncio.run(StationClient().transfer(unit, part))


def test_changed_counter_without_complete_coil_never_confirms_payload(monkeypatch):
    from types import SimpleNamespace

    class CounterOnlySocket:
        def __init__(self, *args, **kwargs):
            self.connected = False
            self.started = False

        async def connect(self):
            self.connected = True
            return True

        async def read_coils(self, *args, **kwargs):
            return SimpleNamespace(
                bits=[True, self.started, self.started, False, False]
            )

        async def read_holding_registers(self, *args, **kwargs):
            return SimpleNamespace(registers=[1, 1, 2 if self.started else 1, 0])

        async def write_register(self, *args, **kwargs):
            return True

        async def write_coil(self, address, value, **kwargs):
            self.started |= address == 2 and value
            return True

        async def write_coils(self, *args, **kwargs):
            return True

        def close(self):
            self.connected = False

    monkeypatch.setattr(
        "modbus_gateway.station_client.AsyncModbusTcpClient", CounterOnlySocket
    )
    result = asyncio.run(StationClient(transfer_timeout=0.02).transfer(1, 1))
    assert not result.success
    assert result.outcome == "UNKNOWN"
