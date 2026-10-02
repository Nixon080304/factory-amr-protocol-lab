"""Bounded Modbus handshakes without replaying ambiguous cycle requests."""

import asyncio
from dataclasses import dataclass
import math

from pymodbus.client import AsyncModbusTcpClient
from pymodbus.exceptions import ModbusException

from . import registers as address


@dataclass(frozen=True)
class TransferResult:
    success: bool
    error_code: str
    message: str
    cycle_counter: int


class NetworkFailure(Exception):
    """The bounded network operation did not return a usable response."""


class StationClient:
    def __init__(
        self,
        host="127.0.0.1",
        port=1502,
        *,
        response_timeout=address.RESPONSE_TIMEOUT,
        transfer_timeout=2.0,
        on_retry=None,
        on_state=None,
    ):
        if any(
            not math.isfinite(value) or value <= 0
            for value in (response_timeout, transfer_timeout)
        ):
            raise ValueError("timeouts must be finite and positive")
        self.host = host
        self.port = port
        self.response_timeout = response_timeout
        self.transfer_timeout = transfer_timeout
        self._lock = asyncio.Lock()
        self._on_retry = on_retry
        self._on_state = on_state

    @staticmethod
    def _notify(callback, *args):
        # Observation must never change the PLC handshake outcome.
        if callback is not None:
            try:
                callback(*args)
            except Exception:
                pass

    async def _network(self, operation, *, retry=True, deadline=None):
        """Retry only idempotent operations, with no hidden library retries."""
        delays = address.RETRY_DELAYS if retry else ()
        last_error = None
        for attempt in range(len(delays) + 1):
            remaining = (
                deadline - asyncio.get_running_loop().time()
                if deadline is not None
                else self.response_timeout
            )
            if remaining <= 0:
                raise NetworkFailure("response deadline exceeded")
            try:
                response = await asyncio.wait_for(
                    operation(), min(self.response_timeout, remaining)
                )
                if response is False or (
                    hasattr(response, "isError") and response.isError()
                ):
                    raise NetworkFailure(f"Modbus operation failed: {response}")
                return response
            except (
                OSError,
                asyncio.TimeoutError,
                ModbusException,
                NetworkFailure,
            ) as error:
                last_error = error
                if attempt == len(delays):
                    break
                delay = delays[attempt]
                if (
                    deadline is not None
                    and asyncio.get_running_loop().time() + delay >= deadline
                ):
                    break
                self._notify(self._on_retry, attempt + 1, delay)
                await asyncio.sleep(delay)
        message = (
            str(last_error) or f"PLC response timeout after {self.response_timeout}s"
        )
        raise NetworkFailure(message) from last_error

    async def transfer(self, unit_id: int, part_code: int) -> TransferResult:
        if type(unit_id) is not int or unit_id not in (1, 2):
            raise ValueError("unit_id must be 1 or 2")
        if type(part_code) is not int or not 0 <= part_code <= 65535:
            raise ValueError("part_code must be a uint16")
        async with self._lock:
            return await self._transfer(unit_id, part_code)

    async def _transfer(self, unit_id, part_code):
        client = AsyncModbusTcpClient(
            self.host,
            port=self.port,
            timeout=self.response_timeout,
            retries=0,
            reconnect_delay=0,
        )
        counter = 0
        stale = False
        connected_once = False
        last_state = None
        result = TransferResult(False, "PLC_TIMEOUT", "PLC did not respond", counter)

        async def request(method, *args, retry=True, deadline=None, **kwargs):
            async def operation():
                if not client.connected:
                    if not await client.connect():
                        raise NetworkFailure("PLC connection failed")
                return await method(*args, device_id=unit_id, **kwargs)

            return await self._network(operation, retry=retry, deadline=deadline)

        async def state(deadline=None):
            nonlocal last_state
            coils = await request(client.read_coils, 0, count=5, deadline=deadline)
            registers = await request(
                client.read_holding_registers, 0, count=4, deadline=deadline
            )
            if len(coils.bits) < 5 or len(registers.registers) != 4:
                raise NetworkFailure("incomplete PLC state response")
            observed = (tuple(coils.bits[:5]), tuple(registers.registers))
            if observed != last_state:
                last_state = observed
                self._notify(
                    self._on_state, unit_id, list(observed[0]), list(observed[1])
                )
            return coils.bits, registers.registers

        def fault(registers):
            return TransferResult(
                False,
                "PLC_FAULT",
                f"PLC station fault: fault_code={registers[address.FAULT_CODE]}",
                registers[address.CYCLE_COUNTER],
            )

        try:
            await self._network(client.connect)
            connected_once = True
            coils, registers = await state()
            counter = registers[address.CYCLE_COUNTER]
            if coils[address.STATION_FAULT]:
                result = fault(registers)
            elif not coils[address.STATION_READY]:
                result = TransferResult(
                    False, "PLC_TIMEOUT", "PLC station is not ready", counter
                )
            else:
                await request(client.write_register, address.PART_CODE, part_code)
                await request(client.write_coil, address.ROBOT_PRESENT, True)
                # A lost acknowledgment can mean the PLC already began the cycle.
                # Never resend the request; observe that same cycle instead.
                deadline = asyncio.get_running_loop().time() + self.transfer_timeout
                try:
                    await request(
                        client.write_coil,
                        address.TRANSFER_REQUEST,
                        True,
                        retry=False,
                        deadline=deadline,
                    )
                except NetworkFailure:
                    pass
                while asyncio.get_running_loop().time() < deadline:
                    coils, registers = await state(deadline)
                    observed = registers[address.CYCLE_COUNTER]
                    if coils[address.STATION_FAULT]:
                        result = fault(registers)
                        break
                    if coils[address.TRANSFER_COMPLETE]:
                        if observed != counter:
                            result = TransferResult(
                                True, "", "PLC transfer complete", observed
                            )
                            break
                        stale = True
                    await asyncio.sleep(
                        min(
                            address.POLL_INTERVAL,
                            max(0, deadline - asyncio.get_running_loop().time()),
                        )
                    )
                else:
                    result = TransferResult(
                        False,
                        "STALE_PLC_STATE" if stale else "PLC_TIMEOUT",
                        "Completion counter did not change"
                        if stale
                        else "PLC transfer completion timed out",
                        counter,
                    )
        except NetworkFailure as error:
            result = TransferResult(
                False,
                "STALE_PLC_STATE" if stale else "PLC_TIMEOUT",
                str(error),
                counter,
            )
        finally:
            # Cleanup is idempotent and separately bounded. Attempt it after every
            # outcome when a connection existed; failed initial connect wrote nothing.
            if connected_once:
                try:
                    await request(
                        client.write_coils,
                        address.ROBOT_PRESENT,
                        [False, False],
                        retry=False,
                        deadline=asyncio.get_running_loop().time()
                        + self.response_timeout,
                    )
                except NetworkFailure as error:
                    message = f"{result.message}; robot coil cleanup failed: {error}"
                    result = TransferResult(
                        False,
                        result.error_code or "PLC_TIMEOUT",
                        message,
                        result.cycle_counter,
                    )
            client.close()
        return result
