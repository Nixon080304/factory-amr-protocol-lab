"""Real PyModbus 3.15 server with a deterministic, ROS-free station core."""
import argparse
import asyncio
import math
import signal

from pymodbus.constants import ExcCodes
from pymodbus.server import ModbusTcpServer
from pymodbus.simulator import DataType, SimData, SimDevice

from .station_cycle import StationCycle


class PlcServer:
    def __init__(self, host="127.0.0.1", port=1502, *, cycle_delay=0.2,
                 timeout=False, stale_completion=False, fault_code=0,
                 response_delay=0.0, request_response_delay=0.0):
        if any(not math.isfinite(value) or value < 0
               for value in (cycle_delay, response_delay, request_response_delay)):
            raise ValueError("delays must be finite and nonnegative")
        self.host = host
        self.port = port
        self.stations = {unit: StationCycle(
            unit, cycle_delay=cycle_delay, timeout=timeout,
            stale_completion=stale_completion, fault_code=fault_code)
            for unit in (1, 2)}
        self.response_delay = response_delay
        self.request_response_delay = request_response_delay
        self._server = None

    def _device(self, unit):
        station = self.stations[unit]

        async def action(function_code, start_address, address, count,
                         current_registers, set_values):
            now = asyncio.get_running_loop().time()
            station.advance(now)
            request_started = False
            if set_values is not None:
                if function_code in (5, 15):
                    if address < 1 or address + len(set_values) > 3:
                        return ExcCodes.ILLEGAL_ADDRESS
                    for offset, value in enumerate(set_values):
                        station.write_coil(address + offset, value, now=now)
                        request_started |= address + offset == 2 and bool(value)
                elif function_code in (6, 16):
                    if address != 1 or len(set_values) != 1:
                        return ExcCodes.ILLEGAL_ADDRESS
                    station.write_register(1, set_values[0])
                else:
                    return ExcCodes.ILLEGAL_FUNCTION
            if function_code in (1, 5, 15):
                # Distinct SimDevice coil blocks store sixteen coils per word.
                current_registers[0] = sum(int(value) << bit
                                           for bit, value in enumerate(station.coils))
            elif function_code in (3, 6, 16):
                current_registers[:4] = station.registers
            if self.response_delay:
                await asyncio.sleep(self.response_delay)
            if request_started and self.request_response_delay:
                await asyncio.sleep(self.request_response_delay)
            return None

        return SimDevice(unit, simdata=(
            [SimData(0, values=station.coils, datatype=DataType.BITS)],
            [SimData(0, values=False, datatype=DataType.BITS)],
            [SimData(0, values=station.registers, datatype=DataType.UINT16)],
            [SimData(0, values=0, datatype=DataType.UINT16)]), action=action)

    async def start(self):
        if self._server is not None:
            raise RuntimeError("PLC server is already started")
        self._server = ModbusTcpServer(
            [self._device(1), self._device(2)], address=(self.host, self.port))
        try:
            await self._server.serve_forever(background=True)
        except BaseException:
            await self._server.shutdown()
            self._server = None
            raise
        self.port = self._server.transport.sockets[0].getsockname()[1]

    async def stop(self):
        if self._server is not None:
            await self._server.shutdown()
            self._server = None


async def _run(options):
    server = PlcServer(**vars(options))
    stopped = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stopped.set)
    await server.start()
    print(f"PLC simulator listening on {server.host}:{server.port}", flush=True)
    try:
        await stopped.wait()
    finally:
        await server.stop()


def main():
    parser = argparse.ArgumentParser(description="Standalone assembly/inspection Modbus PLC")
    parser.add_argument("--host", default="127.0.0.1",
                        help="Bind host; use 0.0.0.0 explicitly inside containers")
    parser.add_argument("--port", type=int, default=1502)
    parser.add_argument("--cycle-delay", type=float, default=0.2)
    parser.add_argument("--response-delay", type=float, default=0.0)
    parser.add_argument("--request-response-delay", type=float, default=0.0)
    parser.add_argument("--timeout", action="store_true", help="Never complete station cycles")
    parser.add_argument("--stale-completion", action="store_true")
    parser.add_argument("--fault-code", type=int, default=0)
    asyncio.run(_run(parser.parse_args()))


if __name__ == "__main__":
    main()
