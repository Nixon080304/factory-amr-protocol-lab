"""Real PyModbus 3.15 server with a deterministic, ROS-free station core."""

import argparse
import asyncio
import json
import math
import signal
import struct

from pymodbus.constants import ExcCodes
from pymodbus.server import ModbusTcpServer
from pymodbus.simulator import DataType, SimData, SimDevice

from .station_cycle import StationCycle


class PlcServer:
    def __init__(
        self,
        host="127.0.0.1",
        port=1502,
        *,
        cycle_delay=0.2,
        timeout=False,
        stale_completion=False,
        fault_code=0,
        response_delay=0.0,
        request_response_delay=0.0,
        fault_control_port=0,
    ):
        if any(
            not math.isfinite(value) or value < 0
            for value in (cycle_delay, response_delay, request_response_delay)
        ):
            raise ValueError("delays must be finite and nonnegative")
        self.host = host
        self.port = port
        self.stations = {
            unit: StationCycle(
                unit,
                cycle_delay=cycle_delay,
                timeout=timeout,
                stale_completion=stale_completion,
                fault_code=fault_code,
            )
            for unit in (1, 2)
        }
        self.response_delay = response_delay
        self.request_response_delay = request_response_delay
        self._server = None
        self.fault_control_port = fault_control_port
        self._control_server = None
        self._effects = {}
        self._control_tasks = set()

    def _reset_effects(self, unit):
        effects = self._effects.pop(unit, None)
        if effects is not None:
            station = self.stations[unit]
            station.timeout, station.stale_completion = effects["original"][:2]
            station.coils[4], station.registers[3] = effects["original"][2:]
            station.coils[0] = not station.coils[4]
            if not station.stale_completion:
                station.coils[3] = False
            effects["timer"].cancel()

    async def _control(self, reader, writer):
        task = asyncio.current_task()
        self._control_tasks.add(task)
        try:
            await self._control_request(reader, writer)
        finally:
            writer.close()
            try:
                await asyncio.wait_for(writer.wait_closed(), 1.0)
            except (OSError, asyncio.TimeoutError):
                pass
            self._control_tasks.discard(task)

    async def _control_request(self, reader, writer):
        try:
            prefix = await asyncio.wait_for(reader.readexactly(1), 1.0)
            if prefix == b"\xff":
                await self._ownership_request(reader, writer)
                return
            # Fixed side-channel frame: operation, unit, duration, fault code.
            # Fault frames and typed ownership frames share this opt-in listener.
            # Neither frame expands the raw Modbus address map.
            operation, unit, duration, code = struct.unpack(
                "!BBdH", prefix + await asyncio.wait_for(reader.readexactly(11), 1.0)
            )
            if unit not in (1, 2):
                raise ValueError("unit_id must be 1 or 2")
            if operation == 0:
                self._reset_effects(unit)
            else:
                if operation not in (1, 2, 3, 4):
                    raise ValueError("unsupported station fault")
                if not math.isfinite(duration) or duration <= 0:
                    raise ValueError("invalid duration")
                if not 1 <= code <= 65535:
                    raise ValueError("invalid fault code")
                self._reset_effects(unit)
                station = self.stations[unit]
                original = (
                    station.timeout,
                    station.stale_completion,
                    station.coils[4],
                    station.registers[3],
                )
                if operation == 2:
                    station.timeout = True
                elif operation == 3:
                    station.stale_completion = True
                    station.coils[3] = True
                elif operation == 4:
                    station.coils[4] = True
                    station.registers[3] = code
                delay = duration if operation == 1 else 0.0
                timer = asyncio.get_running_loop().call_later(
                    duration, self._reset_effects, unit
                )
                self._effects[unit] = dict(original=original, timer=timer, delay=delay)
            reply = b"\x01"
        except (ValueError, asyncio.IncompleteReadError, asyncio.TimeoutError):
            reply = b"\x00"
        writer.write(reply)
        await asyncio.wait_for(writer.drain(), 1.0)

    async def _ownership_request(self, reader, writer):
        try:
            size = struct.unpack(
                "!H", await asyncio.wait_for(reader.readexactly(2), 1.0)
            )[0]
            if not 1 <= size <= 1024:
                raise ValueError("ownership frame exceeds 1024 bytes")
            fields = json.loads(await asyncio.wait_for(reader.readexactly(size), 1.0))
            if not isinstance(fields, dict) or set(fields) != {
                "operation",
                "unit_id",
                "robot_id",
                "mission_id",
                "part",
            }:
                raise ValueError("invalid ownership fields")
            unit = fields["unit_id"]
            if type(unit) is not int or unit not in self.stations:
                raise ValueError("invalid ownership unit")
            station = self.stations[unit]
            identity = (fields["robot_id"], fields["mission_id"], fields["part"])
            operation = fields["operation"]
            if operation == "claim":
                accepted = station.claim(*identity)
            elif operation == "release":
                accepted = station.release(*identity)
            elif operation == "status":
                accepted = station.owner == identity
            else:
                raise ValueError("invalid ownership operation")
            reply = dict(accepted=accepted, **station.status())
        except (
            ValueError,
            TypeError,
            KeyError,
            asyncio.IncompleteReadError,
            asyncio.TimeoutError,
            RecursionError,
        ):
            reply = dict(accepted=False, message="invalid ownership request")
        encoded = json.dumps(reply, allow_nan=False).encode()
        writer.write(b"\xff" + struct.pack("!H", len(encoded)) + encoded)
        await asyncio.wait_for(writer.drain(), 1.0)

    async def start_fault_control(self):
        """Opt-in side channel, always loopback, with no Modbus map changes."""
        if self._control_server is not None:
            raise RuntimeError("fault control is already started")
        self._control_server = await asyncio.start_server(
            self._control, "127.0.0.1", self.fault_control_port, limit=4096
        )
        self.fault_control_port = self._control_server.sockets[0].getsockname()[1]

    def _device(self, unit):
        station = self.stations[unit]

        async def action(
            function_code, start_address, address, count, current_registers, set_values
        ):
            now = asyncio.get_running_loop().time()
            if station.owner is not None:
                station.complete(*station.owner[:2], now=now)
            else:
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
                current_registers[0] = sum(
                    int(value) << bit for bit, value in enumerate(station.coils)
                )
            elif function_code in (3, 6, 16):
                current_registers[:4] = station.registers
            if self.response_delay:
                await asyncio.sleep(self.response_delay)
            effect = self._effects.get(unit)
            if effect is not None and effect["delay"]:
                await asyncio.sleep(effect["delay"])
            if request_started and self.request_response_delay:
                await asyncio.sleep(self.request_response_delay)
            return None

        return SimDevice(
            unit,
            simdata=(
                [SimData(0, values=station.coils, datatype=DataType.BITS)],
                [SimData(0, values=False, datatype=DataType.BITS)],
                [SimData(0, values=station.registers, datatype=DataType.UINT16)],
                [SimData(0, values=0, datatype=DataType.UINT16)],
            ),
            action=action,
        )

    async def start(self):
        if self._server is not None:
            raise RuntimeError("PLC server is already started")
        self._server = ModbusTcpServer(
            [self._device(1), self._device(2)], address=(self.host, self.port)
        )
        try:
            await self._server.serve_forever(background=True)
        except BaseException:
            await self._server.shutdown()
            self._server = None
            raise
        self.port = self._server.transport.sockets[0].getsockname()[1]

    async def stop(self):
        if self._control_server is not None:
            self._control_server.close()
            await self._control_server.wait_closed()
            self._control_server = None
        for task in tuple(self._control_tasks):
            task.cancel()
        if self._control_tasks:
            await asyncio.gather(*self._control_tasks, return_exceptions=True)
        for unit in tuple(self._effects):
            self._reset_effects(unit)
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
    try:
        if options.fault_control_port:
            await server.start_fault_control()
        print(f"PLC simulator listening on {server.host}:{server.port}", flush=True)
        await stopped.wait()
    finally:
        await server.stop()


def main():
    parser = argparse.ArgumentParser(
        description="Standalone assembly/inspection Modbus PLC"
    )
    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="Bind host; use 0.0.0.0 explicitly inside containers",
    )
    parser.add_argument("--port", type=int, default=1502)
    parser.add_argument("--cycle-delay", type=float, default=0.2)
    parser.add_argument("--response-delay", type=float, default=0.0)
    parser.add_argument("--request-response-delay", type=float, default=0.0)
    parser.add_argument(
        "--timeout", action="store_true", help="Never complete station cycles"
    )
    parser.add_argument("--stale-completion", action="store_true")
    parser.add_argument("--fault-code", type=int, default=0)
    parser.add_argument(
        "--fault-control-port",
        type=int,
        default=0,
        help="Opt-in loopback fault-control listener; disabled by default",
    )
    asyncio.run(_run(parser.parse_args()))


if __name__ == "__main__":
    main()
