"""Deterministic PLC cycle; time is supplied by the transport or caller."""

import re


class StationCycle:
    """One assembly/load (1) or inspection/unload (2) station."""

    def __init__(
        self,
        unit_id,
        *,
        cycle_delay=0.2,
        timeout=False,
        stale_completion=False,
        fault_code=0,
    ):
        if unit_id not in (1, 2):
            raise ValueError("unit_id must be 1 or 2")
        if cycle_delay < 0 or not 0 <= fault_code <= 65535:
            raise ValueError("invalid cycle delay or fault code")
        self.coils = [
            not bool(fault_code),
            False,
            False,
            bool(stale_completion),
            bool(fault_code),
        ]
        self.registers = [unit_id, 0, 0, fault_code]
        self.cycle_delay = cycle_delay
        self.timeout = timeout
        self.stale_completion = stale_completion
        self._part_written = False
        self._presence_valid = False
        self._started_at = None
        self.owner = None
        self.last_completion = None

    def claim(self, robot_id, mission_id, part):
        if any(
            not isinstance(value, str)
            or not re.fullmatch(r"[A-Za-z0-9_-]{1,64}", value)
            for value in (robot_id, mission_id, part)
        ):
            raise ValueError("invalid station ownership")
        identity = (robot_id, mission_id, part)
        if self.owner is not None and self.owner != identity:
            return False
        if self.owner is None and (self.coils[1] or self.coils[2]):
            return False
        self.owner = identity
        return True

    def release(self, robot_id, mission_id, part):
        if self.owner != (robot_id, mission_id, part) or self.coils[1] or self.coils[2]:
            return False
        self.owner = None
        return True

    def status(self):
        return dict(
            robot_id=self.owner[0] if self.owner else "",
            mission_id=self.owner[1] if self.owner else "",
            part=self.owner[2] if self.owner else "",
            cycle_counter=self.registers[2],
            last_completion=self.last_completion,
        )

    def complete(self, robot_id, mission_id, *, now):
        if self.owner is None or self.owner[:2] != (robot_id, mission_id):
            return False
        previous = self.registers[2]
        self.advance(now)
        return self.coils[3] and self.registers[2] != previous

    def write_register(self, address, value):
        if address != 1 or not isinstance(value, int) or not 0 <= value <= 65535:
            raise ValueError("only part_code may be written, as a uint16")
        self.registers[1] = value
        self._part_written = not self.coils[1]

    def write_coil(self, address, value, *, now):
        if address not in (1, 2):
            raise ValueError("only robot-owned coils may be written")
        value = bool(value)
        old = self.coils[address]
        self.coils[address] = value
        if address == 1:
            if value and not old:
                self._presence_valid = self._part_written
            elif not value:
                self._presence_valid = False
                self._part_written = False
                self._started_at = None
        elif not value:
            self.coils[3] = False
            self.coils[0] = not self.coils[4]
            self._started_at = None
        elif (
            not old
            and self.coils[1]
            and self._presence_valid
            and self.coils[0]
            and not self.coils[4]
        ):
            self.coils[0] = False
            self._started_at = now

    def advance(self, now):
        if (
            self._started_at is None
            or self.coils[4]
            or self.timeout
            or self.stale_completion
        ):
            return
        if now - self._started_at >= self.cycle_delay:
            self.registers[2] = (self.registers[2] + 1) % 65536
            self.coils[3] = True
            if self.owner is not None:
                self.last_completion = dict(
                    robot_id=self.owner[0],
                    mission_id=self.owner[1],
                    part=self.owner[2],
                    cycle_counter=self.registers[2],
                )
            self._started_at = None
