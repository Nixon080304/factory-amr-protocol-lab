"""Deterministic PLC cycle; time is supplied by the transport or caller."""


class StationCycle:
    """One assembly/load (1) or inspection/unload (2) station."""

    def __init__(self, unit_id, *, cycle_delay=0.2, timeout=False,
                 stale_completion=False, fault_code=0):
        if unit_id not in (1, 2):
            raise ValueError("unit_id must be 1 or 2")
        if cycle_delay < 0 or not 0 <= fault_code <= 65535:
            raise ValueError("invalid cycle delay or fault code")
        self.coils = [not bool(fault_code), False, False,
                      bool(stale_completion), bool(fault_code)]
        self.registers = [unit_id, 0, 0, fault_code]
        self.cycle_delay = cycle_delay
        self.timeout = timeout
        self.stale_completion = stale_completion
        self._part_written = False
        self._presence_valid = False
        self._started_at = None

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
        elif not old and self.coils[1] and self._presence_valid and self.coils[0] and not self.coils[4]:
            self.coils[0] = False
            self._started_at = now

    def advance(self, now):
        if self._started_at is None or self.coils[4] or self.timeout or self.stale_completion:
            return
        if now - self._started_at >= self.cycle_delay:
            self.registers[2] = (self.registers[2] + 1) % 65536
            self.coils[3] = True
            self._started_at = None
