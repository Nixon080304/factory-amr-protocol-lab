"""Zero-based wire addresses shared by the two station unit IDs."""
from typing import Final

ASSEMBLY_UNIT: Final = 1
INSPECTION_UNIT: Final = 2
STATION_READY: Final = 0
ROBOT_PRESENT: Final = 1
TRANSFER_REQUEST: Final = 2
TRANSFER_COMPLETE: Final = 3
STATION_FAULT: Final = 4
STATION_ID: Final = 0
PART_CODE: Final = 1
CYCLE_COUNTER: Final = 2
FAULT_CODE: Final = 3
RETRY_DELAYS: Final = (0.5, 1.0, 2.0)
POLL_INTERVAL: Final = 0.1
RESPONSE_TIMEOUT: Final = 2.0
