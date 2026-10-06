# SPDX-License-Identifier: Apache-2.0
"""Append-only JSONL output, owned by one observer process per path."""

import json
from dataclasses import replace
from pathlib import Path

from .models import ProtocolEventRecord


class TraceWriter:
    """Assign local arrival order, preserving each source timestamp.

    Reopening an existing trace continues its sequence. Writers must not
    concurrently share a path; the later ROS adapter owns one writer.
    """

    def __init__(self, path: str | Path, *, allow_legacy=False):
        self.path = Path(path)
        self._sequence = 0
        if self.path.exists():
            with self.path.open(encoding="utf-8") as trace:
                for line in trace:
                    if not line.endswith("\n"):
                        raise ValueError(
                            "existing trace record must end with a newline"
                        )
                    record = ProtocolEventRecord.from_dict(
                        json.loads(line), allow_legacy=allow_legacy
                    )
                    self._sequence = max(self._sequence, record.sequence or 0)

    def append(self, event: ProtocolEventRecord) -> None:
        sequence = self._sequence + 1
        record = replace(event, sequence=sequence)
        line = json.dumps(record.to_dict(), ensure_ascii=False, allow_nan=False)
        with self.path.open("a", encoding="utf-8") as trace:
            trace.write(line + "\n")
        self._sequence = sequence
