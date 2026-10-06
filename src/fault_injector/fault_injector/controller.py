"""Thread-safe mission/station matching without random faults."""

import threading
import time

from .models import FaultRequest


class FaultController:
    def __init__(self, *, clock=time.monotonic, on_event=None):
        self._clock, self._on_event = clock, on_event
        self._entries = {}
        self._lock = threading.RLock()

    def enable(self, request: FaultRequest):
        if not isinstance(request, FaultRequest):
            raise TypeError("expected FaultRequest")
        with self._lock:
            self._entries[
                (
                    request.name,
                    request.mission_id,
                    request.station,
                    request.robot_id,
                    request.activation_point,
                )
            ] = (request, None)

    def _event(self, name, request):
        if self._on_event is not None:
            self._on_event(name, request)

    def query(self):
        with self._lock:
            for key, (request, started) in tuple(self._entries.items()):
                if started is not None and self._clock() >= started + request.duration:
                    del self._entries[key]
                    self._event("fault_reset", request)
            return tuple(request for request, _ in self._entries.values())

    def consume(self, name, mission_id, station, activation_point, *, robot_id=None):
        with self._lock:
            self.query()
            for key, (request, started) in tuple(self._entries.items()):
                if (
                    request.name == name
                    and request.mission_id == mission_id
                    and request.station in (None, station)
                    and request.robot_id in (None, robot_id)
                    and request.activation_point == activation_point
                ):
                    self._event("fault_activated", request)
                    if request.one_shot:
                        del self._entries[key]
                        self._event("fault_consumed", request)
                    else:
                        self._entries[key] = (
                            request,
                            self._clock() if started is None else started,
                        )
                    return request
            return None

    def reset(self):
        with self._lock:
            for request, _ in self._entries.values():
                self._event("fault_reset", request)
            self._entries.clear()

    def finish(self, request):
        """Report the end of a consumed effect, separately from its armed state."""
        with self._lock:
            self._event("fault_reset", request)
