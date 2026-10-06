"""Thread-safe mission/station matching without random faults."""

import threading
import time
from dataclasses import dataclass

from .models import FaultRequest


@dataclass(frozen=True, eq=False)
class FaultEffect:
    """An immutable activation identity, separate from the requested fault scope."""

    request: FaultRequest
    activation_robot_id: str

    def __getattr__(self, name):
        return getattr(self.request, name)

    def __eq__(self, other):
        if isinstance(other, FaultEffect):
            return (self.request, self.activation_robot_id) == (
                other.request,
                other.activation_robot_id,
            )
        return self.request == other

    def __hash__(self):
        return hash(self.request)


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
            ] = (request, None, set())

    def _event(self, name, request):
        if self._on_event is not None:
            self._on_event(name, request)

    def query(self):
        with self._lock:
            for key, (request, started, effects) in tuple(self._entries.items()):
                if started is not None and self._clock() >= started + request.duration:
                    del self._entries[key]
                    for effect in effects or (request,):
                        self._event("fault_reset", effect)
            return tuple(request for request, _, _ in self._entries.values())

    def consume(self, name, mission_id, station, activation_point, *, robot_id=None):
        with self._lock:
            self.query()
            for key, (request, started, effects) in tuple(self._entries.items()):
                if (
                    request.name == name
                    and request.mission_id == mission_id
                    and request.station in (None, station)
                    and request.robot_id in (None, robot_id)
                    and request.activation_point == activation_point
                ):
                    effect = FaultEffect(request, robot_id or request.robot_id or "")
                    self._event("fault_activated", effect)
                    if request.one_shot:
                        del self._entries[key]
                        self._event("fault_consumed", effect)
                    else:
                        self._entries[key] = (
                            request,
                            self._clock() if started is None else started,
                            effects | {effect},
                        )
                    return effect
            return None

    def reset(self):
        with self._lock:
            for request, _, effects in self._entries.values():
                for effect in effects or (request,):
                    self._event("fault_reset", effect)
            self._entries.clear()

    def finish(self, request):
        """Report the end of a consumed effect, separately from its armed state."""
        with self._lock:
            self._event("fault_reset", request)
