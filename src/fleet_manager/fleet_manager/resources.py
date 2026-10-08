# SPDX-License-Identifier: Apache-2.0
"""Coordinate shared resources without transport or clock side effects."""

from dataclasses import dataclass, replace
import math
import secrets
from threading import Lock
from typing import Sequence

from fleet_manager.config import ResourceConfig


@dataclass(frozen=True)
class LeaseRequest:
    robot_id: str
    mission_id: str
    resource_id: str


@dataclass(frozen=True)
class LeaseKey:
    robot_id: str
    mission_id: str
    resource_id: str
    lease_id: str


@dataclass(frozen=True)
class Lease:
    robot_id: str
    mission_id: str
    resource_id: str
    lease_id: str
    expires_at: float


@dataclass(frozen=True)
class LeaseDecision:
    granted: bool
    lease: Lease | None
    current_owner: str | None
    reason: str


@dataclass(frozen=True)
class ResourceSnapshot:
    resource_id: str
    kind: str
    capacity: int
    lease: Lease | None
    waiters: tuple[LeaseRequest, ...]
    reconciliation_required: bool
    former_lease: Lease | None
    # All unresolved identities; former_lease remains the first for old callers.
    former_leases: tuple[Lease, ...] = ()


@dataclass(frozen=True)
class WaiterResolution:
    cancelled: bool
    lease: Lease | None
    reconciliation_required: bool
    reason: str


@dataclass(frozen=True)
class _Waiter:
    requested_at: float
    sequence: int
    request: LeaseRequest


def _finite(value: float, name: str) -> float:
    if type(value) not in (int, float):
        raise ValueError(f"{name} must be a finite number")
    try:
        result = float(value)
    except OverflowError as exc:
        raise ValueError(f"{name} must be a finite number") from exc
    if not math.isfinite(result):
        raise ValueError(f"{name} must be a finite number")
    return result


def _identity(value: str, name: str) -> None:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be a nonempty string")


class ResourceManager:
    """Serialize all state changes under one lock and return frozen values.

    Callers supply monotonic timestamps; no clock is read by this core. Initial
    acquisition grants immediately. Contended requests are ordered by their first
    receipt time, robot ID, then stable insertion sequence. A queued handoff grants
    under the same lock as release or clearance, so new callers cannot overtake it.
    """

    def __init__(
        self, resources: Sequence[ResourceConfig], lease_ttl_sec: float = 10.0
    ):
        self._ttl = _finite(lease_ttl_sec, "lease_ttl_sec")
        if self._ttl <= 0:
            raise ValueError("lease_ttl_sec must be positive")
        self._resources: dict[str, ResourceConfig] = {}
        for resource in resources:
            _identity(resource.resource_id, "resource_id")
            if resource.resource_id in self._resources:
                raise ValueError(f"duplicate resource_id: {resource.resource_id}")
            if type(resource.capacity) is not int or resource.capacity != 1:
                raise ValueError("resource capacity must be integer 1")
            if resource.kind not in ("station", "traffic_zone", "dock"):
                raise ValueError("resource kind must be station, traffic_zone, or dock")
            self._resources[resource.resource_id] = resource
        self._leases: dict[str, Lease] = {}
        self._former: dict[str, tuple[Lease, ...]] = {}
        self._queues: dict[str, list[_Waiter]] = {
            resource_id: [] for resource_id in self._resources
        }
        self._sequence = 0
        self._lock = Lock()

    def _time(self, now: float) -> float:
        now = _finite(now, "now")
        if not math.isfinite(now + self._ttl):
            raise ValueError("lease expiry must be finite")
        return now

    def _request(self, request: LeaseRequest | LeaseKey) -> None:
        for field in ("robot_id", "mission_id", "resource_id"):
            _identity(getattr(request, field), field)
        if request.resource_id not in self._resources:
            raise KeyError(request.resource_id)

    @staticmethod
    def _same_request(lease: Lease, request: LeaseRequest | LeaseKey) -> bool:
        return (
            lease.robot_id == request.robot_id
            and lease.mission_id == request.mission_id
            and lease.resource_id == request.resource_id
        )

    @classmethod
    def _same_key(cls, lease: Lease | None, key: LeaseKey) -> bool:
        return (
            lease is not None
            and cls._same_request(lease, key)
            and lease.lease_id == key.lease_id
        )

    def _grant(self, request: LeaseRequest, now: float) -> Lease:
        lease = Lease(
            request.robot_id,
            request.mission_id,
            request.resource_id,
            secrets.token_urlsafe(32),
            now + self._ttl,
        )
        self._leases[request.resource_id] = lease
        return lease

    def _grant_next(self, resource_id: str, now: float) -> None:
        queue = self._queues[resource_id]
        if queue and resource_id not in self._former:
            self._grant(queue.pop(0).request, now)

    def _expire(self, now: float) -> tuple[Lease, ...]:
        expired = tuple(
            lease for lease in self._leases.values() if now >= lease.expires_at
        )
        for lease in expired:
            del self._leases[lease.resource_id]
            self._former[lease.resource_id] = (lease,)
        return expired

    def _decision(self, resource_id: str, granted: bool, reason: str) -> LeaseDecision:
        lease = self._leases.get(resource_id)
        former = self._former.get(resource_id, ())
        owner = lease or (former[0] if former else None)
        return LeaseDecision(
            granted,
            lease if granted else None,
            owner.robot_id if owner else None,
            reason,
        )

    def acquire(self, request: LeaseRequest, now: float) -> LeaseDecision:
        self._request(request)
        now = self._time(now)
        with self._lock:
            self._expire(now)
            lease = self._leases.get(request.resource_id)
            if lease is not None and self._same_request(lease, request):
                return self._decision(request.resource_id, True, "granted")
            queue = self._queues[request.resource_id]
            if not any(waiter.request == request for waiter in queue):
                queue.append(_Waiter(now, self._sequence, request))
                self._sequence += 1
                queue.sort(
                    key=lambda waiter: (
                        waiter.requested_at,
                        waiter.request.robot_id,
                        waiter.sequence,
                    )
                )
            if lease is None and request.resource_id not in self._former:
                self._grant_next(request.resource_id, now)
                lease = self._leases.get(request.resource_id)
                if lease is not None and self._same_request(lease, request):
                    return self._decision(request.resource_id, True, "granted")
            reason = (
                "reconciliation required"
                if request.resource_id in self._former
                else "queued"
            )
            return self._decision(request.resource_id, False, reason)

    def renew(self, key: LeaseKey, now: float) -> LeaseDecision:
        self._request(key)
        now = self._time(now)
        with self._lock:
            self._expire(now)
            lease = self._leases.get(key.resource_id)
            if not self._same_key(lease, key):
                return self._decision(key.resource_id, False, "lease mismatch")
            self._leases[key.resource_id] = replace(lease, expires_at=now + self._ttl)
            return self._decision(key.resource_id, True, "renewed")

    def release(self, key: LeaseKey, now: float) -> bool:
        self._request(key)
        now = self._time(now)
        with self._lock:
            self._expire(now)
            if not self._same_key(self._leases.get(key.resource_id), key):
                return False
            del self._leases[key.resource_id]
            self._grant_next(key.resource_id, now)
            return True

    def cancel_waiter(self, request: LeaseRequest, now: float) -> bool:
        """Cancel a queued request without releasing any held lease."""
        return self.resolve_waiter(request, now).cancelled

    def resolve_waiter(self, request: LeaseRequest, now: float) -> WaiterResolution:
        """Remove an exact waiter and resolve its ownership under the same lock.

        Automatic handoff may already have granted the request. Return only that
        request's live lease; never release it or reveal another owner's token.
        A matching quarantined former owner requires physical reconciliation.
        """
        self._request(request)
        now = self._time(now)
        with self._lock:
            self._expire(now)
            queue = self._queues[request.resource_id]
            cancelled = False
            for index, waiter in enumerate(queue):
                if waiter.request == request:
                    queue.pop(index)
                    cancelled = True
                    break
            lease = self._leases.get(request.resource_id)
            exact = (
                lease
                if lease is not None and self._same_request(lease, request)
                else None
            )
            reconciliation = any(
                self._same_request(former, request)
                for former in self._former.get(request.resource_id, ())
            )
            reason = (
                "reconciliation required"
                if reconciliation
                else "exact request owns lease"
                if exact is not None
                else "waiter cancelled"
                if cancelled
                else "no waiter or ownership"
            )
            return WaiterResolution(cancelled, exact, reconciliation, reason)

    def expire(self, now: float) -> tuple[Lease, ...]:
        """Fence expired owners; physical occupancy remains uncertain."""
        now = self._time(now)
        with self._lock:
            return self._expire(now)

    def release_owner(self, robot_id: str, now: float) -> tuple[Lease, ...]:
        """Quarantine an offline owner's holds and discard its queued requests."""
        _identity(robot_id, "robot_id")
        now = self._time(now)
        with self._lock:
            expired = self._expire(now)
            removed = tuple(
                lease for lease in self._leases.values() if lease.robot_id == robot_id
            )
            for lease in removed:
                del self._leases[lease.resource_id]
                self._former[lease.resource_id] = (lease,)
            for resource_id, queue in self._queues.items():
                self._queues[resource_id] = [
                    waiter for waiter in queue if waiter.request.robot_id != robot_id
                ]
            return (
                tuple(lease for lease in expired if lease.robot_id == robot_id)
                + removed
            )

    def clear_reconciliation(self, key: LeaseKey, now: float) -> bool:
        """Clear exact former lease only after caller proves its owner is outside.

        This method accepts the caller's clearance proof as an authorization.
        The core cannot observe physical occupancy. Transport and recovery adapters
        must establish that proof before calling this method.
        """
        self._request(key)
        now = self._time(now)
        with self._lock:
            self._expire(now)
            claims = self._former.get(key.resource_id, ())
            if not any(self._same_key(lease, key) for lease in claims):
                return False
            remaining = tuple(
                lease for lease in claims if not self._same_key(lease, key)
            )
            if remaining:
                self._former[key.resource_id] = remaining
            else:
                del self._former[key.resource_id]
                self._grant_next(key.resource_id, now)
            return True

    def restore_evidence(self, snapshots: Sequence[ResourceSnapshot]) -> None:
        """Load former authority only as quarantine; never restore a live token.

        Call before serving resource operations. Queues and old process-clock
        deadlines cannot confer authority in a new process.
        """
        former = {}
        for snapshot in snapshots:
            claims = snapshot.former_leases or (
                (snapshot.former_lease,) if snapshot.former_lease is not None else ()
            )
            if snapshot.lease is not None:
                claims = (snapshot.lease,) + claims
            for lease in claims:
                self._request(lease)
                _identity(lease.lease_id, "lease_id")
                if snapshot.resource_id != lease.resource_id:
                    raise ValueError("persisted resource identity mismatch")
                existing = former.get(lease.resource_id, ())
                if lease not in existing:
                    former[lease.resource_id] = existing + (lease,)
        with self._lock:
            if self._leases or self._former or any(self._queues.values()):
                raise ValueError("cannot restore evidence after resource activity")
            self._former.update(former)

    def quarantine_evidence(self, lease: Lease) -> None:
        """Fence observed occupancy that has no persisted authority identity."""
        self._request(lease)
        _identity(lease.lease_id, "lease_id")
        with self._lock:
            if lease.resource_id in self._leases:
                raise ValueError("cannot replace existing resource ownership evidence")
            existing = self._former.get(lease.resource_id, ())
            if lease not in existing:
                self._former[lease.resource_id] = existing + (lease,)

    def snapshot(self, now: float) -> tuple[ResourceSnapshot, ...]:
        """Return immutable detached state, fencing leases expired at this time."""
        now = self._time(now)
        with self._lock:
            self._expire(now)
            return tuple(
                ResourceSnapshot(
                    resource.resource_id,
                    resource.kind,
                    resource.capacity,
                    self._leases.get(resource.resource_id),
                    tuple(
                        waiter.request for waiter in self._queues[resource.resource_id]
                    ),
                    resource.resource_id in self._former,
                    next(iter(self._former.get(resource.resource_id, ())), None),
                    self._former.get(resource.resource_id, ()),
                )
                for resource in self._resources.values()
            )
