# SPDX-License-Identifier: Apache-2.0
"""Protect bounded replay and latest-only telemetry while offline."""

from mqtt_gateway.reconnect_queue import ReconnectQueue


def test_replays_states_in_order_then_clears_queue():
    queue = ReconnectQueue()
    assert queue.drain_states() == []
    queue.push_state({"state": "ACCEPTED"})
    queue.push_state({"state": "COMPLETED"})
    assert queue.drain_states() == [{"state": "ACCEPTED"}, {"state": "COMPLETED"}]
    assert queue.drain_states() == []


def test_keeps_100_states_and_evicts_oldest_on_overflow():
    queue = ReconnectQueue()
    for sequence in range(102):
        queue.push_state({"sequence": sequence})
    states = queue.drain_states()
    assert len(states) == 100
    assert [state["sequence"] for state in states] == list(range(2, 102))


def test_telemetry_replaces_older_sample_and_pop_clears_it():
    queue = ReconnectQueue()
    assert queue.pop_telemetry() is None
    queue.set_telemetry({"sequence": 1})
    queue.set_telemetry({"sequence": 2})
    queue.push_state({"state": "COMPLETED"})
    assert queue.pop_telemetry() == {"sequence": 2}
    assert queue.pop_telemetry() is None
    assert queue.drain_states() == [{"state": "COMPLETED"}]


def test_queue_snapshots_nested_payloads_before_caller_mutation():
    queue = ReconnectQueue()
    payload = {"detail": {"events": ["accepted"]}}
    queue.push_state(payload)
    queue.set_telemetry(payload)
    payload["detail"]["events"].clear()
    states = queue.drain_states()
    assert states == [{"detail": {"events": ["accepted"]}}]
    states[0]["detail"]["events"].clear()
    assert queue.pop_telemetry() == {"detail": {"events": ["accepted"]}}
