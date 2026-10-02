# SPDX-License-Identifier: Apache-2.0
"""Test-only assertions for correlated coordinator and gateway phase evidence."""


def assert_successful_transfer_phases(events):
    coordinator = [
        event
        for event in events
        if event.event
        in (
            "perception_pickup_finished",
            "perception_dropoff_finished",
            "transfer_result",
        )
        or (event.event == "state_changed" and event.detail in ("LOADING", "UNLOADING"))
    ]
    assert all(event.protocol == "ROS" for event in coordinator)
    assert [
        (event.event, event.detail if event.event == "state_changed" else "")
        for event in coordinator
    ] == [
        ("perception_pickup_finished", ""),
        ("state_changed", "LOADING"),
        ("transfer_result", ""),
        ("perception_dropoff_finished", ""),
        ("state_changed", "UNLOADING"),
        ("transfer_result", ""),
    ]
    assert all(
        event.outcome == "SUCCEEDED"
        for event in coordinator
        if event.event != "state_changed"
    )
    gateway = [
        event
        for event in events
        if event.event
        in (
            "modbus_pickup_started",
            "modbus_pickup_finished",
            "modbus_dropoff_started",
            "modbus_dropoff_finished",
        )
    ]
    assert all(event.protocol == "MODBUS" for event in gateway)
    assert [event.event for event in gateway] == [
        "modbus_pickup_started",
        "modbus_pickup_finished",
        "modbus_dropoff_started",
        "modbus_dropoff_finished",
    ]
    assert all(
        event.outcome == "SUCCEEDED"
        for event in gateway
        if event.event.endswith("_finished")
    )

    def source(event):
        return event.stamp.sec * 1_000_000_000 + event.stamp.nanosec

    assert all(
        source(left) <= source(right)
        for left, right in zip(coordinator, coordinator[1:])
    )
    assert all(
        source(left) <= source(right) for left, right in zip(gateway, gateway[1:])
    )
    # Generic transfer_result is associated by its unique position in each
    # coordinator leg, never by an ambiguous first match over all arrivals.
    for coordinator_offset, gateway_offset in ((0, 0), (3, 2)):
        confirmed, state, result = coordinator[
            coordinator_offset : coordinator_offset + 3
        ]
        started, finished = gateway[gateway_offset : gateway_offset + 2]
        assert (
            source(confirmed)
            <= source(state)
            <= source(started)
            <= source(finished)
            <= source(result)
        )
