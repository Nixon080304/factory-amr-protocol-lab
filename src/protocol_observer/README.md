# Protocol observer core

This package records protocol events and creates mission summaries without ROS,
network connections, or wall-clock calculations. ROS subscriptions belong to a
later adapter. Observer errors must be isolated from the mission control path.

## Event contract

`ProtocolEventRecord` contains the exact `factory_interfaces/msg/ProtocolEvent`
fields: `stamp`, `mission_id`, `protocol`, `direction`, `event`, `outcome`,
`latency_ms`, and `detail`. `stamp` is a timezone-aware `datetime`, normalized to
UTC. The future DDS adapter converts `stamp.sec` and `stamp.nanosec` to this
representation; Python datetime precision is microseconds. Producers must use a
shared clock domain. The core cannot correct source clock skew.

The additional optional `sequence` is observer-local metadata, not a DDS field.
`TraceWriter(path)` assigns sequence numbers starting at 1 and continues above
the largest sequence when reopening an existing trace. It preserves the original
timestamp and does not mutate the supplied record. One observer owns each path;
concurrent writers are unsupported. The parent directory must already exist.
Each append closes the file. Invalid JSON or malformed existing traces raise an
error rather than overwriting evidence.

Report aggregation filters by exact nonblank mission ID before ordering by
`(stamp, sequence)`. Events without a sequence use 0; exact ties retain input
order. Blank mission IDs represent uncorrelated events and cannot be reported as
one mission. `latency_ms` is preserved in traces but never supplies report
durations.

Producers emit these exact, case-sensitive event pairs:

| Phase | Start event | Finish event |
| --- | --- | --- |
| Mission | `mission_started` | `mission_finished` |
| MQTT acceptance | `mqtt_acceptance_started` | `mqtt_acceptance_finished` |
| Navigation to pickup | `navigation_pickup_started` | `navigation_pickup_finished` |
| Navigation to dropoff | `navigation_dropoff_started` | `navigation_dropoff_finished` |
| Pickup perception | `perception_pickup_started` | `perception_pickup_finished` |
| Dropoff perception | `perception_dropoff_started` | `perception_dropoff_finished` |
| Pickup Modbus handshake | `modbus_pickup_started` | `modbus_pickup_finished` |
| Dropoff Modbus handshake | `modbus_dropoff_started` | `modbus_dropoff_finished` |

Each phase requires alternating start/finish events. Durations sum the elapsed
time of all completed attempts for that phase, including failed attempts.
Missing events, unmatched finish events, overlapping starts, and unfinished
attempts make the whole phase `not available`. Unknown event names do not imply
phase boundaries. Equal timestamps can produce a measured zero interval when
their sequence establishes start before finish. A finish preceding its start
cannot produce a negative duration.

The last `mission_finished` event supplies the final outcome. Use `COMPLETED` or
`FAILED` for terminal missions; absent or empty terminal outcomes produce
`not available`. Phase failures do not imply a terminal mission outcome.
Emit one event named exactly `retry` per actual retry. The summary counts those
events. It separately counts every event with outcome exactly `FAILED` as
**Failure events**, not unique faults or failed missions; a phase failure and its
terminal failure count as two events.

Call `MissionReport.from_events(mission_id, events).to_markdown()` to generate
a deterministic summary with millisecond durations. Failed or incomplete
missions retain available values and mark absent values `not available`.
