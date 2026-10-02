# Protocol contracts

The protocol core provides transport-independent mission, MQTT, Modbus, and
trace logic. ROS node adapters, Nav2, perception, and Gazebo integration belong
to later milestones. The contracts below are the boundaries those adapters
must preserve.

## MQTT

The local broker listens on host `127.0.0.1:1883`.

| Topic | Direction | MQTT QoS | Retained |
| --- | --- | ---: | --- |
| `factory/missions/request` | Dispatcher to robot | 1 | No |
| `factory/missions/<mission_id>/status` | Robot to dispatcher | 1 | No |
| `factory/robots/amr_01/telemetry` | Robot to dispatcher | 0 | No |
| `factory/robots/amr_01/availability` | Robot to dispatcher | 1 | Yes |

The future gateway adapter configures a retained last-will message of `offline`
and publishes retained `online` on connection. Mission status events queue in
order during a disconnect, up to 100 entries. When full, the queue drops the
oldest event. Telemetry keeps only the latest sample and does not replay history.

Mission request:

```json
{
  "mission_id": "M-001",
  "robot_id": "amr_01",
  "pickup": "assembly",
  "dropoff": "inspection",
  "part": "motor"
}
```

| JSON field | Type | Accepted value |
| --- | --- | --- |
| `mission_id` | String | 1–64 ASCII letters, digits, underscores, or hyphens |
| `robot_id` | String | `amr_01` |
| `pickup` | String | `assembly` |
| `dropoff` | String | `inspection` |
| `part` | String | `motor` |

All five fields are required. String fields are limited to 64 characters.
Unknown properties are rejected (`additionalProperties: false`). Requests must
be UTF-8 JSON objects of at most 4096 bytes. The installed schema is
`mqtt_gateway/schemas/mission.schema.json` within the Python package.
`MissionValidator().validate(raw_payload)` returns a `MissionPayload` or raises
`MissionValidationError` with `error_code == "INVALID_MISSION"`.

Mission status payload:

```json
{
  "mission_id": "M-001",
  "robot_id": "amr_01",
  "state": "NAVIGATING_TO_PICKUP",
  "station": "assembly",
  "timestamp": "2026-10-02T12:00:00.000Z",
  "sequence": 3,
  "detail": "Nav2 goal accepted",
  "error_code": null
}
```

| JSON field | Type | Meaning |
| --- | --- | --- |
| `mission_id` | String | Correlation ID from the request |
| `robot_id` | String | Configured robot, `amr_01` |
| `state` | String | Mission state |
| `station` | String | Current station |
| `timestamp` | String | UTC timestamp |
| `sequence` | Integer | Ordered mission-state sequence |
| `detail` | String | Human-readable transition detail |
| `error_code` | String or null | Stable failure code, or no error |

The status object is an adapter contract; only the request schema and core
validation/registry/queue behavior are implemented in this milestone. Telemetry
JSON fields will be defined with the robot adapter.

Duplicate IDs with the same canonical payload return the current or final state
without another execution. JSON key order does not change mission identity.
Changed payloads for the same ID return `MISSION_ID_CONFLICT`. The registry is
in memory; process-restart recovery is outside version 1. Only one mission may
run at a time; another valid mission returns `ROBOT_BUSY`.

## Modbus TCP

The local PLC listens on host `127.0.0.1:1502`. Both station unit IDs use the
same zero-based wire addresses; these are not `00001`/`40001` display offsets.

| Unit ID | Station | Cycle |
| ---: | --- | --- |
| 1 | `assembly` | Load |
| 2 | `inspection` | Unload |

### Coils

| Address | Name | Owner | Meaning |
| ---: | --- | --- | --- |
| 0 | `station_ready` | PLC | Station can begin a transfer. |
| 1 | `robot_present` | Robot | Robot is localized and visually confirmed at station. |
| 2 | `transfer_request` | Robot | Robot requests the station's configured load or unload cycle. |
| 3 | `transfer_complete` | PLC | Station completed the current cycle. |
| 4 | `station_fault` | PLC | Station cannot complete the cycle. |

### Holding registers

| Address | Name | Owner | Meaning |
| ---: | --- | --- | --- |
| 0 | `station_id` | PLC | Numeric station identity. |
| 1 | `part_code` | Robot | Numeric code for the requested part. |
| 2 | `cycle_counter` | PLC | Increments after every completed cycle. |
| 3 | `fault_code` | PLC | Zero for no fault; otherwise a documented station fault. |

Register values are unsigned 16-bit integers. `station_id` equals the unit ID.
The simulator accepts `part_code` in `0..65535`; the `motor` mapping remains an
adapter decision. `cycle_counter` wraps from 65535 to 0, so a changed counter,
including wraparound, represents one completed cycle. `fault_code == 0` means
no fault. `--fault-code N` injects station fault code `N` in `1..65535`;
version 1 has no station-specific fault-code enumeration.

The handshake follows this order:

1. Read `station_ready`, `station_fault`, and the initial `cycle_counter`.
2. Write `part_code` before setting `robot_present`.
3. Set `transfer_request` to start the station cycle.
4. Poll until `transfer_complete` is true **and** `cycle_counter` changes.
5. After success or failure, clear `transfer_request` and `robot_present`.
   The PLC clears `transfer_complete` after observing the cleared request.

The gateway owns all production knowledge of raw addresses. The PLC rejects
writes to PLC-owned values. `StationClient(host="127.0.0.1", port=1502)` exposes
`await transfer(unit_id, part_code)` and returns `TransferResult(success,
error_code, message, cycle_counter)`. It serializes transfers per client. A
lost transfer-request acknowledgment must not trigger another cycle: the
gateway sends that request once and observes the same cycle. Idempotent
network operations may retry; cleanup is separately bounded. A cleanup failure
makes the result unsuccessful and appears in its message.

### Timeouts and retry bounds

| Operation | Bound or behavior |
| --- | --- |
| Modbus response | 2 seconds per attempt |
| Idempotent network retry | Three retries after the initial attempt; delays 0.5, 1.0, and 2.0 seconds |
| Transfer observation | 2-second default deadline; retries are clipped to the remaining deadline |
| Station poll | 10 Hz (0.1-second interval) |
| Coil cleanup | One attempt bounded by the response timeout |
| PLC simulator cycle | 0.2 seconds by default |
| Perception confirmation | Five matching detections within 1.5 seconds |
| Missing required marker | Fail after 10 seconds |
| Nav2 failure | Clear costmaps and retry once per navigation leg |
| Compose readiness | Wait at most 60 seconds after startup in the documented command |

The standalone CLI has no ROS dependency:

```bash
plc-simulator --host 127.0.0.1 --port 1502
plc-simulator --timeout
plc-simulator --stale-completion
plc-simulator --fault-code 7
plc-simulator --response-delay 0.5 --request-response-delay 0.5
```

`--cycle-delay` controls cycle duration; `--timeout` prevents completion;
`--stale-completion` leaves the initial completion coil stale;
`--response-delay` delays responses; `--request-response-delay` delays the
transfer-request acknowledgment. The container explicitly binds `0.0.0.0`
inside its network namespace; Compose publishes only the host loopback port.

## ROS 2 / DDS

| Interface or topic category | Reliability | Durability / history |
| --- | --- | --- |
| Mission actions and transfer services | Reliable ROS action/service semantics | Standard action/service QoS |
| Mission-state and protocol-event topics | Reliable | Depth 100 |
| Factory-state topics | Reliable | Transient-local for late subscribers |
| Camera and LiDAR topics | Best effort | Sensor-data QoS |
| Telemetry sampling | Best effort | Latest sample replaces a lost sample |

The future QoS mismatch scenario intentionally pairs a reliable subscriber with
an incompatible publisher, verifies the discovery warning, and restores
compatible QoS. This milestone generates typed interfaces and tests cores;
it does not yet provide those DDS publishers or the mismatch launch scenario.

Mission states are `RECEIVED`, `NAVIGATING_TO_PICKUP`, `VERIFYING_PICKUP`,
`LOADING`, `NAVIGATING_TO_DROPOFF`, `VERIFYING_DROPOFF`, `UNLOADING`,
`COMPLETED`, `RECOVERING`, and `FAILED`. The C++ core also has an internal
`Idle` state. Cancellation during transfer waits for a safe terminal handshake
outcome. A terminal core requires explicit `reset()` before another mission.

## Error handling

| Failure | Required behavior / error code |
| --- | --- |
| Invalid MQTT JSON | Reject without starting an action; `INVALID_MISSION` |
| Duplicate mission, same payload | Return current or final state; no second execution |
| Duplicate mission, changed payload | Reject with `MISSION_ID_CONFLICT` |
| Another valid mission while busy | Reject with `ROBOT_BUSY` |
| MQTT disconnect | Continue the local mission, queue state transitions, reconnect with backoff |
| Nav2 goal rejected or aborted | Clear costmaps, retry once, then `NAVIGATION_FAILED` |
| Required marker missing | Wait up to 10 seconds, then `STATION_NOT_CONFIRMED` |
| Wrong marker detected | Do not contact PLC; wait for the required marker until timeout |
| Modbus timeout | Retry eligible operations three times, clear request state, then `PLC_TIMEOUT` |
| Stale completion coil | Require a counter change; `STALE_PLC_STATE` after timeout |
| PLC fault coil | Clear request state; `PLC_FAULT` with the register fault code |
| Observer failure | Log locally; mission continues |

Errors include a stable machine-readable code and concise human-readable detail.
Operational adapters must log mission ID, robot ID, station, and current state.

## Observer event contract

`ProtocolEventRecord` preserves the exact DDS fields `stamp`, `mission_id`,
`protocol`, `direction`, `event`, `outcome`, `latency_ms`, and `detail`. Its
timestamp is a timezone-aware UTC `datetime`; producers must share a clock
domain. The optional `sequence` is observer-local metadata, not a DDS field.

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

`TraceWriter(path)` appends JSONL, assigns sequence numbers starting at 1, and
continues above the largest existing sequence. One observer owns each path;
its parent directory must exist. Malformed traces raise an error rather than
being overwritten. Producers isolate observer errors from mission control.

`MissionReport.from_events(mission_id, events).to_markdown()` filters by exact
nonblank mission ID and orders events by timestamp then sequence. Alternating
start/finish pairs determine durations, summed across completed attempts,
including failed attempts. Missing, overlapping, or unfinished pairs make that
whole phase `not available`. `latency_ms` never substitutes for timestamps.
The last `mission_finished` supplies the final outcome (`COMPLETED` or `FAILED`).
Emit one exact `retry` event per retry. Failure count counts every event whose
outcome is exactly `FAILED`, not unique faults or failed missions.

## Local service lifecycle

```bash
docker compose -f docker/compose.yaml config --quiet
docker compose -f docker/compose.yaml up -d --build --wait --wait-timeout 60
docker compose -f docker/compose.yaml ps
docker compose -f docker/compose.yaml down
```

Broker readiness requires a real QoS 1 publish acknowledgment. PLC readiness
requires successful holding-register reads and correct station identity for
both unit IDs. Neither healthcheck treats an open TCP socket alone as readiness.
The default Compose project is `factory-amr-protocol-lab`. For an isolated test,
pass the same unique `-p` value to every command and shut down only that project.
Services use anonymous MQTT within the localhost simulation boundary; no
secrets are committed. Production authentication and safety certification are
outside version 1.
