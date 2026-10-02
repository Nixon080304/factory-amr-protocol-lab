# Repeatable fault scenarios

Run `scripts/run_scenario.sh <name>` or `scripts/run_all_scenarios.sh` from the
repository root after setup and a `colcon build --symlink-install`. Gazebo cases
need a working display even with GUI windows disabled; `DISPLAY=:0` and software
rendering work on the measured local environment.

The runner validates names before starting services. Each case gets a new
directory at `reports/<run-id>/<scenario>/` containing `actual.json`,
`outcome.json`, `outcome.md`, `protocol_events.jsonl`, and `command.log`.
Physical cases also retain launch logs and rendered-image proof. An outcome
matches only when the actual final state, error code, source, action count where
applicable, and all correlated trace predicates match
`tests/scenarios/expected_outcomes.yaml`. The YAML file uses JSON-compatible YAML
syntax so the runner needs no additional parser dependency.

The default deadline is 300 seconds per case. Set
`FACTORY_SCENARIO_TIMEOUT=<seconds>` to lower it. Timeout returns 124;
Ctrl+C and termination return 130 and 143. A failed case command preserves its
exit code. Cleanup may take up to 65 additional seconds. Cleanup stops only
owned processes and services and verifies that their ports close. Every new
case resets its fault controls. Physical missions launch a fresh fixed-part
simulation so a completed pickup cannot contaminate another case. Matrix runs
continue after failures, save `summary.json` and `summary.md`, and exit nonzero
if any outcome differs or the matrix is interrupted.

The supervisor holds each DDS domain lease until owned descendants and services
are verified stopped, including hard escalation after a driver timeout. A failed
cleanup quarantines that domain in the fixed host lock file
`/tmp/factory-amr-scenario-domain-<domain>.lock`, independent of `TMPDIR`. The record
retains the domain, process identities, service identities, and cleanup failures.
Cooperating runners skip quarantine even when no process holds the lock. Clearing
quarantine requires explicit manual verification of those recorded resources and
exclusive acquisition of the same lock before updating its metadata. There is no
automatic reclaim or general process-crash recovery guarantee.

Commands that cannot start produce failed outcomes with explicit startup errors.
If a case supplies no outcome, the matrix preserves a failed row with cleanup
marked unverified, continues ordinary failures, and still saves both summaries.
An interruption instead saves the partial matrix and preserves its signal exit.

## Measured matrix

Fresh local run `20261003T040407-44f8e160e765` completed all 12 cases with zero
unexpected outcomes. The table is derived from its actual outcome files.
Elapsed time includes setup, assertions, and cleanup; it is not protocol
latency. Trace counts are runner predicates, in addition to the underlying
system assertions.

| Scenario | Actual and expected result | Evidence source | Elapsed seconds | Trace checks |
| --- | --- | --- | --- | --- |
| `success` | `COMPLETED` | Real Gazebo | 113.853 | 4/4 |
| `mqtt_duplicate` | `COMPLETED` | Real protocols and navigation driver | 3.561 | 6/6 |
| `mqtt_conflict` | `COMPLETED`; changed request rejected | Real protocols and navigation driver | 2.051 | 6/6 |
| `mqtt_disconnect` | `COMPLETED` | Real protocols and navigation driver | 2.534 | 5/5 |
| `modbus_delay` | `COMPLETED` | Real protocols and navigation driver | 2.654 | 6/6 |
| `modbus_timeout` | `FAILED/PLC_TIMEOUT` | Real protocols and navigation driver | 1.969 | 5/5 |
| `modbus_stale_completion` | `FAILED/STALE_PLC_STATE` | Real protocols and navigation driver | 2.010 | 5/5 |
| `plc_fault` | `FAILED/PLC_FAULT` | Real protocols and navigation driver | 1.650 | 5/5 |
| `wrong_marker` | `FAILED/STATION_NOT_CONFIRMED` | Real Gazebo | 49.882 | 8/8 |
| `nav_retry` | `COMPLETED` | Real Gazebo | 83.294 | 8/8 |
| `nav_failure` | `FAILED/NAVIGATION_FAILED` | Real Gazebo | 9.874 | 8/8 |
| `qos_mismatch` | `RECOVERED` experiment | Isolated real DDS | 2.977 | 7/7 |

This matrix measures scenario behavior. Subsequent supervisor cleanup changes
have separate timeout and interruption regression checks; the measured elapsed
times above remain the original run values.

## What the evidence shows

MQTT duplicate delivery does not imply another physical action. Duplicate and
conflict cases each execute one authoritative action and one pickup/dropoff
PLC cycle. `mqtt_conflict` separately checks `mqtt_rejected/FAILED` with
`MISSION_ID_CONFLICT`; the original accepted mission completes. The rejection
must not replace the original action's final result. Disconnect occurs after
acceptance: the local action completes while transport is disconnected, then
status replay restores the ordered sequences 1 through 5 after reconnection.
MQTT delivery acknowledgment alone is not a mission completion guarantee.

Modbus delay completes after exactly one retry with a 0.5-second backoff. Timeout,
stale completion, and PLC fault stop at pickup, retain the part at assembly,
leave both robot request coils clear, and do not increment the pickup counter.
The PLC fault carries code 91. A completion bit from an old cycle cannot prove
the current transfer succeeded; the gateway also checks the cycle counter.
Ambiguous writes must not be duplicated. Trace evidence includes the unique
`modbus_pickup_finished` result and fault activation, consumption, and reset.

Wrong-marker evidence comes from rendered Gazebo camera images, independently
decoded marker 20 stamps, and detector output at assembly. The coordinator
fails perception confirmation and dispatches no Modbus transfer. Reset restores
the correct station board and a newly rendered marker 10 observation. Navigation
cases use public names `nav_retry` and `nav_failure`, mapped to
`nav_reject_once` and `nav_reject_twice`. Both exercise real Nav2 adapters, both
actual costmap clear responses, and exactly one retry of the same pickup pose.
The second rejection fails safely without PLC transfer. Successful physical
missions prove five fresh detector stamps per station, an independently decoded
rendered source stamp in that window, actual motion, and both PLC cycles.

DDS mismatch is an experiment, not a physical mission. A
BEST_EFFORT/VOLATILE publisher and RELIABLE/VOLATILE subscriber report both actual
RELIABILITY incompatibility callbacks. The measured mismatch lasts 2.001 seconds
with zero samples and zero matches. A compatible BEST_EFFORT subscriber then
receives one sample with one match. Reset removes the experiment endpoints.
These results distinguish discovery and QoS compatibility from application
success, and do not alter the camera or LiDAR QoS.

The protocol fault cases use a bounded navigation action driver and claim real
MQTT, ROS, and Modbus coverage only. Full Gazebo assertions run locally; no CI
Gazebo coverage is claimed. The measured headless run logs Nav2's inherited
default goal-checker warning. The RViz renderer limitation remains documented
in [architecture](architecture.md). The full Python verification also reports
the inherited Xacro warnings `invalid escape sequence '\$'` and
`invalid escape sequence '\s'`. A vanished or hung transfer
gateway can leave a stopped pending mission with unknown PLC state. The runner's
deadline bounds its test process; it does not establish crash recovery or a
deadline for every production mission.
