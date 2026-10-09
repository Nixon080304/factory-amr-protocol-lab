# Fleet operations

The supported fleet commands run two real Gazebo/Nav2 stacks and real local
MQTT/Modbus peers. The robot-loss and ten-agent checks use real production fleet
nodes and DDS endpoints with bounded synthetic navigation peers. They are not
physical-motion proof.

Each lease-enabled route leg supports one traffic segment. Configuration and
the robot coordinator reject multi-segment routes until protected handoff bays
exist. This restriction prevents unsafe adjacent-zone handoff and does not
limit robot count. Separate station waiting and terminal parking poses keep
the outgoing owner away from the next robot's staging point. The supplied
0.60 m planned clearance includes both robots' arrival error; actual sampled
physical non-overlap uses the two 0.15 m robot footprints, not arrival error.
The supplied physical waiting layout is proven for two simulated robots.
More than two physical contenders require additional distinct protected waiting
bays; the ten-agent check proves fleet orchestration with synthetic DDS peers,
not a ten-robot collision-free layout. Terminal parking uses the outward station
side, away from the dock and shared approach waypoints. Every configured robot
has a distinct `station_exit_poses` bay at each waiting station. The validator
checks the other bays and holding poses along each direct parking approach;
a completed robot must not occupy the next robot's terminal goal.
`station_approach` is the authoritative fleet map pose. Startup rejects a
different pose or non-map frame in the retained Version 1 `stations.yaml`.

The supplied dock layout uses an east-side waiting pose and robot-specific
`docks.dock_01.exit_poses`. The robot holds its dock lease through fresh localized
arrival at that exit; it does not return across traffic to a captured mission
pose. A robot may reuse its own vacated startup or terminal bay, but never
another robot's bay. `departure_stations: [inspection]` selects the configured
terminal origins whose direct dock approach is checked, along with each
configured below-threshold startup approach. These checks retain the 0.60 m
holding-point clearance and exact segment-to-resource distances.

The demonstrated payload direction is assembly to inspection. The reverse
`inspection_to_assembly` traffic route is infrastructure, not proof of safe
direct docking from assembly parking. Direct docking from other origins is
outside the demonstrated layout. There is no general multi-agent path planner
or reservation of all free-floor travel: idle robots outside exclusive regions
rely on Nav2 obstacle avoidance. Additional physical robots require configured
holding, parking and dock-exit geometry and a new real-motion audit.

Startup also checks every configured initial traffic route from each spawn
through staging and traffic exit against every other idle startup bay, using
the unchanged 0.60 m planned clearance. The supplied west worker bay is
`[-1.1, -3.4, 0]`; the east dock-side bay is `[5.2, -3.4, pi]`. Moving the worker
west prevents the incoming east robot from crossing a queued worker's bay.
The nearest configured initial-route clearance is approximately 0.645 m.
This is a check of those explicit lifecycle segments, not general free-floor
path reservation or a proof of the exact path chosen by Nav2.

## Run and stop

Run dependency setup and `colcon build --symlink-install` first. Use a working
Ubuntu 22.04/ROS 2 Humble rendering display even when omitting GUI windows.

```bash
DISPLAY=:0 scripts/run_fleet_demo.sh [--headless] [output-directory]
DISPLAY=:0 scripts/run_fleet_acceptance.sh --headless --scenario all --timeout 900
```

Acceptance selectors are `nominal`, `contention`, `robot_failure`, `charging`,
`scale` and `all`. Unknown, missing or duplicate options fail before services
start. Each invocation allocates an isolated ROS domain independently of the
caller's domain, fresh local ports, a fresh broker container, a real local PLC,
a new SQLite journal and a unique UTC-stamped artifact directory. Concurrent
invocations share no mission journal or application endpoint.
The fleet and Version 1 scenario runners hold the same cross-process locks,
`/tmp/factory-fleet-domain-<domain>.lock`, on domains in 20–69 and exclude
the inherited domain. They check the host's Linux ephemeral UDP range before
allocation: the full RTPS port envelope must remain below that range. If no
safe unlocked domain is available, startup fails without launching services.
Version 1 cleanup quarantine is respected by both runners. Unknown, malformed
or mismatched lock metadata is not overwritten or silently reclaimed.

For source-bound verification against a fresh CI install, set
`FACTORY_INSTALL_SETUP` to that run's absolute readable `install/setup.bash`
before repository-level pytest, `scripts/run_scenario.sh` or
`scripts/run_all_scenarios.sh`. The nested Version 1 `run_demo.sh` honors that
same explicit setup and skips its default incremental rebuild only in this
mode. With the variable unset, the Version 1 demo still builds and sources the
checkout's `install/setup.bash`. Invalid setup paths fail before
domain reservation or command launch; a path containing spaces is passed as
one quoted argument. Preserve historical result and artifact directories.

The Version 1 demo keeps its root coordinator and root Nav2 stack while its
robot agent publishes robot-local state and cost services. Explicit compatibility
modes target the root mission/planner/navigation/localization endpoints directly;
Humble action-base remaps did not redirect those clients in the measured probe.
Normal fleet mode remains namespaced and cannot discover a root Version 1
server as its robot-local server. The Version 1 demo now uses the validated
configured worker spawn `[-1.1, -3.4, 0]` for both simulation and initial AMCL
localization, preserving the conservative central-aisle recovery margin.
`FACTORY_DASHBOARD_PORT` optionally forwards an integer in 1–65535 and
`FACTORY_JOURNAL_PATH` an absolute journal path; unset Version 1 launch defaults
remain unchanged. Test scenarios use a fresh journal and an owned loopback
ephemeral dashboard port for exact manager-view startup evidence.

Ctrl+C and SIGTERM enter bounded cleanup. Only recorded child process groups
and the exact newly created broker container are stopped. Logs, journals and
receipts remain for inspection. A timeout or missing proof returns nonzero;
the final PASS line requires scenario receipts and clean launch-child exits.
Never use broad process-name kills to clean a fleet run.

## Failure semantics

Heartbeat age of 3 seconds makes a robot UNHEALTHY; 5 seconds makes it OFFLINE.
These bounds use wall monotonic receipt time even when Gazebo is paused. A cost
timeout excludes that candidate for the current round. Automatic work remains
queued when no eligible robot exists; a pinned unavailable robot receives a
bounded explicit rejection.

Automatic requests reserve robots in durable acceptance order: the first
journal event sequence breaks same-tick ties and survives clock changes and
restart. Only one automatic cost round runs at once, so a faster later reply
cannot take the earlier request's cheapest eligible robot. After reservation,
robot missions still execute concurrently. A no-decision head yields during
its bounded backoff, allowing a feasible tail to progress; pinned cost rounds
remain independent. Late replies from a retired round cannot assign a robot.

Loss before confirmed pickup enters REASSIGNING but does not grant another
robot permission to proceed. The former goal must be stopped, a fresh robot
observation must show EMPTY payload, and pose evidence must clear the exact
quarantined resource identities. Offline time alone is not stop or clearance
proof. Confirmed or uncertain pickup enters RECOVERY_REQUIRED and preserves
the recorded carrier; automatic reassignment is prohibited.
Stop proof is a fenced terminal action result, or a fresh post-loss observation
from the same robot reporting ONLINE/AVAILABLE, EMPTY payload and no mission.
Its DDS source timestamp must be after the loss or restart fence. Stale state
and late results from an older mission generation cannot clear quarantine.

Lease expiry removes live authority and retains quarantine. A robot stops at
the next boundary and cannot renew while unhealthy. Manager restart treats old
leases as evidence, waits for fresh observations and reconciles the journal.
An unknown physical position, uncertain station outcome, missing contact or
conflicting payload claim requires explicit recovery. Restart the full
simulation to reset its one-part-per-robot visual lifecycle.

Below 30%, an idle empty robot joins the shared dock queue. It approaches the
staging pose, obtains the exact dock lease and enters. Only independent Gazebo
world-pose contact, current localization and live authority permit simulated
charge. At 80% the robot exits to its configured dedicated bay, proves clearance and
releases the dock. A following robot waits throughout entry, charge and exit.
The charging acceptance starts the dock-side robot at 25% and the worker at 31%; delivery consumes enough
energy to take it below 30%. It then charges to 80% on a distinct later lease.
The audit requires independent contact for both robots and verified clearance
of the former owner before the handoff.
The energy model is deterministic simulation, not a calibrated hardware battery.
The generated fleet world moves the preserved dynamic obstacle from the dock
center to `[5.0, 3.0, 0.25]`. Its collision, visual and inertial geometry remain
unchanged. This is a fleet-layout relocation, not removal of an obstacle.

## Artifact anatomy

Each run contains `summary.json`. Each Gazebo scenario contains:

- `run.json`: isolated domain, ports, dashboard URL and source configuration hash.
- `startup-snapshot.json`, `dispatch-endpoints.json`, `initial-costs.json`: pre-mission eligibility, exact configured robot-local cost/mission endpoint discovery and nominal production cost ordering.
- `launch.log`, `plc.log`, `broker.log`, `broker-runtime.log`: real process and service output.
- `fleet.yaml`, `fleet.world`: exact scenario inputs, including independent parts and visible dock geometry.
- `protocol_events.jsonl`: robot/mission-correlated transport events.
- `navigation-plans.jsonl`: private per-robot plan source stamps, frames, pose counts and final planned poses; action transition events correlate accepted/result UUIDs and generations independently.
- `fleet-snapshots.jsonl`, `world-poses.jsonl`: separate authority and physical samples, batteries and contact.
- `fleet.sqlite3`, `lease-history.json`: durable transitions and exact lease identities.
- `exact-clearances.json`: query-only exact releases, including atomic waiter promotion.
- `mqtt-statuses.json`, `payload-states.json`: observed public results and confirmed attachment/delivery.
- `receipt.json`, `cleanup.json`: asserted behavior, bounds and every owned child exit.

A successful scenario writes its receipt only after its evidence assertions.
A failed scenario retains its available traces, snapshots and cleanup report,
but never fabricates a success receipt. The enclosing `summary.json` records
the failure and the command exits nonzero.

Failure and scale directories retain real DDS journals and their typed receipts.
Generated artifacts are ignored by Git; public media must have separate capture
provenance. Sampled physical occupancy is evidence at the recorded cadence,
not a safety certification or continuous mathematical collision proof.
World samples retain each queried quaternion, derived yaw and actual observer
receipt time. One owned Gazebo service client persists for the scenario;
each physical sample still has a one-second response deadline and no retry.
Missing responses fail acceptance instead of fabricating or repeating poses.
Lease tokens in the private journal are authority credentials. Do not publish
those artifacts or copy live tokens into dashboard events or public reports.
The dashboard's HTTP/SSE evidence contains only short one-way lease fingerprints,
not raw lease IDs. Exact identity checks use the private query-only journal.
An atomic handoff can move directly from one owner to another without an
intermediate owner-less snapshot. Its audit still requires the previous exact
release, a distinct new lease identity and fresh sampled physical clearance.

## Troubleshooting

If startup fails, inspect `launch.log` before retrying. Both robots require nine
ACTIVE lifecycle nodes, namespaced descriptions, odometry, joint states, sensor
topics and isolated dynamic/static TF chains. Full node paths in lifecycle bond
IDs are invalid: Nav2 server bonds use basenames under their namespace.
The acceptance startup gate also requires every configured robot's cost service
and mission action server before it samples nominal costs or submits any
mission. It checks the manager participant's read-only per-robot
`cost_service_ready` and `mission_action_ready` snapshot fields as well as local
observer discovery. Another DDS participant's discovery cannot prove the
manager's view. This discovery uses the existing bounded startup deadline, not
a fixed sleep. A cost service alone does not prove dispatch readiness:
production scheduling correctly excludes a robot whose mission action is
unavailable.

If RGB detection is absent, check the display, rendered camera topic and fresh
ArUco detections. A TCP port opening is not station readiness. Confirm ownership
listener wiring and both PLC unit IDs. Inspect changed cycle counters and
cleanup outcomes; never infer pickup from a request or Nav2 arrival alone.

If charging stalls, inspect independent contact, current world pose, battery,
dock lease and exit clearance. If navigation stalls, inspect real world poses,
local costmaps and resource waiters before increasing any deadline. A physical
station approach must be leased before the final station goal, and release
must follow verified exit. Dashboard failure does not command or stop a robot.
Only charger entry selects the installed docking behavior tree and its
`DockFollowPath` controller and 0.05 m / 0.05 rad goal checker, leaving margin inside the independent
0.15 m / 0.2 rad contact gates. Missions, dock staging and dock exit retain
`GeneralFollowPath` controller and general 0.15 m / 0.15 rad checker. Humble
DWB's rotation critic uses its controller's fixed XY tolerance, not the chosen
goal checker; a shared precise controller would also slow general staging
goals. Both default navigation trees explicitly select the matching controller
and checker; configured custom trees remain explicit overrides.
Custom trees must name a configured controller and its matching checker;
the runner does not infer an ambiguous default when two plugins are installed.
Contact absence still stops the session and retains
uncertain dock occupancy; neither localization success nor a service ACK proves
physical contact. Do not enlarge contact or lease TTL to conceal a failed entry.
An acknowledged AMCL no-motion update can consume a buffered scan older than
the refresh epoch. The coordinator may reissue that valid-but-stale candidate
at a 0.1-second cadence, at most ten attempts, inside the original 1.0-second
steady deadline. It never resets that deadline, source floor or epoch, and no
stale pose can authorize motion or resource clearance. Inspect
`localization_refresh_requested` dispatch elapsed times and
`localization_refresh_pose` source/epoch/receipt evidence, not service receipt
spacing, when diagnosing this fence.
Both the service acknowledgement and qualifying pose must arrive within that
original deadline. Timely complete evidence remains usable if executor pressure
delays its timer consumption; a late acknowledgement or late pose cannot
complete the proof. A fresh off-target pose still fails the independent station
geometry guard and waits for its normal station-confirmation timeout. Refresh
does not turn geometric disagreement into successful arrival.
The dashboard's mission mode is not independent stop evidence: a coordinator
can retain its last resource-wait mode while navigating to a verified exit.
Use current simulator poses, robot-local action results and exact resource
clearance evidence when deciding whether a stopped or departing robot is safe.

Run `scripts/run_ci_checks.sh` for interface, pure, DDS, protocol, restart and
real-browser checks. Run the explicit 900-second acceptance command above for
full Gazebo evidence; hosted non-Gazebo results do not imply rendered proof.
CI prints its unique `artifacts/ci/<run-id>/` root. It retains that run's fresh
build, install, logs and `test-results`; every result command uses the exact
printed root. Inspect it with `colcon test-result --verbose --test-result-base
<printed-root>/test-results`. Do not infer current status from a bare recursive
scan of an old `build/` tree containing historical RED XML. The runner never
deletes previous runs. Artifacts remain ignored for local inspection; archive
or remove only an explicitly identified run after preserving its evidence.
For full local coverage, run unrestricted colcon tests and inspect their exact
fresh result root, then run `.venv/bin/python3 -m pytest -q tests` for all
repository-level suites. Do not merge every package's `test` directory into one
pytest import namespace: ROS launch_testing imports basenames, and package-local
`test_node` modules and helpers would collide. Package-isolated colcon runs
preserve all assertions and plugins; they are not a test exclusion.
