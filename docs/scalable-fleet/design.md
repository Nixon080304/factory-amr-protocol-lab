# Scalable Multi-Robot Fleet

## What and why

Version 1 demonstrates one simulated AMR carrying one motor from assembly to
inspection through MQTT, ROS 2 DDS, Nav2, perception, and Modbus TCP. Version 2
will preserve those protocol boundaries while adding centralized fleet
coordination, two visible AMRs, shared traffic resources, energy-aware mission
assignment, automatic charging, restart recovery, and a read-only operator
dashboard.

The first demonstration will run two complete robot stacks in one Gazebo world.
The fleet interfaces, scheduler, registry, and resource manager will be tested
with ten robots. Adding a robot must require configuration, not copied business
logic or robot-specific branches.

## Requirements

- Launch two AMRs from the same parameterized robot stack under distinct ROS 2
  namespaces.
- Represent up to ten registered robots without changing scheduler or robot
  agent source code.
- Keep one central fleet manager responsible for assignments, mission state,
  resource leases, and energy policy.
- Keep navigation, localization, obstacle response, and stopping local to each
  robot stack.
- Select the healthy, available robot with enough predicted battery reserve and
  the lowest path cost to the pickup station. Break equal costs by robot ID.
- Prevent simultaneous ownership of capacity-one traffic zones, stations, and
  charging docks.
- Use one shared charging dock in the two-robot demonstration. Make robots,
  docks, zones, and stations configuration-driven collections.
- Begin automatic docking when an idle robot falls below 30% battery. Refuse a
  mission predicted to finish below 20%. Charge to 80% before releasing the
  dock.
- Reassign a mission after a robot failure only when pickup has not been
  confirmed. Preserve payload ownership and enter `RECOVERY_REQUIRED` after
  pickup.
- Persist mission identity, transitions, assignment, payload ownership, and
  completed results in SQLite. Reconcile persistent state with fresh robot
  heartbeats after fleet-manager restart.
- Preserve the external MQTT, internal ROS 2 DDS, and station Modbus TCP
  boundaries.
- Provide a read-only dashboard for robots, missions, batteries, resource
  leases, faults, and history.
- Preserve the Version 1 single-robot request as a compatible explicitly
  assigned mission.
- Bind the dashboard and protocol development services to loopback by default.
- Make no physical-safety or production-readiness claim.

## Acceptance criteria

- Two AMRs launch in one Gazebo world from one reusable launch description and
  one robot implementation.
- Each running robot has an isolated namespace, Nav2 stack, localization,
  transforms, command velocity, sensor topics, battery state, and mission
  action server.
- Adding `amr_03` through `amr_10` to a test configuration requires no source
  change and registers ten distinct robot agents.
- An automatically assigned mission selects the lowest-cost eligible robot.
- A requested robot ID pins the mission to that robot or produces a bounded,
  explicit rejection when it is unknown or ineligible.
- Equal-cost assignment is deterministic by robot ID.
- A robot whose predicted final charge is below 20% is not eligible.
- No two robots hold the same capacity-one resource lease at the same time.
- The two-robot simulation never records simultaneous entry into an exclusive
  traffic-zone polygon.
- Assembly and inspection transfers retain the existing Modbus completion rule:
  `transfer_complete` is true and `cycle_counter` changed.
- The simulated payload is attached only after confirmed loading and detached
  only after confirmed unloading.
- An idle robot below 30% reserves the shared dock, navigates through its
  staging pose, charges to 80%, releases the lease, and becomes available.
- A second low-battery robot waits outside the occupied dock resource.
- Loss of a robot before confirmed pickup releases safe leases and returns its
  mission to the assignment queue.
- Loss of a robot after confirmed pickup retains payload ownership and produces
  `RECOVERY_REQUIRED` without automatic reassignment.
- Restart reloads queued and completed missions, rejects duplicate conflicting
  mission IDs, never reruns a completed mission, and safely reconciles active
  work after fresh heartbeats.
- MQTT status identifies the assigned robot after assignment.
- The dashboard updates live and offers no endpoint that commands a robot, PLC,
  fault injector, or resource owner.
- All Version 1 protocol, navigation, perception, payload, observer, and fault
  tests remain green or are migrated with equivalent coverage.
- Hosted checks cover pure fleet logic, ROS interface contracts, two namespaced
  robot agents with bounded navigation drivers, protocol integration, and the
  dashboard in a real browser.
- A local display-backed system test proves two real Nav2 stacks, Gazebo motion,
  exclusive-zone coordination, pickup, visible carrying, drop-off, and charging.

## Design

### Selected architecture

The design uses a centralized fleet manager with autonomous robot-local
execution. This matches the ownership split used by common fleet systems:
fleet-wide optimization is central, while immediate motion and stopping remain
on each robot.

```text
Mission client
    │ MQTT/TCP/IP
    ▼
MQTT gateway
    │ ExecuteFleetMission action through ROS 2 DDS
    ▼
Fleet manager
    ├── robot registry and health
    ├── deterministic dispatcher
    ├── resource lease manager
    ├── energy policy
    └── SQLite mission journal
          │ ExecuteFactoryMission action through ROS 2 DDS
          ├──────────────────────┐
          ▼                      ▼
      /amr_01                /amr_02
      robot agent            robot agent
      Nav2 + AMCL            Nav2 + AMCL
      perception             perception
      battery                battery
          │                      │
          └──── resource leases ─┘
                     │
                     ├── traffic zones
                     ├── PLC stations
                     └── charging dock
```

The fleet manager coordinates but is not a safety controller. A resource grant
allows a robot agent to proceed to a configured boundary; the robot's local
Nav2 stack, sensor processing, and stop behavior remain authoritative for
motion.

### Alternatives rejected

A fully decentralized auction among robot agents avoids a central scheduler,
but it adds distributed consensus, lease conflict, and recovery problems that
do not improve the intended ten-robot demonstration. One ROS domain per robot
provides stronger network isolation, but bridging ten domains adds operational
complexity without improving one-world simulation. Both remain possible later
because robot agents communicate through explicit interfaces.

### Fleet manager

The new `fleet_manager` package will separate pure policy from ROS adapters:

- `RobotRegistry` owns the latest state, heartbeat deadline, and eligibility of
  each configured robot.
- `Dispatcher` filters eligible robots, requests bounded cost estimates, and
  selects `(path_cost, robot_id)` minimum order.
- `ResourceManager` owns capacity, queueing, lease IDs, renewal, expiration,
  and release for traffic zones, stations, and docks.
- `EnergyPolicy` checks the predicted mission energy, automatic-docking
  threshold, reserve, and charge target.
- `MissionJournal` is the single SQLite writer and records immutable
  transitions plus current snapshots.
- `FleetManagerNode` exposes ROS actions and services, connects the pure
  components, and publishes fleet events.

The fleet manager accepts multiple queued missions but assigns at most one
active mission to a robot. Mission IDs are globally unique. A repeated ID with
an identical payload replays the recorded state; a different payload is a
conflict.

### Reusable robot agent

The existing mission coordinator will become a reusable namespaced robot agent.
Every instance uses parameters for robot ID, namespace, station poses, marker
IDs, topic names, and action names. It keeps the established mission state
machine and executes one assigned mission:

```text
RECEIVED
NAVIGATING_TO_PICKUP
VERIFYING_PICKUP
LOADING
NAVIGATING_TO_DROPOFF
VERIFYING_DROPOFF
UNLOADING
COMPLETED or FAILED
```

Before entering a configured exclusive zone or station, the agent obtains a
lease. The route configuration provides staging waypoints outside resource
polygons. The first version permits only one exclusive zone lease per robot at
a time, preventing hold-and-wait cycles between zones. The agent renews its
current lease while healthy and releases it after its verified exit.

Each robot owns a namespaced Nav2 stack and queries its own `ComputePathToPose`
service. A fleet cost-estimate request returns pickup path length, full mission
energy estimate, and feasibility. A bounded timeout makes that robot ineligible
for the current assignment round rather than blocking the fleet.

### Station transfers

The shared Modbus gateway remains the sole owner of raw PLC addresses. A robot
agent with a station lease sends a typed transfer request containing robot ID,
mission ID, station ID, and part. The gateway keeps per-station serialization
as a second defense against concurrent transfers.

Successful loading or unloading still requires both a true
`transfer_complete` coil and a changed `cycle_counter`. Protocol events include
the robot ID so payload state, observer traces, dashboard history, and fault
injection remain correlated in a multi-robot run.

### Traffic resources

`fleet.yaml` defines exclusive polygons, staging poses, and route segments.
Robot agents navigate to a staging pose, acquire the next resource lease, then
enter. Pose observations and lease events are recorded independently so tests
can prove that no two robot footprints entered a capacity-one polygon at the
same time.

Lease IDs are unguessable process-generated values. Each lease records the
resource ID, robot ID, mission ID, grant time, expiration time, and renewal
time. A robot heartbeat does not renew a lease implicitly; the holder renews
the lease explicitly. This distinguishes a healthy but stalled agent from an
agent still authorized to enter a resource.

### Battery and charging

Each robot publishes `sensor_msgs/BatteryState`. The simulation drains charge
from measured odometry distance plus a small configurable idle rate. It charges
only while the robot is inside the dock tolerance, owns the dock lease, and has
confirmed contact.

An idle robot below 30% requests docking. It reserves the dock before leaving
its staging location, navigates to the configured dock staging pose, then moves
to the charging pose. Other robots wait at their configured safe staging poses.
At 80%, the robot releases the dock and becomes available.

The assignment estimate includes travel to pickup, pickup to drop-off, fixed
transfer energy, and a conservative route-to-dock allowance. A predicted final
charge below 20% makes the robot ineligible. All rates and thresholds are
configuration values with the approved defaults.

### Persistence and restart

SQLite uses write-ahead logging and one fleet-manager writer. Tables store
mission identity and payload hash, current mission snapshot, immutable mission
transitions, assignments, payload ownership, robot observations, and resource
lease history. Active leases loaded after restart are evidence, not immediate
authorization.

After restart, the fleet manager remains in `RECONCILING` until each configured
robot supplies a fresh state or reaches the offline deadline. Queued missions
return to scheduling. Completed missions remain terminal. A pre-pickup active
mission may return to the queue only after its former robot is unavailable and
safe resource ownership is reconciled. Any mission with confirmed or uncertain
payload pickup becomes `RECOVERY_REQUIRED`.

Robot agents treat local lease expiry as loss of permission to enter the next
resource and stop before its boundary. The restart protocol does not claim safe
automatic recovery from arbitrary physical positions.

### Dashboard

A `fleet_dashboard` ROS 2 node consumes fleet state and protocol events. A
loopback-only HTTP server provides a snapshot endpoint, a server-sent event
stream, and static HTML/CSS/JavaScript. Only `GET` endpoints exist in Version 2.
The browser displays:

- robot pose, mode, mission, payload, health, and battery;
- queued, active, completed, failed, and recovery-required missions;
- traffic-zone, station, and dock owners and waiters;
- charging progress, faults, and recent protocol events; and
- persistent mission history after restart.

The dashboard is an observer. It never publishes ROS commands or writes fleet
state.

### Configuration-driven scaling

`fleet.yaml` is the scaling boundary:

```yaml
robots:
  - id: amr_01
    namespace: /amr_01
    initial_pose: [-3.0, -1.0, 0.0]
  - id: amr_02
    namespace: /amr_02
    initial_pose: [-3.0, 1.0, 0.0]

docks:
  - id: dock_01
    staging_pose: [0.0, -2.0, 0.0]
    charging_pose: [1.0, -2.0, 0.0]

traffic_zones:
  - id: central_aisle
    capacity: 1
    staging_poses:
      east: [1.5, 0.0, 3.14159]
      west: [-1.5, 0.0, 0.0]
```

The launcher iterates over `robots` and creates the same model, localization,
Nav2, perception, battery, and agent descriptions with namespace substitutions.
Fleet code discovers no robot through hard-coded identifiers. Configuration
validation rejects duplicate IDs, duplicate namespaces, missing frames,
overlapping initial poses, unknown resource references, invalid thresholds,
and routes without safe staging poses before launching any robot.

## Interfaces and data

### MQTT compatibility

The request topic remains `factory/missions/request` with QoS 1. The Version 2
request keeps the existing fields. `robot_id` becomes optional:

- omitted means automatic fleet assignment;
- a configured ID requests that specific robot; and
- the existing `"robot_id": "amr_01"` request remains valid.

Dynamic registry validation replaces the Version 1 single-value JSON-schema
enumeration. All other strict validation and duplicate-ID behavior remain.
Status continues on `factory/missions/<mission_id>/status`. Before assignment,
automatic missions omit `robot_id`; after assignment, every status includes the
assigned ID. Explicit Version 1 requests therefore retain their existing status
shape. Fleet states add `QUEUED`, `ASSIGNING`, `REASSIGNING`, and
`RECOVERY_REQUIRED` without changing the robot agent's internal states.

### ROS 2 interfaces

New or extended typed interfaces will include:

- `ExecuteFleetMission.action`: mission fields and optional requested robot;
  feedback includes fleet state, assigned robot, detail, and progress; result
  includes success, terminal state, assigned robot, error code, and message.
- Existing `ExecuteFactoryMission.action`: one explicit robot mission, hosted at
  `/<robot_id>/execute_factory_mission`.
- `RobotState.msg`: robot ID, mode, pose, battery percentage, payload state,
  mission ID, health detail, and timestamp.
- `EstimateMissionCost.srv`: pickup, drop-off, and part; response contains
  feasible, path cost, predicted final battery, and rejection reason.
- `AcquireResource.srv`: robot, mission, and resource IDs; response contains
  granted, lease ID, expiration, current owner, and reason.
- `RenewResource.srv` and `ReleaseResource.srv`: exact lease identity with an
  idempotent response.
- Existing `TransferPart.srv`: add robot ID for station ownership and tracing.
- Existing `ProtocolEvent.msg`: add robot ID for fleet-wide correlation.

Resource services are central fleet interfaces. Navigation, perception, and
battery topics remain namespaced per robot. Fleet summaries and protocol events
use shared topics with robot ID fields.

### Mission and robot states

Fleet mission states are:

```text
QUEUED
ASSIGNING
ASSIGNED
EXECUTING
REASSIGNING
COMPLETED
FAILED
RECOVERY_REQUIRED
```

Robot modes are:

```text
AVAILABLE
RESERVED
EXECUTING
WAITING_FOR_RESOURCE
DOCKING
CHARGING
UNHEALTHY
OFFLINE
RECOVERY_REQUIRED
```

Every transition has one owner and is written transactionally with the updated
snapshot. Unsupported transitions fail without mutating the snapshot.

## Failure behavior

- Robots publish heartbeats at 2 Hz. Missing heartbeats for 3 seconds mark a
  robot `UNHEALTHY`; 5 seconds mark it `OFFLINE`.
- Unhealthy or offline robots receive no assignments and cannot renew leases.
- Cost-estimate timeout excludes only that candidate from the current round.
- With no eligible robot, the mission remains queued and reports the bounded
  reason instead of failing or busy-looping.
- Resource requests queue fairly by request time and robot ID. Cancellation or
  mission termination removes the waiter.
- An expired lease cannot authorize entry. The resource remains unavailable
  until pose reconciliation proves the former holder is outside or an operator
  resolves uncertainty.
- PLC station fault, stale completion, timeout, and unknown transfer outcomes
  retain their Version 1 behavior and never fabricate payload movement.
- Before confirmed pickup, robot failure releases proven-safe resources and
  requeues the mission.
- After confirmed or uncertain pickup, robot failure preserves payload
  ownership and produces `RECOVERY_REQUIRED`.
- Dock contact loss stops charging and keeps the robot unavailable until pose
  and lease state agree.
- SQLite write failure stops new assignment and resource grants. Active robot
  agents finish only actions that remain locally safe, then report failure.
- Dashboard failure has no effect on fleet operation.
- Fleet-manager restart requires reconciliation before new work or lease grants.

## Test approach

### Pure tests

- Table-driven scheduler cases cover health, mode, requested robot, path cost,
  energy reserve, timeouts, and deterministic ties.
- A ten-robot fixture proves registry and assignment behavior without robot-ID
  branches.
- Resource-manager tests prove mutual exclusion, fair queueing, renewal,
  expiration, idempotent release, cancellation, and restart reconciliation.
- Energy tests cover distance drain, idle drain, mission prediction, 20% reserve,
  30% docking threshold, 80% target, contact loss, and dock contention.
- Mission-state tests cover every valid and invalid transition, duplicate IDs,
  payload ownership, pre-pickup reassignment, and post-pickup recovery.
- SQLite tests use real temporary databases and process restart boundaries.

### ROS and protocol integration

- Interface contract tests assert complete message and service fields.
- Two namespaced robot agents run against bounded navigation and perception
  drivers in hosted checks.
- Real DDS tests prove robot state isolation, action routing, deadlines, and
  resource lease loss.
- Real Mosquitto tests prove automatic and pinned requests, status robot IDs,
  duplicate replay, queueing, reconnect, and malformed input rejection.
- Real Modbus tests prove per-station serialization, robot correlation, stale
  completion rejection, cleanup, and payload ownership.
- Dashboard tests use a real browser at desktop and mobile widths and assert
  live updates, restart history, no horizontal overflow, accessibility, and the
  absence of command endpoints.

### Simulation and fault scenarios

- A display-backed two-robot Gazebo test records both poses and resource leases,
  proving exclusive-zone entry, independent Nav2 operation, station transfer,
  visible payload carrying, and final delivery.
- A charging scenario proves shared-dock staging, contact, charge progression,
  release, and the waiting robot's eventual admission.
- Fault scenarios cover robot loss before and after pickup, lease expiry,
  blocked traffic zone, dock contact loss, PLC fault, MQTT disconnect, and
  fleet-manager restart.
- Ten full Gazebo/Nav2 stacks are not a required laptop test. Ten-robot scale is
  proved at registry, scheduling, lease, DDS agent, and bounded-driver levels;
  the resource-intensive visual demonstration uses two complete stacks.

## Risks

- **Two Nav2 stacks may exceed laptop resources.** Use composed launch where
  stable, reduce sensor rates, keep meshes bounded, and measure CPU and memory
  before adding visual detail.
- **Namespace leaks can couple robots.** Contract tests will reject global robot
  topics, duplicate frames, and un-namespaced actions.
- **Central manager failure interrupts coordination.** SQLite journaling,
  expiring leases, local stop boundaries, and explicit reconciliation limit the
  failure; high availability is outside this version.
- **Static traffic zones may be conservative.** Begin with explicit bottlenecks
  and staging poses. Do not claim general multi-agent path finding.
- **Battery simulation may look more precise than it is.** Label it as a
  deterministic energy model and publish its configured rates.
- **A read-only dashboard can drift from authoritative state.** Render sequence
  numbers and reconnect from a fresh snapshot before applying later events.
- **Multi-robot interface changes can weaken Version 1 guarantees.** Keep
  compatibility fixtures and migrate each protocol contract with tests before
  changing implementation.

## Out of scope

- Physical robots, hardware charging, and safety certification
- More than ten registered robots
- Multiple factories, ROS domains, or network sites
- Fully decentralized task allocation or consensus
- General multi-agent path finding
- Robotic-arm grasping or manipulation
- Production authentication, authorization, and internet exposure
- VDA 5050 compatibility
- Browser-issued robot, PLC, resource, or fault commands
- Fleet-manager high availability
