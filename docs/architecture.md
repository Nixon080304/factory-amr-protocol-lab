# Architecture and successful mission

The fleet adds one central owner for assignment, resource authority and durable
mission state. Robot-local navigation and immediate stopping remain local.
The dashboard is a loopback GET/SSE observer; it has no command endpoint.

```mermaid
flowchart LR
    Client[Mission client] -->|MQTT QoS 1| Gateway[MQTT gateway]
    Gateway -->|ExecuteFleetMission / DDS| Fleet[Fleet manager]
    Fleet --> Journal[(SQLite WAL journal)]
    Fleet -->|ExecuteFactoryMission / DDS| Robot1[Namespaced robot stack 1]
    Fleet -->|ExecuteFactoryMission / DDS| Robot2[Namespaced robot stack 2]
    Robot1 & Robot2 -->|Exact lease services / DDS| Fleet
    Robot1 & Robot2 -->|TransferPart / DDS| Modbus[Modbus gateway]
    Modbus -->|Modbus TCP| Stations[Assembly and inspection PLC]
    Fleet -->|Read-only snapshots / SSE| Dashboard[Local dashboard]
```

Each namespaced stack has its own prefixed map/odom/base/sensor frame tree,
Nav2 and AMCL nodes, camera perception, cost endpoint, energy agent and local
mission action. The manager selects `(path_cost, robot_id)` among healthy,
available candidates predicted to retain at least 20% charge. Idle robots below
30% enter the dock queue; charging requires independently simulated contact,
fresh localization and an exact live dock lease, and stops at 80%.
The dock lease remains held through arrival at a configured robot-specific
exit bay. The supplied two-robot layout validates low-battery startup and
inspection-parking dock approaches; reverse-route docking and general
free-floor multi-agent path planning are not demonstrated guarantees.

Automatic assignment rounds run in first-journal-event acceptance order, so
later cost replies cannot reserve an earlier request's cheapest eligible robot.
Only cost evaluation is serialized; assigned robots execute concurrently.
Bounded no-decision backoff lets feasible later work progress, while pinned
rounds remain independent.

Capacity-one leases cover the central aisle, complete station approaches and
dock. A station lease precedes final approach and remains until verified exit;
a successful transfer alone is not clearance. Expiry quarantines authority
until fresh world-frame observations prove clearance. Before-pickup loss blocks
reassignment until the former goal is stopped, payload is EMPTY, and exact
quarantine is cleared. After-pickup loss retains the carrier and becomes
`RECOVERY_REQUIRED`. Restart loads durable evidence and reconciles fresh robot
observations before granting authority. See [fleet operations](fleet-operations.md).

## Original single-robot architecture

The diagram and measurements below describe the retained Version 1 command.
The fleet action and namespaces above replace its direct northbound action
routing in the new demonstration. Historical hosted results do not certify a
new fleet revision.

The lab separates mission decisions from communication transports and Gazebo
APIs. All robot nodes share simulation time. The committed map and station
poses share world coordinates.

![Implemented architecture](assets/architecture.svg)

```mermaid
flowchart LR
    Dispatcher[Mission publisher] -->|MQTT QoS 1| Broker[Mosquitto]
    Broker <--> MQTT[mqtt_gateway]
    MQTT -->|ExecuteFactoryMission action| Coordinator[mission_coordinator]
    Coordinator -->|NavigateToPose| Nav[AMCL and Nav2]
    Nav -->|sole smoothed cmd_vel| Simulation[Gazebo robot and factory]
    Simulation -->|scan, odom, TF, clock| Nav
    Simulation -->|RGB images| Perception[station_perception]
    Perception -->|station identity| Coordinator
    Nav -->|amcl_pose| Coordinator
    Coordinator -->|TransferPart service| Modbus[modbus_gateway]
    Modbus <-->|Modbus TCP| PLC[PLC simulator]
    Modbus -->|successful transfer events| Payload[payload_simulator]
    Payload -->|entity animation| Simulation
    MQTT & Coordinator & Modbus -->|ProtocolEvent| Observer[protocol_observer]
    Observer --> Artifacts[JSONL trace and Markdown report]
    Nav & Perception & Coordinator --> RViz[RViz visualization]
```

Only the Gazebo drive plugin publishes wheel odometry and `odom` to
`base_footprint`. Robot state publication owns the remaining robot TF tree.
AMCL owns `map` to `odom`. Nav2 controller and recovery behaviors feed the
velocity smoother; only its output publishes `/cmd_vel`. Gazebo entity service
access stays in simulation packages.

| Package | Responsibility |
| --- | --- |
| `factory_interfaces` | Typed mission action, transfer service, station detection, protocol events |
| `mission_coordinator` | C++ state machine, current-goal navigation, localization and camera gates, transfer sequencing |
| `mqtt_gateway` | JSON validation, mission registry, action bridge, status and telemetry buffering |
| `modbus_gateway` | Unit mapping, PLC handshake retries and cleanup, exact transfer events |
| `plc_simulator` | Standalone deterministic TCP PLC: assembly unit 1 and inspection unit 2 |
| `station_perception` | OpenCV ArUco detection from RGB images, preserving source timestamps |
| `payload_simulator` | Deduplicated transfer state, conveyor and part animation |
| `protocol_observer` | Passive JSONL trace and terminal duration report |
| `factory_simulation` | World, Burger-compatible drive, LiDAR, RGB camera, IMU, odometry, entity probes |
| `factory_bringup` | Map, station configuration, AMCL/Nav2, full composition, RViz visualization |
| `fault_injector` | Explicit deterministic controls, bounded owner acknowledgements and reset |

## Mission sequence

![Protocol and transfer sequence](assets/protocol-sequence.svg)

```mermaid
sequenceDiagram
    participant D as Dispatcher
    participant M as MQTT gateway
    participant C as Coordinator
    participant N as AMCL/Nav2
    participant V as Camera perception
    participant G as Modbus gateway
    participant P as PLC
    participant B as Payload simulator
    participant O as Observer
    D->>M: Standard M-001 JSON, QoS 1
    M->>C: ExecuteFactoryMission
    C-->>M: Goal accepted
    M-->>D: RECEIVED
    C->>N: Assembly [-3, 0.8, pi/2]
    N-->>C: Current goal succeeds; current-leg AMCL agrees
    V-->>C: Five fresh marker 10 images within 1.5 seconds
    C->>G: Load motor at assembly
    G->>P: Part code, robot_present, transfer_request
    P-->>G: Completion flag and changed counter
    G-->>B: One successful LOADING cycle event
    B-->>B: IN_TRANSIT; animate assembly conveyor
    G-->>C: Safe handshake result
    C->>N: Inspection [3, 0.8, pi/2]
    N-->>C: Current goal succeeds; current-leg AMCL agrees
    V-->>C: Five fresh marker 20 images within 1.5 seconds
    C->>G: Unload motor at inspection
    G->>P: Part code, robot_present, transfer_request
    P-->>G: Completion flag and changed counter
    G-->>B: One successful UNLOADING cycle event
    B-->>B: AT_INSPECTION; animate inspection conveyor
    G-->>C: Safe handshake result
    C-->>M: COMPLETED
    M-->>D: COMPLETED, error_code null
    C-->>O: Terminal event
    O-->>O: Write report; refresh for late correlated events
    D->>M: Identical M-001 again
    M-->>D: Replay COMPLETED without another action
```

Each verification leg resets its confirmation window. Marker 10 identifies
assembly; marker 20 identifies inspection. Transfer requires current goal
success, a finite current-leg map-frame AMCL pose within 0.25 m and 0.25 rad,
and five increasing correct image timestamps within 1.5 seconds. Stationary
AMCL updates can be sparse; camera evidence still expires against simulation
time. Only the Modbus adapter owns transfer phase pairs. Successful finish
details carry station, `LOADING` or `UNLOADING`, and the actual uint16 counter.
Payload deduplication uses mission, transfer kind, and counter. Observer sequence
numbers record local arrival; source timestamps use simulation time. Reports
refresh for late correlated events because cross-publisher DDS delivery can
follow service completion.

Sensor observations may arrive before their corresponding `/clock` update.
The coordinator retains one current-leg AMCL candidate and at most five pending
camera observations. It processes camera observations only after their source
timestamps are current, then applies the same identity, order, freshness, and
window checks. Invalid observations clear partial confirmation; each leg and
terminal reset clears pending evidence. No future evidence permits a transfer.
The action result owns the terminal MQTT status, including its final error code;
terminal feedback does not publish an additional completion.

Gazebo publishes camera images, camera calibration, and LiDAR with sensor-data
QoS: best-effort, volatile, keep-last depth 5. Independent best-effort subscribers
can receive different subsets. Acceptance requires five fresh correct detector
source timestamps within the perception phase and independently decodes at least
one actual rendered frame with an exact timestamp in that five-image window.
It records all detector timestamps separately from received rendered frames.
The coordinator still requires all five distinct source observations; the
independent probe does not assert that every subscriber receives every image.

## Running and observing

Run `scripts/setup_dev.sh` once, then `DISPLAY=:0 scripts/run_demo.sh`.
Publish with `scripts/send_demo_mission.sh` in a second terminal. The standard
mission is `M-001`, `amr_01`, assembly pickup, inspection dropoff, motor.
See the [README](../README.md) for setup, port/domain overrides, cleanup,
headless operation, expected states, and verification commands.

RViz uses fixed frame `map`. It displays `/map`, `/amcl_pose`, `/scan`, both
costmaps, `/plan`, `/factory/navigation_goal`, `/factory/station_markers`, and
the robot. The visualization adapter derives goals from coordinator navigation
events and committed station poses. Camera identity labels sit at configured
station locations and expire after 1.5 seconds. These labels show observed
identity, not measured 3D marker position. Initial-pose and goal tools are available.

On success the robot ends within 0.25 m and 0.25 rad of inspection. Each PLC
counter increments once, robot-owned presence/request coils clear, and the
part rests at world `(3, 2, 0.65)`. MQTT emits `COMPLETED` once for the original
request; duplicates replay that status. The printed output directory contains
`protocol_events.jsonl` and, for `M-001`,
`mission_02acf5bdbbaa226dc29a07a86e64c1b2d630385779533ed70d193a8c258149dc.md`.

## Current limitations

Version 1 supports one robot, one motor part, and one fixed route. Resetting the
single-part lifecycle requires restarting the simulation. Registries and active
aggregation are process-local. Deterministic reliability
scenarios and local CI gates are implemented; restart recovery is unsupported.
The hosted CI is observed passing, but it excludes local-only Gazebo rendering
and navigation. See [validation results](validation/latest-results.md).

Confirmed transfers drive bounded, best-effort visuals. The part travels 0.2 m
along each station belt in four intermediate poses. Loading then places the part
at local pose `(-0.03, 0, 0.28)` in the `factory_amr` reference frame. A nominal
150 ms refresh while simulation time advances keeps the visual pose attached
during transport.
Unloading stops that refresh and places the part at `(3, 2, 0.65)`. Its kinematic
link has gravity disabled so commanded poses stay stable and Gazebo publishes
motion. This is an animation,
not a physical grasp or conveyor dynamics model. Logical payload state and PLC
cycle counters remain authoritative even if the visual service fails.

The first successful pickup commits the process-local lifecycle, including a
pickup that finishes during cancellation. Completion, later failure, and later
cancellation do not release it. Another valid idle request acknowledges action
transport and immediately aborts with `FAILED/RESTART_REQUIRED` and a
`mission_rejected` event, without navigation, transfer, or `mission_started`.
Action acceptance here acknowledges transport, not physical execution. Restart
the full simulation, not only the coordinator, before another payload mission.
Failures before successful pickup permit a new request. Identical MQTT mission
IDs still replay their existing status without another action.

A vanished or hung gateway after transfer dispatch can leave the mission
pending with the robot stopped and PLC state unknown. Cancellation during
transfer requires an explicit safe handshake outcome. No bound on every
mission or crash safety is claimed. The system test has a 300-second outer
deadline and stops only its owned harness. Payload visual failures are best
effort and do not undo confirmed logical transfers.

Humble clients can omit initial action feedback before registering the accepted
goal. Complete reliable mission-state and protocol-event streams are authoritative.
Headless RGB still requires a working display. Localhost MQTT and Modbus have
no production authentication or production security model.

The installed Humble RViz renderer can log a first-map GLSL sampler diagnostic
(`active samplers with a different type refer to the same texture image unit`),
also reported in [upstream RViz issue 463](https://github.com/ros2/rviz/issues/463).
The actual RViz view on the verified display shows the map, localization,
scan, both costmaps, plan, goals, and live station identity labels despite this
diagnostic. Rendering on every graphics driver is not guaranteed.
