# Scalable Multi-Robot Fleet Implementation Plan

> **For agentic workers:** Use either superpowers:subagent-driven-development or superpowers:executing-plans to execute this plan task by task. Do not batch tasks, skip RED/GREEN evidence, or merge before the final review gate.

**Goal:** Extend the existing single-AMR protocol lab into a two-robot Gazebo fleet demo whose configuration, scheduling, resource ownership, energy policy, persistence, and dashboard scale to ten robots without copied business logic.

**Architecture:** A global Python fleet manager owns mission assignment, robot health, shared-resource leases, charging decisions, and an SQLite mission journal. Each namespaced robot keeps local navigation, localization, mission execution, perception, safety, battery simulation, and cost estimation. MQTT remains the northbound mission boundary, ROS 2/DDS remains the internal robot boundary, and Modbus TCP remains the station boundary. A read-only loopback HTTP/SSE dashboard observes the same authoritative state without becoming a control path.

**Tech stack:** ROS 2 Humble, Python 3, C++17, rclpy/rclcpp, Nav2, Gazebo Classic, MQTT, Modbus TCP, SQLite, pytest, GoogleTest, launch_testing, Bash, HTML/CSS/JavaScript, Chrome DevTools Protocol.

**Specification:** [docs/scalable-fleet/design.md](../../scalable-fleet/design.md)

## Global constraints

- Preserve the existing explicit amr_01 mission as a valid V1-compatible request.
- An omitted robot_id means automatic assignment. A configured robot_id means a pinned assignment.
- Adding a robot must require only one fleet.yaml entry and its spawn pose; no robot-ID branches or copied launch blocks.
- Use ROS namespaces for robot topics, actions, and services and unique frame prefixes for every robot.
- Use monotonic process time for heartbeat aging and lease expiry. ROS time is telemetry only.
- Only the fleet manager writes the SQLite journal. Use WAL mode and parameterized SQL.
- Resource capacity is one for every V1 station, traffic zone, and dock.
- A robot that fails before confirmed pickup may be replaced. A failure after pickup or with uncertain payload ownership must enter RECOVERY_REQUIRED.
- Battery eligibility requires a predicted post-mission reserve of at least 20%. Automatic charging begins below 30% and ends at 80%.
- The dashboard binds to 127.0.0.1 by default, exposes GET endpoints only, and never publishes commands.
- Full Gazebo/Nav2 proof is required for two robots. Ten-robot proof is configuration, pure-core, agent-contract, and bounded-driver coverage.
- Keep dependencies minimal. Use Python’s sqlite3 and HTTP server libraries; do not add a web framework.
- Preserve the handwritten, direct style of the existing repository. Avoid generated-looking abstraction layers and decorative code.

## Review focus

Reviewers must pay special attention to:

1. Duplicate robot IDs, namespaces, TF prefixes, or spawn poses in fleet.yaml.
2. Stale or out-of-order heartbeats, including behavior while simulation time is paused.
3. Process restart at every mission boundary, especially after pickup and during a resource lease.
4. Concurrent acquire, renew, release, expiry, cancellation, and offline-owner cleanup for shared resources.
5. Malformed MQTT input, broker reconnect, duplicate mission IDs, slow SSE clients, and browser reconnect.

## Planned file ownership

- **factory_interfaces:** public ROS actions, messages, and services only.
- **fleet_manager:** global configuration, registry, dispatcher, leases, journal, fleet state machine, ROS node, and read-only web snapshot.
- **robot_agent:** per-robot health, energy model, cost estimate, and docking action.
- **mission_coordinator:** per-robot mission execution and route/station lease usage.
- **factory_simulation:** reusable prefixed robot description and multi-robot Gazebo spawning.
- **factory_bringup:** fleet.yaml, reusable namespaced launch, Nav2 rewriting, scenario launch, and system drivers.
- **mqtt_gateway:** V1/V2 mission parsing and the fleet action client.
- **modbus_gateway / plc_simulator / payload_simulator / protocol_observer:** robot-aware station and payload correlation.
- **tests/browser:** dependency-free Chrome verification of the dashboard.

## Execution prerequisite

Execute the plan from its own implementation worktree. If that worktree does not
contain .venv, run scripts/setup_dev.sh there before Task 1; do not silently use
an editable environment from another checkout. Source /opt/ros/humble/setup.bash
and .venv/bin/activate before build or ROS tests. Each task command assumes the
repository root and this prepared environment.

---

## Milestone 1: Fleet contracts and pure core

### Task 1: Add fleet, resource, energy, and robot-correlation interfaces

**Files:**

- Modify: src/factory_interfaces/CMakeLists.txt
- Modify: src/factory_interfaces/package.xml
- Modify: src/factory_interfaces/msg/ProtocolEvent.msg
- Modify: src/factory_interfaces/srv/TransferPart.srv
- Create: src/factory_interfaces/action/ExecuteFleetMission.action
- Create: src/factory_interfaces/action/DockRobot.action
- Create: src/factory_interfaces/msg/RobotState.msg
- Create: src/factory_interfaces/srv/EstimateMissionCost.srv
- Create: src/factory_interfaces/srv/AcquireResource.srv
- Create: src/factory_interfaces/srv/RenewResource.srv
- Create: src/factory_interfaces/srv/ReleaseResource.srv
- Modify: src/mqtt_gateway/mqtt_gateway/models.py
- Modify: src/mqtt_gateway/mqtt_gateway/validator.py
- Modify: src/mqtt_gateway/config/mission.schema.json
- Modify: src/mqtt_gateway/test/test_validator.py
- Modify: tests/contracts/test_interface_contracts.py

**Interfaces:**

    # ExecuteFleetMission.action
    string mission_id
    string requested_robot_id
    string pickup_station
    string dropoff_station
    string part
    ---
    bool success
    string final_state
    string assigned_robot_id
    string error_code
    string message
    ---
    string state
    string assigned_robot_id
    string detail
    float32 progress

    # RobotState.msg
    builtin_interfaces/Time stamp
    string robot_id
    string mode
    geometry_msgs/Pose pose
    string frame_id
    float32 battery_percent
    string payload_state
    string mission_id
    string health_detail

    # EstimateMissionCost.srv
    string mission_id
    string pickup_station
    string dropoff_station
    string part
    ---
    bool feasible
    float64 path_cost
    float32 predicted_final_battery
    string reason

    # AcquireResource.srv
    string robot_id
    string mission_id
    string resource_id
    ---
    bool granted
    string lease_id
    float64 lease_ttl_sec
    string current_owner
    string reason

    # RenewResource.srv and ReleaseResource.srv
    # Both carry robot_id, mission_id, resource_id, and lease_id.
    # Renew returns renewed, lease_ttl_sec, and reason.
    # Release returns released and reason.

    # DockRobot.action
    string dock_id
    geometry_msgs/PoseStamped staging_pose
    geometry_msgs/PoseStamped charging_pose
    float32 target_percent
    ---
    bool success
    string error_code
    string message
    ---
    string state
    float32 battery_percent
    string detail

Add robot_id immediately after mission_id in ProtocolEvent.msg and TransferPart.srv. Change MissionPayload to accept robot_id as optional; normalize an omitted value to None before building the fleet action goal.

- [ ] Write failing contract tests that assert every field and prove both payloads below validate:

      {"mission_id":"m1","pickup":"assembly","dropoff":"inspection","part":"gear"}
      {"mission_id":"m2","robot_id":"amr_01","pickup":"assembly","dropoff":"inspection","part":"gear"}

- [ ] Run RED:

      .venv/bin/python3 -m pytest -q tests/contracts/test_interface_contracts.py src/mqtt_gateway/test/test_validator.py

- [ ] Add the interface files, dependency declarations, schema migration, and optional MissionPayload field.
- [ ] Build only the interface and gateway packages:

      colcon build --packages-select factory_interfaces mqtt_gateway --symlink-install

- [ ] Run GREEN:

      .venv/bin/python3 -m pytest -q tests/contracts/test_interface_contracts.py src/mqtt_gateway/test/test_validator.py

- [ ] Commit:

      git add src/factory_interfaces src/mqtt_gateway tests/contracts
      git commit -m "feat: add multi-robot fleet interfaces"

### Task 2: Add one validated fleet configuration source

**Files:**

- Create: src/fleet_manager/package.xml
- Create: src/fleet_manager/setup.py
- Create: src/fleet_manager/setup.cfg
- Create: src/fleet_manager/resource/fleet_manager
- Create: src/fleet_manager/fleet_manager/__init__.py
- Create: src/fleet_manager/fleet_manager/config.py
- Create: src/fleet_manager/test/test_config.py
- Create: src/factory_bringup/config/fleet.yaml

**Interfaces:**

    @dataclass(frozen=True)
    class RobotConfig:
        robot_id: str
        namespace: str
        frame_prefix: str
        spawn: Pose2D
        battery_start_percent: float

    @dataclass(frozen=True)
    class ResourceConfig:
        resource_id: str
        kind: str
        capacity: int

    @dataclass(frozen=True)
    class FleetConfig:
        robots: tuple[RobotConfig, ...]
        resources: tuple[ResourceConfig, ...]
        routes: Mapping[str, tuple[str, ...]]
        docks: Mapping[str, DockConfig]
        energy: EnergyPolicyConfig

    def load_fleet_config(path: Path) -> FleetConfig

fleet.yaml must declare amr_01 and amr_02, unique spawn poses, one dock with staging and charging poses, assembly and inspection station resources, and the central_aisle traffic zone. Include commented amr_03 through amr_10 examples only if they remain valid configuration snippets.

- [ ] Write failing tests for the two-robot happy path and rejection of duplicate IDs, namespaces, frame prefixes, spawn poses, unknown route resources, capacities other than one, invalid thresholds, and unknown docks.
- [ ] Run RED:

      .venv/bin/python3 -m pytest -q src/fleet_manager/test/test_config.py

- [ ] Implement immutable config models and strict validation with useful path-specific errors.
- [ ] Run GREEN and the repository config checks:

      .venv/bin/python3 -m pytest -q src/fleet_manager/test/test_config.py
      .venv/bin/python3 -m pytest -q src/factory_bringup/test

- [ ] Commit:

      git add src/fleet_manager src/factory_bringup/config/fleet.yaml
      git commit -m "feat: add validated fleet configuration"

### Task 3: Implement robot health, energy eligibility, and deterministic assignment

**Files:**

- Create: src/fleet_manager/fleet_manager/models.py
- Create: src/fleet_manager/fleet_manager/registry.py
- Create: src/fleet_manager/fleet_manager/energy.py
- Create: src/fleet_manager/fleet_manager/dispatcher.py
- Create: src/fleet_manager/test/test_registry.py
- Create: src/fleet_manager/test/test_energy.py
- Create: src/fleet_manager/test/test_dispatcher.py

**Interfaces:**

    class RobotRegistry:
        def observe(self, state: RobotSnapshot, received_at: float) -> None
        def get(self, robot_id: str, now: float) -> RobotSnapshot
        def eligible(self, now: float) -> tuple[RobotSnapshot, ...]

    class EnergyPolicy:
        def mission_is_safe(self, estimate: CostEstimate) -> bool
        def should_charge(self, state: RobotSnapshot) -> bool
        def charge_complete(self, state: RobotSnapshot) -> bool

    class Dispatcher:
        def choose(
            self,
            mission: MissionRequest,
            robots: Sequence[RobotSnapshot],
            estimates: Mapping[str, CostEstimate],
        ) -> AssignmentDecision

Health uses receipt monotonic time, never message stamps. State becomes UNHEALTHY after 3 seconds and OFFLINE after 5 seconds. Eligibility requires ONLINE, IDLE, known pose, known payload, no fault, and energy-safe estimate. Choose minimum path_cost, then lexicographically smallest robot_id.

- [ ] Write failing tests using explicit monotonic values. Include out-of-order message timestamps, paused ROS timestamps, a pinned unavailable robot, a 19.9% predicted reserve, and an exact-cost tie between amr_01 and amr_02.
- [ ] Run RED:

      .venv/bin/python3 -m pytest -q src/fleet_manager/test/test_registry.py src/fleet_manager/test/test_energy.py src/fleet_manager/test/test_dispatcher.py

- [ ] Implement only the pure models and policies.
- [ ] Run GREEN:

      .venv/bin/python3 -m pytest -q src/fleet_manager/test/test_registry.py src/fleet_manager/test/test_energy.py src/fleet_manager/test/test_dispatcher.py

- [ ] Commit:

      git add src/fleet_manager
      git commit -m "feat: add deterministic fleet dispatch policy"

### Task 4: Add the SQLite mission journal

**Files:**

- Create: src/fleet_manager/fleet_manager/journal.py
- Create: src/fleet_manager/test/test_journal.py

**Interfaces:**

    class MissionJournal:
        def register(self, request: MissionRequest, payload_hash: str, now: float) -> RegisterResult
        def transition(self, mission_id: str, expected: MissionState, target: MissionState,
                       detail: Mapping[str, object], now: float) -> MissionRecord
        def assign(self, mission_id: str, robot_id: str, now: float) -> MissionRecord
        def load_active(self) -> tuple[MissionRecord, ...]
        def events(self, mission_id: str) -> tuple[MissionEvent, ...]
        def close(self) -> None

Create missions and mission_events tables. Use one transaction for state mutation plus audit event. Reject the same mission_id with a different canonical payload hash; return the existing record for an identical duplicate. Enable WAL, foreign keys, and a busy timeout.

- [ ] Write failing tests for first registration, idempotent duplicate, conflicting duplicate, compare-and-set transition, rollback after injected write failure, close/reopen persistence, and two handles contending on the same database.
- [ ] Run RED:

      .venv/bin/python3 -m pytest -q src/fleet_manager/test/test_journal.py

- [ ] Implement the journal with parameterized SQL and explicit transactions.
- [ ] Run GREEN:

      .venv/bin/python3 -m pytest -q src/fleet_manager/test/test_journal.py

- [ ] Commit:

      git add src/fleet_manager/fleet_manager/journal.py src/fleet_manager/test/test_journal.py
      git commit -m "feat: persist fleet mission state"

### Task 5: Implement fair capacity-one resource leases

**Files:**

- Create: src/fleet_manager/fleet_manager/resources.py
- Create: src/fleet_manager/test/test_resources.py

**Interfaces:**

    class ResourceManager:
        def acquire(self, request: LeaseRequest, now: float) -> LeaseDecision
        def renew(self, key: LeaseKey, now: float) -> LeaseDecision
        def release(self, key: LeaseKey, now: float) -> bool
        def expire(self, now: float) -> tuple[Lease, ...]
        def release_owner(self, robot_id: str, now: float) -> tuple[Lease, ...]
        def snapshot(self, now: float) -> tuple[ResourceSnapshot, ...]

Queue order is request sequence then robot_id. Repeating the same acquisition is idempotent. A wrong owner or stale lease_id cannot renew or release. Leases use monotonic expiry and a configurable TTL.

- [ ] Write failing tests for FIFO order, deterministic same-tick ordering, idempotent retry, renewal, wrong-owner rejection, cancellation, expiry handoff, offline-owner cleanup, and concurrent acquire/release calls from threads.
- [ ] Run RED:

      .venv/bin/python3 -m pytest -q src/fleet_manager/test/test_resources.py

- [ ] Implement the lock-protected lease table and wait queues without sleeping inside the core.
- [ ] Run GREEN, including 100 repeated concurrent trials:

      .venv/bin/python3 -m pytest -q src/fleet_manager/test/test_resources.py

  Parameterize 100 concurrent trials inside the test; do not add pytest-repeat.

- [ ] Commit:

      git add src/fleet_manager/fleet_manager/resources.py src/fleet_manager/test/test_resources.py
      git commit -m "feat: coordinate shared fleet resources"

### Task 6: Implement the durable fleet mission state machine

**Files:**

- Create: src/fleet_manager/fleet_manager/core.py
- Create: src/fleet_manager/test/test_core.py

**Interfaces:**

    class FleetCore:
        def submit(self, request: MissionRequest, now: float) -> MissionRecord
        def assign(self, mission_id: str, estimates: Mapping[str, CostEstimate],
                   now: float) -> AssignmentDecision
        def record_robot_feedback(self, mission_id: str, feedback: RobotMissionFeedback,
                                  now: float) -> MissionRecord
        def record_robot_result(self, mission_id: str, result: RobotMissionResult,
                                now: float) -> MissionRecord
        def handle_robot_offline(self, robot_id: str, now: float) -> tuple[MissionRecord, ...]
        def reconcile(self, observations: ReconciliationSnapshot, now: float) -> ReconcileResult

States are RECEIVED, QUEUED, ASSIGNED, EXECUTING, SUCCEEDED, FAILED, CANCELLED, and RECOVERY_REQUIRED. Persist transitions before emitting external effects. Before confirmed pickup, loss of the assigned robot returns the mission to QUEUED. At or after confirmed pickup, loss enters RECOVERY_REQUIRED and blocks automatic reassignment.

- [ ] Write failing tests for automatic assignment, pinned assignment, no eligible robot, duplicate submission, cancellation before/after pickup, robot loss before/after pickup, stale feedback from a previous assignment, terminal idempotency, and restart reconciliation at each state.
- [ ] Run RED:

      .venv/bin/python3 -m pytest -q src/fleet_manager/test/test_core.py

- [ ] Implement the core using Dispatcher, RobotRegistry, ResourceManager, and MissionJournal as injected dependencies.
- [ ] Run GREEN and all pure fleet tests:

      .venv/bin/python3 -m pytest -q src/fleet_manager/test

- [ ] Commit:

      git add src/fleet_manager/fleet_manager/core.py src/fleet_manager/test/test_core.py
      git commit -m "feat: orchestrate durable fleet missions"

---

## Milestone 2: ROS vertical slice and two-robot simulation

### Task 7: Connect MQTT to a fleet manager ROS node

**Files:**

- Create: src/fleet_manager/fleet_manager/node.py
- Create: src/fleet_manager/fleet_manager/main.py
- Modify: src/fleet_manager/setup.py
- Modify: src/fleet_manager/package.xml
- Create: src/fleet_manager/test/test_node.py
- Create: src/fleet_manager/test/fake_robot_agent.py
- Modify: src/mqtt_gateway/mqtt_gateway/node.py
- Modify: src/mqtt_gateway/test/test_node.py
- Modify: src/factory_bringup/launch/demo.launch.py
- Modify: src/factory_bringup/test/test_launch_description.py

**Interfaces:**

- Action server: /factory/execute_fleet_mission
- Per-robot action clients: /<namespace>/factory/execute_mission
- Per-robot cost clients: /<namespace>/factory/estimate_mission_cost
- Per-robot state subscriptions: /<namespace>/factory/robot_state
- Resource services: /factory/resources/acquire, renew, and release

The MQTT gateway must target the fleet action. It sends an empty requested_robot_id for automatic assignment and preserves an explicit configured ID. FleetCore remains ROS-free; the node translates messages and schedules side effects.

- [ ] Write failing node tests with two fake agents. Prove automatic selection, pinned routing, action feedback propagation, duplicate mission behavior, cancellation, a missing cost service, and an agent that disappears during execution.
- [ ] Run RED:

      .venv/bin/python3 -m pytest -q src/fleet_manager/test/test_node.py src/mqtt_gateway/test/test_node.py

- [ ] Implement the ROS adapter and update the gateway and single-robot demo launch.
- [ ] Build and run GREEN:

      colcon build --packages-select factory_interfaces fleet_manager mqtt_gateway factory_bringup --symlink-install
      . install/setup.bash && .venv/bin/python3 -m pytest -q src/fleet_manager/test/test_node.py src/mqtt_gateway/test/test_node.py src/factory_bringup/test/test_launch_description.py

- [ ] Run one bounded vertical driver: publish an automatic MQTT mission, observe an assigned robot, and receive a terminal MQTT status without Gazebo.
- [ ] Commit:

      git add src/fleet_manager src/mqtt_gateway src/factory_bringup
      git commit -m "feat: route mqtt missions through fleet manager"

### Task 8: Make the local mission stack namespace-safe

**Files:**

- Modify: src/mission_coordinator/include/mission_coordinator/coordinator_node.hpp
- Modify: src/mission_coordinator/include/mission_coordinator/navigation_adapter.hpp
- Modify: src/mission_coordinator/src/coordinator_node.cpp
- Modify: src/mission_coordinator/src/navigation_adapter.cpp
- Modify: src/mission_coordinator/test/test_coordinator_node.cpp
- Modify: src/mission_coordinator/test/test_navigation_adapter.cpp
- Modify: src/station_perception/station_perception/node.py
- Modify: src/station_perception/test/test_node.py
- Modify: src/protocol_observer/protocol_observer/node.py
- Modify: src/protocol_observer/test/test_node.py

**Interfaces:**

The coordinator declares robot_id and frame_prefix parameters. Use relative names for factory/execute_mission, navigate_to_pose, amcl_pose, camera/image_raw, factory/station_detection, and local costmap services. Every ProtocolEvent producer populates robot_id.

- [ ] Add failing tests that instantiate amr_01 and amr_02 under distinct namespaces and assert that action, pose, detection, navigation, costmap, and observer endpoints never resolve to a shared robot-local name.
- [ ] Run RED:

      colcon test --packages-select mission_coordinator station_perception protocol_observer --event-handlers console_direct+

- [ ] Replace absolute robot-local names with relative names, read robot_id once, and reject a goal whose explicit robot ID does not match the local parameter.
- [ ] Run GREEN:

      colcon build --packages-select mission_coordinator station_perception protocol_observer --symlink-install
      colcon test --packages-select mission_coordinator station_perception protocol_observer --event-handlers console_direct+
      colcon test-result --verbose

- [ ] Commit:

      git add src/mission_coordinator src/station_perception src/protocol_observer
      git commit -m "refactor: isolate robot-local ros namespaces"

### Task 9: Spawn two prefixed robots from fleet.yaml

**Files:**

- Convert: src/factory_simulation/urdf/factory_amr.urdf to src/factory_simulation/urdf/factory_amr.urdf.xacro
- Modify: src/factory_simulation/CMakeLists.txt
- Modify: src/factory_simulation/package.xml
- Modify: src/factory_simulation/launch/simulation.launch.py
- Create: src/factory_simulation/factory_simulation/description.py
- Create: src/factory_simulation/test/test_description.py
- Modify: src/factory_simulation/test/test_simulation_launch.py
- Modify: src/factory_bringup/launch/navigation.launch.py
- Create: src/factory_bringup/factory_bringup/fleet_launch.py
- Modify: src/factory_bringup/package.xml
- Modify: src/factory_bringup/config/nav2_params.yaml
- Create: src/factory_bringup/test/test_fleet_launch.py

**Interfaces:**

    def robot_launch_specs(config: FleetConfig) -> tuple[RobotLaunchSpec, ...]

One Gazebo server loads the world. For each config entry, xacro receives namespace and frame_prefix, robot_state_publisher is namespaced, spawn_entity receives a unique entity name and pose, and Nav2 is included once under that namespace. Frame IDs are prefixed, including map/odom/base links and sensors. Nav2 topics are relative and parameters are rewritten per robot.

- [ ] Write failing pure tests for two generated specs and a ten-robot synthetic config. Assert unique entity names, namespaces, descriptions, frame IDs, initial poses, Nav2 roots, and lifecycle manager sets.
- [ ] Write a failing bounded launch test that sees amr_01 and amr_02 robot descriptions and spawn requests from one simulation launch.
- [ ] Run RED:

      .venv/bin/python3 -m pytest -q src/factory_simulation/test src/factory_bringup/test/test_fleet_launch.py

- [ ] Implement xacro prefixing and data-driven launch generation. Do not copy launch blocks for individual robot IDs.
- [ ] Run GREEN:

      colcon build --packages-select factory_simulation factory_bringup --symlink-install
      . install/setup.bash && .venv/bin/python3 -m pytest -q src/factory_simulation/test src/factory_bringup/test/test_fleet_launch.py

- [ ] Start Gazebo headless with two robots and capture topic, TF, entity, and Nav2 lifecycle evidence with a timeout.
- [ ] Commit:

      git add src/factory_simulation src/factory_bringup
      git commit -m "feat: launch a data-driven two-robot simulation"

### Task 10: Gate robot movement and station use with leases

**Files:**

- Create: src/mission_coordinator/include/mission_coordinator/resource_adapter.hpp
- Create: src/mission_coordinator/src/resource_adapter.cpp
- Create: src/mission_coordinator/test/test_resource_adapter.cpp
- Modify: src/mission_coordinator/CMakeLists.txt
- Modify: src/mission_coordinator/package.xml
- Modify: src/mission_coordinator/include/mission_coordinator/coordinator_node.hpp
- Modify: src/mission_coordinator/src/coordinator_node.cpp
- Modify: src/mission_coordinator/test/test_coordinator_node.cpp
- Modify: src/factory_bringup/config/fleet.yaml

**Interfaces:**

    class ResourceAdapter {
      void acquire(std::string resource_id, std::string mission_id,
                   AcquireCallback callback);
      void release(std::string resource_id, std::string mission_id);
      void release_all(std::string mission_id);
    };

Each configured mission leg lists ordered traffic resources. The coordinator acquires a leg’s resource before sending Nav2 through it, renews while held, and releases after verified arrival. It acquires the target station before TransferPart and releases after the station transaction. Waiting publishes action feedback with state WAITING_FOR_RESOURCE. Cancel and every terminal path release all held leases.

- [ ] Write failing adapter tests for acquire, denial/retry, renewal, expiry, cancel, service loss, wrong lease, and cleanup.
- [ ] Write a failing coordinator test in which two robots request central_aisle; only the owner may send a navigation goal and the waiting robot proceeds after release.
- [ ] Run RED:

      colcon test --packages-select mission_coordinator --event-handlers console_direct+

- [ ] Implement the adapter and coordinator gates without blocking executor threads.
- [ ] Run GREEN:

      colcon build --packages-select mission_coordinator --symlink-install
      colcon test --packages-select mission_coordinator --event-handlers console_direct+
      colcon test-result --verbose

- [ ] Commit:

      git add src/mission_coordinator src/factory_bringup/config/fleet.yaml
      git commit -m "feat: lease shared routes and stations"

### Task 11: Correlate station, payload, fault, and trace state by robot

**Files:**

- Modify: src/modbus_gateway/modbus_gateway/node.py
- Modify: src/modbus_gateway/test/test_node.py
- Modify: src/payload_simulator/payload_simulator/state_machine.py
- Modify: src/payload_simulator/payload_simulator/node.py
- Modify: src/payload_simulator/test/test_state_machine.py
- Modify: src/payload_simulator/test/test_node.py
- Modify: src/plc_simulator/plc_simulator/server.py
- Modify: src/plc_simulator/test/test_server.py
- Modify: src/fault_injector/fault_injector/node.py
- Modify: src/fault_injector/test/test_node.py
- Modify: src/protocol_observer/protocol_observer/models.py
- Modify: src/protocol_observer/protocol_observer/report.py
- Modify: src/protocol_observer/protocol_observer/trace.py
- Modify: src/protocol_observer/test/test_report.py
- Modify: src/protocol_observer/test/test_trace.py

**Interfaces:**

TransferPart.robot_id is required at runtime. Payload state is keyed by robot ID and mission ID. Station ownership and trace rows include robot ID. Fault commands may target global, station, or robot scope. The observer’s correlation key is mission_id plus robot_id while still retaining one fleet-level mission view.

- [ ] Write failing tests for two simultaneous missions using different stations, station contention, wrong-robot unload, duplicate Modbus request, robot-scoped navigation fault, and interleaved protocol events with the same event names.
- [ ] Run RED:

      .venv/bin/python3 -m pytest -q src/modbus_gateway/test src/payload_simulator/test src/plc_simulator/test src/fault_injector/test src/protocol_observer/test

- [ ] Add correlation fields through the full station and trace path and keep single-robot defaults only at the public V1 input boundary.
- [ ] Run GREEN:

      .venv/bin/python3 -m pytest -q src/modbus_gateway/test src/payload_simulator/test src/plc_simulator/test src/fault_injector/test src/protocol_observer/test

- [ ] Commit:

      git add src/modbus_gateway src/payload_simulator src/plc_simulator src/fault_injector src/protocol_observer
      git commit -m "feat: correlate multi-robot station operations"

---

## Milestone 3: Energy, docking, and restart safety

### Task 12: Add the per-robot state and energy agent

**Files:**

- Create: src/robot_agent/package.xml
- Create: src/robot_agent/setup.py
- Create: src/robot_agent/setup.cfg
- Create: src/robot_agent/resource/robot_agent
- Create: src/robot_agent/robot_agent/__init__.py
- Create: src/robot_agent/robot_agent/energy_model.py
- Create: src/robot_agent/robot_agent/cost_model.py
- Create: src/robot_agent/robot_agent/node.py
- Create: src/robot_agent/test/test_energy_model.py
- Create: src/robot_agent/test/test_cost_model.py
- Create: src/robot_agent/test/test_node.py
- Modify: src/factory_bringup/launch/demo.launch.py

**Interfaces:**

    class EnergyModel:
        def advance(self, elapsed_sec: float, distance_m: float,
                    mode: RobotMode) -> float
        def predict(self, path_cost: float, operation_count: int) -> float

    class CostModel:
        def estimate(self, pose: Pose2D, mission: MissionRequest,
                     path_lengths: PathLengths, battery_percent: float) -> CostEstimate

The namespaced node publishes factory/robot_state at 2 Hz and serves factory/estimate_mission_cost. It listens to local odometry, mission feedback/result, and payload state. Battery decreases from movement plus operations and increases only in CHARGING mode while a valid dock lease is held.

- [ ] Write failing pure tests for idle drain, movement drain, pickup/dropoff cost, charge gain, clamping, deterministic prediction, and 20% reserve classification.
- [ ] Write failing node tests for namespaced state, cost response, stale odometry, unknown pose, payload updates, and heartbeat cadence under paused simulation time.
- [ ] Run RED:

      .venv/bin/python3 -m pytest -q src/robot_agent/test

- [ ] Implement the pure models and ROS adapter; use a wall timer for health heartbeat receipt and ROS stamps for telemetry.
- [ ] Run GREEN:

      colcon build --packages-select robot_agent factory_bringup --symlink-install
      . install/setup.bash && .venv/bin/python3 -m pytest -q src/robot_agent/test

- [ ] Commit:

      git add src/robot_agent src/factory_bringup/launch/demo.launch.py
      git commit -m "feat: publish robot health and energy estimates"

### Task 13: Implement shared-dock charging through a robot action

**Files:**

- Create: src/robot_agent/robot_agent/docking.py
- Create: src/robot_agent/test/test_docking.py
- Modify: src/robot_agent/robot_agent/node.py
- Modify: src/robot_agent/test/test_node.py
- Modify: src/fleet_manager/fleet_manager/core.py
- Modify: src/fleet_manager/fleet_manager/node.py
- Modify: src/fleet_manager/test/test_core.py
- Modify: src/fleet_manager/test/test_node.py
- Modify: src/factory_bringup/config/fleet.yaml
- Create: src/factory_bringup/test/test_charging_scenario.py

**Interfaces:**

- Per-robot action server: /<namespace>/factory/dock_robot
- Fleet charging states: CHARGE_QUEUED, DOCKING, CHARGING

The fleet manager queues an idle robot below 30%. The robot agent waits at the staging pose without the dock lease, acquires the dock, navigates into the charging pose, reports CHARGING, and releases only after reaching 80% or on safe cancellation. One robot charges at a time. Mission assignment excludes charge-queued, docking, and charging robots.

- [ ] Write failing docking controller tests for stage, lease wait, enter, charge, target completion, cancel before lease, cancel while docked, navigation failure, lease loss, and agent restart.
- [ ] Write a failing fleet test where both robots are below 30%; amr_01 wins the deterministic dock queue, amr_02 waits, and then receives the dock.
- [ ] Run RED:

      .venv/bin/python3 -m pytest -q src/robot_agent/test/test_docking.py src/fleet_manager/test/test_core.py src/fleet_manager/test/test_node.py src/factory_bringup/test/test_charging_scenario.py

- [ ] Implement action behavior and fleet charging policy.
- [ ] Run GREEN:

      colcon build --packages-select factory_interfaces robot_agent fleet_manager factory_bringup --symlink-install
      . install/setup.bash && .venv/bin/python3 -m pytest -q src/robot_agent/test src/fleet_manager/test src/factory_bringup/test/test_charging_scenario.py

- [ ] Commit:

      git add src/robot_agent src/fleet_manager src/factory_bringup
      git commit -m "feat: coordinate shared dock charging"

### Task 14: Reconcile missions, leases, payloads, and charging after restart

**Files:**

- Create: src/fleet_manager/fleet_manager/recovery.py
- Create: src/fleet_manager/test/test_recovery.py
- Modify: src/fleet_manager/fleet_manager/node.py
- Modify: src/fleet_manager/fleet_manager/journal.py
- Modify: src/fleet_manager/fleet_manager/resources.py
- Modify: src/factory_bringup/test/test_restart_scenarios.py

**Interfaces:**

    class RecoveryPlanner:
        def plan(self, active: Sequence[MissionRecord],
                 robots: Mapping[str, RobotSnapshot],
                 resources: Sequence[ResourceSnapshot]) -> tuple[RecoveryDecision, ...]

On startup, the node loads nonterminal missions, waits for bounded robot observations, expires old in-memory leases, and derives a conservative plan. RECEIVED/QUEUED return to queue. ASSIGNED without execution may requeue. EXECUTING before confirmed pickup may requeue only after the old robot is known not to hold payload. Any uncertain or confirmed payload ownership becomes RECOVERY_REQUIRED. Dock state is reconstructed from robot pose/mode and a new lease, never from stale lease memory.

- [ ] Write a failure matrix that restarts the fleet process at each persisted state, before and after pickup, during station use, while holding central_aisle, at dock staging, and while charging.
- [ ] Run RED:

      .venv/bin/python3 -m pytest -q src/fleet_manager/test/test_recovery.py src/factory_bringup/test/test_restart_scenarios.py

- [ ] Implement the recovery planner and bounded startup reconciliation.
- [ ] Run GREEN, then kill and restart the actual fleet node during a bounded fake-agent mission:

      colcon build --packages-select fleet_manager factory_bringup --symlink-install
      . install/setup.bash && .venv/bin/python3 -m pytest -q src/fleet_manager/test/test_recovery.py src/factory_bringup/test/test_restart_scenarios.py

- [ ] Commit:

      git add src/fleet_manager src/factory_bringup/test/test_restart_scenarios.py
      git commit -m "feat: reconcile fleet state after restart"

---

## Milestone 4: Operations view and release evidence

### Task 15: Add the read-only fleet dashboard with SSE

**Files:**

- Create: src/fleet_manager/fleet_manager/web.py
- Create: src/fleet_manager/fleet_manager/static/index.html
- Create: src/fleet_manager/fleet_manager/static/styles.css
- Create: src/fleet_manager/fleet_manager/static/app.js
- Create: src/fleet_manager/test/test_web.py
- Create: tests/browser/chrome_cdp.mjs
- Create: tests/browser/test_fleet_dashboard.mjs
- Modify: src/fleet_manager/setup.py
- Modify: src/fleet_manager/fleet_manager/node.py
- Modify: src/factory_bringup/launch/demo.launch.py

**Interfaces:**

- GET / serves the dashboard.
- GET /api/snapshot returns robots, missions, resources, dock queue, and updated_at.
- GET /api/events streams SSE snapshot events.
- Any non-GET method returns 405.

Use a loopback ThreadingHTTPServer owned by the fleet node with bounded per-client queues. A slow or disconnected client must not block ROS callbacks. The page must show both robots, state, battery, assigned mission, payload, current resource, dock queue, and a recent fleet event list. Reconnect with exponential backoff and refresh from /api/snapshot.

- [ ] Write failing HTTP tests for loopback binding, snapshot schema, SSE framing, slow-client eviction, disconnect cleanup, 404, method rejection, static path traversal, and clean shutdown.
- [ ] Write a failing real-Chrome test that loads the page, receives a simulated state transition, renders both robots and dock ownership, disconnects SSE, and visibly recovers after reconnect.
- [ ] Run RED:

      .venv/bin/python3 -m pytest -q src/fleet_manager/test/test_web.py
      node tests/browser/test_fleet_dashboard.mjs

- [ ] Implement the HTTP adapter and hand-authored responsive page.
- [ ] Run GREEN:

      .venv/bin/python3 -m pytest -q src/fleet_manager/test/test_web.py
      node tests/browser/test_fleet_dashboard.mjs

- [ ] Capture desktop and narrow-phone screenshots and inspect them for clipping, unreadable state, overflow, and misleading stale connection state.
- [ ] Commit:

      git add src/fleet_manager tests/browser src/factory_bringup/launch/demo.launch.py
      git commit -m "feat: add live read-only fleet dashboard"

### Task 16: Prove the two-robot system and publish recruiter-facing evidence

**Files:**

- Create: tests/system/test_two_robot_fleet.py
- Create: tests/system/test_ten_robot_scale.py
- Create: tests/scenarios/fleet_nominal.yaml
- Create: tests/scenarios/fleet_contention.yaml
- Create: tests/scenarios/fleet_robot_failure.yaml
- Create: tests/scenarios/fleet_charging.yaml
- Create: scripts/run_fleet_demo.sh
- Create: scripts/run_fleet_acceptance.sh
- Modify: scripts/run_ci_checks.sh
- Modify: tests/test_run_ci_checks.py
- Modify: README.md
- Modify: docs/architecture.md
- Modify: docs/protocols.md
- Create: docs/fleet-operations.md
- Create: docs/assets/fleet-dashboard.png
- Create: docs/assets/two-robot-fleet.gif

**Interfaces:**

- scripts/run_fleet_demo.sh accepts --headless and an optional output directory.
- scripts/run_fleet_acceptance.sh accepts --headless, --scenario, and --timeout.
- Each driver creates one timestamped artifact directory containing process
  logs, protocol traces, fleet snapshots, test receipts, and cleanup status.
- Every owned child process has a bounded startup, execution, and shutdown.

**Acceptance scenarios:**

1. Two automatic missions select eligible robots deterministically and finish.
2. A pinned amr_01 mission remains compatible with the original external payload.
3. Two robots cannot own central_aisle, a station, or the dock simultaneously.
4. A robot failure before pickup reassigns; after pickup it enters RECOVERY_REQUIRED.
5. A low-battery robot charges from below 30% to 80% while another eligible robot handles work.
6. Ten synthetic agents register, heartbeat, estimate, schedule, and complete bounded missions without special-case code.

- [ ] Write the system tests and scenario files first. Run them and record the expected RED causes:

      .venv/bin/python3 -m pytest -q tests/system/test_two_robot_fleet.py tests/system/test_ten_robot_scale.py

- [ ] Add fail-fast demo and acceptance drivers with explicit timeouts, process cleanup traps, artifact directories, and nonzero exits on missing evidence.
- [ ] Make run_ci_checks.sh include pure fleet, agent, interface, launch, dashboard, and ten-agent bounded checks while keeping full Gazebo acceptance as an explicit supported command.
- [ ] Run the focused two-robot headless acceptance:

      scripts/run_fleet_acceptance.sh --headless --scenario all --timeout 900

- [ ] Run the full repository gate:

      scripts/run_ci_checks.sh
      colcon test-result --verbose
      git diff --check

- [ ] Start the visible demo, record the robot transfer and charging sequence, update the screenshot/GIF, and verify that the media shows actual station-to-station payload movement.
- [ ] Update README with a 60-second quick start, architecture diagram, dashboard image, protocol boundaries, two-versus-ten scaling claim, test evidence, and honest laptop constraints.
- [ ] Commit:

      git add tests scripts README.md docs
      git commit -m "docs: publish scalable fleet demo evidence"

---

## Final integration and review gate

- [ ] Rebase the implementation branch on the latest public main without rewriting unrelated user work.
- [ ] Run every command from Task 16 on the rebased branch.
- [ ] Verify git status contains no generated logs, databases, Gazebo caches, browser profiles, or test artifacts.
- [ ] Request an independent review focused on the five Review focus items and all specification acceptance criteria.
- [ ] Fix each confirmed finding with a new regression test and focused commit.
- [ ] Rerun the complete gate after the final fix.
- [ ] Only then push the branch and open a public pull request. Do not manufacture filler commits; the task-sized commits above provide a truthful, reviewable history.
