# Measured validation results

The evidence uses real Gazebo rendering, Nav2/AMCL, ROS 2 DDS, Mosquitto, and a
deterministic Modbus TCP PLC simulator. A test's total wall time includes setup,
assertions, and cleanup. Mission duration instead comes from the correlated
`mission_started` and `mission_finished` source timestamps in simulation time.

## Current local verification

The verified development source is revision
`6b4f300f64cbd22270f69722278df0d083904340`, checked on 3 October 2026 from a
clean worktree. Its filtered public equivalent is
`baef48bc9af5e9ce62b54ca3319de7f355f72d75`; identifiers differ because private
planning paths are absent from every public revision. Since the earlier recording,
one production shutdown correction lets the visualization callback finish before
ROS shutdown, disables automatic rclpy signal handling for that process, and
restores its original handlers. Later changes only strengthen bounded test
cleanup across direct callbacks and real `setsid` transitions. Runtime protocol
contracts, mission events, physical assertions and dependency pins are unchanged.

| Command or gate | Observed result |
| --- | --- |
| `colcon build --symlink-install --event-handlers console_direct+` | All ten packages pass in the retained pinned environment; current incremental build: 3.98 wall seconds |
| Unrestricted `colcon test --return-code-on-test-failure` and `colcon test-result --verbose` | 174 fresh wrapper/Python records, zero errors/failures/skips, including Gazebo camera, payload, topic and navigation coverage |
| `python3 -m pytest -q tests src/*/test` | 445 passed, two inherited Xacro warnings, 574.36 wall seconds |
| `scripts/run_ci_checks.sh` | All ten non-Gazebo stages pass; 170 fresh wrapper/Python records with zero errors/failures/skips; separate stages: 166 pure, 36 contract/schema, 94 real protocol/DDS checks |
| `scripts/run_all_scenarios.sh` | 12/12 expected outcomes, 73/73 trace predicates, all owned cleanups verified on 12 distinct leased domains |
| Supported GUI success case | COMPLETED; 24/24 child exits zero; wrapper exit 130; owned Compose project absent and ports closed |
| Environment, Compose configuration, shell syntax, public links/assets, whitespace | Pass; ROS 2 Humble, Python 3.10.12, Compose 2.21.0, system OpenCV 4.5.4 |

These categories overlap and must not be summed as distinct tests. The current
build reused the pinned checkout-local interpreter environment; fresh dependency
bootstrap is verified separately in the release clone. Local unrestricted
Gazebo coverage is not replaced with hosted non-Gazebo coverage.
Hosted setup sets `FACTORY_AMR_ROSDEP_SKIP_KEYS=nav2_bringup` because that
launch meta-package is outside the hosted non-Gazebo gate and its current runner
dependency is unsatisfiable. Individual Nav2 API packages remain declared and
installed. Local setup still installs all declared dependencies by default.

The current matrix's successful mission spans **72.3 simulated seconds**, from
3.8 s to 76.1 s; its complete case takes 97.367 wall seconds. Those clocks are
distinct from both the current GUI receipt below and the earlier recording.
Four matrix cases use physical Gazebo/Nav2, seven real protocol cases use a
bounded navigation driver, and one is an isolated real DDS experiment.

| Current matrix case | Expected/observed result | Case wall seconds |
| --- | --- | --- |
| `success` | COMPLETED | 97.367 |
| `mqtt_duplicate` | COMPLETED, one action | 2.670 |
| `mqtt_conflict` | COMPLETED, changed request rejected | 2.160 |
| `mqtt_disconnect` | COMPLETED after connection recovery | 2.670 |
| `modbus_delay` | COMPLETED | 2.830 |
| `modbus_timeout` | FAILED / PLC_TIMEOUT | 2.140 |
| `modbus_stale_completion` | FAILED / STALE_PLC_STATE | 2.080 |
| `plc_fault` | FAILED / PLC_FAULT | 1.750 |
| `wrong_marker` | FAILED / STATION_NOT_CONFIRMED; zero PLC contact | 48.530 |
| `nav_retry` | COMPLETED after real costmap clear and same-leg retry | 81.170 |
| `nav_failure` | FAILED / NAVIGATION_FAILED | 8.840 |
| `qos_mismatch` | RECOVERED after incompatible then compatible DDS readers | 3.000 |

Earlier combined runs exposed a direct Modbus test clearing its caller's asyncio
loop and a false aggregate arrival-order assertion across independent DDS
writers. Focused RED/GREEN tests preceded their corrections. A later current-
candidate command recorded 444 passes and one prelaunch Docker registry failure;
a separately focused retry passed after the required base image became available.
The fresh complete command above now establishes all 445 passes in one run. The
failed infrastructure receipts remain preserved rather than being relabeled.

## Current public GUI receipt

Status: observed. Source revision: `6b4f300f64cbd22270f69722278df0d083904340`.
The supported `scripts/run_demo.sh` GUI path runs through the production scenario
supervisor with the unchanged successful-mission assertions and bounded command
`timeout 300s python3 -m pytest -q tests/system/test_successful_mission.py -s`.
An ignored task-local observer records owned identities and shutdown stacks;
it does not change production calls, exception behavior or deadlines. No new
capture asset or injected traffic is used.

| Measurement | Observed result |
| --- | --- |
| Correlated mission source stamps | 71.9 simulated seconds, from 3.9 s to 75.8 s |
| Inner pytest | 1 passed, 1 warning, 96.03 wall seconds |
| Complete supervised case | 97.139 wall seconds; COMPLETED; 4/4 trace predicates |
| Assembly camera gate | Fresh rendered-marker window passed for marker 10 |
| Inspection camera gate | Fresh rendered-marker window passed for marker 20 |
| PLC counters | `[0, 0]` before; `[1, 1]` after; robot request/presence coils clear |
| Child lifecycle | 24/24 exit zero, including Gazebo server/client, RViz and visualization; wrapper exit 130 |
| Owned cleanup | All launch children and group gone, Compose removed, ports closed, preexisting containers preserved |
| Final Gazebo position / yaw error | 0.0812 m / 0.1678 rad |
| Final AMCL position / yaw error | 0.1265 m / 0.1919 rad |

The warning is pytest's existing `PytestReturnNotNoneWarning`: under a scenario
the product system test returns its evidence dictionary. It is not an Xacro
warning or a shutdown failure and was not suppressed. The ignored observer only
selects GUI mode and records provenance; all physical and child-exit assertions
remain those of the product test.

An earlier public GUI attempt at development revision
`f47a4b779c90c17ae3fdf53416e98d8361726433` completed its mission but reported 23 zero
child exits and a **null** visualization status. Its wrapper was terminated with
**-15**, and owned Compose still existed at the public test's cleanup assertion;
the outer supervisor reclaimed the resources afterward. Separately, a bounded
matched-reader visualization diagnostic exited with a post-SIGINT **RuntimeError**
(`Unable to convert call argument to Python object`). Their historical cause remains unknown.
The production shutdown correction is covered by focused real-DDS lifecycle
tests and the current supported GUI pass, but it does not rewrite or diagnose
either historical receipt and does not prove reliability on every graphics stack.

An earlier verification attempt failed before launch because the Docker registry's
anonymous-token hostname timed out in DNS. Normal DNS/TLS connectivity recovered
without host repair; later complete verification produced the receipt above. This
infrastructure failure is separate from the retained shutdown risks.

## Visible payload mission recording

The current mission GIF uses product source revision
`9837ce27362309aca717929dbf4d9aee2b2dade6`, captured on 4 October 2026. The
supported successful mission reached every state from `RECEIVED` through
`COMPLETED`; both PLC cycle counters reached one, and the payload progressed
from `AT_ASSEMBLY` through `IN_TRANSIT` to `AT_INSPECTION`. The final part pose
was `(3, 2, 0.65)`.

The recorder made 564 successful owned-window captures from 564 attempts over
75.145 wall seconds: 282 Gazebo frames and 282 RViz frames. All 24 launch
children exited cleanly, the interrupted wrapper returned 130, the owned
Compose project was absent, and owned ports were closed after cleanup.

The renderer selects 137 real Gazebo source frames: 9 pickup frames, 120
transport frames, and 8 drop-off frames. GIF optimization stores 78 encoded GIF
frames at 960 × 600 over 12.99 seconds. Pickup and drop-off play at captured
wall speed; transport plays at 4× wall speed. A nearest-neighbor crop follows
the detected orange payload and labels the three phases. Playback holds sampled
frames and creates no intermediate images. This establishes visible simulated
carrying, not a physical grasp or hardware result.

## Superseded recording evidence

The earlier recording uses product source revision
`759af4f8901787c4476f24e744642b5b662e1796`, captured on 3 October 2026.
Mission `M-001` completes once; replaying its ID does not create another action
or PLC cycle. A distinct post-pickup request fails with `RESTART_REQUIRED`.

| Measurement | Observed result |
| --- | --- |
| Mission duration from correlated source stamps | 72.6 simulated seconds, from 3.7 s to 76.3 s |
| Whole scenario wall time, including setup and cleanup | 104.611 seconds |
| Assembly camera gate | 5 fresh stamps in 0.699 simulated seconds; 2 independently decoded exact source overlaps |
| Inspection camera gate | 5 fresh stamps in 0.498 simulated seconds; 5 exact rendered overlaps |
| PLC counters, assembly and inspection | `[0, 0]` before; `[1, 1]` after; robot request/presence coils clear |
| Gazebo final position / yaw error | 0.0816 m / 0.1751 rad |
| AMCL final position / yaw error | 0.1152 m / 0.1884 rad |
| Payload lifecycle | `AT_ASSEMBLY`, `IN_TRANSIT`, `AT_INSPECTION` |
| Scenario trace predicates | 4/4 pass, in addition to full system assertions |
| Cleanup | Owned launch children exited, owned Compose project removed, owned ports closed |

Model and child-link probes prove 0.2 m travel on each belt, stable hidden
storage during transport, final placement at `(3, 2, 0.65)`, and no replay of
successful cycles. Native rendered frames separately show intermediate loading
and unloading positions. A static part initially left the loading client view
stale despite updated service poses; the final part uses a kinematic link with
gravity disabled, preserving stable command-owned poses while showing motion.

### Evidence and clock boundaries

The demo assets are captured from owned Gazebo and RViz application windows.
Native frames retain their capture times. The navigation view plays at 4× wall
speed; the two zoomed conveyor clips play at their captured wall speed. Scaling
and cropping expose the original pixels. Playback holds sampled frames and
does not create intermediate motion. This recording is visual evidence of the
simulator, not hardware testing or a physical grasp simulation.

There are 513 successful native frames from 514 capture attempts over 77.967
wall seconds. One RViz frame reports an X11 `BadMatch` and is omitted. Actual
timestamps preserve that gap; the GIF holds captured frames rather than
inventing motion. Both conveyor clips contain real intermediate part positions.

## Supplemental AMR visibility

The [native robot close-up](../assets/amr-closeup.png) uses source revision
`a33a31f36a1c1e014b07d9c45eddb4d39bd57719`, captured on 3 October 2026. It
shows the stacked base, wheels, and cylindrical LiDAR body. Earlier wide views
did not establish a recognizable robot: Gazebo could not resolve the installed
description meshes and rendered only the camera box. The simulation now appends
the installed description's share parent to its existing model search path.
Robot geometry, mesh scale, collisions, sensors, and protocol behavior are
unchanged. The camera remains at its configured elevated mount.

This capture launches only Gazebo and the robot's description/spawn. It is not
a mission rerun or new navigation, sensor-gate, or protocol evidence. Both Gazebo
processes receive the corrected search path, and their logs no longer report
the missing body, wheel, or LiDAR meshes. Capture validates the owned drawable's
PID, start time, and process group. Owned resource cleanup passes, but the
Gazebo client exits with SIGSEGV (signal 11) during this diagnostic's SIGINT
teardown. This additional lifecycle concern is not suppressed or resolved here.

Two standalone close-up receipts record Gazebo client SIGSEGV during teardown;
the cause remains unverified. The full public GUI mission instead records a
clean client exit. The current source-bound GUI receipt above records every
child exit, but the retained shutdown risks still require release review.
A supported-path client SIGSEGV blocks release. The resolved-mesh receipt also contains 407
model-browser missing-`model.config` errors for unrelated ROS share packages.
This accepted GUI noise does not mean the AMR meshes are missing and does not
establish a crash cause. Actual asset-resolution failures remain separate.

## Earlier complete fault matrix

The earlier local matrix at revision
`b91c36ded7fcf5c71519e698e0a52c68f6e9359d` completed 12/12 expected outcomes and
73/73 runner trace predicates. It predates later supervisor/domain amendments
and the payload rendering correction. It is not a final candidate matrix.
The [fault scenario table](../fault-scenarios.md) preserves its exact results,
elapsed wall seconds, and source classifications.

Four physical cases use real Gazebo. Seven protocol cases use real MQTT, ROS,
and Modbus services with a bounded navigation driver; they do not prove physical
autonomy. The remaining case is an isolated real DDS experiment. It measured
2.001 seconds of incompatible reliability with zero data and zero matches,
followed by a compatible subscriber receiving data. Fault controls are explicit
and deterministic; normal operation does not enable random corruption.

## Earlier local CI receipt

Revision `d1a5767c95d6ee4edc76acf2044d17d36a71926f` passed all ten local
non-Gazebo stages, including all ten ROS packages. Its fresh colcon result
directory contains 160 wrapper/Python records with zero errors, failures, or
skips. Separate stage receipts are 92 pure checks, 36 contract/schema checks,
and 38 real protocol/DDS integration checks. These categories overlap and must
not be summed as distinct tests. This earlier receipt predates the added public
document checks and payload animation tests. Run `scripts/run_ci_checks.sh`
for the same current selection used by the hosted workflow. At that earlier
revision, no hosted workflow run had been observed.

## Current hosted CI

Status: success. The non-Gazebo workflow passed at public commit
`2290822c0f0da742627933b6b4b3dec42a7c3e2d` in
[GitHub Actions run 37126082544](https://github.com/Nixon080304/factory-amr-protocol-lab/actions/runs/37126082544).
This hosted gate does not replace the local Gazebo evidence above.

## Limits and diagnostics

Full Gazebo verification is local-only and requires a working rendering display.
The captured RViz view includes the map, AMCL localization, LiDAR scan, global
and local costmaps, Nav2 plan, current goal, and camera station labels. Labels
use configured station positions; they are not measured 3D marker poses.

The source-bound recording logs include Xacro escape warnings, Nav2's default goal-checker warning,
and RViz GLSL sampler, covariance, and message-filter diagnostics. Some Python
adapters also report `rcl_shutdown already called on the given context` during
SIGINT teardown; six children exit 1, while eighteen exit cleanly. This shutdown
concern does not alter the measured mission result. The subsequent reliability
fix handles expected signal shutdown and drains owned executors before node
destruction. The system gate now records and requires clean exits for every
normal child, separately from wrapper interruption and process absence. These
changes do not rewrite the six historical failures. The current GUI receipt
above is an observed pass, not proof that the later retained visualization
failures are fixed. Rendering on
every graphics driver is not guaranteed.

Version 1 supports one fixed part and route. A confirmed or uncertain pickup consumes the
lifecycle; another distinct mission fails with `RESTART_REQUIRED` until the
full simulation restarts. Same-ID replay remains supported. Registries are
process-local, and a hung transfer gateway can leave a stopped pending mission
with unknown PLC state. No general crash recovery or production security model
is claimed. See [architecture](../architecture.md) and
[protocol contracts](../protocols.md) for the exact safety boundaries.
Optional MQTT password-file authentication and obstacle-triggered physical
replanning/recovery are explicitly deferred and are not delivered. The
controllable non-static obstacle remains; current recovery evidence uses Nav2
adapter rejection, real costmap clears, and retry of the same leg.
