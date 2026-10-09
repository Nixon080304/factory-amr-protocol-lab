# Measured validation results

The fleet simulation evidence below is from 8 October 2026. Final local gates
are measured on 9 October; the historical Version 1 sections retain their
3 October source boundaries and do not establish fleet verification.
See the [fleet quick start](../../README.md#fleet-quick-start) and
[fleet operations](../fleet-operations.md) for commands, artifact anatomy and
the distinction between physical simulation and synthetic DDS scaling proof.

## Fleet evidence: 8 October 2026

The ordinary, unmodified-middleware command
`scripts/run_fleet_acceptance.sh --headless --scenario all --timeout 900`
passes from one frozen Task 16 source tree. Its retained, ignored artifact
directory is `artifacts/fleet/20261008T130900Z-acceptance-wjqmyunu`.
Private receipts retain exact lease identities; public state and screenshots
publish no lease credentials. The run predates a later, isolated Version 1
endpoint compatibility repair. That repair selects root mission and Nav2
endpoints only when the existing explicit legacy flags are true; the normal
fleet endpoint strings remain unchanged and real DDS isolation tests verify
that boundary. The fleet receipt and media therefore remain applicable to
normal fleet behavior, but they are not a byte-identical source receipt for
the final commit. The final-tree CI and Version 1 checks below verify the
later compatibility branches separately. A subsequent shared contact-process
shutdown guard handles only the exact native WaitSet error after ROS context
invalidation; contact, geometry and freshness behavior are unchanged. Its
active-context and unrelated-error regressions, complete simulation package
and actual Version 1 clean-child receipt verify that shutdown-only change.

| Scenario | Observed proof | Case wall seconds |
| --- | --- | --- |
| Nominal | Two automatic missions select distinct eligible robots deterministically and deliver both payloads; 1,099 actual-world samples | 236.245 |
| Contention | Original pinned `amr_01` payload remains compatible; both robots complete under exact capacity-one lease and physical-occupancy checks; 1,130 world samples | 242.833 |
| Robot failure | Pre-pickup reassignment is blocked until stop and exact clearance proof; post-pickup mission remains `RECOVERY_REQUIRED` with its carrier | Bounded DDS probe |
| Charging | Initially low robot reaches 80% with independent simulated contact while its peer delivers; peer then drops below 30%, receives a distinct later dock lease, reaches 80% and exits; 1,163 world samples | 249.979 |
| Ten agents | Ten synthetic DDS agents register, heartbeat, cost, schedule and complete ten bounded missions without changing production configuration or code | 1.353 |

Each physical scenario observes 18 active namespaced Nav2 lifecycle nodes.
Every scenario's 42 launch children exits zero, its launch wrapper exits zero,
owned process groups clear without escalation, and owned PLC and broker cleanup
passes. Failure and scale probes also leave no owned process group.
The physical overlap threshold remains twice the configured robot radius,
0.30 m, with only the documented numerical tolerance. Static waypoint and
holding-bay validation retains its separate conservative 0.60 m clearance.
These samples are simulation evidence, not a physical safety certification or
continuous-time collision proof.

One contention startup log reports FastDDS
`failed to send response to /amr_02/smoother_server/get_state (timeout)`.
The lifecycle readiness gate subsequently passes; all mission evidence and child
exits pass. Charging shutdown also records an upstream
`rclpy.task.Future.__del__` `KeyboardInterrupt` after the fleet manager has
already exited zero. Both diagnostics are retained, not suppressed. Neither
causes missing evidence, nonzero child status or surviving processes. No
middleware profile, timeout extension or weakened assertion is used.

### Current fleet media provenance

The supported visible `DISPLAY=:0 scripts/run_fleet_demo.sh` passes charging
and a fresh-world nominal scenario in
`artifacts/fleet/20261008T132337Z-demo-2hb6spye`. Charging takes 249.352 wall
seconds with 1,161 world samples; nominal takes 234.648 wall seconds with
1,084 samples. Both scenarios retain their receipts and clean child exits.

The [fleet GIF](../assets/two-robot-fleet.gif) uses only the charging scenario's
owned native Gazebo window: 1,840 source frames at 1920 × 950 and 10 frames/s
over 184 wall seconds. The recorder ends when the owned window closes, leaving
a retained X11 end-of-window diagnostic; its valid MP4 exits zero. The published
GIF selects 460 real frames, crops and scales the world view, adds descriptive
labels, and plays at 4× wall speed. It is 1200 × 658, 46 seconds and 1,089,043
bytes. There is no interpolation, generated image or fabricated robot motion.
The inspected source and GIF frames show an empty pickup robot, orange motor
riding that robot, motor arriving on inspection, and the peer on the charging
pad. The far lower-right departure is partly outside the original camera view;
the numeric clearance and exit proof comes from the separate world-pose audit.

The [dashboard screenshot](../assets/fleet-dashboard.png) is an unmodified
full-page browser capture from the final headless charging run. It shows
`amr_01` loaded at 30% and `amr_02` charging at 67%, with their actual resource
ownership and mission. Full-page inspection confirms readable identities,
no clipped page content and no raw lease credentials. The dashboard is a
read-only observer; its pixels do not replace physical simulation assertions.

The fresh complete CI, unrestricted Gazebo package checks and Version 1
restart/scenario matrix are separate gates. Their counts come from their own
fresh result roots, not from historical aggregates or the receipts above.
No new hosted workflow result is claimed for the fleet source.

The evidence uses real Gazebo rendering, Nav2/AMCL, ROS 2 DDS, Mosquitto, and a
deterministic Modbus TCP PLC simulator. A test's total wall time includes setup,
assertions, and cleanup. Mission duration instead comes from the correlated
`mission_started` and `mission_finished` source timestamps in simulation time.

## Final Task 16 local gates: 9 October 2026

The final `scripts/run_ci_checks.sh` exits zero from a frozen runtime, test
and script tree. A retained SHA-256 manifest matches before and after the run.
Its unique root is `artifacts/ci/20261009T033003Z-run.MJ3aZg`; subsequent stages
source only that root's fresh install. The table is a measured receipt, not a
claim about a future hosted workflow or independent review. This receipt prose
is added after the run and checked separately; no runtime, test or script is
changed after the frozen gate.

| Command or gate | Observed result |
| --- | --- |
| Fresh CI build and serial package tests | All 12 packages build in 55.1 wall seconds and test in 4 minutes |
| Explicit `colcon test-result --verbose --test-result-base artifacts/ci/20261009T033003Z-run.MJ3aZg/test-results` | 1,608 records; zero errors, failures or skips |
| CI pure / contract and schema / bounded fleet stages | 297 passed in 220.50 seconds / 63 passed / 30 passed in 22.97 seconds |
| Real Chrome dashboard | Desktop and 360 px mobile layout, live transitions, stale/reconnect handling, fresh snapshot and credential redaction pass without console or network exceptions |
| Real protocol and DDS integration | 95 passed in 67.88 seconds, including the complete protocol-fault suite |
| Complete repository `.venv/bin/python3 -m pytest -q tests` | 395 passed in 534.98 seconds; zero failures, errors or skips |
| Current Version 1 `scripts/run_all_scenarios.sh` | 12/12 expected outcomes, 73/73 trace predicates and 12/12 verified owned cleanups; root `reports/20261009T112506-cb66baeb7b89` |
| Complete affected simulation package | 50 passed, including real Gazebo topics and navigation; two inherited Xacro warnings |
| Focused actual Version 1 success and cleanup | All physical, transfer, duplicate and restart assertions pass; 25/25 launch children exit zero, wrapper exits 130 and owned services/processes disappear |

CI intentionally excludes the explicit rendered Gazebo package checks described
in the README. The earlier unrestricted package run has its own fresh root,
`artifacts/ci/20261008T140635Z-full.fqpltn`: all 12 packages and 1,610 records
pass with zero errors, failures or skips. That run predates the isolated legacy
endpoint and shutdown-only repairs described above; it is not relabeled as a
byte-identical final-tree run. The final CI includes all affected agent and
fleet packages, real legacy-root/normal-namespace isolation, and the 23-case
restart matrix. Current real Version 1 physical cases and the complete affected
simulation package verify the later compatibility and shutdown changes.

The repository suite and Version 1 matrix use the explicit fresh unrestricted
install rather than the checkout's older `install/`. The subsequent CI-only
change selects serial package scheduling and is verified by its real runner
contract in final CI. Package-local tests remain isolated under colcon; they
are not merged into one incompatible pytest import namespace. These categories
overlap and must not be added as distinct tests. Historical failing XML and
diagnostic artifacts remain retained, not removed to make an aggregate green.
Independent final review and a new hosted run remain separate, unobserved gates.

## Current local verification — historical Version 1 (3 October 2026)

This section retains the original Version 1 receipt and its historical use of
"current". Its revisions, counts and timings are not the final fleet results.
Use the 9 October Task 16 table above for the final exact-tree local gates.

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
