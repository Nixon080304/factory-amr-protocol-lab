# Measured validation results

The evidence uses real Gazebo rendering, Nav2/AMCL, ROS 2 DDS, Mosquitto, and a
deterministic Modbus TCP PLC simulator. A test's total wall time includes setup,
assertions, and cleanup. Mission duration instead comes from the correlated
`mission_started` and `mission_finished` source timestamps in simulation time.

## Fresh successful mission

The recording uses product source revision
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

## Evidence and clock boundaries

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
for the same current selection used by the hosted workflow. Hosted workflow
success remains unobserved until publication and an actual workflow run.

## Limits and diagnostics

Full Gazebo verification is local-only and requires a working rendering display.
The captured RViz view includes the map, AMCL localization, LiDAR scan, global
and local costmaps, Nav2 plan, current goal, and camera station labels. Labels
use configured station positions; they are not measured 3D marker poses.

Actual logs include Xacro escape warnings, Nav2's default goal-checker warning,
and RViz GLSL sampler, covariance, and message-filter diagnostics. Some Python
adapters also report `rcl_shutdown already called on the given context` during
SIGINT teardown. This shutdown concern does not alter the measured mission
result; owned process/service cleanup is checked independently. Rendering on
every graphics driver is not guaranteed.

Version 1 supports one fixed part and route. A successful pickup consumes the
lifecycle; another distinct mission fails with `RESTART_REQUIRED` until the
full simulation restarts. Same-ID replay remains supported. Registries are
process-local, and a hung transfer gateway can leave a stopped pending mission
with unknown PLC state. No general crash recovery or production security model
is claimed. See [architecture](../architecture.md) and
[protocol contracts](../protocols.md) for the exact safety boundaries.
