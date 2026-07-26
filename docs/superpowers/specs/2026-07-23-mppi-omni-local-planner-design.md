# Pure-Dora Nav2 MPPI Omni Local Planner Design

Date: 2026-07-23 (Asia/Shanghai)

## Objective

Replace the production DWB local planner with a ROS-free adaptation of the
mature open-source Navigation2 MPPI controller. The controller must run as a
Dora C++ node, support the Adora Nano Kiwi omnidirectional chassis, command
translation and rotation concurrently, avoid lidar-observed obstacles, and
require both position and heading accuracy before declaring the goal reached.

The project must not require a ROS installation, ROS runtime, ROS messages,
TF, pluginlib, lifecycle nodes, ament, or a ROS workspace. No Git commit is
part of this work.

## Acceptance Criteria

- The controller can output non-zero `vx`, `vy`, and `wz` in the same command.
- The Omni motion model respects configured velocity and acceleration limits.
- Lidar-observed collision trajectories are rejected before command output.
- The controller follows the Boost.Graph A* global path while optimizing local
  obstacle clearance and target progress.
- A goal is not reached until Euclidean position error is at most `0.08 m` and
  wrapped yaw error is at most `0.12 rad`.
- When position is within tolerance but yaw is not, the controller performs a
  collision-checked heading correction instead of reporting success.
- Localization loss, stale pose, stale path, stale scan, no valid trajectory,
  supervisor timeout, or terminal supervisor state results in zero chassis
  velocity.
- The navigation supervisor remains the only producer connected to the
  chassis `CmdVelTwist` input.
- The production project contains no DWB executable, source directory, YAML
  path, build reference, runtime reference, or DWB-specific documentation after
  MPPI verification succeeds.
- Production executables have no ROS-linked libraries and do not access the
  former source repository at runtime.

## Selected Approach

Adapt the Navigation2 MPPI algorithm core rather than implementing a new
sampling controller inspired by MPPI. Preserve the upstream algorithmic
structure and behavior where they do not depend on ROS:

- omnidirectional motion model;
- batched noisy control-sequence sampling;
- forward trajectory prediction;
- critic-based batch scoring;
- softmax-weighted control update;
- rolling reuse of the optimal control sequence;
- optimal-trajectory collision validation.

Replace ROS integration only:

- ROS messages become small C++17 POD types;
- TF transforms are unnecessary because Karto supplies map-frame pose and the
  adapter transforms path and lidar data explicitly;
- costmap inputs become the existing lidar-derived rolling local costmap;
- pluginlib critic loading becomes a documented static critic composition;
- ROS parameters become environment variables in Dora YAML;
- publishers and subscriptions become Dora node inputs and outputs.

## Architecture and Boundaries

Create `planning/mppi_local_planner_dora_node/` with focused units:

- `types`: pose, twist, path, scan obstacle, trajectory, control sequence, and
  status POD types.
- `omni_motion_model`: velocity/acceleration constraints and simultaneous
  `vx/vy/wz` forward integration.
- `noise_generator`: deterministic seeded Gaussian samples for tests and a
  configurable runtime seed.
- `trajectory_batch`: batched forward prediction over the control horizon.
- `costmap`: rolling robot-centered occupancy and inflation built from MS200
  scans using the calibrated lidar transform.
- `critics`: path-follow, path-align, goal-distance, goal-angle, obstacle,
  velocity-deadband, control-effort, and smoothness costs.
- `optimizer`: MPPI softmax update, sequence shifting, retry, and optimal
  trajectory validation.
- `goal_checker`: the authoritative local position/yaw tolerance decision.
- `mppi_local_planner_dora_node`: JSON parsing, freshness checks, coordinate
  conversion, Dora events, status serialization, and zero-command fail-safe.

Algorithm code must not include Dora headers. Only the Dora adapter links the
Dora C API. This keeps the controller independently testable.

## Data Flow and Interfaces

The node keeps the current production wiring contract:

Inputs:

- `path` from `path_planning_node/path`;
- `LaserScan` from `ms200/LaserScan`;
- `CorrectedPose` from `karto_slam/CorrectedPose`;
- `GoalPose` from `pose_goal/goal_pose`;
- a 100 ms Dora timer tick.

Outputs:

- `CmdVelTwist`, containing `linear.x`, `linear.y`, and `angular.z`;
- `LocalPlannerStatus`, containing mode, reason, pose/goal errors, selected
  command, trajectory validity, obstacle clearance, and localization health.

The command chain remains:

```text
mppi_local_planner/CmdVelTwist
  -> nav_supervisor/CmdVelTwist
  -> nav_supervisor/SafeCmdVelTwist
  -> chassis/CmdVelTwist
```

The supervisor must also apply the same position and yaw goal tolerances. A
terminal state stays latched until a different goal arrives.

## Controller Behavior

At each control tick the adapter:

1. Rejects stale or unhealthy inputs and emits zero velocity.
2. Transforms the relevant global path segment and lidar points into the
   robot-local planning frame.
3. Builds and inflates the rolling local costmap.
4. Samples control sequences for all three axes around the shifted previous
   optimum.
5. Applies the Omni motion model to predict trajectory batches.
6. Marks collision trajectories invalid and evaluates the remaining critics.
7. Performs the MPPI softmax update and revalidates the selected trajectory.
8. Rate-limits the first `vx/vy/wz` command and publishes it through Dora.

Near the goal, goal-distance and goal-angle costs increase in influence. When
position is within `0.08 m` and yaw error exceeds `0.12 rad`, translational
commands are reduced or zeroed while collision-checked angular correction
continues. `GOAL_REACHED` is emitted only when both tolerances pass, followed by
a zero command.

## Initial Hardware Limits

The first real-robot configuration is deliberately conservative:

- `vx`: `-0.045 .. 0.045 m/s`;
- `vy`: `-0.045 .. 0.045 m/s`;
- `wz`: `-0.20 .. 0.20 rad/s`;
- linear acceleration: at most `0.10 m/s^2` per axis;
- angular acceleration: at most `0.50 rad/s^2`;
- control period: `0.10 s`;
- position tolerance: `0.08 m`;
- yaw tolerance: `0.12 rad`;
- robot radius: `0.17 m` plus configured safety margin;
- supervisor navigation timeout: `180000 ms`.

Batch size and horizon will be selected by benchmark on the Ubuntu host. The
controller must sustain the 10 Hz control deadline with margin before physical
motion is enabled.

## Safety and Failure Handling

- Every non-running state publishes explicit zero velocity.
- The previous command is never reused when pose, scan, path, or localization
  health is stale.
- No valid trajectory produces `BLOCKED/no_valid_trajectory` and zero velocity.
- The selected trajectory receives a final independent collision check.
- Non-finite input, cost, or command values produce zero velocity and a fault
  status.
- The supervisor continues to latch localization loss, timeout, and terminal
  states and remains the final safety gate.
- DWB remains present but disconnected during initial MPPI development. It is
  deleted only after automated, Dora dry-run, bench, and real-robot MPPI tests
  pass.

## Test Strategy

Development follows test-first red/green cycles.

Unit tests cover:

- simultaneous non-zero `vx/vy/wz` Omni integration;
- velocity and acceleration constraint enforcement;
- deterministic noise generation;
- batch dimensions and sequence shifting;
- obstacle collision and inflation;
- each critic's intended preference;
- finite softmax updates under extreme costs;
- final optimal-trajectory validation;
- combined position/yaw goal checking, including angle wrapping;
- zero output for stale data and localization loss.

Integration tests feed recorded/synthetic Dora JSON inputs through the adapter
and verify status and command output. Project tests verify YAML wiring, build
entry points, standalone paths, licenses, no ROS-linked libraries, and removal
of all production DWB references.

Physical verification proceeds in gates:

1. wheels lifted: simultaneous translation/rotation command and emergency stop;
2. clear floor: short pose goal with heading change;
3. clear floor: the recorded absolute goal `(-0.634, -0.185, 1.545)` with a
   `180 s` timeout;
4. temporary obstacle: safe stop or collision-free local avoidance;
5. repeated goal: position and yaw tolerance plus latched zero velocity.

## Open-Source Provenance

The implementation must record the exact Navigation2 repository revision and
the upstream MPPI package files used. The upstream license must be copied into
the MPPI directory, and `UPSTREAM.md` plus `MODIFICATIONS.md` must distinguish
unchanged algorithm concepts from ROS-removal adaptations. Any additional
dependency must be installable without ROS and documented in
`scripts/install_dependencies.sh` and `docs/OPEN_SOURCE.md`.

## Migration and Cleanup

During development, generated YAML selects MPPI while DWB remains available
only as rollback source. After all acceptance gates pass:

- replace every production local-planner path with the MPPI executable;
- update root and application build scripts;
- update component, operation, interface, open-source, and validation docs;
- update the strict standalone manifest;
- delete `planning/dwb_local_planner_dora_node/` and its generated build output;
- rescan the project for `dwb`, ROS libraries, and old repository paths;
- retain no Git commit, as explicitly requested by the user.
