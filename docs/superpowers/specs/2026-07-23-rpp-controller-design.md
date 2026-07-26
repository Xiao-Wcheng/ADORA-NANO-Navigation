# Pure Dora Nav2 RPP Controller Design

Date: 2026-07-23

## Goal

Replace the incomplete MPPI local controller with a deterministic, ROS-free
port of Nav2 Regulated Pure Pursuit (RPP) for the Adora Nano Kiwi chassis.
Autonomous navigation will deliberately use differential-style motion:
forward/reverse translation plus rotation, with lateral velocity always zero.

## Constraints

- Dora remains the only runtime framework.
- No ROS 2 installation, runtime, messages, TF, pluginlib, or Costmap2D.
- Reuse the existing Dora JSON interfaces, localization, Boost A* global
  planner, rolling laser costmap, velocity smoother, supervisor, and chassis.
- Keep the existing circular collision radius of 0.17 m and 0.04 m safety
  margin.
- Preserve upstream source attribution and applicable Nav2 license text.
- Do not create a Git commit.

## Selected Architecture

The control pipeline is:

1. The global planner publishes a map-frame path.
2. A Nav2-style path handler finds the closest forward path point, prunes
   already-traversed poses, limits the local segment by integrated distance,
   and transforms that segment into the current robot frame.
3. The existing Rotation Shim rotates in place when the local path heading
   exceeds its entry threshold.
4. RPP selects a lookahead point, computes curvature, and produces `vx` and
   `wz`, with `vy` fixed to zero.
5. RPP regulates linear speed using curvature, obstacle proximity, approach
   distance, and configured velocity limits.
6. Forward simulation checks the complete commanded arc against the rolling
   laser costmap before publishing.
7. The existing velocity smoother applies acceleration limits and sends the
   command to the chassis.

## Components

### Path Handler

The path handler ports the behavioral structure of Nav2 RPP rather than using
the raw full global path on every cycle. It:

- searches only within a bounded integrated path distance for the closest pose;
- removes poses behind the robot after the closest pose is accepted;
- preserves a minimum useful local path length;
- transforms the pruned path into `base_footprint`;
- rejects empty, non-finite, disconnected, or stale paths;
- retains the last valid path only until its normal timeout, never indefinitely.

This prevents the controller from steering toward already-traversed path
segments after frequent global replans.

### Regulated Pure Pursuit Core

The controller uses a velocity-scaled lookahead distance clamped between
minimum and maximum bounds. It interpolates the carrot point on the local path,
computes curvature from the robot-frame carrot, and applies:

- desired linear velocity;
- curvature-based speed reduction;
- obstacle-cost/clearance-based speed reduction;
- approach-speed reduction near the goal;
- minimum regulated speed;
- maximum angular velocity;
- optional reverse motion only when the selected path direction requires it.

The production default disables arbitrary reverse selection: the controller
prefers forward tracking after Rotation Shim alignment. Every command satisfies
`vy == 0`.

### Rotation Shim

The existing Rotation Shim stays in front of RPP. It rotates in place when the
initial local path heading error is greater than 0.75 rad and returns control
when the error is below 0.35 rad. Rotation is collision checked at the current
footprint and capped at 0.20 rad/s.

### Collision Prediction

The controller projects the constant-curvature command over a configurable
time horizon using small time steps. Every projected pose is checked by the
existing exact continuous-distance rolling costmap. Any collision produces a
zero command and `BLOCKED/collision_ahead`.

### Supervisor Semantics

Global planning failure is a first-class failure state. When the global planner
publishes `path_found=false`, the local controller reports
`BLOCKED/no_global_path` immediately. The supervisor must not continue to label
this state as ordinary `RUNNING`. Motion may resume only after a newly published
valid path passes normal freshness and collision checks.

## Interfaces and Status

The Dora node executable path and input/output IDs remain unchanged so existing
dataflows do not need rewiring. Status adds:

- `controller_impl: nav2_regulated_pure_pursuit_port`
- pruned/local path point counts
- lookahead distance and carrot coordinates
- curvature
- applied velocity regulation factor
- collision projection result

Modes remain `ROTATE_TO_PATH`, `FOLLOW_PATH`, `ALIGN_GOAL`, `BLOCKED`,
`LOCALIZATION_LOST`, `GOAL_REACHED`, and `FAULT`.

## Error Handling

- Invalid or stale pose/scan/path: publish zero velocity.
- `path_found=false`: publish zero velocity and `BLOCKED/no_global_path`.
- Non-finite geometry or command: publish zero velocity and `FAULT`.
- Collision on projected arc: publish zero velocity and
  `BLOCKED/collision_ahead`.
- Localization lost: publish zero velocity and `LOCALIZATION_LOST`.
- No recovery behavior is allowed to move the robot without a valid path.

## Testing and Acceptance

Unit tests cover path pruning, robot-frame transformation, interpolated
lookahead, straight and curved commands, `vy=0`, curvature regulation, approach
regulation, collision projection, and no-path status propagation.

Offline integration tests replay:

- a path initially behind the robot;
- a path updated every second;
- a rectangular detour around an obstacle;
- transition from valid path to `path_found=false`;
- goal position followed by final-yaw alignment.

All existing Karto, global planner, supervisor, smoother, ROS-free, and readiness
checks must continue to pass.

Real-machine acceptance requires:

- initial in-place alignment without a large translation arc;
- monotonically meaningful progress along the pruned global path;
- no lateral command;
- stop on lost path or collision;
- arrival within the existing 0.05 m position and 0.035 rad yaw tolerances.
