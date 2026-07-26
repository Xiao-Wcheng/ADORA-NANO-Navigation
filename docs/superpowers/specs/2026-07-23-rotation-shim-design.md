# Pure Dora Rotation Shim Design

## Goal

Prevent the omni-drive robot from taking a large translation arc when the
global path initially points far away from the current body heading. Align the
robot to the first usable path segment before handing control to MPPI, matching
the behavior pattern of Nav2's Rotation Shim Controller without requiring ROS.

## Scope

- Pure Dora C++ implementation inside `mppi_local_planner_dora_node`.
- No ROS runtime or ROS messages.
- No changes to the saved map, Karto localization, A*, robot radius, or safety
  margin.
- No Git commit.

## State and Thresholds

The local planner adds a `ROTATE_TO_PATH` mode in front of the existing MPPI
controller.

- Enter rotation shim when absolute path-heading error exceeds `0.75 rad`.
- Exit rotation shim when absolute path-heading error is below `0.35 rad`.
- The separate enter and exit thresholds provide hysteresis.
- Rotation command has zero linear velocity: `vx=0`, `vy=0`.
- Angular speed is proportional to heading error and clamped to `0.20 rad/s`.
- A minimum useful angular speed is applied outside the exit threshold so the
  chassis does not stall in its deadband.

Thresholds and angular limits are configurable through environment variables,
with the values above as defaults.

## Path Heading

The desired heading is computed from the first non-degenerate segment of the
current body-frame global path. Degenerate points are skipped. If no usable
segment exists, the shim does not generate a command and the existing planner
health path handles the invalid path.

Whenever a new path arrives, the heading is evaluated again. Hysteresis keeps
small replanning changes from repeatedly switching modes.

## Safety

Before publishing an in-place rotation command, the planner verifies that the
current robot center has continuous-distance clearance greater than
`robot_radius + safety_margin`.

Because the current robot model is circular, rotation does not change its
occupied area. If current clearance is insufficient, output is zero and the
planner reports `BLOCKED`; it must not attempt translation to escape.

Stale pose, scan, path, localization loss, and goal completion retain priority
over the rotation shim.

## Control Flow

At each fixed 100 ms control tick:

1. Validate pose, scan, path, localization, and goal state.
2. Compute path-heading error.
3. Update the rotation-shim state using the enter/exit thresholds.
4. If rotating and current circular footprint is collision-free, publish only
   angular velocity and report `ROTATE_TO_PATH`.
5. Otherwise run the existing MPPI optimizer.
6. Near the final position, retain the existing goal-yaw alignment behavior.

## Tests

Automated tests cover:

- Entering shim above `0.75 rad`.
- Remaining in shim inside the hysteresis band.
- Exiting below `0.35 rad`.
- Correct shortest-turn direction across the `-pi/pi` boundary.
- Zero `vx` and `vy` while rotating.
- Angular speed clamp and deadband floor.
- Blocking rotation when continuous clearance is insufficient.
- Hand-off to MPPI after alignment.
- Existing MPPI, costmap, localization, global planner, supervisor, and
  velocity-smoother regression suites.

## Acceptance

On the real robot at the approximate map origin, navigation to position 1 must:

- rotate in place before substantial translation when the path is behind it;
- avoid the previous large outward arc;
- show a decreasing target-distance trend after alignment;
- stop safely on localization loss or true insufficient clearance;
- proceed under MPPI after the heading error falls below `0.35 rad`.
