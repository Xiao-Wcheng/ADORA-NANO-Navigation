# Robot Self-Return Filtering Design

## Goal

Prevent lidar returns from the robot body from entering the global dynamic
obstacle layer or MPPI rolling costmap, while retaining real obstacles outside
the footprint. Reduce the MPPI safety margin from 0.08 m to 0.04 m.

## Design

Lidar points are transformed into `base_footprint` coordinates first. A shared
axis-aligned rectangular footprint predicate rejects points inside the robot
body plus a small configurable self-filter padding. The same predicate is used
by the global planner and MPPI adapter so both planners see consistent obstacle
data.

Defaults describe the current Kiwi chassis and remain configurable through
environment variables. Collision inflation still uses the circular
`ROBOT_RADIUS`; only sensor returns originating inside the physical footprint
are removed.

## Verification

Unit tests cover inside, boundary, and outside points. An optimizer regression
proves body returns no longer invalidate every trajectory, while a genuine
external obstacle still blocks colliding trajectories. Existing planner and
supervisor tests must remain green.
