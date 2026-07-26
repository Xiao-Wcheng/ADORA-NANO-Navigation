# Goal Tolerance Design

## Goal

Reduce the software navigation arrival error while preserving stable behavior on
the existing 0.05 m occupancy grid.

## Design

All production navigation templates use one consistent position tolerance of
0.05 m and one consistent yaw tolerance of 0.035 rad. Both the MPPI local
planner and navigation supervisor receive these same values so neither component
can declare success before the other.

The map resolution, obstacle safety margin, controller velocity limits, waypoint
coordinates, and localization parameters remain unchanged. No robot process is
started during this configuration change.

## Verification

Verify all six production-template occurrences, generate a navigation flow with
the existing dry-run path, and run the complete project build. A later physical
navigation run is required to measure real-world repeatability.
