# Open-source dense global path and Nav2 SimpleSmoother for RPP

## Problem

The custom `SmoothPathLineOfSight()` stage collapses a valid Boost A* path from
52 grid poses to two endpoints. The RPP local window then contains one pose,
so RPP reports `invalid_path` and safely commands zero velocity.

## Design

- Delete the custom line-of-sight path compression.
- Keep the dense path produced by Boost Graph Library A*.
- Port the Apache-2.0 Nav2 `SimpleSmoother` core to standalone C++ without ROS
  lifecycle, plugin, message, or costmap dependencies.
- Preserve the official update formula and defaults: tolerance `1e-10`, 1000
  iterations, data weight `0.2`, smooth weight `0.3`, two refinements.
- Validate each smoothed pose against the existing inflated `MapLoader`; roll
  back to the last collision-free path when a smoothing update is infeasible.
- Represent smoothed grid poses as doubles and update the RPP JSON adapter to
  preserve fractional coordinates instead of truncating to integers.
- Keep the standalone Nav2 RPP port responsible for pruning, lookahead,
  velocity regulation, and projected collision checks.

## Safety and compatibility

- Keep existing JSON field names and Dora wiring.
- Preserve start/goal nearest-free adjustment and obstacle-aware A*.
- Preserve zero velocity on stale, missing, invalid, or colliding paths.
- Preserve path endpoints and number of poses.
- Include upstream attribution, URL, and Apache-2.0 notices.

## Success criteria

- Unit tests prove dense-point preservation, endpoint preservation, smoothing,
  collision rollback, and fractional JSON coordinate preservation.
- The position1 global plan contains more than two poses.
- RPP receives at least two local poses and no longer reports `invalid_path`.
- All builds, CTest suites, Python tests, Dora validation, and readiness checks
  pass.
- No Git commit is created.
