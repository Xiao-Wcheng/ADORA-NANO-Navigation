# Open-source algorithm provenance

This project uses mature open-source algorithms through ROS-free C++ adapters. No ROS runtime, ROS messages, pluginlib, TF, ament, or ROS workspace is required.

## OpenKarto / slam_toolbox

- Upstream project: `slam_toolbox`
- Release: `2.6.9`
- Commit: `a69c0fea1d66ff65893e022df950adb58f2a165e`
- Imported role: Karto scan matching, graph construction, constraints, and mapping core.
- Local integration: Dora messages, odometry/scan timestamp synchronization, Ceres pose-graph solver, map/pose-graph archive, localization mode.
- Records: `third_party/slam_toolbox_core/UPSTREAM.md` and `MODIFICATIONS.md`.
- Licenses: `third_party/slam_toolbox_core/LICENSE-KARTO` and `LICENSE-SLAM-TOOLBOX`.

## Nav2 Regulated Pure Pursuit

- Upstream project: Navigation2 `nav2_regulated_pure_pursuit_controller`.
- Reference date: 2026-07-23.
- Imported role: velocity-scaled lookahead, interpolated carrot selection, pure-pursuit curvature, curvature and approach speed regulation, rotate-to-path behavior, and projected-command collision checking.
- Local integration: Dora inputs, plain C++ types, environment configuration, rolling laser costmap, exact `GoalPose`, velocity smoothing, and Supervisor terminal command gating.
- Records: `planning/rpp_local_controller_dora_node/UPSTREAM.md`.
- License: `planning/rpp_local_controller_dora_node/LICENSE.nav2-rpp-apache-2.0`.

## Boost.Graph A*

The global planner uses Boost.Graph's mature `astar_search` implementation over the occupancy grid. Local code provides map loading, obstacle inflation, graph construction, admissible Euclidean heuristic, and path smoothing. Boost is installed as an Ubuntu dependency and retains its upstream Boost Software License.

## Ceres Solver and Eigen

Ceres Solver performs nonlinear pose-graph optimization and Eigen provides linear algebra. Both are Ubuntu/system dependencies rather than copied source trees.
