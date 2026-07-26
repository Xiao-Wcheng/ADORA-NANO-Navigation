# Component inventory

| Path | Responsibility | Runtime executable / data | Inputs | Outputs | Main dependencies |
|---|---|---|---|---|---|
| `apps/adora_nano_navigation` | Generates, validates, starts, summarizes, and stops Dora flows | Python tools and production YAML | CLI arguments, maps, node binaries | Generated dataflow and flow UUID | Python 3, PyYAML, Dora CLI |
| `driver/ms200_dora_node` | Reads and normalizes MS200 lidar scans | `build/ms200_dora_node` | MS200 serial packets | `LaserScan` | Oradar vendor SDK, Dora C API |
| `chassis/feetech_kiwi_chassis_dora_node` | Three-wheel Kiwi motor control and feedback odometry | Rust SDK chassis node | `SafeCmdVelTwist`, servo feedback | `Odometry` | Rust, Feetech serial protocol, Dora |
| `chassis/.../src/keyboard_control_dora_node.cpp` | Latched keyboard driving for mapping; space stops | `build/keyboard_control_dora_node` | Terminal keys | `CmdVelTwist` | Dora C API |
| `localization/initial_pose_dora_node` | Injects an operator-selected initial map pose | `build/initial_pose_dora_node` | FIFO commands | `InitialPose` | Dora C API |
| `localization/karto_slam_dora_node` | OpenKarto-based mapping, pose-graph optimization, map localization, synchronization, and map export | `build/karto_slam_dora_node` | `LaserScan`, `Odometry`, `InitialPose` | `CorrectedPose`, SLAM/map status | OpenKarto core, Ceres, Eigen, Dora |
| `third_party/slam_toolbox_core` | ROS-free imported Karto SDK core from slam_toolbox | static library | prepared scans and odometry | graph constraints and optimized poses | C++17 |
| `planning/pose_goal_dora_node` | Converts relative/absolute goals and corrected poses to planner inputs | `build/pose_goal_dora_node` | `CorrectedPose`, replan request | start cell, goal cell, exact `GoalPose` | Dora C API |
| `planning/adora_nano_global_planner_node` | Occupancy-map loading, inflation, Boost.Graph A*, path smoothing | `build/adora_nano_global_planner_node` | map, start/goal, dynamic scan obstacles | global path | Boost.Graph, yaml-cpp, Dora |
| `planning/rpp_local_controller_dora_node` | ROS-free Nav2 Regulated Pure Pursuit path tracking, speed regulation, rotation shim, and projected collision checking | `build/rpp_local_controller_dora_node` | exact goal, global path, pose, scan | local velocity and status | Nav2-derived C++ port, Dora |
| `planning/velocity_smoother_dora_node` | Limits command acceleration and smooths controller output | `build/velocity_smoother_dora_node` | RPP velocity command | smoothed velocity command | C++17, Dora |
| `planning/nav_supervisor_dora_node` | Navigation state, timeout/replan policy, terminal latch, chassis command gate | `build/nav_supervisor_dora_node` | pose, exact goal, RPP command/status | `SafeCmdVelTwist`, navigation status, replan request | Dora C API |
| `mapping/maps` | Current portable map and pose graph | PGM/YAML/JSON/Dora archive | mapper output | localization/global-planner input | Karto node |
| `scripts` | Root-level install, build, readiness, and shutdown entry points | executable shell scripts | command-line options | builds/check reports | Bash |
| `tests` and component `tests/` | Structural and algorithm regression tests | Python/CTest binaries | source tree and fixtures | pass/fail evidence | Python, CTest |

Generated `build/`, Rust `target/`, `out/`, generated YAML, sessions, caches, and logs are intentionally excluded from the source layout and can be regenerated.
