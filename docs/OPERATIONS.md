# Operations

All commands start from the standalone project:

```bash
cd ~/adora_nano_dora_navigation
```

## Install and build

Check dependencies without changing the machine:

```bash
./scripts/install_dependencies.sh --check
```

Install missing Ubuntu packages (Dora itself is installed separately):

```bash
./scripts/install_dependencies.sh
```

Build every node and retained test:

```bash
./scripts/build_all.sh
```

## Readiness

```bash
./scripts/check_ready.sh --mode mapping
./scripts/check_ready.sh --mode navigation
./scripts/check_ready.sh --mode localize
```

Expected hardware paths are configured in the YAML files under `apps/adora_nano_navigation`. Use `/dev/serial/by-id` names so USB enumeration changes do not swap lidar and chassis.

## New mapping

Place the robot at the desired map origin, leave clearance around it, then run:

```bash
python3 apps/adora_nano_navigation/adora_nav.py map --clean
```

The keyboard node latches motion after a key press; press space to stop. Drive slowly around the boundary, revisit the starting area for loop closure, then stop the complete flow:

```bash
./scripts/stop_all.sh
```

Finalize and inspect the map:

```bash
python3 apps/adora_nano_navigation/adora_nav.py finalize-map --padding 0.50
python3 apps/adora_nano_navigation/adora_nav.py quality
```

Continue an existing map/pose graph with:

```bash
python3 apps/adora_nano_navigation/adora_nav.py map --continue
```

## Initial pose and localization

Put the robot near the mapped reference location and start localization navigation with a short safe goal. If an explicit pose is required, use `localization/initial_pose_dora_node/set_initial_pose.py --help` for the FIFO command syntax.

Dry-run validation:

```bash
python3 apps/adora_nano_navigation/adora_nav.py nav \
  --localize --relative 0.30 -0.15 --global-relative --dry-run
```

## Autonomous navigation

Relative target: forward is positive; lateral is positive left and negative right.

```bash
python3 apps/adora_nano_navigation/adora_nav.py nav \
  --localize --relative 0.30 -0.15 --global-relative
```

Absolute map target:

```bash
python3 apps/adora_nano_navigation/adora_nav.py nav \
  --localize --absolute 0.50 -0.30 --theta 0.0
```

Periodic global replanning:

```bash
python3 apps/adora_nano_navigation/adora_nav.py nav \
  --localize --relative 0.50 0.00 --global-relative --replan
```

SLAM navigation, which updates the map/graph:

```bash
python3 apps/adora_nano_navigation/adora_nav.py nav \
  --slam --relative 0.30 0.00 --global-relative
```

## Status and logs

```bash
python3 apps/adora_nano_navigation/adora_nav.py status
python3 apps/adora_nano_navigation/summarize_localization.py --help
```

Important states are `RUNNING`, `BLOCKED`, `REACHED`, `TIMEOUT`, and `LOCALIZATION_LOST`. `REACHED`, `TIMEOUT`, and localization-loss terminal outcomes latch until a different `GoalPose` arrives.

## Normal and emergency stop

```bash
./scripts/stop_all.sh
```

If a terminal is attached to keyboard mapping, press space first. The chassis also has a command watchdog, but the supervisor and stop script are the primary software stop paths.

## Troubleshooting order

1. `./scripts/check_ready.sh --mode localize`: executable, map, port, wiring failure.
2. MS200 log: connected, activated, scan frequency and timestamp freshness.
3. Chassis log: motors initialized and odometry changing under command.
4. Karto log/status: localization matched and lost count.
5. Global planner: `path_found` and map inflation.
6. RPP: `TRACK_PATH`, `ALIGN_GOAL`, `GOAL_REACHED`, or a collision/blocking reason.
7. Supervisor: distance, terminal state, and command gate.
