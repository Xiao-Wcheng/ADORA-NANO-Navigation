# Adora Nano Dora Navigation

Pure-Dora 2D mapping, localization, and autonomous navigation for the Adora Nano robot. This directory is self-contained except for Ubuntu system packages and a separately installed Dora runtime. ROS is not required.

## Quick start

```bash
cd ~/adora_nano_dora_navigation
./scripts/install_dependencies.sh --check
./scripts/build_all.sh
./scripts/check_ready.sh --mode localize
python3 apps/adora_nano_navigation/adora_nav.py nav \
  --localize --relative 0.30 -0.15 --global-relative
```

Stop the robot software safely:

```bash
./scripts/stop_all.sh
```

For mapping and full operating procedures, read `docs/OPERATIONS.md`. For every retained directory and node, read `docs/COMPONENTS.md`. Open-source provenance and license locations are in `docs/OPEN_SOURCE.md` and `LICENSES/README.md`.

## Runtime chain

```text
MS200 LaserScan + chassis Odometry
        -> Karto mapping/localization -> CorrectedPose
        -> Boost.Graph A* -> global path
        -> Nav2 RPP ROS-free port -> local velocity
        -> navigation supervisor -> SafeCmdVelTwist -> chassis
```

The navigation supervisor is the only node connected to the chassis velocity input. It publishes zero velocity while waiting, blocked, timed out, localization-lost, or after a terminal state is latched.

## Active map

The portable map set is stored under `mapping/maps/`:

- `ms200_keyboard_map.pgm`: occupancy grid image.
- `ms200_keyboard_map.yaml`: resolution and map origin.
- `ms200_keyboard_map.metadata.json`: capture/calibration metadata.
- `ms200_keyboard_map.posegraph.dora`: Karto localization graph.

## Supported workflows

- New or continued keyboard-controlled mapping.
- Map finalization and quality reporting.
- Saved-map localization with explicit initial pose support.
- Relative or absolute goal navigation through global A* and local RPP.
- Periodic replanning and SLAM-navigation variants.
- Terminal-state command gating and safe shutdown.
