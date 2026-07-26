# Validation report

Date: 2026-07-24 (Asia/Shanghai)

## Closeout scope

The standalone project now uses one production local controller:
`planning/rpp_local_controller_dora_node`. The retired DWB and MPPI source,
build, configuration, and generated-run artifacts were removed from the
deliverable. Historical design documents under `docs/superpowers/` are
non-production records.

## Build

`./scripts/build_all.sh` completed successfully for:

- MS200 lidar driver and bounds test.
- Keyboard controller and Rust Feetech Kiwi chassis driver.
- Initial-pose node.
- Karto/OpenKarto/Ceres mapping and localization node.
- Pose-goal adapter.
- Boost.Graph A* global planner.
- Nav2 RPP ROS-free local controller.
- Velocity smoother.
- Navigation Supervisor and chassis command gate.

The MS200 vendor CMake project emitted deprecation warnings and the Rust chassis
driver emitted one dead-code warning; neither affected the successful build.

## Automated tests

- MS200 driver: 1/1 PASS.
- Karto/OpenKarto: 20/20 PASS.
- Boost.Graph A*, smoothing, and self-filter: 3/3 PASS.
- Nav2 RPP port: 2/2 PASS.
- Velocity smoother: 1/1 PASS.
- Navigation Supervisor: 2/2 PASS, including terminal command gating.
- Total compiled algorithm/driver tests: 29/29 PASS.
- Root entry-point, standalone-path, Karto dataflow, RPP production wiring,
  and strict standalone-manifest tests: PASS.
- Strict manifest: PASS, 236 retained source/configuration/documentation/map files.

The closeout also reproduced and fixed a shutdown defect: writing to an unread
keyboard FIFO could block `stop_all.sh` indefinitely. The FIFO notification is
now bounded by a timeout, and the regression contract passes.

## Dora dataflow validation

The following production YAML files passed Dora input/output and type
validation without starting hardware:

- `adora_nano_mapping.yml`
- `adora_nano_navigation.yml`
- `adora_nano_localization_navigation.yml`
- `adora_nano_slam_navigation.yml`

The command chain is:

```text
RPP CmdVelTwist
  -> velocity smoother
  -> nav_supervisor/CmdVelTwist
  -> nav_supervisor/SafeCmdVelTwist
  -> chassis/CmdVelTwist
```

The Supervisor only forwards commands in a non-terminal `RUNNING` state.
`REACHED`, `TIMEOUT`, and `LOCALIZATION_LOST` latch a zero safe command until a
different goal is received or the flow stops.

## Dependency and readiness validation

- `./scripts/check_ready.sh`: `READY`.
- Dora CLI: `/home/ubuntu2204/dora-main/target/release/dora`.
- Current map YAML/PGM: present; known-cell coverage 37.47%.
- Both configured serial-by-id hardware paths: present.
- ROS-linked-library scan: `ROS_FREE=PASS`.
- No ROS runtime is required.

## Physical acceptance evidence

No motor dataflow was started during the 2026-07-24 closeout.

The immediately preceding RPP physical run navigated to position 1 at
`(-0.993, -0.330, 3.126)` with a final localized position error of
approximately 0.044–0.047 m. Localization remained healthy
(`loc_lost=0`), the Supervisor latched `REACHED`, and the project was then
stopped. This evidence covers the current 0.15 m robot radius, 0.03 m safety
margin, 0.15 m static-map inflation, and RPP production wiring.

## Installation

- Canonical path: `/home/ubuntu2204/adora_nano_dora_navigation`.
- Install dependencies: `./scripts/install_dependencies.sh`.
- Build: `./scripts/build_all.sh`.
- Verify: `./scripts/check_ready.sh`.
- No Git commit was created during closeout, per operator request.
