#!/usr/bin/env python3
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
templates = [
    ROOT / "apps/adora_nano_navigation/adora_nano_navigation.yml",
    ROOT / "apps/adora_nano_navigation/adora_nano_localization_navigation.yml",
    ROOT / "apps/adora_nano_navigation/adora_nano_slam_navigation.yml",
]
required = {
    "../../planning/rpp_local_controller_dora_node/build/rpp_local_controller_dora_node",
    "RPP_DESIRED_LINEAR_VEL: '0.05'",
    "RPP_MIN_LOOKAHEAD_DIST: '0.20'",
    "RPP_MAX_LOOKAHEAD_DIST: '0.45'",
    "RPP_MAX_ANGULAR_SPEED: '0.23'",
    "ROTATION_SHIM_MAX_ANGULAR_SPEED: '0.23'",
    "MAX_LINEAR_SPEED: '0.055'",
    "MAX_ANGULAR_SPEED: '0.23'",
    "MAX_LINEAR_ACCEL: '0.12'",
    "MAX_ANGULAR_ACCEL: '0.60'",
    "MAX_VX: '0.055'",
    "MAX_WZ: '0.23'",
    "MAX_ACCEL_X: '0.12'",
    "MAX_ACCEL_WZ: '0.60'",
    "MAX_DECEL_X: '-0.12'",
    "MAX_DECEL_WZ: '-0.60'",
    "ROBOT_RADIUS: '0.15'",
    "SAFETY_MARGIN: '0.03'",
    "DYNAMIC_OBSTACLE_INFLATION_M: '0.18'",
    "INFLATION_RADIUS_CELLS: '3'",
}
for path in templates:
    text = path.read_text(encoding="utf-8")
    missing = sorted(token for token in required if token not in text)
    if missing:
        raise AssertionError(f"{path} missing RPP settings: {missing}")
    if "../../planning/mppi_local_planner_dora_node/build/" in text:
        raise AssertionError(f"production MPPI reference remains in {path}")
    if "../../planning/dwb_local_planner_dora_node/build/" in text:
        raise AssertionError(f"production DWB reference remains in {path}")
print("RPP production wiring: PASS")
