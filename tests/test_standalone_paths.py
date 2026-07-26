#!/usr/bin/env python3
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
APP = ROOT / "apps/adora_nano_navigation"
PRODUCTION_FLOWS = (
    "adora_nano_mapping.yml",
    "adora_nano_navigation.yml",
    "adora_nano_localization_navigation.yml",
    "adora_nano_slam_navigation.yml",
)
REQUIRED_COMPONENTS = (
    "driver/ms200_dora_node",
    "chassis/feetech_kiwi_chassis_dora_node",
    "localization/initial_pose_dora_node",
    "localization/karto_slam_dora_node",
    "planning/pose_goal_dora_node",
    "planning/adora_nano_global_planner_node",
    "planning/rpp_local_controller_dora_node",
    "planning/velocity_smoother_dora_node",
    "planning/nav_supervisor_dora_node",
    "third_party/slam_toolbox_core",
    "mapping/maps/ms200_keyboard_map.yaml",
)


def fail(message: str) -> None:
    raise AssertionError(message)


for relative in REQUIRED_COMPONENTS:
    if not (ROOT / relative).exists():
        fail(f"required standalone path missing: {relative}")

for retired in (
    "planning/dwb_local_planner_dora_node",
    "planning/mppi_local_planner_dora_node",
):
    if (ROOT / retired).exists():
        fail(f"retired planner directory remains: {retired}")

for flow_name in PRODUCTION_FLOWS:
    flow_path = APP / flow_name
    data = yaml.safe_load(flow_path.read_text())
    text = flow_path.read_text()
    forbidden_repo = "dora-rs-" + "DORA_NAV"
    forbidden_home = "/home/" + "ubuntu2204"
    if forbidden_repo in text or forbidden_home in text:
        fail(f"external repository path in {flow_name}")
    if (
        "body_relative_controller" in text
        or "planning/local_planner_dora_node" in text
        or "dwb_local_planner_dora_node" in text
        or "mppi_local_planner_dora_node" in text
    ):
        fail(f"retired planner reference in {flow_name}")
    if "rpp_local_controller_dora_node" not in text and flow_name != "adora_nano_mapping.yml":
        fail(f"RPP controller missing from {flow_name}")
    for node in data.get("nodes", []):
        executable = node.get("path")
        if isinstance(executable, str):
            resolved = (APP / executable).resolve()
            if not resolved.is_relative_to(ROOT):
                fail(f"path escapes standalone root in {flow_name}: {executable}")
            if not resolved.is_file():
                fail(f"node executable is not built inside standalone root in {flow_name}: {executable}")
        for key, value in node.get("env", {}).items():
            if key in {"MAP_PREFIX", "MAP_YAML_PATH"} and isinstance(value, str):
                resolved = (APP / value).resolve()
                if not resolved.is_relative_to(ROOT / "mapping/maps"):
                    fail(f"map path is outside mapping/maps in {flow_name}: {value}")

runner = (APP / "run_navigation.py").read_text()
checker = (APP / "check_navigation_ready.py").read_text()
builder = (APP / "build_navigation_stack.sh").read_text()
global_adapter = (ROOT / "planning/adora_nano_global_planner_node/src/dora_planning_node.cpp").read_text()
for retired in ("adora_nano_body_relative", "body_relative_controller_dora_node",
                'planning/local_planner_dora_node', "adora_nano_replanning_navigation"):
    if retired in runner + checker + builder:
        fail(f"retired runtime component remains referenced: {retired}")
if "dijkstra" in global_adapter.lower():
    fail("retired custom Dijkstra branch remains in the Boost A* adapter")
if (APP / "adora_nano_replanning_navigation.yml").exists():
    fail("obsolete scan-matching replanning dataflow still exists")

print("standalone paths: PASS")
