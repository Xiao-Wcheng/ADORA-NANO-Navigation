#!/usr/bin/env python3
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
MANIFEST = ROOT / "docs/standalone_navigation_manifest.txt"
REQUIRED_ROOTS = {
    "apps/adora_nano_navigation",
    "chassis/feetech_kiwi_chassis_dora_node",
    "driver/ms200_dora_node",
    "localization/initial_pose_dora_node",
    "localization/karto_slam_dora_node",
    "planning/pose_goal_dora_node",
    "planning/adora_nano_global_planner_node",
    "planning/rpp_local_controller_dora_node",
    "planning/velocity_smoother_dora_node",
    "planning/nav_supervisor_dora_node",
    "third_party/slam_toolbox_core",
}
FORBIDDEN_PARTS = {
    "build",
    "target",
    "out",
    "__pycache__",
    "backups",
    ".git",
}


def fail(message: str) -> None:
    raise AssertionError(message)


if not MANIFEST.is_file():
    fail(f"manifest missing: {MANIFEST}")

entries = [line.strip() for line in MANIFEST.read_text().splitlines()
           if line.strip() and not line.lstrip().startswith("#")]
if not entries:
    fail("manifest is empty")
if entries != sorted(set(entries)):
    fail("manifest entries must be sorted and unique")

seen_roots = set()
for entry in entries:
    path = Path(entry)
    if path.is_absolute() or ".." in path.parts:
        fail(f"unsafe manifest path: {entry}")
    if any(part in FORBIDDEN_PARTS for part in path.parts):
        fail(f"generated or historical path in manifest: {entry}")
    if any(part.startswith("generated_") for part in path.parts):
        fail(f"generated path in manifest: {entry}")
    if not (ROOT / path).is_file():
        fail(f"listed source file does not exist: {entry}")
    for required in REQUIRED_ROOTS:
        if entry == required or entry.startswith(required + "/"):
            seen_roots.add(required)

missing = sorted(REQUIRED_ROOTS - seen_roots)
if missing:
    fail(f"required component roots missing: {missing}")

print(f"standalone manifest: PASS ({len(entries)} files)")
