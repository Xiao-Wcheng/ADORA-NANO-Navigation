#!/usr/bin/env python3
from pathlib import Path
import os


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = (
    "install_dependencies.sh",
    "build_all.sh",
    "check_ready.sh",
    "stop_all.sh",
)


for name in SCRIPTS:
    path = ROOT / "scripts" / name
    if not path.is_file():
        raise AssertionError(f"missing root entry point: scripts/{name}")
    if not os.access(path, os.X_OK):
        raise AssertionError(f"entry point is not executable: scripts/{name}")
    text = path.read_text()
    if not text.startswith("#!/usr/bin/env bash\nset -euo pipefail\n"):
        raise AssertionError(f"entry point is not strict bash: scripts/{name}")
    if 'BASH_SOURCE[0]' not in text:
        raise AssertionError(f"entry point does not derive its own location: scripts/{name}")
    lowered = text.lower()
    if "apt install ros-" in lowered or "setup.bash" in lowered or "ros2" in lowered:
        raise AssertionError(f"ROS dependency in scripts/{name}")

stop_script = (ROOT / "scripts/stop_all.sh").read_text()
if "/tmp/feetech_kiwi_keyboard_fifo" in stop_script and "timeout " not in stop_script:
    raise AssertionError("stop_all.sh can block forever while opening an unread keyboard FIFO")

build = (ROOT / "scripts/build_all.sh").read_text()
for required in (
    "driver/ms200_dora_node",
    "chassis/feetech_kiwi_chassis_dora_node",
    "localization/initial_pose_dora_node",
    "localization/karto_slam_dora_node",
    "planning/pose_goal_dora_node",
    "planning/adora_nano_global_planner_node",
    "planning/rpp_local_controller_dora_node",
    "planning/velocity_smoother_dora_node",
    "planning/nav_supervisor_dora_node",
):
    if required not in build:
        raise AssertionError(f"build entry point misses {required}")

for retired in (
    "planning/dwb_local_planner_dora_node",
    "planning/mppi_local_planner_dora_node",
):
    if (ROOT / retired).exists():
        raise AssertionError(f"retired planner directory remains: {retired}")
    if retired in build:
        raise AssertionError(f"retired planner remains in build entry point: {retired}")

print("standalone entry points: PASS")
