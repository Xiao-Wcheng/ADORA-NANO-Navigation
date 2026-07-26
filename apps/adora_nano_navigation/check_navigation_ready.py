#!/usr/bin/env python3
import argparse
import json
import os
import subprocess
from pathlib import Path


APP_DIR = Path(__file__).resolve().parent
ROOT = APP_DIR.parent.parent

EXECUTABLES = [
    "driver/ms200_dora_node/build/ms200_dora_node",
    "localization/initial_pose_dora_node/build/initial_pose_dora_node",
    "localization/karto_slam_dora_node/build/karto_slam_dora_node",
    "planning/pose_goal_dora_node/build/pose_goal_dora_node",
    "planning/adora_nano_global_planner_node/build/adora_nano_global_planner_node",
    "planning/rpp_local_controller_dora_node/build/rpp_local_controller_dora_node",
    "planning/velocity_smoother_dora_node/build/velocity_smoother_dora_node",
    "planning/nav_supervisor_dora_node/build/nav_supervisor_dora_node",
    "chassis/feetech_kiwi_chassis_dora_node/sdk_node/target/release/feetech-kiwi-chassis-sdk-node",
]

MAP_FILES = [
    "mapping/maps/ms200_keyboard_map.yaml",
    "mapping/maps/ms200_keyboard_map.pgm",
]

POSE_GRAPH = "mapping/maps/ms200_keyboard_map.posegraph.dora"
LIDAR_PORT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AA6084348-if00"
CHASSIS_PORT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AE6086267-if00"
DEFAULT_DORA = Path.home() / "dora-main" / "target" / "release" / "dora"

DATAFLOW_BY_MODE = {
    "mapping": "apps/adora_nano_navigation/adora_nano_mapping.yml",
    "navigation": "apps/adora_nano_navigation/adora_nano_navigation.yml",
    "slam": "apps/adora_nano_navigation/adora_nano_slam_navigation.yml",
    "localize": "apps/adora_nano_navigation/adora_nano_localization_navigation.yml",
}


def check_path(path, executable=False):
    full = ROOT / path
    ok = full.exists() and (not executable or os.access(full, os.X_OK))
    print(("OK   " if ok else "MISS ") + str(full))
    return ok


def port_busy(path):
    if not Path(path).exists():
        return None
    result = subprocess.run(["fuser", path], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
    return bool(result.stdout.strip())

def validate_dataflow(dora, mode):
    dataflow = ROOT / DATAFLOW_BY_MODE[mode]
    if not dataflow.exists():
        print(f"MISS {dataflow}")
        return False
    result = subprocess.run([str(dora), "validate", str(dataflow)],
                            cwd=str(APP_DIR), stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True)
    ok = result.returncode == 0
    print(("OK   " if ok else "FAIL ") + f"dora validate {dataflow.name}")
    if not ok:
        print(result.stdout.rstrip())
    return ok

def summarize_pose_graph():
    full = ROOT / POSE_GRAPH
    if not full.exists():
        return
    try:
        graph = json.loads(full.read_text(encoding="utf-8"))
    except Exception as exc:
        print(f"WARN pose graph cannot be parsed: {exc}")
        return
    keyframes = graph.get("num_keyframes", len(graph.get("keyframes", [])))
    constraints = graph.get("num_constraints", len(graph.get("constraints", [])))
    loops = graph.get("loop_closures", 0)
    print(f"INFO pose_graph keyframes={keyframes} constraints={constraints} loop_closures={loops}")
    points = [(k.get("x"), k.get("y")) for k in graph.get("keyframes", [])
              if isinstance(k, dict) and "x" in k and "y" in k]
    if points:
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        print(f"INFO trajectory_bbox_m={max(xs)-min(xs):.2f}x{max(ys)-min(ys):.2f}")
    if keyframes < 20:
        print("WARN pose graph has few keyframes; navigation may be weak")

def summarize_map():
    full = ROOT / MAP_FILES[1]
    if not full.exists():
        return
    try:
        data = full.read_bytes()
        header_end = data.find(b"\n255\n")
        if header_end < 0:
            return
        payload = data[header_end + 5:]
        if not payload:
            return
        unknown = payload.count(205)
        known = len(payload) - unknown
        known_pct = 100.0 * known / len(payload)
        print(f"INFO map_known={known_pct:.2f}%")
        if known_pct < 3.0:
            print("WARN map coverage is low; global navigation may plan poorly")
    except Exception as exc:
        print(f"WARN map cannot be summarized: {exc}")

def main():
    parser = argparse.ArgumentParser(description="Check Adora Nano navigation files and hardware ports.")
    parser.add_argument("--mode", choices=["mapping", "navigation", "slam", "localize"], default="navigation")
    parser.add_argument("--dora", default=str(DEFAULT_DORA), help="path to dora CLI")
    parser.add_argument("--skip-validate", action="store_true", help="skip dora validate")
    args = parser.parse_args()

    ok = True
    dora = Path(args.dora)
    print("Dora CLI:")
    if dora.exists() and os.access(dora, os.X_OK):
        print(f"OK   {dora}")
    else:
        print(f"MISS {dora}")
        ok = False

    print("Executables:")
    for item in EXECUTABLES:
        ok = check_path(item, executable=True) and ok

    if args.mode in {"navigation", "slam", "localize"}:
        print("\nMap files:")
        for item in MAP_FILES:
            ok = check_path(item) and ok
        summarize_map()

    if args.mode == "localize":
        print("\nPose graph:")
        ok = check_path(POSE_GRAPH) and ok
        summarize_pose_graph()
    elif args.mode == "slam" and (ROOT / POSE_GRAPH).exists():
        print("\nPose graph:")
        summarize_pose_graph()

    print("\nHardware ports:")
    for port in [LIDAR_PORT, CHASSIS_PORT]:
        exists = Path(port).exists()
        busy = port_busy(port)
        if not exists:
            print(f"MISS {port}")
            ok = False
        elif busy:
            print(f"BUSY {port}")
            ok = False
        else:
            print(f"OK   {port}")

    if not args.skip_validate:
        print("\nDora dataflow:")
        ok = validate_dataflow(dora, args.mode) and ok

    print("\nResult:", "READY" if ok else "NOT READY")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
