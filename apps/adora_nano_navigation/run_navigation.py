#!/usr/bin/env python3
import argparse
import subprocess
import sys
from pathlib import Path

try:
    import yaml
except ImportError:
    print("python3-yaml is required for this helper", file=sys.stderr)
    sys.exit(2)


APP_DIR = Path(__file__).resolve().parent
ROOT = APP_DIR.parent.parent
DEFAULT_DORA = Path.home() / "dora-main" / "target" / "release" / "dora"
MAP_YAML = ROOT / "mapping" / "maps" / "ms200_keyboard_map.yaml"
MAP_PGM = ROOT / "mapping" / "maps" / "ms200_keyboard_map.pgm"
POSE_GRAPH = ROOT / "mapping" / "maps" / "ms200_keyboard_map.posegraph.dora"


def find_node(data, node_id):
    for node in data.get("nodes", []):
        if node.get("id") == node_id:
            return node
    raise KeyError(f"node not found: {node_id}")


def read_pgm_size(path):
    with path.open("rb") as f:
        if f.readline().strip() not in {b"P5", b"P2"}:
            raise ValueError(f"unsupported PGM format: {path}")
        line = f.readline()
        while line.startswith(b"#"):
            line = f.readline()
        width, height = (int(value) for value in line.split())
    if width <= 0 or height <= 0:
        raise ValueError(f"invalid map dimensions: {width}x{height}")
    return width, height


def read_map_metadata(yaml_path, pgm_path):
    with yaml_path.open("r", encoding="utf-8") as f:
        metadata = yaml.safe_load(f) or {}
    resolution = float(metadata["resolution"])
    origin = metadata["origin"]
    if resolution <= 0.0 or not isinstance(origin, list) or len(origin) < 2:
        raise ValueError(f"invalid map metadata: {yaml_path}")
    width, height = read_pgm_size(pgm_path)
    return {
        "RESOLUTION": f"{resolution:.9g}",
        "MAP_WIDTH": str(width),
        "MAP_HEIGHT": str(height),
        "ORIGIN_X": f"{float(origin[0]):.9g}",
        "ORIGIN_Y": f"{float(origin[1]):.9g}",
    }


def apply_map_metadata(data, metadata):
    for node_id in ("pose_goal", "local_planner"):
        try:
            env = find_node(data, node_id).setdefault("env", {})
        except KeyError:
            continue
        for key, value in metadata.items():
            if node_id == "local_planner" and key == "MAP_WIDTH":
                continue
            env[key] = value


def set_goal(env, args):
    for key in ["GOAL_DISTANCE", "GOAL_LATERAL", "GOAL_X", "GOAL_Y", "GOAL_THETA"]:
        env.pop(key, None)

    if args.relative is not None:
        distance, lateral = args.relative
        env["GOAL_DISTANCE"] = f"{distance:.3f}"
        env["GOAL_LATERAL"] = f"{lateral:.3f}"
        return lateral

    x, y = args.absolute
    env["GOAL_X"] = f"{x:.3f}"
    env["GOAL_Y"] = f"{y:.3f}"
    env["GOAL_THETA"] = f"{args.theta:.3f}"
    return 0.0


def clear_static_goal(env):
    for key in ["GOAL_X", "GOAL_Y", "GOAL_THETA"]:
        env.pop(key, None)


def absolutize_paths(data):
    env_path_keys = {
        "MAP_YAML_PATH",
        "MAP_PATH_PREFIX",
        "GRAPH_PATH",
    }
    for node in data.get("nodes", []):
        path = node.get("path")
        if isinstance(path, str) and not Path(path).is_absolute():
            node["path"] = str((APP_DIR / path).resolve())
        env = node.get("env", {})
        if isinstance(env, dict):
            for key in env_path_keys:
                value = env.get(key)
                if isinstance(value, str) and value.startswith("."):
                    env[key] = str((APP_DIR / value).resolve())

def check_mode(args):
    if args.localize or args.replan:
        return "localize"
    if args.slam:
        return "slam"
    return "navigation"

def run_preflight(args):
    cmd = [sys.executable, str(APP_DIR / "check_navigation_ready.py"),
           "--mode", check_mode(args), "--dora", args.dora, "--skip-validate"]
    return subprocess.call(cmd, cwd=str(APP_DIR))

def validate_generated(args, generated):
    cmd = [args.dora, "validate", str(generated)]
    return subprocess.call(cmd, cwd=str(APP_DIR))


def main():
    parser = argparse.ArgumentParser(description="Generate and optionally start an Adora Nano navigation dataflow.")
    goal = parser.add_mutually_exclusive_group(required=True)
    goal.add_argument("--relative", nargs=2, type=float, metavar=("FORWARD_M", "LEFT_M"),
                      help="relative goal in robot frame; left is positive, right is negative")
    goal.add_argument("--absolute", nargs=2, type=float, metavar=("X_M", "Y_M"),
                      help="map/world-frame goal")
    parser.add_argument("--theta", type=float, default=0.0, help="absolute goal yaw, radians")
    parser.add_argument("--initial", nargs=3, type=float, metavar=("X_M", "Y_M", "YAW_RAD"),
                        help="required initial map pose for saved-map localization")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--replan", action="store_true", help="use periodic replanning dataflow")
    mode.add_argument("--slam", action="store_true", help="use mapping plus periodic replanning dataflow")
    mode.add_argument("--localize", action="store_true", help="use saved map and pose graph without updating the map")
    parser.add_argument("--global-relative", action="store_true",
                        help="compatibility option; relative goals always use global planning")
    parser.add_argument("--dry-run", action="store_true", help="only write generated yaml")
    parser.add_argument("--force", action="store_true", help="skip local safety checks")
    parser.add_argument("--name", default=None, help="Dora flow name")
    parser.add_argument("--dora", default=str(DEFAULT_DORA), help="path to dora CLI")
    args = parser.parse_args()

    if not args.slam and args.initial is None:
        parser.error("saved-map navigation requires --initial X Y YAW_RAD")

    if (args.localize or args.replan) and not args.force:
        missing = [str(p) for p in [MAP_YAML, MAP_PGM, POSE_GRAPH] if not p.exists()]
        if missing:
            print("localize mode requires saved map and pose graph. Missing:", file=sys.stderr)
            for path in missing:
                print(f"  {path}", file=sys.stderr)
            print("Run --slam/mapping first, or pass --force only if you know the files are generated elsewhere.", file=sys.stderr)
            return 2

    if args.slam and not args.force:
        print("warning: --slam mode updates the saved map and pose graph", file=sys.stderr)

    if args.slam:
        template = APP_DIR / "adora_nano_slam_navigation.yml"
    elif args.localize or args.replan:
        template = APP_DIR / "adora_nano_localization_navigation.yml"
    else:
        template = APP_DIR / "adora_nano_navigation.yml"
    with template.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    absolutize_paths(data)

    if MAP_YAML.exists() and MAP_PGM.exists():
        metadata = read_map_metadata(MAP_YAML, MAP_PGM)
        apply_map_metadata(data, metadata)
        print("map metadata: " + " ".join(f"{key}={value}" for key, value in metadata.items()))

    local_planner = find_node(data, "local_planner")
    pose_goal = find_node(data, "pose_goal")
    nav_supervisor = find_node(data, "nav_supervisor")
    initial_pose = find_node(data, "initial_pose_control")
    if args.initial is not None:
        initial_env = initial_pose.setdefault("env", {})
        initial_env["INITIAL_POSE_X"] = f"{args.initial[0]:.6f}"
        initial_env["INITIAL_POSE_Y"] = f"{args.initial[1]:.6f}"
        initial_env["INITIAL_POSE_YAW"] = f"{args.initial[2]:.6f}"
    lateral = set_goal(pose_goal.setdefault("env", {}), args)
    clear_static_goal(nav_supervisor.setdefault("env", {}))

    local_env = local_planner.setdefault("env", {})
    if args.relative is not None:
        if lateral > 0.002:
            local_env["FIXED_LATERAL_PREFERENCE"] = "1"
        elif lateral < -0.002:
            local_env["FIXED_LATERAL_PREFERENCE"] = "-1"
        else:
            local_env["FIXED_LATERAL_PREFERENCE"] = "0"
    else:
        local_env["FIXED_LATERAL_PREFERENCE"] = "0"

    generated = APP_DIR / "generated_navigation.yml"
    with generated.open("w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False)

    print(f"generated: {generated}")
    if not args.force and run_preflight(args) != 0:
        print("preflight failed; pass --force to skip checks", file=sys.stderr)
        return 2
    if validate_generated(args, generated) != 0:
        print("generated navigation dataflow is invalid", file=sys.stderr)
        return 2

    if args.dry_run:
        return 0

    flow_name = args.name
    if flow_name is None:
        if args.slam:
            flow_name = "adora-nano-slam-navigation"
        elif args.replan:
            flow_name = "adora-nano-replanning"
        elif args.localize:
            flow_name = "adora-nano-localization"
        else:
            flow_name = "adora-nano-navigation"

    cmd = [args.dora, "start", "--attach", "-n", flow_name, str(generated)]
    return subprocess.call(cmd, cwd=str(APP_DIR))


if __name__ == "__main__":
    raise SystemExit(main())
