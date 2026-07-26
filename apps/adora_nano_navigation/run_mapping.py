#!/usr/bin/env python3
import argparse
import shutil
import subprocess
import sys
from datetime import datetime
from pathlib import Path

try:
    import yaml
except ImportError:
    print("python3-yaml is required for this helper", file=sys.stderr)
    sys.exit(2)

from sync_map_metadata import sync_current_map_metadata


APP_DIR = Path(__file__).resolve().parent
ROOT = APP_DIR.parent.parent
DEFAULT_DORA = Path.home() / "dora-main" / "target" / "release" / "dora"
MAP_DIR = ROOT / "mapping" / "maps"
MAP_ARTIFACTS = [
    "ms200_keyboard_map.pgm",
    "ms200_keyboard_map.yaml",
    "ms200_keyboard_map.metadata.json",
    "ms200_keyboard_map.posegraph.dora",
]


def absolutize_paths(data):
    env_path_keys = {"MAP_PATH_PREFIX"}
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

def backup_current_map(tag):
    backup = MAP_DIR / "backups" / f"{datetime.now().strftime('%Y%m%d-%H%M%S')}-{tag}"
    backup.mkdir(parents=True, exist_ok=True)
    moved = []
    for name in MAP_ARTIFACTS:
        src = MAP_DIR / name
        if src.exists():
            shutil.move(str(src), str(backup / name))
            moved.append(name)
    if moved:
        print(f"backed up current map artifacts to: {backup}")
    else:
        print("no existing map artifacts to back up")

def run_check(args):
    cmd = [sys.executable, str(APP_DIR / "check_navigation_ready.py"),
           "--mode", "mapping", "--dora", args.dora, "--skip-validate"]
    return subprocess.call(cmd, cwd=str(APP_DIR))

def validate_generated(args, generated):
    cmd = [args.dora, "validate", str(generated)]
    return subprocess.call(cmd, cwd=str(APP_DIR))

def main():
    parser = argparse.ArgumentParser(description="Start Adora Nano Dora-native map building.")
    parser.add_argument("--dry-run", action="store_true", help="only write generated mapping yaml")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--clean", action="store_true", help="backup current artifacts and start a new map")
    mode.add_argument("--continue", dest="continue_map", action="store_true", help="continue from the current .posegraph.dora archive")
    parser.add_argument("--force", action="store_true", help="skip preflight checks")
    parser.add_argument("--name", default="adora-nano-mapping", help="Dora flow name")
    parser.add_argument("--dora", default=str(DEFAULT_DORA), help="path to dora CLI")
    args = parser.parse_args()

    if args.clean:
        backup_current_map("before-mapping")
    if args.continue_map and not (MAP_DIR / "ms200_keyboard_map.posegraph.dora").exists():
        print("cannot continue: ms200_keyboard_map.posegraph.dora is missing", file=sys.stderr)
        return 2

    template = APP_DIR / "adora_nano_mapping.yml"
    with template.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    absolutize_paths(data)
    for node in data.get("nodes", []):
        if node.get("id") == "karto_slam":
            node.setdefault("env", {})["SLAM_MODE"] = "continue_mapping" if args.continue_map else "new_mapping"

    generated = APP_DIR / "generated_mapping.yml"
    with generated.open("w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False)
    print(f"generated: {generated}")

    if not args.force and run_check(args) != 0:
        print("preflight failed; pass --force to skip checks", file=sys.stderr)
        return 2
    if validate_generated(args, generated) != 0:
        print("generated mapping dataflow is invalid", file=sys.stderr)
        return 2

    if args.dry_run:
        return 0

    cmd = [args.dora, "start", "--attach", "-n", args.name, str(generated)]
    result = subprocess.call(cmd, cwd=str(APP_DIR))
    if result == 0:
        sync_current_map_metadata(ROOT)
    return result


if __name__ == "__main__":
    raise SystemExit(main())
