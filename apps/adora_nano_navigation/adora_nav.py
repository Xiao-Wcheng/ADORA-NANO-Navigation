#!/usr/bin/env python3
import argparse
import shutil
import subprocess
import sys
from pathlib import Path


APP_DIR = Path(__file__).resolve().parent
ROOT = APP_DIR.parent.parent
DEFAULT_DORA = Path.home() / "dora-main" / "target" / "release" / "dora"


def run(cmd, cwd=APP_DIR):
    print("+ " + " ".join(str(x) for x in cmd))
    return subprocess.call([str(x) for x in cmd], cwd=str(cwd))


def clean_runtime_files():
    for path in [
        APP_DIR / "generated_mapping.yml",
        APP_DIR / "generated_navigation.yml",
        APP_DIR / "out",
        APP_DIR / "__pycache__",
    ]:
        if path.is_dir():
            shutil.rmtree(path)
            print(f"removed: {path}")
        elif path.exists():
            path.unlink()
            print(f"removed: {path}")
    return 0


def main():
    parser = argparse.ArgumentParser(description="One-command entry point for the Adora Nano Dora navigation project.")
    parser.add_argument("--dora", default=str(DEFAULT_DORA), help="path to dora CLI")
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("build", help="build all navigation stack nodes")

    check = sub.add_parser("check", help="check executables, maps, ports, and generated dataflow")
    check.add_argument("--mode", choices=["mapping", "navigation", "slam", "localize"], default="navigation")

    mapping = sub.add_parser("map", help="start mapping")
    map_mode = mapping.add_mutually_exclusive_group()
    map_mode.add_argument("--clean", action="store_true", help="backup current map/graph and start a new map")
    map_mode.add_argument("--continue", dest="continue_map", action="store_true", help="load the current Dora pose graph and continue mapping")
    mapping.add_argument("--dry-run", action="store_true")
    mapping.add_argument("--force", action="store_true")

    nav = sub.add_parser("nav", help="navigate to a relative or absolute goal")
    nav_goal = nav.add_mutually_exclusive_group(required=True)
    nav_goal.add_argument("--relative", nargs=2, type=float, metavar=("FORWARD_M", "LEFT_M"))
    nav_goal.add_argument("--absolute", nargs=2, type=float, metavar=("X_M", "Y_M"))
    nav.add_argument("--theta", type=float, default=0.0)
    nav.add_argument("--initial", nargs=3, type=float,
                     metavar=("X_M", "Y_M", "YAW_RAD"),
                     help="initial map pose required for saved-map localization")
    nav.add_argument("--replan", action="store_true")
    nav.add_argument("--slam", action="store_true")
    nav.add_argument("--localize", action="store_true")
    nav.add_argument("--global-relative", action="store_true",
                     help="compatibility option; relative goals always use global planning")
    nav.add_argument("--dry-run", action="store_true")
    nav.add_argument("--force", action="store_true")

    sub.add_parser("stop", help="stop all running Dora flows")
    sub.add_parser("status", help="summarize latest navigation logs")
    sub.add_parser("quality", help="report current map and pose-graph quality")
    finalize = sub.add_parser("finalize-map", help="crop unknown map borders and update its origin")
    finalize.add_argument("--padding", type=float, default=0.50, help="padding around known cells in meters")
    finalize.add_argument("--dry-run", action="store_true")
    sync_map = sub.add_parser("sync-map", help="synchronize localization helpers with the saved map")
    sync_map.add_argument("--dry-run", action="store_true")
    sub.add_parser("clean", help="remove generated runtime files")

    args = parser.parse_args()

    if args.command == "build":
        return run(["bash", "apps/adora_nano_navigation/build_navigation_stack.sh"], cwd=ROOT)

    if args.command == "check":
        return run([sys.executable, APP_DIR / "check_navigation_ready.py",
                    "--mode", args.mode, "--dora", args.dora])

    if args.command == "map":
        cmd = [sys.executable, APP_DIR / "run_mapping.py", "--dora", args.dora]
        if args.clean:
            cmd.append("--clean")
        if args.continue_map:
            cmd.append("--continue")
        if args.dry_run:
            cmd.append("--dry-run")
        if args.force:
            cmd.append("--force")
        return run(cmd)

    if args.command == "nav":
        cmd = [sys.executable, APP_DIR / "run_navigation.py", "--dora", args.dora]
        if args.relative is not None:
            cmd += ["--relative", str(args.relative[0]), str(args.relative[1])]
        else:
            cmd += ["--absolute", str(args.absolute[0]), str(args.absolute[1]), "--theta", str(args.theta)]
        if args.initial is not None:
            cmd += ["--initial", str(args.initial[0]), str(args.initial[1]),
                    str(args.initial[2])]
        if args.replan:
            cmd.append("--replan")
        if args.slam:
            cmd.append("--slam")
        if args.localize:
            cmd.append("--localize")
        if args.global_relative:
            cmd.append("--global-relative")
        if args.dry_run:
            cmd.append("--dry-run")
        if args.force:
            cmd.append("--force")
        return run(cmd)

    if args.command == "stop":
        return run(["bash", APP_DIR / "stop_all_navigation.sh"])

    if args.command == "status":
        return run([sys.executable, APP_DIR / "summarize_last_run.py"])

    if args.command == "quality":
        return run([sys.executable, APP_DIR / "map_quality_report.py"])

    if args.command == "finalize-map":
        cmd = [sys.executable, APP_DIR / "finalize_map.py", "--padding", str(args.padding)]
        if args.dry_run:
            cmd.append("--dry-run")
        return run(cmd)

    if args.command == "sync-map":
        cmd = [sys.executable, APP_DIR / "sync_map_metadata.py"]
        if args.dry_run:
            cmd.append("--dry-run")
        return run(cmd)

    if args.command == "clean":
        return clean_runtime_files()

    return 2


if __name__ == "__main__":
    raise SystemExit(main())
