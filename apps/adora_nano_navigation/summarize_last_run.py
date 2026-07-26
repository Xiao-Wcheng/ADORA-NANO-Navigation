#!/usr/bin/env python3
from pathlib import Path


APP_DIR = Path(__file__).resolve().parent
OUT_DIR = APP_DIR / "out"


def latest_run_dir():
    dirs = [p for p in OUT_DIR.iterdir() if p.is_dir()]
    if not dirs:
      return None
    return max(dirs, key=lambda p: p.stat().st_mtime)


def tail_matches(path, patterns, limit=8):
    if not path.exists():
        return []
    hits = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if any(pattern in line for pattern in patterns):
            hits.append(line)
    return hits[-limit:]


def main():
    run_dir = latest_run_dir()
    if run_dir is None:
        print("No Dora run logs found.")
        return 1

    print(f"Latest run: {run_dir}")
    checks = [
        ("navigation", run_dir / "log_nav_supervisor.jsonl",
         ["nav state=", "LOCALIZATION_LOST", "POSE_TIMEOUT", "BLOCKED", "REACHED"]),
        ("local planner", run_dir / "log_local_planner.jsonl",
         ["local_planner mode=", "localization_lost", "pose_status_timeout"]),
        ("map localization", run_dir / "log_map_localization.jsonl",
         ["map_localization matched="]),
        ("pose graph", run_dir / "log_pose_graph_slam.jsonl",
         ["localization matched=", "keyframes", "loop closure"]),
    ]

    for title, path, patterns in checks:
        print(f"\n[{title}]")
        lines = tail_matches(path, patterns)
        if not lines:
            print("  no matching lines")
            continue
        for line in lines:
            print(line)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
