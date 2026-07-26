#!/usr/bin/env python3
import argparse
import json
import math
import re
from collections import Counter
from pathlib import Path


APP_DIR = Path(__file__).resolve().parent
ROOT = APP_DIR.parent.parent
MAP_DIR = ROOT / "mapping" / "maps"
DEFAULT_PGM = MAP_DIR / "ms200_keyboard_map.pgm"
DEFAULT_GRAPH = MAP_DIR / "ms200_keyboard_map.posegraph.dora"


def read_pgm(path):
    data = path.read_bytes()
    match = re.match(rb"P5\s+(?:#.*\n\s*)*(\d+)\s+(\d+)\s+(\d+)\s", data, re.S)
    if not match:
        raise ValueError(f"unsupported PGM format: {path}")
    width, height, max_value = map(int, match.groups())
    pixels = data[match.end():match.end() + width * height]
    if len(pixels) != width * height:
        raise ValueError(f"truncated PGM payload: {path}")
    return width, height, max_value, pixels


def map_stats(path, resolution):
    width, height, _, pixels = read_pgm(path)
    counter = Counter(pixels)
    occupied = sum(v for k, v in counter.items() if k < 100)
    free = sum(v for k, v in counter.items() if k > 240)
    unknown = len(pixels) - occupied - free
    known = occupied + free
    known_indices = [i for i, value in enumerate(pixels) if value != 205]
    bbox = None
    if known_indices:
        xs = [i % width for i in known_indices]
        ys = [i // width for i in known_indices]
        bbox = ((max(xs) - min(xs) + 1) * resolution,
                (max(ys) - min(ys) + 1) * resolution)
    return {
        "width": width,
        "height": height,
        "occupied": occupied,
        "free": free,
        "unknown": unknown,
        "known_percent": 100.0 * known / len(pixels),
        "bbox_m": bbox,
    }


def graph_stats(path):
    graph = json.loads(path.read_text(encoding="utf-8"))
    if graph.get("format") == "dora-karto-posegraph":
        payload = graph.get("payload", {})
        scans = payload.get("scans", [])
        constraint_items = payload.get("constraints", [])
        solver = payload.get("solver", {})
        points = []
        for item in scans:
            pose = item.get("optimized_pose", []) if isinstance(item, dict) else []
            if isinstance(pose, list) and len(pose) == 3:
                points.append(tuple(pose))
        loop_constraints = sum(
            1 for item in constraint_items
            if isinstance(item, dict) and item.get("category") == "loop_closure")
        odom_constraints = sum(
            1 for item in constraint_items
            if isinstance(item, dict) and item.get("category") == "sequential")
        graph = {
            "num_keyframes": len(scans),
            "num_constraints": len(constraint_items),
            "loop_closures": solver.get("loop_closure_count", loop_constraints),
            "backend": graph.get("source_tag", "dora-karto"),
            "last_optimization": {
                "iterations": solver.get("iterations", 0),
                "mean_error": solver.get("final_cost", 0.0),
                "max_error": solver.get("final_cost", 0.0),
                "status": solver.get("status", "not_run"),
            },
            "constraints": [
                {"type": "loop" if item.get("category") == "loop_closure" else
                 "odom" if item.get("category") == "sequential" else "near_chain"}
                for item in constraint_items if isinstance(item, dict)
            ],
            "keyframes": [
                {"x": pose[0], "y": pose[1], "theta": pose[2]}
                for pose in points
            ],
        }
    keyframes = graph.get("num_keyframes", len(graph.get("keyframes", [])))
    constraints = graph.get("num_constraints", len(graph.get("constraints", [])))
    loop_closures = graph.get("loop_closures", 0)
    backend = graph.get("backend", "unknown")
    optimization = graph.get("last_optimization", {})
    constraint_items = graph.get("constraints", [])
    loop_constraints = 0
    odom_constraints = 0
    if isinstance(constraint_items, list):
        for item in constraint_items:
            ctype = item.get("type", "") if isinstance(item, dict) else ""
            if str(ctype).startswith("loop"):
                loop_constraints += 1
            elif ctype == "odom":
                odom_constraints += 1
    points = [(item.get("x"), item.get("y"), item.get("theta", 0.0))
              for item in graph.get("keyframes", [])
              if isinstance(item, dict) and "x" in item and "y" in item]
    trajectory_bbox = None
    start = None
    end = None
    if points:
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        trajectory_bbox = (max(xs) - min(xs), max(ys) - min(ys))
        start = points[0]
        end = points[-1]
    return {
        "keyframes": keyframes,
        "constraints": constraints,
        "loop_closures": loop_closures,
        "backend": backend,
        "optimization": optimization if isinstance(optimization, dict) else {},
        "odom_constraints": odom_constraints,
        "loop_constraints": loop_constraints,
        "trajectory_bbox_m": trajectory_bbox,
        "start": start,
        "end": end,
    }


def main():
    parser = argparse.ArgumentParser(description="Report Adora Nano SLAM map/pose-graph quality.")
    parser.add_argument("--pgm", default=str(DEFAULT_PGM))
    parser.add_argument("--graph", default=str(DEFAULT_GRAPH))
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--min-known-percent", type=float, default=3.0)
    parser.add_argument("--min-keyframes", type=int, default=40)
    parser.add_argument("--min-trajectory-span", type=float, default=1.0)
    parser.add_argument("--strict", action="store_true", help="return non-zero when quality is weak")
    args = parser.parse_args()

    pgm = Path(args.pgm)
    graph = Path(args.graph)
    ok = True
    if not pgm.exists():
        print(f"MISS map: {pgm}")
        return 2
    if not graph.exists():
        print(f"MISS pose graph: {graph}")
        return 2

    ms = map_stats(pgm, args.resolution)
    gs = graph_stats(graph)

    print(f"map: {pgm}")
    print(f"  size: {ms['width']}x{ms['height']} resolution={args.resolution}")
    print(f"  occupied={ms['occupied']} free={ms['free']} unknown={ms['unknown']}")
    print(f"  known={ms['known_percent']:.2f}%")
    if ms["bbox_m"]:
        print(f"  known_bbox={ms['bbox_m'][0]:.2f}m x {ms['bbox_m'][1]:.2f}m")

    print(f"pose_graph: {graph}")
    print(f"  backend={gs['backend']}")
    print(f"  keyframes={gs['keyframes']} constraints={gs['constraints']} loop_closures={gs['loop_closures']}")
    print(f"  odom_constraints={gs['odom_constraints']} loop_constraints={gs['loop_constraints']}")
    opt = gs["optimization"]
    if opt:
        print(f"  optimization_iterations={opt.get('iterations', 0)} "
              f"mean_error={float(opt.get('mean_error', 0.0)):.4f} "
              f"max_error={float(opt.get('max_error', 0.0)):.4f}")
    if gs["trajectory_bbox_m"]:
        print(f"  trajectory_bbox={gs['trajectory_bbox_m'][0]:.2f}m x {gs['trajectory_bbox_m'][1]:.2f}m")
    if gs["start"] and gs["end"]:
        print(f"  start=({gs['start'][0]:.2f},{gs['start'][1]:.2f},{math.degrees(gs['start'][2]):.1f}deg)")
        print(f"  end=({gs['end'][0]:.2f},{gs['end'][1]:.2f},{math.degrees(gs['end'][2]):.1f}deg)")

    if ms["known_percent"] < args.min_known_percent:
        print("WARN map known area is low")
        ok = False
    if gs["keyframes"] < args.min_keyframes:
        print("WARN pose graph has too few keyframes")
        ok = False
    if gs["keyframes"] > 20 and gs["constraints"] < gs["keyframes"] - 1:
        print("WARN pose graph is missing odom constraints")
        ok = False
    if gs["trajectory_bbox_m"] and max(gs["trajectory_bbox_m"]) < args.min_trajectory_span:
        print("WARN trajectory span is small")
        ok = False

    print("quality:", "OK" if ok else "WEAK")
    return 1 if args.strict and not ok else 0


if __name__ == "__main__":
    raise SystemExit(main())
