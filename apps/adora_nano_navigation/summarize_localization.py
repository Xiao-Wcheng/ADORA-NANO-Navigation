#!/usr/bin/env python3
import argparse
import json
import math
from pathlib import Path


def angle_delta(a, b):
    return math.atan2(math.sin(b - a), math.cos(b - a))


def summarize(path):
    poses = []
    summary = {}
    with Path(path).open(encoding="utf-8") as stream:
        for line in stream:
            record = json.loads(line)
            if record.get("type") == "pose":
                poses.append(record)
            elif record.get("type") == "summary":
                summary = record
    if not poses:
        raise ValueError("localization log contains no pose records")
    distance = sum(math.hypot(b["pose"]["x"] - a["pose"]["x"],
                              b["pose"]["y"] - a["pose"]["y"])
                   for a, b in zip(poses, poses[1:]))
    start, end = poses[0]["pose"], poses[-1]["pose"]
    return {
        "samples": len(poses),
        "trajectory_length_m": distance,
        "start_end_distance_m": math.hypot(end["x"] - start["x"], end["y"] - start["y"]),
        "start_end_heading_deg": math.degrees(angle_delta(start["yaw"], end["yaw"])),
        "minimum_match_response": min(p["match_response"] for p in poses),
        "maximum_covariance_xx": max(p["covariance"]["xx"] for p in poses),
        "maximum_covariance_yy": max(p["covariance"]["yy"] for p in poses),
        "maximum_covariance_yaw": max(p["covariance"]["yaw_yaw"] for p in poses),
        "loss_events": summary.get("loss_events", sum(p.get("lost_count", 0) > 0 for p in poses)),
        "rejected_scans": summary.get("rejected_scans", 0),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("path", nargs="?", default="out/localization_quality.jsonl")
    args = parser.parse_args()
    print(json.dumps(summarize(args.path), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
