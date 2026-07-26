#!/usr/bin/env python3
"""Send an initial localization pose to the running pure-Dora navigation flow."""

import argparse
import json
import math
import os


def main() -> None:
    parser = argparse.ArgumentParser(description="Set Dora localization initial pose")
    parser.add_argument("x", type=float, help="map-frame x in metres")
    parser.add_argument("y", type=float, help="map-frame y in metres")
    parser.add_argument("yaw", type=float, help="heading in radians")
    parser.add_argument("--fifo", default="/tmp/dora_initial_pose_fifo")
    args = parser.parse_args()

    if not all(math.isfinite(v) for v in (args.x, args.y, args.yaw)):
        parser.error("x, y and yaw must be finite")
    if not os.path.exists(args.fifo):
        parser.error(f"initial-pose FIFO does not exist: {args.fifo}; start localization first")

    payload = json.dumps({"x": args.x, "y": args.y, "yaw": args.yaw}, separators=(",", ":"))
    with open(args.fifo, "w", encoding="utf-8") as fifo:
        fifo.write(payload + "\n")
    print(f"InitialPose queued: {payload}")


if __name__ == "__main__":
    main()
