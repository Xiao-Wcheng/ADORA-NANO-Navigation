#!/usr/bin/env python3
import json
from datetime import datetime, timezone
from pathlib import Path


APP_DIR = Path(__file__).resolve().parent
ROOT = APP_DIR.parent.parent
MAP_DIR = ROOT / "mapping" / "maps"
OUT = MAP_DIR / "ms200_navigation_metadata.json"


def main():
    MAP_DIR.mkdir(parents=True, exist_ok=True)
    metadata = {
        "created_at": datetime.now(timezone.utc).isoformat(),
        "robot": "Adora Nano / Feetech Kiwi three-wheel chassis",
        "lidar": "MS200P",
        "map": {
            "yaml": "ms200_keyboard_map.yaml",
            "pgm": "ms200_keyboard_map.pgm",
            "resolution": 0.05,
            "width": 400,
            "height": 400,
            "origin": [-10.0, -10.0, 0.0],
        },
        "pose_graph": "ms200_keyboard_map.posegraph.dora",
        "lidar_extrinsic": {
            "x": 0.09,
            "y": 0.06,
            "yaw": 0.0,
        },
        "chassis": {
            "wheel_ids": [13, 14, 15],
            "wheel_angles_deg": [60, 180, 300],
            "linear_ticks_per_mps": 13350,
            "angular_ticks_per_radps": 1988,
            "odom_source": "feedback",
        },
    }
    tmp = OUT.with_suffix(OUT.suffix + ".tmp")
    tmp.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    tmp.replace(OUT)
    print(f"wrote {OUT}")


if __name__ == "__main__":
    raise SystemExit(main())
