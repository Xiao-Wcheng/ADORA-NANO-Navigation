#!/usr/bin/env python3
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
configs = [
    ROOT / "apps/adora_nano_navigation/localization_keyboard.yml",
    ROOT / "apps/adora_nano_navigation/localization_readonly.yml",
]
required = {
    "MAP_WIDTH: '89'",
    "MAP_HEIGHT: '85'",
    "ORIGIN_X: '-0.8'",
    "ORIGIN_Y: '-3.1'",
    "RESOLUTION: '0.05'",
}

for path in configs:
    text = path.read_text(encoding="utf-8")
    missing = sorted(token for token in required if token not in text)
    if missing:
        raise AssertionError(f"{path} has stale map metadata: {missing}")

print("Auxiliary localization map metadata: PASS")
