#!/usr/bin/env python3
from pathlib import Path


APP_DIR = Path(__file__).resolve().parents[1]

run_mapping = (APP_DIR / "run_mapping.py").read_text(encoding="utf-8")
finalize_map = (APP_DIR / "finalize_map.py").read_text(encoding="utf-8")
entrypoint = (APP_DIR / "adora_nav.py").read_text(encoding="utf-8")

if "sync_current_map_metadata(ROOT)" not in run_mapping:
    raise AssertionError("mapping completion does not synchronize map metadata")
if "sync_current_map_metadata(ROOT)" not in finalize_map:
    raise AssertionError("map finalization does not synchronize map metadata")
if 'sub.add_parser("sync-map"' not in entrypoint:
    raise AssertionError("manual sync-map entrypoint is missing")

print("Map metadata synchronization wiring: PASS")
