#!/usr/bin/env python3
"""Synchronize auxiliary localization dataflows with the saved map metadata."""

import argparse
import tempfile
from pathlib import Path

import yaml


APP_DIR = Path(__file__).resolve().parent
ROOT = APP_DIR.parent.parent
MAP_BASENAME = "ms200_keyboard_map"
AUXILIARY_CONFIGS = (
    "localization_keyboard.yml",
    "localization_readonly.yml",
)


def read_pgm_size(path):
    with path.open("rb") as stream:
        if stream.readline().strip() != b"P5":
            raise ValueError(f"only binary P5 PGM is supported: {path}")
        line = stream.readline()
        while line.startswith(b"#"):
            line = stream.readline()
        width, height = (int(value) for value in line.split())
    if width <= 0 or height <= 0:
        raise ValueError(f"invalid map dimensions: {width}x{height}")
    return width, height


def current_map_metadata(root):
    map_dir = root / "mapping/maps"
    yaml_path = map_dir / f"{MAP_BASENAME}.yaml"
    pgm_path = map_dir / f"{MAP_BASENAME}.pgm"
    data = yaml.safe_load(yaml_path.read_text(encoding="utf-8")) or {}
    resolution = float(data["resolution"])
    origin = data["origin"]
    if resolution <= 0.0 or not isinstance(origin, list) or len(origin) < 2:
        raise ValueError(f"invalid map metadata: {yaml_path}")
    width, height = read_pgm_size(pgm_path)
    return {
        "MAP_WIDTH": str(width),
        "MAP_HEIGHT": str(height),
        "ORIGIN_X": f"{float(origin[0]):.9g}",
        "ORIGIN_Y": f"{float(origin[1]):.9g}",
        "RESOLUTION": f"{resolution:.9g}",
    }


def find_pose_monitor(data, path):
    for node in data.get("nodes", []):
        if node.get("id") == "pose_monitor":
            return node
    raise ValueError(f"pose_monitor node is missing: {path}")


def write_yaml_atomically(path, data):
    with tempfile.NamedTemporaryFile(
        mode="w",
        dir=path.parent,
        prefix=path.name + ".",
        encoding="utf-8",
        delete=False,
    ) as stream:
        temporary = Path(stream.name)
        yaml.safe_dump(data, stream, sort_keys=False)
    temporary.replace(path)


def sync_current_map_metadata(root=ROOT, dry_run=False):
    root = Path(root)
    app_dir = root / "apps/adora_nano_navigation"
    metadata = current_map_metadata(root)
    for name in AUXILIARY_CONFIGS:
        path = app_dir / name
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        env = find_pose_monitor(data, path).setdefault("env", {})
        env.update(metadata)
        if not dry_run:
            write_yaml_atomically(path, data)

    generated = app_dir / "generated_navigation.yml"
    if generated.exists() and not dry_run:
        generated.unlink()

    print("map metadata synchronized: " + " ".join(
        f"{key}={value}" for key, value in metadata.items()
    ))
    if generated.exists():
        print(f"stale generated navigation would be removed: {generated}")
    elif not dry_run:
        print(f"stale generated navigation removed/absent: {generated}")
    return metadata


def main():
    parser = argparse.ArgumentParser(
        description="Synchronize localization helpers with the current saved map."
    )
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    sync_current_map_metadata(ROOT, args.dry_run)


if __name__ == "__main__":
    main()
