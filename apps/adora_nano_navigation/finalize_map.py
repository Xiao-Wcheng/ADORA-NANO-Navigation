#!/usr/bin/env python3
"""Crop unknown map borders and update the map origin atomically."""

import argparse
import shutil
import tempfile
from datetime import datetime
from pathlib import Path

import yaml

from sync_map_metadata import sync_current_map_metadata


APP_DIR = Path(__file__).resolve().parent
ROOT = APP_DIR.parent.parent
MAP_DIR = ROOT / "mapping" / "maps"
DEFAULT_YAML = MAP_DIR / "ms200_keyboard_map.yaml"


def read_pgm(path):
    with path.open("rb") as f:
        magic = f.readline().strip()
        if magic != b"P5":
            raise ValueError(f"only binary P5 PGM is supported: {path}")
        line = f.readline()
        while line.startswith(b"#"):
            line = f.readline()
        width, height = (int(v) for v in line.split())
        max_value = int(f.readline())
        pixels = f.read(width * height)
    if max_value != 255 or len(pixels) != width * height:
        raise ValueError(f"invalid or truncated PGM: {path}")
    return width, height, pixels


def crop_bounds(width, height, pixels, padding_cells):
    known = [i for i, value in enumerate(pixels) if value != 205]
    if not known:
        raise ValueError("map contains no known cells")
    xs = [i % width for i in known]
    rows = [i // width for i in known]
    left = max(0, min(xs) - padding_cells)
    right = min(width - 1, max(xs) + padding_cells)
    top = max(0, min(rows) - padding_cells)
    bottom = min(height - 1, max(rows) + padding_cells)
    return left, right, top, bottom


def cropped_pixels(width, pixels, bounds):
    left, right, top, bottom = bounds
    rows = []
    for row in range(top, bottom + 1):
        start = row * width + left
        rows.append(pixels[start:start + right - left + 1])
    return b"".join(rows)


def finalize_map(yaml_path, padding_m, dry_run=False):
    metadata = yaml.safe_load(yaml_path.read_text(encoding="utf-8")) or {}
    resolution = float(metadata["resolution"])
    origin = list(metadata["origin"])
    pgm_path = yaml_path.parent / metadata["image"]
    width, height, pixels = read_pgm(pgm_path)
    padding_cells = max(0, int(round(padding_m / resolution)))
    bounds = crop_bounds(width, height, pixels, padding_cells)
    left, right, top, bottom = bounds
    new_width = right - left + 1
    new_height = bottom - top + 1
    bottom_removed = height - 1 - bottom
    new_origin = [float(origin[0]) + left * resolution,
                  float(origin[1]) + bottom_removed * resolution,
                  float(origin[2]) if len(origin) > 2 else 0.0]

    print(f"map crop: {width}x{height} -> {new_width}x{new_height}")
    print(f"origin: {origin[:2]} -> {new_origin[:2]}")
    if dry_run or (new_width == width and new_height == height):
        return {"width": new_width, "height": new_height, "origin": new_origin}

    backup = yaml_path.parent / "backups" / f"{datetime.now().strftime('%Y%m%d-%H%M%S')}-before-crop"
    backup.mkdir(parents=True, exist_ok=False)
    shutil.copy2(yaml_path, backup / yaml_path.name)
    shutil.copy2(pgm_path, backup / pgm_path.name)

    metadata["image"] = pgm_path.name
    metadata["origin"] = new_origin
    new_pixels = cropped_pixels(width, pixels, bounds)
    with tempfile.NamedTemporaryFile(dir=pgm_path.parent, prefix=pgm_path.name + ".", delete=False) as f:
        pgm_tmp = Path(f.name)
        f.write(f"P5\n{new_width} {new_height}\n255\n".encode("ascii"))
        f.write(new_pixels)
    with tempfile.NamedTemporaryFile(mode="w", dir=yaml_path.parent, prefix=yaml_path.name + ".",
                                     encoding="utf-8", delete=False) as f:
        yaml_tmp = Path(f.name)
        yaml.safe_dump(metadata, f, sort_keys=False)
    pgm_tmp.replace(pgm_path)
    yaml_tmp.replace(yaml_path)
    print(f"backup: {backup}")
    return {"width": new_width, "height": new_height, "origin": new_origin}


def main():
    parser = argparse.ArgumentParser(description="Crop unknown borders from a completed occupancy map.")
    parser.add_argument("--map-yaml", type=Path, default=DEFAULT_YAML)
    parser.add_argument("--padding", type=float, default=0.50, help="known-map border padding in meters")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    finalize_map(args.map_yaml.resolve(), args.padding, args.dry_run)
    if not args.dry_run:
        sync_current_map_metadata(ROOT)


if __name__ == "__main__":
    main()
