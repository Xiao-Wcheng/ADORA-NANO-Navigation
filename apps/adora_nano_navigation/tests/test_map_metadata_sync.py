#!/usr/bin/env python3
import importlib.util
import tempfile
from pathlib import Path

import yaml


APP_SOURCE = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "sync_map_metadata", APP_SOURCE / "sync_map_metadata.py"
)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


with tempfile.TemporaryDirectory() as tmp:
    root = Path(tmp)
    app_dir = root / "apps/adora_nano_navigation"
    map_dir = root / "mapping/maps"
    app_dir.mkdir(parents=True)
    map_dir.mkdir(parents=True)

    (map_dir / "ms200_keyboard_map.pgm").write_bytes(
        b"P5\n# test map\n3 2\n255\n" + bytes([205, 254, 205, 0, 254, 0])
    )
    (map_dir / "ms200_keyboard_map.yaml").write_text(
        yaml.safe_dump(
            {
                "image": "ms200_keyboard_map.pgm",
                "resolution": 0.05,
                "origin": [-0.8, -3.1, 0.0],
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    for name in ("localization_keyboard.yml", "localization_readonly.yml"):
        (app_dir / name).write_text(
            yaml.safe_dump(
                {
                    "nodes": [
                        {
                            "id": "pose_monitor",
                            "env": {
                                "MAP_WIDTH": "74",
                                "MAP_HEIGHT": "51",
                                "ORIGIN_X": "-2.15",
                                "ORIGIN_Y": "-1.55",
                                "RESOLUTION": "0.05",
                            },
                        }
                    ]
                },
                sort_keys=False,
            ),
            encoding="utf-8",
        )
    generated = app_dir / "generated_navigation.yml"
    generated.write_text("stale: true\n", encoding="utf-8")

    result = module.sync_current_map_metadata(root)

    assert result == {
        "MAP_WIDTH": "3",
        "MAP_HEIGHT": "2",
        "ORIGIN_X": "-0.8",
        "ORIGIN_Y": "-3.1",
        "RESOLUTION": "0.05",
    }
    for name in ("localization_keyboard.yml", "localization_readonly.yml"):
        data = yaml.safe_load((app_dir / name).read_text(encoding="utf-8"))
        env = data["nodes"][0]["env"]
        assert all(env[key] == value for key, value in result.items())
    assert not generated.exists()

print("Map metadata synchronization: PASS")
