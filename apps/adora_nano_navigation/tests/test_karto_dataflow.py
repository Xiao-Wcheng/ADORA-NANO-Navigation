import unittest
import os
from pathlib import Path
import yaml

_default = Path(__file__).parents[1] / "yaml"
if not _default.exists():
    _default = Path(__file__).parents[1]
ROOT = Path(os.environ.get("KARTO_DATAFLOW_DIR", _default))
MODES = {
    "adora_nano_mapping.yml": "new_mapping",
    "adora_nano_navigation.yml": "localization",
    "adora_nano_slam_navigation.yml": "continue_mapping",
    "adora_nano_localization_navigation.yml": "localization",
}

class KartoDataflowTest(unittest.TestCase):
    def test_single_mature_slam_node(self):
        for filename, mode in MODES.items():
            with self.subTest(filename=filename):
                data = yaml.safe_load((ROOT / filename).read_text())
                nodes = {n["id"]: n for n in data["nodes"]}
                self.assertTrue({"scan_matching", "pose_graph_slam", "map_rebuilder", "map_localization"}.isdisjoint(nodes))
                self.assertIn("karto_slam", nodes)
                slam = nodes["karto_slam"]
                self.assertIn("karto_slam_dora_node", slam["path"])
                self.assertEqual(slam["env"]["SLAM_MODE"], mode)
                self.assertEqual(slam["inputs"]["LaserScan"]["source"], "ms200/LaserScan")
                self.assertEqual(slam["inputs"]["Odometry"]["source"], "chassis/Odometry")
                self.assertIn("CORRELATION_SEARCH_SPACE_RESOLUTION", slam["env"])
                self.assertIn("LOOP_SEARCH_MAXIMUM_DISTANCE", slam["env"])
                text=(ROOT/filename).read_text()
                self.assertNotIn("source: scan_matching/", text)
                self.assertNotIn("source: pose_graph_slam/", text)
                self.assertNotIn("source: map_localization/", text)

if __name__ == "__main__": unittest.main()
