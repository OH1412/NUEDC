import json
import math
import sys
import tempfile
import unittest
from pathlib import Path


H_DIR = Path(__file__).resolve().parents[1]
if str(H_DIR) not in sys.path:
    sys.path.insert(0, str(H_DIR))

from mode5_equilibrium import (  # noqa: E402
    load_mode5_equilibrium_points,
    nearest_mode5_equilibrium,
    save_mode5_equilibrium_point,
)


class Mode5EquilibriumTests(unittest.TestCase):
    def make_table(self, directory: str) -> Path:
        path = Path(directory) / "points.json"
        path.write_text(
            json.dumps(
                {
                    "points": [
                        {
                            "position_cm": 1.0,
                            "equilibrium_angle_bias_deg": -0.5,
                        },
                        {
                            "position_cm": -1.0,
                            "equilibrium_angle_bias_deg": 0.5,
                        },
                    ]
                }
            ),
            encoding="utf-8",
        )
        return path

    def test_nearest_position_returns_angle_and_derived_height(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = self.make_table(directory)
            angle, position, height = nearest_mode5_equilibrium(0.8, path)
            self.assertEqual(position, 1.0)
            self.assertEqual(angle, -0.5)
            self.assertAlmostEqual(
                height, 250.0 * math.tan(math.radians(-0.5)), places=8
            )

    def test_coordinated_save_replaces_same_position(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = self.make_table(directory)
            saved_path, replaced, height = save_mode5_equilibrium_point(
                1.0, -0.75, path
            )
            self.assertEqual(saved_path, path.resolve())
            self.assertTrue(replaced)
            self.assertAlmostEqual(
                height, 250.0 * math.tan(math.radians(-0.75)), places=8
            )
            points = load_mode5_equilibrium_points(path)
            self.assertEqual(len(points), 2)
            self.assertEqual(points[0]["position_cm"], 1.0)
            self.assertEqual(points[0]["equilibrium_angle_bias_deg"], -0.75)

    def test_coordinated_save_adds_and_sorts_new_position(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = self.make_table(directory)
            _saved_path, replaced, _height = save_mode5_equilibrium_point(
                0.25, -0.1, path
            )
            self.assertFalse(replaced)
            self.assertEqual(
                [point["position_cm"] for point in load_mode5_equilibrium_points(path)],
                [1.0, 0.25, -1.0],
            )


if __name__ == "__main__":
    unittest.main()
