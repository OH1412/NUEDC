#!/usr/bin/env python3
"""验证15～20 FPS、由新视觉测量触发的纯数值控制仿真。"""

from dataclasses import fields
import json
import math
from pathlib import Path
import sys
import unittest
from unittest import mock

import numpy as np


H_DIR = Path(__file__).resolve().parents[1]
if str(H_DIR) not in sys.path:
    sys.path.insert(0, str(H_DIR))

from ball_control import KinematicEstimate  # noqa: E402
import simulate_ball_control as simulation  # noqa: E402


class CountingController:
    def __init__(self) -> None:
        self.calls = []

    def update(
        self,
        position_m: float,
        velocity_m_s: float,
        target_m: float,
        dt_s: float,
    ) -> float:
        self.calls.append(
            (position_m, velocity_m_s, target_m, dt_s)
        )
        return 0.0


class AlwaysAcceptingEstimator:
    def __init__(self, **_: object) -> None:
        pass

    def update(
        self, measurement_m: float, timestamp_s: float
    ) -> KinematicEstimate:
        return KinematicEstimate(
            position_m=float(measurement_m),
            velocity_m_s=0.0,
            acceleration_m_s2=0.0,
            timestamp_s=float(timestamp_s),
            measurement_accepted=True,
        )


class MeasurementDrivenSimulationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.config = json.loads(
            (H_DIR / "ball_control_config.json").read_text(
                encoding="utf-8"
            )
        )

    def make_scenario(
        self, vision_rate_hz: float
    ) -> simulation.PlantScenario:
        return simulation.PlantScenario(
            start_m=0.05,
            target_m=0.20,
            acceleration_gain_m_s2=7.0,
            coulomb_accel_m_s2=0.0,
            viscous_drag_s_inv=0.0,
            constant_bias_accel_m_s2=0.0,
            vision_noise_std_m=0.0,
            vision_rate_hz=vision_rate_hz,
            vision_delay_s=0.0,
            dropout_probability=0.0,
        )

    def test_config_and_scenario_expose_fifteen_to_twenty_fps_field(
        self,
    ) -> None:
        self.assertEqual(
            self.config["expected_vision_rate_hz"],
            [15.0, 20.0],
        )
        field_names = {
            field.name for field in fields(simulation.PlantScenario)
        }
        self.assertIn("vision_rate_hz", field_names)

        scenarios = simulation.random_scenarios(
            200,
            np.random.default_rng(20260730),
        )
        self.assertTrue(
            all(
                15.0 <= scenario.vision_rate_hz <= 20.0
                for scenario in scenarios
            )
        )
        self.assertTrue(
            any(scenario.vision_rate_hz < 17.0 for scenario in scenarios)
        )
        self.assertTrue(
            any(scenario.vision_rate_hz > 18.0 for scenario in scenarios)
        )

    def test_controller_updates_once_per_new_visual_measurement(self) -> None:
        duration_s = 1.0
        control_steps = int(
            math.ceil(
                duration_s
                * float(self.config["control_rate_hz"])
            )
        )
        for vision_rate_hz in (15.0, 17.5, 20.0):
            with self.subTest(vision_rate_hz=vision_rate_hz):
                controller = CountingController()
                scenario = self.make_scenario(vision_rate_hz)
                with mock.patch.object(
                    simulation,
                    "make_controller",
                    return_value=controller,
                ), mock.patch.object(
                    simulation,
                    "KinematicKalmanFilter",
                    AlwaysAcceptingEstimator,
                ):
                    simulation.simulate(
                        controller_name="cascade_pid",
                        config=self.config,
                        scenario=scenario,
                        duration_s=duration_s,
                        working_limit_deg=10.0,
                        seed=7,
                    )

                # 仿真窗口为[0, 1 s)，更新次数服从视觉FPS而非
                # 25 Hz控制循环；17.5 Hz经控制时刻量化后为17次。
                expected_measurements = int(
                    math.floor(vision_rate_hz * duration_s)
                )
                self.assertEqual(
                    len(controller.calls),
                    expected_measurements,
                )
                self.assertLess(len(controller.calls), control_steps)
                self.assertAlmostEqual(
                    controller.calls[0][3],
                    1.0 / vision_rate_hz,
                )
                self.assertTrue(
                    all(call[3] > 0.0 for call in controller.calls)
                )

    def test_stress_scenarios_fix_worst_case_vision_timing(self) -> None:
        scenarios = simulation.stress_scenarios()
        self.assertEqual(len(scenarios), 8)
        self.assertTrue(
            all(item.vision_rate_hz == 15.0 for item in scenarios)
        )
        self.assertTrue(
            all(item.vision_delay_s == 0.12 for item in scenarios)
        )
        self.assertTrue(
            all(item.vision_noise_std_m == 0.0025 for item in scenarios)
        )
        self.assertTrue(
            all(item.dropout_probability == 0.06 for item in scenarios)
        )


if __name__ == "__main__":
    unittest.main()
