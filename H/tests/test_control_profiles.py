#!/usr/bin/env python3
"""PID参数文件与实时应用测试。"""

import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest


H_DIR = Path(__file__).resolve().parents[1]
if str(H_DIR) not in sys.path:
    sys.path.insert(0, str(H_DIR))

from ball_control import CascadePIDController  # noqa: E402
from ball_control_runtime import (  # noqa: E402
    apply_control_parameters,
    control_ui_values,
)
from control_profiles import (  # noqa: E402
    active_profile_name,
    list_profiles,
    load_active_profile,
    load_profile,
    rename_profile,
    save_profile,
    set_active_profile,
)


class ControlProfileTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = json.loads(
            (H_DIR / "ball_control_config.json").read_text(encoding="utf-8")
        )
        self.values = control_ui_values(self.config, 2.0, 0.0)

    def test_save_select_load_and_rename_profile(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            save_profile("温和参数", self.values, directory)
            self.assertEqual(list_profiles(directory), ["温和参数"])
            set_active_profile("温和参数", directory)
            self.assertEqual(active_profile_name(directory), "温和参数")
            active_name, loaded = load_active_profile(directory)
            self.assertEqual(active_name, "温和参数")
            self.assertEqual(loaded, self.values)

            renamed = rename_profile("温和参数", "比赛参数", directory)
            self.assertEqual(renamed, "比赛参数")
            self.assertEqual(active_profile_name(directory), "比赛参数")
            self.assertEqual(load_profile("比赛参数", directory), self.values)
            self.assertFalse((directory / "温和参数.json").exists())

    def test_profile_name_cannot_escape_directory(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            with self.assertRaises(ValueError):
                save_profile("../outside", self.values, Path(temporary))

    def test_old_profile_without_local_zero_time_uses_two_seconds(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            old_values = dict(self.values)
            old_values.pop("local_zero_stall_time_s")
            path = directory / "旧方案.json"
            path.write_text(
                json.dumps({"parameters": old_values}, ensure_ascii=False),
                encoding="utf-8",
            )
            loaded = load_profile("旧方案", directory)
            self.assertEqual(loaded["local_zero_stall_time_s"], 2.0)

    def test_runtime_apply_changes_limits_and_resets_integral(self) -> None:
        config = copy.deepcopy(self.config)
        controller = CascadePIDController(
            config["cascade_pid"], -2.0, 2.0, 0.25, config["safety"]
        )
        controller.inner_integral = 0.4
        values = control_ui_values(config, 1.5, 0.2)
        values["position_kp_s_inv"] = 0.4
        values["max_angle_step_deg"] = 0.1
        limit, bias, angle = apply_control_parameters(
            controller, config, values, 1.8
        )
        self.assertEqual(limit, 1.5)
        self.assertEqual(bias, 0.2)
        self.assertEqual(angle, 1.5)
        self.assertEqual(controller.inner_integral, 0.0)
        self.assertEqual(controller.angle_max_deg, 1.5)
        self.assertEqual(controller.rate_limiter.max_step_deg, 0.1)
        self.assertEqual(config["cascade_pid"]["position_kp_s_inv"], 0.4)


if __name__ == "__main__":
    unittest.main()
