#!/usr/bin/env python3
"""带视觉延迟、噪声和滚动参数不确定性的钢珠闭环仿真与PID搜索。"""

import argparse
import copy
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import sys
import time
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

from ball_control import (
    CascadePIDController,
    CompetitionTargetMonitor,
    ConstrainedMPCController,
    KinematicEstimate,
    KinematicKalmanFilter,
)


H_DIR = Path(__file__).resolve().parent
DEFAULT_CONFIG = H_DIR / "ball_control_config.json"


@dataclass(frozen=True)
class PlantScenario:
    start_m: float
    target_m: float
    acceleration_gain_m_s2: float
    coulomb_accel_m_s2: float
    viscous_drag_s_inv: float
    constant_bias_accel_m_s2: float
    vision_noise_std_m: float
    vision_rate_hz: float
    vision_delay_s: float
    dropout_probability: float


@dataclass(frozen=True)
class SimulationResult:
    controller: str
    start_cm: float
    target_cm: float
    competition_failed: bool
    settled: bool
    settle_time_s: Optional[float]
    final_position_cm: float
    final_velocity_cm_s: float
    final_error_cm: float
    max_wrong_side_overshoot_cm: float
    max_abs_angle_deg: float
    max_controller_time_ms: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="在不连接相机/串口/电机的情况下仿真钢珠位置控制"
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument(
        "--controller",
        choices=("cascade_pid", "mpc", "both"),
        default="both",
    )
    parser.add_argument("--trials", type=int, default=30)
    parser.add_argument("--duration-s", type=float, default=20.0)
    parser.add_argument("--seed", type=int, default=20260730)
    parser.add_argument("--working-angle-limit-deg", type=float, default=2.0)
    parser.add_argument(
        "--tune-pid",
        action="store_true",
        help="对同一批不确定场景随机搜索串级PID参数",
    )
    parser.add_argument("--candidate-count", type=int, default=60)
    parser.add_argument(
        "--stress-test",
        action="store_true",
        help=(
            "使用8个15FPS/120ms/高噪声边界组合场景；"
            "用于暴露参数范围两端，严苛程度高于随机均匀场景"
        ),
    )
    parser.add_argument(
        "--write-config",
        action="store_true",
        help="与--tune-pid同时使用，显式把最佳PID参数写回配置",
    )
    return parser.parse_args()


def make_controller(
    name: str, config: Dict[str, Any], working_limit_deg: float
) -> Any:
    if name == "cascade_pid":
        return CascadePIDController(
            config["cascade_pid"],
            -working_limit_deg,
            working_limit_deg,
            float(config["max_angle_step_deg"]),
            config["safety"],
        )
    if name == "mpc":
        return ConstrainedMPCController(
            config["mpc"],
            config["motion_model"],
            -working_limit_deg,
            working_limit_deg,
            float(config["max_angle_step_deg"]),
            config["safety"],
        )
    raise ValueError("未知控制器：{}".format(name))


def predict_estimate(
    estimate: KinematicEstimate, now_s: float
) -> Tuple[float, float]:
    age = max(0.0, min(now_s - estimate.timestamp_s, 0.20))
    return (
        estimate.position_m + age * estimate.velocity_m_s,
        estimate.velocity_m_s,
    )


def plant_acceleration(
    velocity_m_s: float,
    angle_deg: float,
    scenario: PlantScenario,
) -> float:
    drive = (
        -scenario.acceleration_gain_m_s2
        * math.sin(math.radians(angle_deg))
        + scenario.constant_bias_accel_m_s2
        - scenario.viscous_drag_s_inv * velocity_m_s
    )
    coulomb = scenario.coulomb_accel_m_s2
    if abs(velocity_m_s) < 1e-4:
        if abs(drive) <= coulomb:
            return 0.0
        return drive - math.copysign(coulomb, drive)
    return drive - math.copysign(coulomb, velocity_m_s)


def simulate(
    controller_name: str,
    config: Dict[str, Any],
    scenario: PlantScenario,
    duration_s: float,
    working_limit_deg: float,
    seed: int,
) -> SimulationResult:
    rng = np.random.default_rng(seed)
    controller = make_controller(
        controller_name, config, working_limit_deg
    )
    estimator = KinematicKalmanFilter(**config["estimator"])
    safety = config["safety"]
    monitor = CompetitionTargetMonitor(
        scenario.target_m,
        float(safety["internal_tolerance_m"]),
        float(safety["competition_tolerance_m"]),
        float(safety["settle_velocity_m_s"]),
        float(safety["settle_time_s"]),
    )
    dt = 1.0 / float(config["control_rate_hz"])
    substeps = 4
    plant_dt = dt / substeps
    position = scenario.start_m
    velocity = 0.0
    angle_deg = 0.0
    measurement_queue: List[Tuple[float, float, float]] = []
    next_vision_capture_s = 0.0
    latest_estimate: Optional[KinematicEstimate] = None
    last_valid_delivery_s: Optional[float] = None
    last_control_measurement_timestamp_s: Optional[float] = None
    control_was_valid = False
    settled_time: Optional[float] = None
    max_overshoot = 0.0
    max_abs_angle = 0.0
    max_control_ms = 0.0
    approach_direction = (
        1 if scenario.target_m > scenario.start_m else -1
    )
    status = monitor.update(position, velocity, 0.0)

    total_steps = int(math.ceil(duration_s / dt))
    for step in range(total_steps):
        now_s = step * dt
        while now_s + 1e-12 >= next_vision_capture_s:
            if rng.random() >= scenario.dropout_probability:
                measurement = position + rng.normal(
                    0.0, scenario.vision_noise_std_m
                )
                measurement_queue.append(
                    (
                        now_s + scenario.vision_delay_s,
                        now_s,
                        measurement,
                    )
                )
            next_vision_capture_s += 1.0 / scenario.vision_rate_hz
        new_measurement_accepted = False
        while measurement_queue and measurement_queue[0][0] <= now_s + 1e-12:
            _, capture_time, measurement = measurement_queue.pop(0)
            estimate = estimator.update(measurement, capture_time)
            if estimate.measurement_accepted:
                latest_estimate = estimate
                last_valid_delivery_s = now_s
                new_measurement_accepted = True

        timeout_s = float(config["measurement_timeout_s"])
        valid_control = (
            latest_estimate is not None
            and last_valid_delivery_s is not None
            and now_s - last_valid_delivery_s <= timeout_s
            and now_s - latest_estimate.timestamp_s <= timeout_s
        )
        if not valid_control:
            if control_was_valid:
                controller.reset(0.0)
                monitor.clear_settle_timer()
                last_control_measurement_timestamp_s = None
            angle_deg = 0.0
        elif new_measurement_accepted:
            assert latest_estimate is not None
            estimated_position, estimated_velocity = predict_estimate(
                latest_estimate, now_s
            )
            if last_control_measurement_timestamp_s is None:
                controller_dt = 1.0 / scenario.vision_rate_hz
            else:
                controller_dt = (
                    latest_estimate.timestamp_s
                    - last_control_measurement_timestamp_s
                )
            last_control_measurement_timestamp_s = (
                latest_estimate.timestamp_s
            )
            started = time.perf_counter()
            angle_deg = controller.update(
                estimated_position,
                estimated_velocity,
                scenario.target_m,
                controller_dt,
            )
            max_control_ms = max(
                max_control_ms,
                (time.perf_counter() - started) * 1000.0,
            )
        control_was_valid = valid_control
        max_abs_angle = max(max_abs_angle, abs(angle_deg))

        for _ in range(substeps):
            acceleration = plant_acceleration(
                velocity, angle_deg, scenario
            )
            position += (
                velocity * plant_dt
                + 0.5 * acceleration * plant_dt ** 2
            )
            velocity += acceleration * plant_dt

        status = monitor.update(position, velocity, now_s + dt)
        wrong_side = approach_direction * (
            position - scenario.target_m
        )
        max_overshoot = max(max_overshoot, wrong_side)
        if status.settled:
            settled_time = (
                now_s + dt - status.settled_duration_s
            )
        else:
            settled_time = None
        if status.competition_failed:
            angle_deg = 0.0
            break

    return SimulationResult(
        controller=controller_name,
        start_cm=round(scenario.start_m * 100.0, 3),
        target_cm=round(scenario.target_m * 100.0, 3),
        competition_failed=status.competition_failed,
        settled=status.settled,
        settle_time_s=(
            None if settled_time is None else round(settled_time, 3)
        ),
        final_position_cm=round(position * 100.0, 4),
        final_velocity_cm_s=round(velocity * 100.0, 4),
        final_error_cm=round(
            abs(position - scenario.target_m) * 100.0, 4
        ),
        max_wrong_side_overshoot_cm=round(
            max(0.0, max_overshoot) * 100.0, 4
        ),
        max_abs_angle_deg=round(max_abs_angle, 4),
        max_controller_time_ms=round(max_control_ms, 4),
    )


def random_scenarios(
    count: int, rng: np.random.Generator
) -> List[PlantScenario]:
    scenarios = []
    useful_positions = np.array([0.02, 0.05, 0.10, 0.15, 0.20, 0.23])
    while len(scenarios) < count:
        start, target = rng.choice(useful_positions, size=2, replace=False)
        scenarios.append(
            PlantScenario(
                start_m=float(start),
                target_m=float(target),
                acceleration_gain_m_s2=float(rng.uniform(5.0, 8.5)),
                coulomb_accel_m_s2=float(rng.uniform(0.0, 0.30)),
                viscous_drag_s_inv=float(rng.uniform(0.0, 0.8)),
                constant_bias_accel_m_s2=float(rng.uniform(-0.10, 0.10)),
                vision_noise_std_m=float(rng.uniform(0.0008, 0.0025)),
                vision_rate_hz=float(rng.uniform(15.0, 20.0)),
                vision_delay_s=float(rng.uniform(0.04, 0.12)),
                dropout_probability=float(rng.uniform(0.0, 0.06)),
            )
        )
    return scenarios


def stress_scenarios() -> List[PlantScenario]:
    """返回固定的8个联合边界压力场景，便于结果可复现。"""

    scenarios: List[PlantScenario] = []
    # 3 cm短行程：最大驱动增益、无摩擦、恒定偏置助推，最考验制动。
    for start, target in (
        (0.02, 0.05),
        (0.05, 0.02),
        (0.20, 0.23),
        (0.23, 0.20),
    ):
        direction = 1.0 if target > start else -1.0
        scenarios.append(
            PlantScenario(
                start_m=start,
                target_m=target,
                acceleration_gain_m_s2=8.5,
                coulomb_accel_m_s2=0.0,
                viscous_drag_s_inv=0.0,
                constant_bias_accel_m_s2=direction * 0.10,
                vision_noise_std_m=0.0025,
                vision_rate_hz=15.0,
                vision_delay_s=0.12,
                dropout_probability=0.06,
            )
        )
    # 高阻行程：最小驱动增益、最大摩擦/黏阻、恒定偏置反向。
    for start, target in (
        (0.02, 0.23),
        (0.23, 0.02),
        (0.05, 0.10),
        (0.10, 0.05),
    ):
        direction = 1.0 if target > start else -1.0
        scenarios.append(
            PlantScenario(
                start_m=start,
                target_m=target,
                acceleration_gain_m_s2=5.0,
                coulomb_accel_m_s2=0.30,
                viscous_drag_s_inv=0.8,
                constant_bias_accel_m_s2=-direction * 0.10,
                vision_noise_std_m=0.0025,
                vision_rate_hz=15.0,
                vision_delay_s=0.12,
                dropout_probability=0.06,
            )
        )
    return scenarios


def summarize(results: Sequence[SimulationResult]) -> Dict[str, Any]:
    if not results:
        raise ValueError("没有仿真结果。")
    failures = sum(item.competition_failed for item in results)
    settled = [item for item in results if item.settled]
    overshoots = np.array(
        [item.max_wrong_side_overshoot_cm for item in results]
    )
    errors = np.array([item.final_error_cm for item in results])
    return {
        "controller": results[0].controller,
        "trials": len(results),
        "competition_failures": failures,
        "competition_pass_rate": round(
            (len(results) - failures) / len(results), 4
        ),
        "settled_trials": len(settled),
        "settled_rate": round(len(settled) / len(results), 4),
        "max_wrong_side_overshoot_cm": round(float(overshoots.max()), 4),
        "p95_wrong_side_overshoot_cm": round(
            float(np.percentile(overshoots, 95)), 4
        ),
        "max_final_error_cm": round(float(errors.max()), 4),
        "median_final_error_cm": round(float(np.median(errors)), 4),
        "max_controller_time_ms": round(
            max(item.max_controller_time_ms for item in results), 4
        ),
        "median_settle_time_s": (
            None
            if not settled
            else round(
                float(
                    np.median(
                        [item.settle_time_s for item in settled]
                    )
                ),
                3,
            )
        ),
    }


def evaluate_pid_config(
    config: Dict[str, Any],
    scenarios: Sequence[PlantScenario],
    duration_s: float,
    working_limit_deg: float,
    seed: int,
) -> Tuple[float, Dict[str, Any]]:
    results = [
        simulate(
            "cascade_pid",
            config,
            scenario,
            duration_s,
            working_limit_deg,
            seed + index,
        )
        for index, scenario in enumerate(scenarios)
    ]
    summary = summarize(results)
    unsettled = len(results) - summary["settled_trials"]
    score = (
        summary["competition_failures"] * 1e8
        + unsettled * 1e5
        + summary["p95_wrong_side_overshoot_cm"] * 1e4
        + summary["median_final_error_cm"] * 1e3
    )
    return float(score), summary


def tune_pid(
    base_config: Dict[str, Any],
    scenarios: Sequence[PlantScenario],
    duration_s: float,
    working_limit_deg: float,
    candidate_count: int,
    rng: np.random.Generator,
    seed: int,
) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    candidates = [copy.deepcopy(base_config["cascade_pid"])]
    for _ in range(max(0, candidate_count - 1)):
        candidate = copy.deepcopy(base_config["cascade_pid"])
        candidate.update(
            {
                "position_kp_s_inv": float(rng.uniform(0.3, 1.5)),
                "max_velocity_m_s": float(rng.uniform(0.02, 0.07)),
                "braking_accel_m_s2": float(rng.uniform(0.3, 1.5)),
                "velocity_kp_deg_per_m_s": float(
                    rng.uniform(8.0, 50.0)
                ),
                "velocity_ki_deg_per_m": float(rng.uniform(0.0, 10.0)),
                "static_friction_compensation_deg": float(
                    rng.uniform(1.5, 5.0)
                ),
                "static_compensation_ramp_deg_s": float(
                    rng.uniform(1.0, 8.0)
                ),
                "static_compensation_max_speed_m_s": float(
                    rng.uniform(0.006, 0.025)
                ),
                "static_compensation_min_error_m": float(
                    rng.uniform(0.005, 0.015)
                ),
            }
        )
        candidates.append(candidate)

    best_score = math.inf
    best_pid = candidates[0]
    best_summary: Dict[str, Any] = {}
    for index, candidate in enumerate(candidates):
        trial_config = copy.deepcopy(base_config)
        trial_config["cascade_pid"] = candidate
        score, summary = evaluate_pid_config(
            trial_config,
            scenarios,
            duration_s,
            working_limit_deg,
            seed,
        )
        if score < best_score:
            best_score = score
            best_pid = candidate
            best_summary = summary
        if (index + 1) % 10 == 0:
            print(
                "PID搜索进度：{}/{}，当前最佳失败={}，稳定率={:.1%}"
                .format(
                    index + 1,
                    len(candidates),
                    best_summary.get("competition_failures", 0),
                    best_summary.get("settled_rate", 0.0),
                ),
                file=sys.stderr,
            )
    output = copy.deepcopy(base_config)
    output["cascade_pid"] = best_pid
    best_summary["tuning_score"] = round(best_score, 4)
    best_summary["best_pid"] = best_pid
    return output, best_summary


def atomic_write_config(path: Path, data: Dict[str, Any]) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(data, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def main() -> int:
    args = parse_args()
    if (
        args.trials < 1
        or args.duration_s <= 0
        or args.candidate_count < 1
        or not 0 < args.working_angle_limit_deg <= 30
    ):
        print("配置错误：请检查仿真次数、时长、候选数和限角。", file=sys.stderr)
        return 2
    config_path = args.config.expanduser().resolve()
    try:
        config = json.loads(config_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        print("配置错误：{}".format(error), file=sys.stderr)
        return 2
    rng = np.random.default_rng(args.seed)
    scenarios = (
        stress_scenarios()
        if args.stress_test
        else random_scenarios(args.trials, rng)
    )

    if args.tune_pid:
        tuned, tuning_summary = tune_pid(
            config,
            scenarios,
            args.duration_s,
            args.working_angle_limit_deg,
            args.candidate_count,
            rng,
            args.seed + 10000,
        )
        if args.write_config:
            atomic_write_config(config_path, tuned)
            tuning_summary["config_updated"] = str(config_path)
        else:
            tuning_summary["config_updated"] = False
        print(json.dumps(tuning_summary, ensure_ascii=False, indent=2))
        return 0

    names = (
        ("cascade_pid", "mpc")
        if args.controller == "both"
        else (args.controller,)
    )
    summaries = []
    for name in names:
        results = [
            simulate(
                name,
                config,
                scenario,
                args.duration_s,
                args.working_angle_limit_deg,
                args.seed + 1000 * names.index(name) + index,
            )
            for index, scenario in enumerate(scenarios)
        ]
        summaries.append(summarize(results))
    print(json.dumps(summaries, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
