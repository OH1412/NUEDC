#!/usr/bin/env python3
"""使用合成轨迹验证调参诊断，不连接相机、串口或电机。"""

import json
from pathlib import Path
import sys
import threading
import time
import unittest
from typing import Dict, Iterable, List, Optional, Tuple


H_DIR = Path(__file__).resolve().parents[1]
if str(H_DIR) not in sys.path:
    sys.path.insert(0, str(H_DIR))

from tuning_diagnostics import (  # noqa: E402
    DiagnosticEvent,
    DiagnosticReport,
    TuningDebugReporter,
    TuningDiagnostics,
    TuningHealthSample,
    TuningSample,
)


def event_codes(reports: Iterable[DiagnosticReport]) -> set:
    return {
        event.code
        for report in reports
        for event in report.events
    }


def event_by_code(
    reports: Iterable[DiagnosticReport],
) -> Dict[str, DiagnosticEvent]:
    return {
        event.code: event
        for report in reports
        for event in report.events
    }


class TuningDiagnosticsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.config = json.loads(
            (H_DIR / "ball_control_config.json").read_text(
                encoding="utf-8"
            )
        )

    def make_diagnostics(
        self,
        *,
        enabled: bool = True,
        expected_fps_min: float = 15.0,
        report_interval_s: float = 1.0,
        window_s: float = 6.0,
        suggestion_cooldown_s: float = 6.0,
    ) -> TuningDiagnostics:
        return TuningDiagnostics(
            pid_config=self.config["cascade_pid"],
            safety_config=self.config["safety"],
            working_angle_limit_deg=10.0,
            enabled=enabled,
            expected_fps_min=expected_fps_min,
            report_interval_s=report_interval_s,
            window_s=window_s,
            suggestion_cooldown_s=suggestion_cooldown_s,
        )

    def make_sample(
        self,
        monotonic_s: float,
        *,
        position_m: float = 0.05,
        velocity_m_s: float = 0.01,
        target_position_m: float = 0.20,
        control_error_m: Optional[float] = None,
        velocity_reference_m_s: float = 0.02,
        angle_command_deg: float = -2.0,
        measurement_age_s: float = 0.02,
        processing_latency_ms: Optional[float] = 10.0,
        approach_direction: int = 1,
        within_internal_tolerance: Optional[bool] = None,
        settled: bool = False,
        capture_timestamp_s: Optional[float] = None,
        frame: Optional[int] = None,
        accepted_sequence: Optional[int] = None,
        serial_enabled: bool = False,
    ) -> TuningSample:
        error = (
            target_position_m - position_m
            if control_error_m is None
            else control_error_m
        )
        within = (
            abs(target_position_m - position_m) <= 0.003
            if within_internal_tolerance is None
            else within_internal_tolerance
        )
        capture = (
            monotonic_s - measurement_age_s
            if capture_timestamp_s is None
            else capture_timestamp_s
        )
        return TuningSample(
            monotonic_s=monotonic_s,
            capture_timestamp_s=capture,
            position_m=position_m,
            velocity_m_s=velocity_m_s,
            target_position_m=target_position_m,
            control_error_m=error,
            velocity_reference_m_s=velocity_reference_m_s,
            angle_command_deg=angle_command_deg,
            measurement_age_s=measurement_age_s,
            processing_latency_ms=processing_latency_ms,
            approach_direction=approach_direction,
            within_internal_tolerance=within,
            settled=settled,
            frame=frame,
            accepted_sequence=accepted_sequence,
            serial_enabled=serial_enabled,
        )

    def feed(
        self,
        diagnostics: TuningDiagnostics,
        samples: Iterable[TuningSample],
    ) -> List[DiagnosticReport]:
        reports = []
        for sample in samples:
            report = diagnostics.update(sample)
            if report is not None:
                reports.append(report)
        return reports

    def test_default_is_disabled_and_does_not_accumulate_samples(self) -> None:
        diagnostics = TuningDiagnostics(
            self.config["cascade_pid"],
            self.config["safety"],
            10.0,
        )
        pathological = self.make_sample(
            1.0,
            velocity_m_s=0.0,
            angle_command_deg=-8.0,
            measurement_age_s=0.3,
            processing_latency_ms=250.0,
            serial_enabled=True,
        )

        self.assertIsNone(diagnostics.update(pathological))
        self.assertEqual(diagnostics.total_samples, 0)
        self.assertIn("未启用", diagnostics.summary_line())

    def test_low_tracker_fps_and_high_delay_are_reported(self) -> None:
        diagnostics = self.make_diagnostics(
            report_interval_s=2.0,
        )
        samples = [
            self.make_sample(
                1.0 + index / 10.0,
                position_m=0.05 + index * 0.001,
                velocity_m_s=0.01,
                measurement_age_s=0.18,
                processing_latency_ms=130.0,
                frame=index,
            )
            for index in range(21)
        ]

        reports = self.feed(diagnostics, samples)
        codes = event_codes(reports)

        self.assertIn("low_vision_fps", codes)
        self.assertIn("high_vision_delay", codes)
        self.assertNotIn("stiction", codes)
        self.assertTrue(
            any(
                "处理/有效FPS=10.0/10.0" in report.status_line
                for report in reports
            )
        )
        for event in event_by_code(reports).values():
            self.assertTrue(event.severity)
            self.assertTrue(event.message)

    def test_frame_numbers_distinguish_low_acceptance_from_low_fps(self) -> None:
        diagnostics = self.make_diagnostics(
            report_interval_s=1.0,
            window_s=2.0,
        )
        samples = [
            self.make_sample(
                1.0 + index / 10.0,
                position_m=0.05 + index * 0.001,
                frame=2 * index,
            )
            for index in range(11)
        ]

        reports = self.feed(diagnostics, samples)
        codes = event_codes(reports)

        self.assertIn("low_vision_acceptance", codes)
        self.assertNotIn("low_vision_fps", codes)
        self.assertTrue(
            any(
                "处理/有效FPS=20.0/10.0" in report.status_line
                for report in reports
            )
        )

    def test_accepted_sequence_survives_background_sample_drops(self) -> None:
        diagnostics = self.make_diagnostics(
            expected_fps_min=5.0,
            report_interval_s=0.5,
            window_s=2.0,
        )
        first = self.make_sample(
            1.0,
            capture_timestamp_s=1.0,
            frame=0,
            accepted_sequence=0,
        )
        last = self.make_sample(
            2.0,
            capture_timestamp_s=2.0,
            frame=20,
            accepted_sequence=10,
        )

        self.assertIsNone(diagnostics.update(first))
        report = diagnostics.update(last)

        self.assertIsNotNone(report)
        assert report is not None
        self.assertIn(
            "处理/有效FPS=20.0/10.0 接受率=50%",
            report.status_line,
        )
        self.assertNotIn(
            "vision_gaps",
            {event.code for event in report.events},
        )

    def test_stiction_requires_real_serial_enable(self) -> None:
        real_diagnostics = self.make_diagnostics(
            expected_fps_min=5.0,
            report_interval_s=0.5,
            window_s=2.0,
        )
        dry_run_diagnostics = self.make_diagnostics(
            expected_fps_min=5.0,
            report_interval_s=0.5,
            window_s=2.0,
        )
        real_samples = [
            self.make_sample(
                1.0 + index / 10.0,
                velocity_m_s=0.0,
                velocity_reference_m_s=0.02,
                angle_command_deg=-3.0,
                frame=index,
                serial_enabled=True,
            )
            for index in range(21)
        ]
        dry_run_samples = [
            self.make_sample(
                sample.monotonic_s,
                velocity_m_s=sample.velocity_m_s,
                velocity_reference_m_s=sample.velocity_reference_m_s,
                angle_command_deg=sample.angle_command_deg,
                capture_timestamp_s=sample.capture_timestamp_s,
                frame=sample.frame,
                serial_enabled=False,
            )
            for sample in real_samples
        ]

        real_reports = self.feed(real_diagnostics, real_samples)
        dry_reports = self.feed(dry_run_diagnostics, dry_run_samples)

        self.assertIn("stiction", event_codes(real_reports))
        self.assertNotIn("stiction", event_codes(dry_reports))
        self.assertEqual(
            event_by_code(real_reports)["stiction"].severity,
            "warning",
        )

    def test_approach_too_fast_is_symmetric(self) -> None:
        cases = (
            (1, 0.13, 0.145, 0.05, 0.03, -2.0),
            (-1, 0.17, 0.155, -0.05, -0.03, 2.0),
        )
        for (
            direction,
            start_position,
            near_position,
            velocity,
            velocity_reference,
            angle,
        ) in cases:
            with self.subTest(direction=direction):
                diagnostics = self.make_diagnostics(
                    expected_fps_min=1.0,
                    report_interval_s=0.5,
                    window_s=2.0,
                )
                samples = (
                    self.make_sample(
                        1.0,
                        position_m=start_position,
                        velocity_m_s=direction * 0.02,
                        target_position_m=0.15,
                        velocity_reference_m_s=velocity_reference,
                        angle_command_deg=angle,
                        approach_direction=direction,
                        frame=0,
                        serial_enabled=True,
                    ),
                    self.make_sample(
                        1.5,
                        position_m=near_position,
                        velocity_m_s=velocity,
                        target_position_m=0.15,
                        velocity_reference_m_s=velocity_reference,
                        angle_command_deg=angle,
                        approach_direction=direction,
                        frame=1,
                        serial_enabled=True,
                    ),
                )

                reports = self.feed(diagnostics, samples)
                events = event_by_code(reports)

                self.assertIn("approach_too_fast", events)
                self.assertEqual(
                    events["approach_too_fast"].severity,
                    "danger",
                )
                self.assertNotIn(
                    "approach_target_offset_m",
                    events["approach_too_fast"].message,
                )

    def test_recovery_motion_is_not_reported_as_direction_mismatch(
        self,
    ) -> None:
        diagnostics = self.make_diagnostics(
            report_interval_s=0.5,
            window_s=2.0,
        )
        target_m = 0.15
        samples = [
            self.make_sample(
                1.0 + index / 20.0,
                position_m=0.165 - index * 0.0005,
                velocity_m_s=-0.01,
                target_position_m=target_m,
                control_error_m=-0.01,
                velocity_reference_m_s=-0.02,
                angle_command_deg=2.0,
                approach_direction=1,
                frame=index,
                serial_enabled=True,
            )
            for index in range(41)
        ]

        reports = self.feed(diagnostics, samples)

        self.assertNotIn("direction_mismatch", event_codes(reports))

    def test_motion_opposite_to_current_vref_reports_direction_mismatch(
        self,
    ) -> None:
        diagnostics = self.make_diagnostics(
            report_interval_s=0.5,
            window_s=2.0,
        )
        samples = [
            self.make_sample(
                1.0 + index / 20.0,
                position_m=0.08 - index * 0.0005,
                velocity_m_s=0.0 if index == 0 else -0.01,
                target_position_m=0.20,
                control_error_m=0.10,
                velocity_reference_m_s=0.02,
                angle_command_deg=-2.0,
                approach_direction=1,
                frame=index,
                serial_enabled=True,
            )
            for index in range(41)
        ]

        reports = self.feed(diagnostics, samples)

        self.assertIn("direction_mismatch", event_codes(reports))

    def test_repeated_target_crossings_report_oscillation(self) -> None:
        diagnostics = self.make_diagnostics(
            expected_fps_min=1.0,
            report_interval_s=1.0,
        )
        target_m = 0.15
        errors = [
            0.004 if index % 2 == 0 else -0.004
            for index in range(41)
        ]
        samples = [
            self.make_sample(
                1.0 + index / 10.0,
                position_m=target_m - error,
                velocity_m_s=0.01 if error > 0 else -0.01,
                target_position_m=target_m,
                control_error_m=error,
                velocity_reference_m_s=0.01 if error > 0 else -0.01,
                angle_command_deg=-1.0 if error > 0 else 1.0,
                approach_direction=1,
                within_internal_tolerance=False,
                frame=index,
                serial_enabled=True,
            )
            for index, error in enumerate(errors)
        ]

        reports = self.feed(diagnostics, samples)
        events = event_by_code(reports)

        self.assertIn("target_oscillation", events)
        self.assertEqual(events["target_oscillation"].severity, "warning")

    def test_reports_are_rate_limited_and_event_codes_are_independent(
        self,
    ) -> None:
        diagnostics = self.make_diagnostics(
            report_interval_s=1.0,
            suggestion_cooldown_s=3.0,
        )
        timed_reports = []  # type: List[Tuple[float, DiagnosticReport]]
        for index in range(41):
            timestamp = 1.0 + index / 10.0
            report = diagnostics.update(
                self.make_sample(
                    timestamp,
                    position_m=0.05 + index * 0.001,
                    velocity_m_s=0.01,
                    processing_latency_ms=(
                        10.0 if index <= 10 else 130.0
                    ),
                    frame=index,
                )
            )
            if report is not None:
                timed_reports.append((timestamp, report))

        report_times = [item[0] for item in timed_reports]
        self.assertTrue(
            all(
                later - earlier >= 0.999999
                for earlier, later in zip(report_times, report_times[1:])
            )
        )
        self.assertEqual(
            diagnostics.event_counts.get("low_vision_fps"),
            2,
        )
        self.assertEqual(
            diagnostics.event_counts.get("high_vision_delay"),
            1,
        )

        # high_vision_delay首次出现时，low_vision_fps尚在自己的冷却期；
        # 两种建议不能因为另一种刚报告过而相互吞掉。
        reports = [item[1] for item in timed_reports]
        codes_per_report = [
            {event.code for event in report.events}
            for report in reports
        ]
        self.assertTrue(
            any(
                "high_vision_delay" in codes
                and "low_vision_fps" not in codes
                for codes in codes_per_report
            )
        )

        summary = diagnostics.summary_line()
        self.assertIn("样本=41", summary)
        self.assertIn("处理/有效FPS=10.0/10.0", summary)
        self.assertIn("low_vision_fps×2", summary)
        self.assertIn("high_vision_delay×1", summary)
        self.assertIn("曾稳定=否", summary)


class TuningDebugReporterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.config = json.loads(
            (H_DIR / "ball_control_config.json").read_text(
                encoding="utf-8"
            )
        )

    make_diagnostics = TuningDiagnosticsTests.make_diagnostics
    make_sample = TuningDiagnosticsTests.make_sample

    def test_slow_output_never_backpressures_offer_or_close(self) -> None:
        diagnostics = self.make_diagnostics(
            expected_fps_min=5.0,
            report_interval_s=1.0,
        )
        emit_entered = threading.Event()
        release_emit = threading.Event()

        def slow_emit(_: str) -> None:
            emit_entered.set()
            release_emit.wait(timeout=2.0)

        reporter = TuningDebugReporter(diagnostics, emit=slow_emit)
        # 先同步放入基准样本，使后台处理下一样本时必然形成报告。
        self.assertIsNone(
            diagnostics.update(
                self.make_sample(
                    1.0,
                    frame=0,
                )
            )
        )
        reporter.start()
        self.assertTrue(
            reporter.offer(
                self.make_sample(
                    2.0,
                    frame=1,
                )
            )
        )
        self.assertTrue(emit_entered.wait(timeout=0.5))

        samples = [
            self.make_sample(
                2.1 + index / 1000.0,
                position_m=0.05 + index * 0.00001,
                frame=index + 2,
            )
            for index in range(1000)
        ]

        offer_started = time.perf_counter()
        accepted = [reporter.offer(sample) for sample in samples]
        offer_elapsed_s = time.perf_counter() - offer_started

        self.assertTrue(all(accepted))
        self.assertLess(offer_elapsed_s, 0.5)
        self.assertEqual(reporter.pending, samples[-1])
        self.assertEqual(reporter.dropped_samples, len(samples) - 1)

        close_started = time.perf_counter()
        closed = reporter.close(timeout_s=0.02)
        close_elapsed_s = time.perf_counter() - close_started
        try:
            self.assertFalse(closed)
            self.assertLess(close_elapsed_s, 0.2)
        finally:
            release_emit.set()
            self.assertTrue(reporter.close(timeout_s=1.0))

        self.assertFalse(reporter.offer(samples[-1]))

    def test_normal_close_emits_summary(self) -> None:
        diagnostics = self.make_diagnostics(
            expected_fps_min=5.0,
            report_interval_s=1.0,
        )
        emitted = []  # type: List[str]
        reporter = TuningDebugReporter(diagnostics, emit=emitted.append)
        reporter.start()
        self.assertTrue(
            reporter.offer(
                self.make_sample(
                    1.0,
                    frame=1,
                )
            )
        )

        self.assertTrue(reporter.close(timeout_s=1.0))

        summary_lines = [
            line for line in emitted if line.startswith("[调参汇总]")
        ]
        self.assertEqual(len(summary_lines), 1)
        self.assertIn("后台丢弃旧样本=", summary_lines[0])
        self.assertIn("诊断异常=0", summary_lines[0])

    def test_invalid_frames_still_emit_a_health_heartbeat(self) -> None:
        diagnostics = self.make_diagnostics(
            expected_fps_min=5.0,
            report_interval_s=0.05,
            window_s=0.2,
        )
        emitted = []  # type: List[str]
        heartbeat = threading.Event()

        def emit(line: str) -> None:
            emitted.append(line)
            if "暂无有效球位置" in line:
                heartbeat.set()

        reporter = TuningDebugReporter(diagnostics, emit=emit)
        reporter.start()
        reporter.offer_health(
            TuningHealthSample(
                monotonic_s=time.perf_counter(),
                reason="ball_not_detected",
                frame=17,
                processing_latency_ms=23.0,
            )
        )
        try:
            self.assertTrue(heartbeat.wait(timeout=0.5))
        finally:
            self.assertTrue(reporter.close(timeout_s=1.0))

        heartbeat_lines = [
            line for line in emitted if "暂无有效球位置" in line
        ]
        self.assertTrue(
            any("最近帧=17" in line for line in heartbeat_lines)
        )
        self.assertTrue(
            any("ball_not_detected" in line for line in heartbeat_lines)
        )


if __name__ == "__main__":
    unittest.main()
