#!/usr/bin/env python3
"""Mode 5位置—固定平衡角专用标定表。"""

import json
import math
import os
from pathlib import Path
import tempfile
from typing import Any, Dict, List, Tuple

from angle_serial import LINKAGE_LENGTH_MM, MAX_ANGLE_DEG


H_DIR = Path(__file__).resolve().parent
DEFAULT_MODE5_EQUILIBRIUM_FILE = H_DIR / "mode5_equilibrium_points.json"


def _validated_points(payload: Any) -> List[Dict[str, float]]:
    if not isinstance(payload, dict):
        raise ValueError("Mode 5平衡标定文件根节点必须是JSON对象。")
    raw_points = payload.get("points")
    if not isinstance(raw_points, list) or not raw_points:
        raise ValueError("Mode 5平衡标定文件points不能为空。")
    points: List[Dict[str, float]] = []
    seen = set()
    for index, raw in enumerate(raw_points):
        if not isinstance(raw, dict):
            raise ValueError("Mode 5标定第{}项必须是对象。".format(index + 1))
        position_cm = float(raw["position_cm"])
        angle_deg = float(raw["equilibrium_angle_bias_deg"])
        if not math.isfinite(position_cm) or not math.isfinite(angle_deg):
            raise ValueError("Mode 5标定位置和基准角必须是有限值。")
        if abs(angle_deg) > float(MAX_ANGLE_DEG):
            raise ValueError("Mode 5基准角不能超过±{}°。".format(MAX_ANGLE_DEG))
        if position_cm in seen:
            raise ValueError("Mode 5标定位置重复：{:+.3f}cm。".format(position_cm))
        seen.add(position_cm)
        equivalent_height_mm = LINKAGE_LENGTH_MM * math.tan(
            math.radians(angle_deg)
        )
        points.append(
            {
                "position_cm": position_cm,
                "equilibrium_angle_bias_deg": angle_deg,
                "equivalent_height_mm": equivalent_height_mm,
            }
        )
    return sorted(points, key=lambda item: item["position_cm"], reverse=True)


def load_mode5_equilibrium_points(
    path: Path = DEFAULT_MODE5_EQUILIBRIUM_FILE,
) -> List[Dict[str, float]]:
    source = Path(path).expanduser().resolve()
    payload = json.loads(source.read_text(encoding="utf-8"))
    return _validated_points(payload)


def nearest_mode5_equilibrium(
    centered_position_cm: float,
    path: Path = DEFAULT_MODE5_EQUILIBRIUM_FILE,
) -> Tuple[float, float, float]:
    position_cm = float(centered_position_cm)
    if not math.isfinite(position_cm):
        raise ValueError("Mode 5记录位置必须是有限值。")
    points = load_mode5_equilibrium_points(path)
    nearest = min(
        points,
        key=lambda item: abs(item["position_cm"] - position_cm),
    )
    return (
        nearest["equilibrium_angle_bias_deg"],
        nearest["position_cm"],
        nearest["equivalent_height_mm"],
    )


def save_mode5_equilibrium_point(
    position_cm: float,
    equilibrium_angle_bias_deg: float,
    path: Path = DEFAULT_MODE5_EQUILIBRIUM_FILE,
) -> Tuple[Path, bool, float]:
    """按精确位置覆盖或新增一项，返回(路径, 是否覆盖, 等效高度mm)。"""

    target_position = float(position_cm)
    target_angle = float(equilibrium_angle_bias_deg)
    if not math.isfinite(target_position) or not math.isfinite(target_angle):
        raise ValueError("协同保存的位置和基准角必须是有限值。")
    if abs(target_angle) > float(MAX_ANGLE_DEG):
        raise ValueError("协同保存的基准角不能超过±{}°。".format(MAX_ANGLE_DEG))
    destination = Path(path).expanduser().resolve()
    points = load_mode5_equilibrium_points(destination)
    replaced = False
    for point in points:
        if abs(point["position_cm"] - target_position) <= 1e-9:
            point["equilibrium_angle_bias_deg"] = target_angle
            point["equivalent_height_mm"] = LINKAGE_LENGTH_MM * math.tan(
                math.radians(target_angle)
            )
            replaced = True
            break
    if not replaced:
        points.append(
            {
                "position_cm": target_position,
                "equilibrium_angle_bias_deg": target_angle,
                "equivalent_height_mm": LINKAGE_LENGTH_MM * math.tan(
                    math.radians(target_angle)
                ),
            }
        )
    points.sort(key=lambda item: item["position_cm"], reverse=True)
    output = {
        "_description": (
            "Mode 5按记录球位就近选择固定平衡基准角。position_cm单位cm；"
            "equilibrium_angle_bias_deg单位deg；equivalent_height_mm按"
            "250*tan(theta)生成，仅用于核对。"
        ),
        "points": [
            {
                "position_cm": round(point["position_cm"], 6),
                "equilibrium_angle_bias_deg": round(
                    point["equilibrium_angle_bias_deg"], 6
                ),
                "equivalent_height_mm": round(point["equivalent_height_mm"], 6),
            }
            for point in points
        ],
    }
    destination.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=".{}-".format(destination.name),
        suffix=".tmp",
        dir=str(destination.parent),
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(output, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        Path(temporary_name).replace(destination)
    except Exception:
        try:
            Path(temporary_name).unlink()
        except OSError:
            pass
        raise
    height_mm = LINKAGE_LENGTH_MM * math.tan(math.radians(target_angle))
    return destination, replaced, height_mm
