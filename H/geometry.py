#!/usr/bin/env python3
"""H 题坐标变换工具。"""

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import numpy as np


class TransformError(ValueError):
    """外参配置格式或数值无效。"""


@dataclass(frozen=True)
class RigidTransform:
    """从摄像头坐标系到摄像头底座坐标系的刚体变换。"""

    rotation: np.ndarray
    translation_m: np.ndarray
    source_frame: str = "camera_optical"
    target_frame: str = "camera_base"

    def transform_point(self, point_camera_m: Sequence[float]) -> np.ndarray:
        point = np.asarray(point_camera_m, dtype=np.float64)
        if point.shape != (3,) or not np.all(np.isfinite(point)):
            raise TransformError("三维点必须是包含 3 个有限数值的向量。")
        return self.rotation.dot(point) + self.translation_m

    def matrix4x4(self) -> np.ndarray:
        matrix = np.eye(4, dtype=np.float64)
        matrix[:3, :3] = self.rotation
        matrix[:3, 3] = self.translation_m
        return matrix


def _validated_rotation(value: object) -> np.ndarray:
    rotation = np.asarray(value, dtype=np.float64)
    if rotation.shape != (3, 3) or not np.all(np.isfinite(rotation)):
        raise TransformError("rotation_matrix 必须是 3×3 有限数值矩阵。")
    identity_error = np.max(np.abs(rotation.T.dot(rotation) - np.eye(3)))
    determinant = float(np.linalg.det(rotation))
    if identity_error > 1e-5 or abs(determinant - 1.0) > 1e-5:
        raise TransformError(
            "rotation_matrix 不是有效旋转矩阵：正交误差 {:.3g}，行列式 {:.6g}。"
            .format(identity_error, determinant)
        )
    return rotation


def _validated_translation(value: object) -> np.ndarray:
    translation = np.asarray(value, dtype=np.float64)
    if translation.shape != (3,) or not np.all(np.isfinite(translation)):
        raise TransformError("translation_m 必须是包含 3 个有限数值的向量。")
    return translation


def load_transform(path: Path) -> RigidTransform:
    path = Path(path).expanduser().resolve()
    try:
        with path.open("r", encoding="utf-8") as config_file:
            config = json.load(config_file)
    except OSError as error:
        raise TransformError("无法读取外参文件 {}：{}".format(path, error))
    except json.JSONDecodeError as error:
        raise TransformError("外参文件不是有效 JSON：{}".format(error))

    try:
        rotation = _validated_rotation(config["rotation_matrix"])
        translation = _validated_translation(config["translation_m"])
    except KeyError as error:
        raise TransformError("外参文件缺少字段：{}".format(error))

    return RigidTransform(
        rotation=rotation,
        translation_m=translation,
        source_frame=str(config.get("source_frame", "camera_optical")),
        target_frame=str(config.get("target_frame", "camera_base")),
    )
