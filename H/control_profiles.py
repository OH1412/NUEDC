#!/usr/bin/env python3
"""串级PID参数方案文件的保存、选择、重命名和默认方案管理。"""

import json
import os
from pathlib import Path
import tempfile
from typing import Any, Dict, List, Mapping, Optional, Tuple

from control_tuning_ui import validate_parameter_values


H_DIR = Path(__file__).resolve().parent
DEFAULT_PROFILE_DIR = H_DIR / "control_profiles"
ACTIVE_PROFILE_FILE = ".active_profile"


def normalize_profile_name(name: str) -> str:
    value = str(name).strip()
    if value.lower().endswith(".json"):
        value = value[:-5].strip()
    if not value or value in (".", ".."):
        raise ValueError("参数方案名称不能为空。")
    if len(value) > 80:
        raise ValueError("参数方案名称不能超过80个字符。")
    if any(character in value for character in ("/", "\\", "\0")):
        raise ValueError("参数方案名称不能包含路径分隔符。")
    if value.startswith("."):
        raise ValueError("参数方案名称不能以点开头。")
    return value


def profile_path(profile_dir: Path, name: str) -> Path:
    return Path(profile_dir) / (normalize_profile_name(name) + ".json")


def _atomic_write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=".{}-".format(path.name), suffix=".tmp", dir=str(path.parent)
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        Path(temporary_name).replace(path)
    except Exception:
        try:
            Path(temporary_name).unlink()
        except OSError:
            pass
        raise


def list_profiles(profile_dir: Path = DEFAULT_PROFILE_DIR) -> List[str]:
    directory = Path(profile_dir)
    if not directory.exists():
        return []
    return sorted(
        (path.stem for path in directory.glob("*.json") if path.is_file()),
        key=str.casefold,
    )


def save_profile(
    name: str,
    values: Mapping[str, Any],
    profile_dir: Path = DEFAULT_PROFILE_DIR,
) -> Path:
    normalized = normalize_profile_name(name)
    validated = validate_parameter_values(values)
    path = profile_path(profile_dir, normalized)
    payload = {
        "_description": "H题钢珠串级PID实时调参方案。",
        "name": normalized,
        "parameters": validated,
    }
    _atomic_write_text(
        path,
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
    )
    return path


def load_profile(
    name: str, profile_dir: Path = DEFAULT_PROFILE_DIR
) -> Dict[str, float]:
    path = profile_path(profile_dir, name)
    if not path.is_file():
        raise ValueError("参数方案不存在：{}".format(path.name))
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("参数方案根节点必须是JSON对象。")
    values = payload.get("parameters", payload)
    if not isinstance(values, dict):
        raise ValueError("参数方案缺少parameters对象。")
    return validate_parameter_values(values)


def active_profile_name(
    profile_dir: Path = DEFAULT_PROFILE_DIR,
) -> Optional[str]:
    pointer = Path(profile_dir) / ACTIVE_PROFILE_FILE
    if not pointer.is_file():
        return None
    name = normalize_profile_name(pointer.read_text(encoding="utf-8"))
    if not profile_path(profile_dir, name).is_file():
        return None
    return name


def set_active_profile(
    name: str, profile_dir: Path = DEFAULT_PROFILE_DIR
) -> str:
    normalized = normalize_profile_name(name)
    if not profile_path(profile_dir, normalized).is_file():
        raise ValueError("不能设为默认：参数方案不存在。")
    _atomic_write_text(
        Path(profile_dir) / ACTIVE_PROFILE_FILE, normalized + "\n"
    )
    return normalized


def load_active_profile(
    profile_dir: Path = DEFAULT_PROFILE_DIR,
) -> Tuple[Optional[str], Optional[Dict[str, float]]]:
    name = active_profile_name(profile_dir)
    if name is None:
        return None, None
    return name, load_profile(name, profile_dir)


def rename_profile(
    old_name: str,
    new_name: str,
    profile_dir: Path = DEFAULT_PROFILE_DIR,
) -> str:
    old_normalized = normalize_profile_name(old_name)
    new_normalized = normalize_profile_name(new_name)
    old_path = profile_path(profile_dir, old_normalized)
    new_path = profile_path(profile_dir, new_normalized)
    if not old_path.is_file():
        raise ValueError("原参数方案不存在。")
    if new_path.exists() and new_path != old_path:
        raise ValueError("新名称已经存在。")
    was_active = active_profile_name(profile_dir) == old_normalized
    payload = load_profile(old_normalized, profile_dir)
    if old_path != new_path:
        save_profile(new_normalized, payload, profile_dir)
        old_path.unlink()
    if was_active:
        set_active_profile(new_normalized, profile_dir)
    return new_normalized
