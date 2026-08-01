#!/usr/bin/env python3
"""纯串级PID参数方案的保存、选择、重命名和默认方案管理。"""

import json
import os
from pathlib import Path
import tempfile
from typing import Any, Dict, List, Mapping, Optional, Tuple

from pure_pid_tuning_ui import validate_pure_values


H_DIR = Path(__file__).resolve().parent
DEFAULT_PURE_PROFILE_DIR = H_DIR / "pure_pid_profiles"
ACTIVE_PROFILE_FILE = ".active_profile"


def normalize_name(name: str) -> str:
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


def _path(directory: Path, name: str) -> Path:
    return Path(directory) / (normalize_name(name) + ".json")


def _atomic_write(path: Path, text: str) -> None:
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


def list_pure_profiles(
    directory: Path = DEFAULT_PURE_PROFILE_DIR,
) -> List[str]:
    root = Path(directory)
    if not root.exists():
        return []
    return sorted(
        (path.stem for path in root.glob("*.json") if path.is_file()),
        key=str.casefold,
    )


def save_pure_profile(
    name: str,
    values: Mapping[str, Any],
    directory: Path = DEFAULT_PURE_PROFILE_DIR,
) -> Path:
    normalized = normalize_name(name)
    validated = validate_pure_values(values)
    payload = {
        "_description": "H题纯串级PID实时调参方案。",
        "name": normalized,
        "parameters": validated,
    }
    path = _path(directory, normalized)
    _atomic_write(
        path, json.dumps(payload, ensure_ascii=False, indent=2) + "\n"
    )
    return path


def load_pure_profile(
    name: str,
    directory: Path = DEFAULT_PURE_PROFILE_DIR,
) -> Dict[str, float]:
    path = _path(directory, name)
    if not path.is_file():
        raise ValueError("纯PID参数方案不存在：{}".format(path.name))
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("纯PID参数方案根节点必须是对象。")
    values = payload.get("parameters", payload)
    if not isinstance(values, dict):
        raise ValueError("纯PID参数方案缺少parameters对象。")
    return validate_pure_values(values)


def active_pure_profile(
    directory: Path = DEFAULT_PURE_PROFILE_DIR,
) -> Optional[str]:
    pointer = Path(directory) / ACTIVE_PROFILE_FILE
    if not pointer.is_file():
        return None
    name = normalize_name(pointer.read_text(encoding="utf-8"))
    return name if _path(directory, name).is_file() else None


def set_active_pure_profile(
    name: str,
    directory: Path = DEFAULT_PURE_PROFILE_DIR,
) -> str:
    normalized = normalize_name(name)
    if not _path(directory, normalized).is_file():
        raise ValueError("不能设为默认：纯PID参数方案不存在。")
    _atomic_write(Path(directory) / ACTIVE_PROFILE_FILE, normalized + "\n")
    return normalized


def load_active_pure_profile(
    directory: Path = DEFAULT_PURE_PROFILE_DIR,
) -> Tuple[Optional[str], Optional[Dict[str, float]]]:
    name = active_pure_profile(directory)
    return (None, None) if name is None else (name, load_pure_profile(name, directory))


def rename_pure_profile(
    old_name: str,
    new_name: str,
    directory: Path = DEFAULT_PURE_PROFILE_DIR,
) -> str:
    old = normalize_name(old_name)
    new = normalize_name(new_name)
    old_path = _path(directory, old)
    new_path = _path(directory, new)
    if not old_path.is_file():
        raise ValueError("原纯PID参数方案不存在。")
    if new_path.exists() and new_path != old_path:
        raise ValueError("新名称已经存在。")
    was_active = active_pure_profile(directory) == old
    values = load_pure_profile(old, directory)
    if old_path != new_path:
        save_pure_profile(new, values, directory)
        old_path.unlink()
    if was_active:
        set_active_pure_profile(new, directory)
    return new
