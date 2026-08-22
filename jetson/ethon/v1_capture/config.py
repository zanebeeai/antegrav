"""Configuration loading and validation for the v1 capture service."""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict


@dataclass(frozen=True)
class CameraConfig:
    name: str
    sensor_id: int
    sensor_mode: int
    width: int = 1280
    height: int = 720
    fps: int = 30
    bitrate_mbps: float = 8.0
    exposure_range: str = ""
    gain_range: str = ""
    white_balance_mode: int = 1
    crop: Dict[str, int] = field(
        default_factory=lambda: {"top": 0, "bottom": 720, "left": 0, "right": 1280})


@dataclass(frozen=True)
class CaptureConfig:
    data_root: Path
    max_duration_minutes: int
    reserve_free_gb: float
    storage_safety_factor: float
    parquet_row_group_size: int
    max_alignment_error_ms: float
    max_ctre_latency_ms: float
    heartbeat_timeout_s: float
    telemetry_start_timeout_s: float
    camera_start_timeout_s: float
    cameras: Dict[str, CameraConfig]
    metadata: Dict[str, Any]


def _camera(name: str, value: Any) -> CameraConfig:
    if not isinstance(value, dict):
        raise ValueError("camera %s must be an object" % name)
    allowed = set(CameraConfig.__dataclass_fields__) - {"name"}
    unknown = sorted(set(value) - allowed)
    if unknown:
        raise ValueError("camera %s has unknown keys: %s" % (name, ", ".join(unknown)))
    return CameraConfig(name=name, **value)


def load_config(path: str | Path) -> CaptureConfig:
    path = Path(path)
    with path.open("r", encoding="utf-8") as handle:
        raw = json.load(handle)
    cameras_raw = raw.pop("cameras", None)
    if not isinstance(cameras_raw, dict):
        raise ValueError("cameras must be an object")
    cameras = {name: _camera(name, value) for name, value in cameras_raw.items()}
    if set(cameras) != {"front_wide", "front_narrow"}:
        raise ValueError("cameras must contain exactly front_wide and front_narrow")
    raw["data_root"] = Path(raw["data_root"])
    cfg = CaptureConfig(cameras=cameras, **raw)
    validate_config(cfg)
    return cfg


def validate_config(cfg: CaptureConfig) -> None:
    if cfg.max_duration_minutes <= 0:
        raise ValueError("max_duration_minutes must be positive")
    if cfg.reserve_free_gb < 0 or cfg.storage_safety_factor < 1.0:
        raise ValueError("invalid storage reserve or safety factor")
    if cfg.parquet_row_group_size < 100:
        raise ValueError("parquet_row_group_size must be at least 100")
    if (cfg.max_alignment_error_ms <= 0 or cfg.max_ctre_latency_ms <= 0 or
            cfg.heartbeat_timeout_s <= 0 or
            cfg.telemetry_start_timeout_s <= 0):
        raise ValueError("alignment and heartbeat limits must be positive")
    for camera in cfg.cameras.values():
        if min(camera.width, camera.height, camera.fps, camera.bitrate_mbps) <= 0:
            raise ValueError("camera %s has a non-positive setting" % camera.name)
        crop = camera.crop
        required = {"top", "bottom", "left", "right"}
        if set(crop) != required:
            raise ValueError("camera %s crop must contain %s" %
                             (camera.name, ", ".join(sorted(required))))
        if not (0 <= crop["left"] < crop["right"] <= camera.width and
                0 <= crop["top"] < crop["bottom"] <= camera.height):
            raise ValueError("camera %s crop is outside the recorded image" % camera.name)


def run_setup_issues(cfg: CaptureConfig) -> list[str]:
    """Items an operator must resolve before creating a real dataset run."""
    issues = []
    required_metadata = (
        "track_name", "track_direction", "driver_identifier", "weather",
        "lighting", "nominal_speed_m_s")
    for key in required_metadata:
        value = cfg.metadata.get(key)
        if value is None or str(value).strip().lower() in ("", "unspecified"):
            issues.append("metadata.%s is not set" % key)
    if not bool(cfg.metadata.get("crop_confirmed")):
        issues.append("metadata.crop_confirmed must be true after the fixed crop is verified")
    for camera in cfg.cameras.values():
        if not camera.exposure_range or not camera.gain_range:
            issues.append(
                "%s exposure_range and gain_range must be fixed and logged" % camera.name)
    return issues
