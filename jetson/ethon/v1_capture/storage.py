"""Disk-capacity calculations and runtime free-space guards."""

from __future__ import annotations

import shutil
from dataclasses import dataclass
from pathlib import Path

from .config import CaptureConfig

GB = 1_000_000_000


@dataclass(frozen=True)
class StorageEstimate:
    duration_minutes: float
    video_gb: float
    telemetry_gb: float
    guarded_capture_gb: float
    reserve_gb: float
    required_free_gb: float


def estimate_storage(cfg: CaptureConfig, duration_minutes: float | None = None) -> StorageEstimate:
    minutes = float(duration_minutes or cfg.max_duration_minutes)
    total_mbps = sum(camera.bitrate_mbps for camera in cfg.cameras.values())
    video_gb = total_mbps * 1_000_000 / 8 * (minutes * 60) / GB
    # JSON-over-ROS is converted to Parquet; 0.25 GB/hour is deliberately
    # conservative for the specified rates and leaves room for row metadata.
    telemetry_gb = 0.25 * minutes / 60.0
    guarded = (video_gb + telemetry_gb) * cfg.storage_safety_factor
    return StorageEstimate(
        duration_minutes=minutes,
        video_gb=video_gb,
        telemetry_gb=telemetry_gb,
        guarded_capture_gb=guarded,
        reserve_gb=cfg.reserve_free_gb,
        required_free_gb=guarded + cfg.reserve_free_gb,
    )


def disk_free_gb(path: Path) -> float:
    probe = path
    while not probe.exists() and probe != probe.parent:
        probe = probe.parent
    return shutil.disk_usage(probe).free / GB


def assert_enough_space(cfg: CaptureConfig) -> tuple[float, StorageEstimate]:
    estimate = estimate_storage(cfg)
    free = disk_free_gb(cfg.data_root)
    if free < estimate.required_free_gb:
        raise RuntimeError(
            "insufficient disk space: %.2f GB free, %.2f GB required "
            "(%.2f GB guarded capture + %.2f GB reserve)" %
            (free, estimate.required_free_gb, estimate.guarded_capture_gb,
             estimate.reserve_gb))
    return free, estimate


def below_runtime_reserve(cfg: CaptureConfig) -> bool:
    return disk_free_gb(cfg.data_root) < cfg.reserve_free_gb
