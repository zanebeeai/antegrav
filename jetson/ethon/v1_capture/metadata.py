"""Run identity, hashes, and atomically updated metadata."""

from __future__ import annotations

import hashlib
import json
import os
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from . import __version__
from .config import CaptureConfig


def sha256_file(path: Path) -> str | None:
    try:
        digest = hashlib.sha256()
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
        return digest.hexdigest()
    except OSError:
        return None


def git_commit(repo: Path) -> str | None:
    override = os.environ.get("ETHON_FIRMWARE_COMMIT")
    if override:
        return override.strip()
    try:
        return subprocess.check_output(
            ["git", "-C", str(repo), "rev-parse", "HEAD"],
            text=True, stderr=subprocess.DEVNULL, timeout=2).strip()
    except (OSError, subprocess.SubprocessError):
        try:
            return (repo / ".firmware-version").read_text(
                encoding="utf-8").strip() or None
        except OSError:
            return None


def create_run(cfg: CaptureConfig, repo: Path) -> tuple[str, Path, dict]:
    now = datetime.now(timezone.utc)
    run_id = now.strftime("%Y%m%dT%H%M%S.%fZ")
    run_dir = cfg.data_root / now.strftime("%Y-%m-%d") / run_id
    run_dir.mkdir(parents=True, exist_ok=False)
    cameras = {}
    for name, camera in cfg.cameras.items():
        cameras[name] = {
            "sensor_id": camera.sensor_id,
            "sensor_mode": camera.sensor_mode,
            "resolution": [camera.width, camera.height],
            "fps": camera.fps,
            "bitrate_mbps": camera.bitrate_mbps,
            "crop": camera.crop,
            "exposure_range": camera.exposure_range or "sensor_auto",
            "gain_range": camera.gain_range or "sensor_auto",
            "focus": "fixed_by_lens",
            "white_balance_mode": camera.white_balance_mode,
        }
    metadata = {
        "schema_version": 1,
        "service_version": __version__,
        "run_id": run_id,
        "utc_start": now.isoformat(),
        "monotonic_start_ns": time.monotonic_ns(),
        "git_commit": git_commit(repo),
        "status": "starting",
        "cameras": cameras,
        "camera_calibration_sha256": {
            "front_wide": sha256_file(repo / "calib" / "cam1_fisheye.npz"),
            "front_narrow": sha256_file(repo / "calib" / "cam0_H.json"),
        },
        "steering_calibration_sha256": sha256_file(repo / "vehicle.yaml"),
        **cfg.metadata,
    }
    write_metadata(run_dir, metadata)
    return run_id, run_dir, metadata


def write_metadata(run_dir: Path, metadata: dict[str, Any]) -> None:
    target = run_dir / "metadata.json"
    temporary = run_dir / ".metadata.json.tmp"
    with temporary.open("w", encoding="utf-8", newline="\n") as handle:
        json.dump(metadata, handle, indent=2, sort_keys=True)
        handle.write("\n")
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary, target)
