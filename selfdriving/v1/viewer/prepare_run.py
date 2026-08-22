#!/usr/bin/env python3
"""Prepare one Ethon capture directory for the browser-based run viewer."""

from __future__ import annotations

import argparse
import json
import math
import subprocess
from pathlib import Path
from typing import Any, Iterable


def finite(value: Any) -> Any:
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def read_parquet(run: Path, name: str, warnings: list[str]) -> list[dict[str, Any]]:
    path = run / f"{name}.parquet"
    if not path.exists() or path.stat().st_size < 8:
        size = path.stat().st_size if path.exists() else 0
        warnings.append(f"{name}.parquet is incomplete ({size} bytes)")
        return []
    try:
        import pyarrow.parquet as pq

        return pq.read_table(path).to_pylist()
    except Exception as exc:
        warnings.append(f"{name}.parquet is unreadable: {exc}")
        return []


def downsample(rows: Iterable[dict[str, Any]], period_ns: int) -> list[dict[str, Any]]:
    selected = []
    last = None
    for row in rows:
        timestamp = row.get("timestamp_ns")
        if not isinstance(timestamp, int):
            continue
        if last is None or timestamp - last >= period_ns:
            selected.append(row)
            last = timestamp
    return selected


def video_info(path: Path, warnings: list[str]) -> dict[str, Any]:
    result: dict[str, Any] = {"file": path.name, "size_bytes": path.stat().st_size}
    if path.stat().st_size == 0:
        warnings.append(f"{path.name} is empty")
        result["valid"] = False
        return result
    try:
        probe = subprocess.run(
            [
                "ffprobe", "-v", "error", "-show_entries",
                "stream=codec_name,width,height,r_frame_rate,nb_frames,duration",
                "-show_entries", "format=duration,size", "-of", "json", str(path),
            ],
            check=True,
            capture_output=True,
            text=True,
        )
        payload = json.loads(probe.stdout)
        stream = (payload.get("streams") or [{}])[0]
        fmt = payload.get("format") or {}
        result.update({
            "valid": True,
            "codec": stream.get("codec_name"),
            "width": int(stream["width"]) if stream.get("width") else None,
            "height": int(stream["height"]) if stream.get("height") else None,
            "fps": stream.get("r_frame_rate"),
            "frames": int(stream["nb_frames"]) if stream.get("nb_frames") else None,
            "duration_s": float(fmt.get("duration") or stream.get("duration") or 0),
        })
    except Exception as exc:
        warnings.append(f"{path.name} is not playable: {exc}")
        result["valid"] = False
    return result


def relative(timestamp_ns: Any, base_ns: int) -> float | None:
    if not isinstance(timestamp_ns, int):
        return None
    return round((timestamp_ns - base_ns) / 1_000_000_000, 6)


def prepare(run: Path) -> Path:
    metadata = json.loads((run / "metadata.json").read_text(encoding="utf-8"))
    warnings: list[str] = []
    frames_raw = read_parquet(run, "frames", warnings)
    telemetry_raw = read_parquet(run, "telemetry", warnings)
    events_raw = read_parquet(run, "events", warnings)

    first_frame_ns = next((row.get("timestamp_ns") for row in frames_raw
                           if isinstance(row.get("timestamp_ns"), int)), None)
    first_telemetry_ns = next((row.get("timestamp_ns") for row in telemetry_raw
                               if isinstance(row.get("timestamp_ns"), int)), None)
    base_ns = int(first_frame_ns or first_telemetry_ns or
                  metadata.get("monotonic_start_ns") or 0)

    frames = [{
        "t": relative(row.get("timestamp_ns"), base_ns),
        "wide": row.get("front_wide_frame_index"),
        "narrow": row.get("front_narrow_frame_index"),
        "alignment_ms": finite(row.get("alignment_error_ms")),
        "valid": row.get("alignment_valid"),
        "drops": row.get("dropped_frame_flags") or "",
        "latency_ms": finite(row.get("capture_latency_ms")),
    } for row in downsample(frames_raw, 100_000_000)]

    telemetry = []
    for row in downsample(telemetry_raw, 50_000_000):
        currents = [row.get(f"drive_{index}_current") for index in (1, 2, 3)]
        telemetry.append({
            "t": relative(row.get("timestamp_ns"), base_ns),
            "speed": finite(row.get("vehicle_speed_m_s")),
            "steer": finite(row.get("cancoder_position_rad")),
            "steer_rate": finite(row.get("cancoder_velocity_rad_s")),
            "target": finite(row.get("steering_target")),
            "pedal": finite(row.get("pedal_fraction")),
            "latency": finite(row.get("ctre_latency_ms")),
            "ctre_ok": row.get("ctre_alignment_valid"),
            "voltage": finite(row.get("supply_voltage")),
            "drive_current": finite(sum(value for value in currents
                                        if isinstance(value, (int, float)))),
            "gps_lat": finite(row.get("gps_latitude")),
            "gps_lon": finite(row.get("gps_longitude")),
            "gps_heading": finite(row.get("gps_heading")),
            "gps_fix": row.get("gps_fix_quality"),
            "mode": row.get("manual_or_auto"),
            "estop": row.get("estop"),
            "faults": row.get("can_faults") or "",
        })

    events = [{
        "t": relative(row.get("timestamp_ns"), base_ns),
        "type": row.get("event_type"),
        "value": row.get("event_value"),
        "notes": row.get("notes"),
    } for row in events_raw]

    wide = video_info(run / "front_wide.mp4", warnings)
    narrow = video_info(run / "front_narrow.mp4", warnings)
    wide["start_s"] = 0.0
    first_narrow_ns = next(
        (row.get("front_narrow_timestamp_ns") for row in frames_raw
         if isinstance(row.get("front_narrow_timestamp_ns"), int)), base_ns)
    narrow["start_s"] = relative(first_narrow_ns, base_ns) or 0.0

    if metadata.get("status") != "complete":
        warnings.insert(0, f"Run status is {metadata.get('status', 'unknown')}, not complete")
    duration = max(
        float(wide.get("duration_s") or 0),
        float(narrow.get("duration_s") or 0),
        float(metadata.get("duration_s") or 0),
        max((float(row["t"]) for row in telemetry if row["t"] is not None), default=0),
    )
    payload = {
        "schema_version": 1,
        "run_id": metadata.get("run_id", run.name),
        "metadata": metadata,
        "health": {
            "ok": not warnings,
            "warnings": warnings,
            "source_rows": {
                "frames": len(frames_raw), "telemetry": len(telemetry_raw),
                "events": len(events_raw),
            },
        },
        "timeline": {"base_timestamp_ns": base_ns, "duration_s": round(duration, 3)},
        "videos": {"wide": wide, "narrow": narrow},
        "frames": frames,
        "telemetry": telemetry,
        "events": events,
    }
    output = run / "viewer.json"
    output.write_text(json.dumps(payload, indent=2, allow_nan=False), encoding="utf-8")
    print(output)
    return output


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_directory", type=Path)
    args = parser.parse_args()
    prepare(args.run_directory.resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
