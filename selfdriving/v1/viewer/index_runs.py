#!/usr/bin/env python3
"""Index locally downloaded Ethon captures for the private run viewer."""

from __future__ import annotations

import argparse
import json
import os
import shutil
from pathlib import Path
from typing import Any

from prepare_run import prepare


VIEWER_ROOT = Path(__file__).resolve().parent
DEFAULT_DATA_ROOT = VIEWER_ROOT.parents[2] / "jetson" / "ethon" / "data" / "raw"
DEFAULT_PUBLIC_ROOT = VIEWER_ROOT / "public" / "runs"


def replace_link(source: Path, destination: Path) -> None:
    """Create a zero-copy link to capture media, replacing stale links only."""
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists():
        try:
            if os.path.samefile(source, destination):
                return
        except OSError:
            pass
        destination.unlink()
    try:
        os.link(source, destination)
    except OSError:
        try:
            destination.symlink_to(source)
        except OSError as exc:
            raise RuntimeError(
                f"could not link {source}; the viewer will not copy capture video"
            ) from exc


def option_for(run: Path, public_root: Path) -> dict[str, Any] | None:
    metadata_path = run / "metadata.json"
    if not metadata_path.exists():
        return None
    try:
        manifest_path = run / "viewer.json"
        sources = [
            path for path in (
                metadata_path,
                run / "frames.parquet",
                run / "telemetry.parquet",
                run / "events.parquet",
                run / "front_wide.mp4",
                run / "front_narrow.mp4",
            ) if path.exists()
        ]
        if (not manifest_path.exists() or
                any(path.stat().st_mtime_ns > manifest_path.stat().st_mtime_ns
                    for path in sources)):
            manifest_path = prepare(run)
        payload = json.loads(manifest_path.read_text(encoding="utf-8"))
    except Exception as exc:
        print(f"Skipping {run.name}: {exc}")
        return None

    relative_dir = Path(run.parent.name) / run.name
    published = public_root / relative_dir
    published.mkdir(parents=True, exist_ok=True)
    shutil.copy2(manifest_path, published / "viewer.json")

    urls: dict[str, str | None] = {"wide_url": None, "narrow_url": None}
    for key, video_name in (
        ("wide_url", "front_wide.mp4"),
        ("narrow_url", "front_narrow.mp4"),
    ):
        source = run / video_name
        if source.exists() and source.stat().st_size:
            replace_link(source, published / video_name)
            urls[key] = f"/runs/{relative_dir.as_posix()}/{video_name}"

    metadata = payload.get("metadata") or {}
    duration = float(payload.get("timeline", {}).get("duration_s") or 0)
    status = str(metadata.get("status") or "unknown")
    return {
        "run_id": payload.get("run_id", run.name),
        "utc_start": metadata.get("utc_start"),
        "status": status,
        "duration_s": duration,
        "health_ok": bool(payload.get("health", {}).get("ok")),
        "label": f"{run.name} · {duration:.1f}s · {status}",
        "manifest_url": f"/runs/{relative_dir.as_posix()}/viewer.json",
        **urls,
    }


def build_index(data_root: Path, public_root: Path) -> list[dict[str, Any]]:
    public_root.mkdir(parents=True, exist_ok=True)
    options = []
    if data_root.exists():
        for metadata_path in data_root.glob("*/*/metadata.json"):
            option = option_for(metadata_path.parent, public_root)
            if option is not None:
                options.append(option)
    options.sort(key=lambda item: item.get("utc_start") or "", reverse=True)
    (public_root / "index.json").write_text(
        json.dumps({"runs": options}, indent=2), encoding="utf-8"
    )
    return options


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=Path, default=DEFAULT_DATA_ROOT)
    parser.add_argument("--public-root", type=Path, default=DEFAULT_PUBLIC_ROOT)
    args = parser.parse_args()
    options = build_index(args.data_root.resolve(), args.public_root.resolve())
    print(f"Indexed {len(options)} local run(s)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
