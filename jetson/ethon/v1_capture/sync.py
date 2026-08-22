"""Pair wide-camera anchor frames with the nearest recent narrow frame."""

from __future__ import annotations

import threading
from dataclasses import dataclass


@dataclass(frozen=True)
class FrameSample:
    timestamp_ns: int
    frame_index: int
    dropped_since_previous: int
    capture_latency_ms: float


class FrameSynchronizer:
    def __init__(self, max_alignment_error_ms: float):
        self._max_error_ns = int(max_alignment_error_ms * 1_000_000)
        self._lock = threading.Lock()
        self._narrow: FrameSample | None = None

    def narrow(self, sample: FrameSample) -> None:
        with self._lock:
            self._narrow = sample

    def wide_row(self, wide: FrameSample) -> dict:
        with self._lock:
            narrow = self._narrow
        flags = []
        if wide.dropped_since_previous:
            flags.append("front_wide:%d" % wide.dropped_since_previous)
        if narrow is None:
            flags.append("front_narrow:missing")
            alignment_ns = None
            valid = False
        else:
            if narrow.dropped_since_previous:
                flags.append("front_narrow:%d" % narrow.dropped_since_previous)
            alignment_ns = abs(wide.timestamp_ns - narrow.timestamp_ns)
            valid = alignment_ns <= self._max_error_ns
            if not valid:
                flags.append("alignment")
        latencies = [wide.capture_latency_ms]
        if narrow is not None:
            latencies.append(narrow.capture_latency_ms)
        return {
            "timestamp_ns": wide.timestamp_ns,
            "front_wide_frame_index": wide.frame_index,
            "front_narrow_frame_index": narrow.frame_index if narrow else None,
            "front_narrow_timestamp_ns": narrow.timestamp_ns if narrow else None,
            "dropped_frame_flags": ",".join(flags),
            "capture_latency_ms": max(latencies),
            "front_wide_capture_latency_ms": wide.capture_latency_ms,
            "front_narrow_capture_latency_ms": (
                narrow.capture_latency_ms if narrow else None),
            "alignment_error_ms": (
                alignment_ns / 1_000_000 if alignment_ns is not None else None),
            "alignment_valid": valid,
        }
