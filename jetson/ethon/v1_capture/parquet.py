"""Bounded asynchronous Parquet writers for capture callbacks."""

from __future__ import annotations

import queue
import threading
from pathlib import Path
from typing import Any, Dict, Iterable


FRAME_FIELDS = (
    ("timestamp_ns", "int64"),
    ("front_wide_frame_index", "int64"),
    ("front_narrow_frame_index", "int64"),
    ("front_narrow_timestamp_ns", "int64"),
    ("dropped_frame_flags", "string"),
    ("capture_latency_ms", "float64"),
    ("front_wide_capture_latency_ms", "float64"),
    ("front_narrow_capture_latency_ms", "float64"),
    ("alignment_error_ms", "float64"),
    ("alignment_valid", "bool"),
)

TELEMETRY_FIELDS = (
    ("timestamp_ns", "int64"),
    ("ctre_timestamp_ns", "int64"),
    ("ctre_latency_ms", "float64"),
    ("ctre_alignment_valid", "bool"),
    ("cancoder_position_rad", "float64"),
    ("cancoder_velocity_rad_s", "float64"),
    ("steering_motor_position", "float64"),
    ("steering_motor_velocity", "float64"),
    ("steering_target", "float64"),
    ("steering_voltage", "float64"),
    ("steering_current", "float64"),
    ("drive_1_velocity", "float64"),
    ("drive_2_velocity", "float64"),
    ("drive_3_velocity", "float64"),
    ("drive_1_current", "float64"),
    ("drive_2_current", "float64"),
    ("drive_3_current", "float64"),
    ("supply_voltage", "float64"),
    ("vehicle_speed_m_s", "float64"),
    ("pedal_fraction", "float64"),
    ("pedal_sample_timestamp_ns", "int64"),
    ("gps_latitude", "float64"),
    ("gps_longitude", "float64"),
    ("gps_heading", "float64"),
    ("gps_heading_timestamp_ns", "int64"),
    ("gps_fix_quality", "int32"),
    ("gps_timestamp_ns", "int64"),
    ("gps_fix_timestamp_ns", "int64"),
    ("manual_or_auto", "string"),
    ("estop", "bool"),
    ("can_faults", "string"),
)

EVENT_FIELDS = (
    ("timestamp_ns", "int64"),
    ("event_type", "string"),
    ("event_value", "string"),
    ("notes", "string"),
)


class AsyncParquetWriter:
    """Write rows off the sensor callback threads with bounded memory."""

    def __init__(self, path: Path, fields: Iterable[tuple[str, str]],
                 row_group_size: int, queue_size: int = 100_000):
        try:
            import pyarrow as pa
            import pyarrow.parquet as pq
        except ImportError as exc:
            raise RuntimeError("pyarrow is required for v1 capture") from exc
        self._pa = pa
        self._pq = pq
        self.path = path
        self.fields = tuple(fields)
        arrow_type = {"bool": pa.bool_}
        self.schema = pa.schema([
            (name, (arrow_type[kind] if kind in arrow_type else getattr(pa, kind))())
            for name, kind in self.fields
        ])
        self._row_group_size = row_group_size
        self._queue: queue.Queue = queue.Queue(maxsize=queue_size)
        self._error: BaseException | None = None
        self._rows_written = 0
        self._thread = threading.Thread(target=self._run, name="parquet-%s" % path.stem,
                                        daemon=True)
        self._thread.start()

    @property
    def rows_written(self) -> int:
        return self._rows_written

    def check(self) -> None:
        if self._error is not None:
            raise RuntimeError("Parquet writer failed for %s" % self.path.name) \
                from self._error

    def append(self, row: Dict[str, Any]) -> None:
        if self._error is not None:
            raise RuntimeError("Parquet writer failed") from self._error
        try:
            self._queue.put_nowait(row)
        except queue.Full as exc:
            raise RuntimeError("Parquet queue full for %s" % self.path.name) from exc

    def _run(self) -> None:
        writer = None
        batch = []
        try:
            writer = self._pq.ParquetWriter(
                str(self.path), self.schema, compression="zstd", use_dictionary=True)
            while True:
                item = self._queue.get()
                if item is None:
                    break
                batch.append({name: item.get(name) for name, _ in self.fields})
                if len(batch) >= self._row_group_size:
                    self._flush(writer, batch)
                    batch.clear()
            if batch:
                self._flush(writer, batch)
        except BaseException as exc:
            self._error = exc
        finally:
            if writer is not None:
                writer.close()

    def _flush(self, writer, rows) -> None:
        table = self._pa.Table.from_pylist(rows, schema=self.schema)
        writer.write_table(table, row_group_size=len(rows))
        self._rows_written += len(rows)

    def close(self) -> None:
        self._queue.put(None)
        self._thread.join(timeout=30)
        if self._thread.is_alive():
            raise RuntimeError("timed out finalizing %s" % self.path.name)
        if self._error is not None:
            raise RuntimeError("failed writing %s" % self.path.name) from self._error
