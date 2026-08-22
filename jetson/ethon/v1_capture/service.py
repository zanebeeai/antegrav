"""ROS orchestration for synchronized v1 driving-data capture."""

from __future__ import annotations

import json
import signal
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty, String

from .camera import CameraRecorder
from .config import CaptureConfig
from .metadata import create_run, write_metadata
from .parquet import (EVENT_FIELDS, FRAME_FIELDS, TELEMETRY_FIELDS,
                      AsyncParquetWriter)
from .storage import assert_enough_space, below_runtime_reserve, disk_free_gb
from .sync import FrameSample, FrameSynchronizer

TOPIC_TELEMETRY = "/ethon/capture/telemetry"
TOPIC_GPS = "/ethon/gps_status"
TOPIC_CAPTURE_STATUS = "/ethon/capture/status"
TOPIC_MANUAL_ENABLE = "/ethon/capture/manual_enable"
TOPIC_EVENT = "/ethon/capture/event"


class CaptureService(Node):
    def __init__(self, cfg: CaptureConfig, repo: Path):
        super().__init__("ethon_v1_capture")
        self.cfg = cfg
        self.repo = repo
        self.log = self.get_logger()
        self._closing = False
        self._stop_requested = False
        self._healthy = False
        self._activated = False
        self._telemetry_seen = False
        self._last_telemetry_time = 0.0
        self._last_good_ctre_time = 0.0
        self._stop_reason = "operator_stop"
        self._latest_gps = {}
        self._mode = "manual"
        self._estop = False
        self._started_monotonic = time.monotonic()
        self._cameras = []

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._status_pub = self.create_publisher(String, TOPIC_CAPTURE_STATUS, latched)
        self._manual_pub = self.create_publisher(Bool, TOPIC_MANUAL_ENABLE, 10)
        self._publish_status("starting")

        free_gb, estimate = assert_enough_space(cfg)
        self.run_id, self.run_dir, self.metadata = create_run(cfg, repo)
        self.metadata["storage_preflight"] = {
            "free_gb": round(free_gb, 3),
            "required_free_gb": round(estimate.required_free_gb, 3),
            "guarded_capture_gb": round(estimate.guarded_capture_gb, 3),
            "reserve_gb": estimate.reserve_gb,
        }
        write_metadata(self.run_dir, self.metadata)

        row_group = cfg.parquet_row_group_size
        self.frames = AsyncParquetWriter(
            self.run_dir / "frames.parquet", FRAME_FIELDS, row_group)
        self.telemetry = AsyncParquetWriter(
            self.run_dir / "telemetry.parquet", TELEMETRY_FIELDS, row_group)
        self.events = AsyncParquetWriter(
            self.run_dir / "events.parquet", EVENT_FIELDS, row_group)
        self.sync = FrameSynchronizer(cfg.max_alignment_error_ms)

        self.create_subscription(String, TOPIC_TELEMETRY, self._on_telemetry, 100)
        sensor_qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(String, TOPIC_GPS, self._on_gps, sensor_qos)
        self.create_subscription(String, TOPIC_EVENT, self._on_custom_event, 20)
        self.create_subscription(Empty, "/ethon/lap/mark", self._on_lap, 10)
        self.create_subscription(String, "/ethon/hmi/mode", self._on_mode, 10)
        self.create_subscription(Bool, "/ethon/hmi/armed", self._on_armed, latched)
        self.create_subscription(Bool, "/ethon/estop", self._on_estop, 10)
        self.create_subscription(Bool, "/ethon/estop", self._on_estop, latched)

        for name in ("front_wide", "front_narrow"):
            camera = CameraRecorder(
                cfg.cameras[name], self.run_dir / (name + ".mp4"),
                self._on_frame, self.log)
            self._cameras.append(camera)
            camera.start()
        for camera in self._cameras:
            camera.wait_first_frame(cfg.camera_start_timeout_s)

        self.create_timer(min(0.1, cfg.heartbeat_timeout_s / 3.0), self._heartbeat)
        self.create_timer(1.0, self._monitor)
        self.log.info("recording v1 run %s in %s" % (self.run_id, self.run_dir))

    def _on_frame(self, name: str, sample: FrameSample) -> None:
        if name == "front_narrow":
            self.sync.narrow(sample)
        else:
            self.frames.append(self.sync.wide_row(sample))

    def _on_telemetry(self, msg: String) -> None:
        try:
            row = json.loads(msg.data)
        except (TypeError, ValueError):
            self._event("can_fault", "malformed_telemetry", msg.data[:200])
            return
        now = time.monotonic()
        self._last_telemetry_time = now
        latency = row.get("ctre_latency_ms")
        critical_ok = all(row.get(name) is not None for name in (
            "cancoder_position_rad", "cancoder_velocity_rad_s",
            "steering_motor_position", "steering_motor_velocity"))
        ctre_ok = (critical_ok and isinstance(latency, (int, float)) and
                   0.0 <= latency <= self.cfg.max_ctre_latency_ms)
        row["ctre_alignment_valid"] = ctre_ok
        if ctre_ok:
            self._telemetry_seen = True
            self._last_good_ctre_time = now
        gps = self._latest_gps
        row.update({
            "gps_latitude": gps.get("latitude"),
            "gps_longitude": gps.get("longitude"),
            "gps_heading": gps.get("heading_deg"),
            "gps_heading_timestamp_ns": gps.get("heading_timestamp_ns"),
            "gps_fix_quality": gps.get("fix_quality"),
            "gps_timestamp_ns": gps.get("timestamp_ns"),
            "gps_fix_timestamp_ns": gps.get("fix_timestamp_ns"),
            "manual_or_auto": row.get("manual_or_auto", self._mode),
            "estop": self._estop,
        })
        self.telemetry.append(row)

    def _on_gps(self, msg: String) -> None:
        try:
            value = json.loads(msg.data)
            if isinstance(value, dict):
                self._latest_gps = value
        except (TypeError, ValueError):
            pass

    def _on_custom_event(self, msg: String) -> None:
        try:
            value = json.loads(msg.data)
            self._event(str(value["event_type"]), str(value.get("event_value", "")),
                        str(value.get("notes", "")))
        except (KeyError, TypeError, ValueError):
            self._event("driver_marked_bad_data", "malformed_event", msg.data[:200])

    def _on_lap(self, _msg: Empty) -> None:
        self._event("lap_boundary", "wheel_mark", "")

    def _on_mode(self, msg: String) -> None:
        previous = self._mode
        self._mode = "auto" if msg.data == "autonomy" else "manual"
        if self._mode != previous:
            event = ("manual_to_autonomous" if self._mode == "auto"
                     else "autonomous_to_manual_takeover")
            self._event(event, self._mode, "HMI mode transition")

    def _on_armed(self, msg: Bool) -> None:
        new_mode = "auto" if bool(msg.data) else "manual"
        if new_mode != self._mode:
            event = ("manual_to_autonomous" if new_mode == "auto"
                     else "autonomous_to_manual_takeover")
            self._mode = new_mode
            self._event(event, new_mode, "armed-state transition")

    def _on_estop(self, msg: Bool) -> None:
        if bool(msg.data) and not self._estop:
            self._event("emergency_stop", "true", "")
        self._estop = self._estop or bool(msg.data)

    def _event(self, event_type: str, event_value: str, notes: str) -> None:
        self.events.append({
            "timestamp_ns": time.monotonic_ns(),
            "event_type": event_type,
            "event_value": event_value,
            "notes": notes,
        })

    def _heartbeat(self) -> None:
        self._manual_pub.publish(Bool(data=bool(self._healthy and not self._closing)))

    def _publish_status(self, state: str, error: str = "") -> None:
        started = getattr(self, "_started_monotonic", time.monotonic())
        payload = {
            "timestamp_ns": time.monotonic_ns(),
            "state": state,
            "run_id": getattr(self, "run_id", None),
            "elapsed_s": max(0.0, time.monotonic() - started),
            "free_gb": round(disk_free_gb(self.cfg.data_root), 2),
            "error": error,
        }
        self._status_pub.publish(String(data=json.dumps(payload, separators=(",", ":"))))

    @property
    def stop_requested(self) -> bool:
        return self._stop_requested

    def request_stop(self, reason: str, state: str = "stopping",
                     error: str = "") -> None:
        """Request a clean spin-loop exit while the ROS context remains valid."""
        if self._stop_requested:
            return
        self._stop_reason = reason
        self._healthy = False
        self._stop_requested = True
        if state:
            self._publish_status(state, error)

    def _monitor(self) -> None:
        if self._closing:
            return
        if not self._activated:
            if self._telemetry_seen:
                self._activated = True
                self._healthy = True
                self.metadata["status"] = "recording"
                self.metadata["utc_recording_started"] = \
                    datetime.now(timezone.utc).isoformat()
                write_metadata(self.run_dir, self.metadata)
                self._event("recording_start", self.run_id,
                            "both cameras, telemetry, and writers healthy")
            elif (time.monotonic() - self._started_monotonic >
                  self.cfg.telemetry_start_timeout_s):
                error = "no high-rate drive telemetry received"
                self._event("can_fault", "telemetry_timeout", error)
                self.request_stop("telemetry_fault", "fault", error)
                return
        elif time.monotonic() - self._last_good_ctre_time > 1.0:
            error = "fresh CANcoder/steering telemetry lost"
            self._event("can_fault", "stale_ctre", error)
            self.request_stop("telemetry_fault", "fault", error)
            return
        for writer in (self.frames, self.telemetry, self.events):
            try:
                writer.check()
            except RuntimeError as error:
                self.request_stop("writer_fault", "fault", str(error))
                return
        for camera in self._cameras:
            error = camera.poll_error()
            if error is not None:
                self._event("camera_fault", camera.config.name, str(error))
                self.request_stop("camera_fault", "fault", str(error))
                return
        if below_runtime_reserve(self.cfg):
            self._event("capture_fault", "low_disk", "runtime reserve reached")
            self.request_stop("low_disk", "low_space", "runtime reserve reached")
            return
        if time.monotonic() - self._started_monotonic >= self.cfg.max_duration_minutes * 60:
            self.request_stop("duration_limit")
            return
        self._publish_status("recording" if self._activated else "starting")

    def close(self) -> None:
        if self._closing:
            return
        self._closing = True
        self._healthy = False
        errors = []

        def attempt(label, operation):
            try:
                operation()
            except Exception as exc:
                errors.append("%s: %s" % (label, exc))
                self.log.error("capture finalization %s failed: %s" % (label, exc))

        if rclpy.ok():
            attempt("manual disable",
                    lambda: self._manual_pub.publish(Bool(data=False)))
            attempt("stopping status", lambda: self._publish_status("stopping"))
        attempt("stop event",
                lambda: self._event("recording_stop", self._stop_reason, ""))
        for camera in self._cameras:
            attempt("%s video" % camera.config.name, camera.close)
        for name, writer in (("frames", self.frames),
                             ("telemetry", self.telemetry),
                             ("events", self.events)):
            attempt("%s parquet" % name, writer.close)

        normal_stop = self._stop_reason in ("operator_stop", "duration_limit")
        self.metadata.update({
            "status": "complete" if normal_stop and not errors else "fault",
            "stop_reason": self._stop_reason,
            "utc_stop": datetime.now(timezone.utc).isoformat(),
            "duration_s": round(time.monotonic() - self._started_monotonic, 3),
            "rows": {
                "frames": self.frames.rows_written,
                "telemetry": self.telemetry.rows_written,
                "events": self.events.rows_written,
            },
            "camera_frames": {
                camera.config.name: camera.frame_count for camera in self._cameras
            },
            "free_gb_at_stop": round(disk_free_gb(self.cfg.data_root), 3),
        })
        if errors:
            self.metadata["finalization_errors"] = errors
        attempt("metadata", lambda: write_metadata(self.run_dir, self.metadata))
        if rclpy.ok():
            final_state = "idle" if self.metadata["status"] == "complete" else "fault"
            attempt("final status", lambda: self._publish_status(
                final_state, "; ".join(errors)))


def run(cfg: CaptureConfig, repo: Path) -> int:
    rclpy.init()
    node = None
    signal_stop_requested = False

    def stop(_signum, _frame):
        nonlocal signal_stop_requested
        signal_stop_requested = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        node = CaptureService(cfg, repo)
        while (rclpy.ok() and not signal_stop_requested and
               not node.stop_requested):
            rclpy.spin_once(node, timeout_sec=0.1)
        if signal_stop_requested and not node.stop_requested:
            node.request_stop("operator_stop")
        return 0
    except (KeyboardInterrupt, ExternalShutdownException):
        return 0
    except Exception as exc:
        if node is not None:
            node._stop_reason = "startup_fault"
            if rclpy.ok():
                node._publish_status("fault", str(exc))
        print("v1 capture failed: %s" % exc, flush=True)
        return 1
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
