"""Jetson CSI camera recording through GStreamer hardware H.264 encoding."""

from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import Callable

from .config import CameraConfig
from .sync import FrameSample


class CameraRecorder:
    """Record one CSI camera and report source-buffer timestamps."""

    def __init__(self, config: CameraConfig, output: Path,
                 on_frame: Callable[[str, FrameSample], None], logger):
        try:
            import gi
            gi.require_version("Gst", "1.0")
            from gi.repository import Gst
        except (ImportError, ValueError) as exc:
            raise RuntimeError("GStreamer Python bindings are required") from exc
        Gst.init(None)
        self.Gst = Gst
        self.config = config
        self.output = output
        self.on_frame = on_frame
        self.log = logger
        self.pipeline = None
        self._first_frame = threading.Event()
        self._clock_offset_ns = None
        self._previous_pts_ns = None
        self._frame_index = -1
        self._expected_period_ns = round(1_000_000_000 / config.fps)
        self._error = None

    def pipeline_description(self) -> str:
        c = self.config
        source_options = [
            "sensor-id=%d" % c.sensor_id,
            "sensor-mode=%d" % c.sensor_mode,
        ]
        if c.exposure_range:
            source_options.append('exposuretimerange="%s"' % c.exposure_range)
        if c.gain_range:
            source_options.append('gainrange="%s"' % c.gain_range)
        source_options.append("wbmode=%d" % c.white_balance_mode)
        source = "nvarguscamerasrc " + " ".join(source_options)
        caps = ("video/x-raw(memory:NVMM),format=NV12,width=%d,height=%d,"
                "framerate=%d/1" % (c.width, c.height, c.fps))
        # The tee's appsink branch never maps image memory; it observes PTS only.
        # Encoding therefore remains entirely in NVMM and on Jetson hardware.
        return (
            "%s ! %s ! tee name=t "
            "t. ! queue max-size-buffers=120 ! "
            "appsink name=timestamp_sink emit-signals=true sync=false "
            "max-buffers=120 drop=false "
            "t. ! queue ! nvv4l2h264enc bitrate=%d iframeinterval=%d "
            "insert-sps-pps=true ! h264parse ! qtmux faststart=true ! "
            "filesink location=\"%s\" sync=false"
            % (source, caps, round(c.bitrate_mbps * 1_000_000), c.fps,
               str(self.output).replace("\\", "\\\\").replace('"', '\\"')))

    def start(self) -> None:
        Gst = self.Gst
        self.pipeline = Gst.parse_launch(self.pipeline_description())
        sink = self.pipeline.get_by_name("timestamp_sink")
        if sink is None:
            raise RuntimeError("timestamp appsink was not created")
        sink.connect("new-sample", self._on_sample)
        result = self.pipeline.set_state(Gst.State.PLAYING)
        if result == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError("camera %s failed to enter PLAYING" % self.config.name)

    def wait_first_frame(self, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        bus = self.pipeline.get_bus()
        mask = self.Gst.MessageType.ERROR
        while time.monotonic() < deadline:
            if self._first_frame.wait(timeout=0.05):
                return
            msg = bus.timed_pop_filtered(0, mask)
            if msg is not None:
                error, debug = msg.parse_error()
                raise RuntimeError("camera %s: %s (%s)" %
                                   (self.config.name, error, debug))
        raise RuntimeError("camera %s produced no frame within %.1f seconds" %
                           (self.config.name, timeout_s))

    def _on_sample(self, sink):
        sample = sink.emit("pull-sample")
        if sample is None:
            return self.Gst.FlowReturn.ERROR
        buffer = sample.get_buffer()
        pts = int(buffer.pts)
        now_ns = time.monotonic_ns()
        try:
            clock_now = int(self.pipeline.get_clock().get_time())
            base_time = int(self.pipeline.get_base_time())
            # Convert pipeline-running PTS to the Jetson monotonic domain while
            # preserving the buffer's age at callback time.
            buffer_age_ns = max(0, clock_now - (base_time + pts))
            timestamp_ns = now_ns - buffer_age_ns
        except Exception:
            if self._clock_offset_ns is None:
                self._clock_offset_ns = now_ns - pts
            timestamp_ns = self._clock_offset_ns + pts
        self._frame_index += 1
        dropped = 0
        if self._previous_pts_ns is not None:
            periods = max(1, round((pts - self._previous_pts_ns) /
                                   self._expected_period_ns))
            dropped = max(0, periods - 1)
        self._previous_pts_ns = pts
        frame = FrameSample(
            timestamp_ns=timestamp_ns,
            frame_index=self._frame_index,
            dropped_since_previous=dropped,
            capture_latency_ms=max(0.0, (now_ns - timestamp_ns) / 1_000_000),
        )
        try:
            self.on_frame(self.config.name, frame)
        except BaseException as exc:
            self._error = exc
            self.log.error("camera frame callback failed: %s" % exc)
            return self.Gst.FlowReturn.ERROR
        self._first_frame.set()
        return self.Gst.FlowReturn.OK

    def poll_error(self):
        if self._error is not None:
            return self._error
        bus = self.pipeline.get_bus()
        msg = bus.timed_pop_filtered(0, self.Gst.MessageType.ERROR)
        if msg is None:
            return None
        error, debug = msg.parse_error()
        return RuntimeError("camera %s: %s (%s)" %
                            (self.config.name, error, debug))

    @property
    def frame_count(self) -> int:
        return self._frame_index + 1

    def close(self) -> None:
        if self.pipeline is None:
            return
        Gst = self.Gst
        self.pipeline.send_event(Gst.Event.new_eos())
        bus = self.pipeline.get_bus()
        bus.timed_pop_filtered(
            10 * Gst.SECOND, Gst.MessageType.EOS | Gst.MessageType.ERROR)
        self.pipeline.set_state(Gst.State.NULL)
        self.pipeline = None
