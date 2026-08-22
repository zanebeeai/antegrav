#!/usr/bin/env python3
"""
Bird's-eye fusion v2 — multi-camera detection -> ground-plane projection.

Owns every perception camera on the vehicle (one process, one GPU context):
  * Jetson CSI cams (nvarguscamerasrc sensor-id 0 = narrow, IMX219;
    sensor-id 1 = wide, HQ IMX477 + fisheye)
  * perc-1 Pi5 TCP MJPEG streams (4-byte big-endian length prefix + JPEG)
    on port 5001 = left and port 5002 = right, both Camera Module 3 Wide
    (imx708), reachable via LAN or Tailscale.
    2026-08-11: cameras are grouped by which platform has a working driver
    and ISP tuning for them, NOT by what they point at — see the SOURCES
    comment below for why, and for the pinned sensor modes.

Every capture source runs a background thread that fills a 1-slot
latest-frame buffer (old frames dropped), so the 10 Hz fusion loop never
blocks on I/O.  One shared YOLO TensorRT model runs inference round-robin
over all sources each tick; sources without a fresh frame are skipped.

Ground projection: a detection is assumed to touch the ground at the
bottom-center of its bounding box (true for cones, people, vehicles); that
pixel maps through the camera's 3x3 homography to (x, y) metres in
base_link (x forward, y left, origin = rear axle center).

Calibration (see calibrate_homography.py), in /home/jetson/ethon/calib/:
  CSI source N  -> cam{N}_H.npy
  TCP port P    -> cam_tcp{P}_H.npy
A source with no homography still runs, but its detections are published
ONLY on the raw debug topic — pixel coordinates must never reach the
planner topics.  Missing files are re-checked every 10 s, so calibrating
does not require a node restart.

Publishes:
  /ethon/obstacles       PoseArray  all calibrated detections (class id in z)
  /ethon/cones           PoseArray  cone-class calibrated detections only
  /ethon/curb_points     PoseArray  classical-CV road/curb edge points (side
                                    in z: +1 left, -1 right). v1, 2026-08-13,
                                    see curb_detect.py -- visualisation/tuning
                                    only, NOT YET consumed by the planner.
  /ethon/detections_raw  String     one JSON object per detection (all
                                    sources, calibrated or not):
                                    {source, cls, conf, bbox:[x1,y1,x2,y2]}
  /ethon/fusion_status   String     1 Hz JSON health: per-source alive /
                                    calibrated / fps / detection counts

Parameters:
  model_path      (string) TRT engine path, default ethon_v1.engine
  enable_<source> (bool)   per-source enable flag, default True
"""

import json
import os
import signal
import socket
import struct
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

import curb_detect
import orange_cone
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import String

from ultralytics import YOLO

# ---------------------------------------------------------------- constants
DEFAULT_MODEL_PATH = "/home/jetson/ethon/models/ethon_v1.engine"
CALIB_DIR          = Path("/home/jetson/ethon/calib")
FRAME_ID           = "base_link"

CONF_THRESHOLD = 0.45
IMG_SIZE       = 640
# ── annotated preview for the web dashboard ───────────────────────────────
# Each source's latest frame is drawn with its detections and written as JPEG
# to tmpfs, where web_dashboard.py serves it at /api/cam. tmpfs so this never
# touches the eMMC (this runs at ~10 Hz for the life of the race). Writes are
# atomic (temp + os.replace) so the dashboard can never read a half-file.
ANNOT_DIR      = "/dev/shm"
ANNOT_PREFIX   = "ethon_cam_"
ANNOT_MAX_W    = 800          # downscale before encoding. 800 keeps small
                              # distant cones legible on a laptop screen while
                              # still encoding in ~6 ms (vs ~14 at full 1280).
ANNOT_QUALITY  = 85           # the source frame is ALREADY JPEG from the
                              # camera, so this is a second compression pass —
                              # too low and the ringing around cone edges gets
                              # bad enough to misread what the model saw.
# BGR per class id, matching the 14-class ethon_v1 order. Cone is deliberately
# the loudest since it is the only class the planner steers by.
ANNOT_COLORS = {
    0: (0, 200, 255),   1: (255, 160, 60),  2: (60, 60, 255),
    3: (0, 140, 255),   4: (200, 200, 60),  5: (255, 255, 80),
    6: (140, 140, 140), 7: (60, 220, 60),   8: (200, 60, 200),
    9: (60, 200, 200), 10: (120, 120, 120), 11: (0, 180, 220),
    12: (0, 0, 255),   13: (255, 120, 60),
}
ANNOT_NAMES = ['cone', 'car', 'person', 'pothole', 'traffic_sign',
               'traffic_light', 'barrier', 'cyclist', 'motorcycle', 'animal',
               'debris', 'construction', 'emergency', 'truck']
# Two cameras seeing the same object publish it twice, offset by however much
# their calibrations disagree. Anything closer than this (metres, ground
# frame) and of the same class is treated as one object. Must stay comfortably
# BELOW the real spacing between adjacent track cones — merging two genuine
# cones into one would widen the apparent corridor, which is the dangerous
# direction — and ABOVE plausible calibration disagreement.
MERGE_RADIUS_M = 0.30
USE_HALF       = True
CONE_CLS       = 0            # ethon_v1 / road_v1 class 0 = cone
# Ground-plausibility window for colour-detected cones (see _process_orange).
# Chosen to match the planner's own max_cone_range_m (12.0) and
# max_cone_lateral_m (6.0) so fusion does not publish cones the planner would
# only discard. MIN_X keeps the car's own orange bodywork and anything
# projecting behind the front axle out.
CONE_MIN_X_M     = 0.5
CONE_MAX_X_M     = 12.0
CONE_MAX_ABS_Y_M = 6.0

FUSION_PERIOD_S  = 0.10       # 10 Hz fusion loop
STATUS_PERIOD_S  = 1.0        # 1 Hz /ethon/fusion_status
HOUSEKEEP_PERIOD_S = 10.0     # stats log + homography re-check

CSI_WIDTH, CSI_HEIGHT, CSI_FPS = 1280, 720, 30
CSI_RETRY_S = 10.0            # re-open attempt period for a dead CSI cam

# Tried in order, rotating on failure with TCP_BACKOFF_S between attempts.
# The mDNS name is FIRST on purpose: perc-1 takes a DHCP lease, so a
# hardcoded IP goes stale the moment the router hands out a different one
# (192.168.0.101 below is exactly that — a leftover from an old subnet).
# avahi-daemon is installed on perc-1 and the Jetson resolves .local via
# nsswitch mdns4_minimal, so the name is the durable way to find it.
PERC1_HOSTS = [
    "10.10.10.2",           # DIRECT ETHERNET CABLE, Jetson eno1 <-> perc-1
                            # eth0 (10.10.10.1 <-> .2, /24, static, no gateway,
                            # never-default). Permanent on-vehicle wiring as of
                            # 2026-08-11, so this is the primary path: it is
                            # point-to-point, so there is no DHCP server, no
                            # other host, and no wifi contention -- ~0.2 ms and
                            # it works with wlan0 down entirely.
    "perception-1.local",   # mDNS fallback, if perc-1 is also on wifi
    "100.107.192.42",       # Tailscale (node registers as perception-1-1
                            # because the dead node still holds the name)
]
# The 10.10.10.2 entry above is a STATICALLY ASSIGNED point-to-point address on
# a dedicated cable -- it cannot be reassigned to anything else, because there
# is nothing else on that link. That is NOT the same as hardcoding a DHCP
# lease, which this list used to do and which caused a real failure: a
# "192.168.2.61  # current LAN lease" entry sat here until 2026-08-11, by which
# time the router had handed that address to an unrelated Pi (hostname
# sentryusb). A leased IP names an ADDRESS, not a HOST; if whatever currently
# holds it happened to serve anything on 5001/5002, fusion would ingest a
# stranger's frames and, once calibrated, feed them to the planner as real cone
# positions. Never reintroduce a DHCP-assigned address here. The other two
# entries are identity-bound (mDNS name, Tailscale node) and cannot drift.
TCP_BACKOFF_S         = 3.0   # reconnect backoff (rotates through hosts)
TCP_CONNECT_TIMEOUT_S = 2.0
TCP_RECV_TIMEOUT_S    = 5.0
TCP_MAX_JPEG_BYTES    = 8 * 1024 * 1024            # length-prefix sanity cap
LEN_PREFIX            = struct.Struct(">I")


# ---------------------------------------------------------- source registry
# ── fisheye support ───────────────────────────────────────────────────────
# A homography is a PINHOLE construct: it assumes straight lines stay
# straight. The HQ's wide lens is a 184.6-deg-diagonal fisheye, where they do
# not — radial position departs from the pinhole assumption by ~4% at 20 deg
# off-axis, ~20% at 40 deg and ~65% at 60 deg. Fitting H straight onto raw
# fisheye pixels is therefore only valid in a small central patch, which
# defeats the point of a wide lens.
#
# Fix: undistort the DETECTION POINT (not the whole frame — only a handful of
# points per tick, so the cost is negligible) into normalised pinhole
# coordinates, and fit/apply H in that space. Whether a source does this is
# decided by its calibration metadata, so the calibrator and this node can
# never silently disagree about which space H lives in.
UNDISTORT_MAX_NORM = 5.0     # |x'| or |y'| beyond this ~= 79 deg off-axis.
                             # Rectilinear coords blow up towards 90 deg and
                             # cannot represent anything past it, so drop
                             # those detections rather than emit a wild
                             # position. This lens reaches 92 deg diagonal.


def undistort_points_fisheye(pix, K, D):
    """Fisheye pixels -> normalised pinhole coords. None if unrepresentable.

    `pix` is an (N, 2) array. Returns an (N, 2) array of (x', y') = (X/Z, Y/Z)
    with entries set to NaN where the point sits too far off-axis to be
    expressed rectilinearly.
    """
    pts = np.asarray(pix, dtype=np.float64).reshape(-1, 1, 2)
    und = cv2.fisheye.undistortPoints(pts, K, D).reshape(-1, 2)
    bad = ~np.isfinite(und).all(axis=1) | (
        np.abs(und) > UNDISTORT_MAX_NORM).any(axis=1)
    und[bad] = np.nan
    return und


def _merge_duplicates(poses, radius):
    """Collapse same-class detections lying within `radius` into one pose.

    Greedy single-pass clustering: walk the poses, and for each one either
    attach it to an existing cluster of the same class whose running centroid
    is within `radius`, or start a new cluster. Each cluster is published as
    the mean of its members, which also averages away some of the per-camera
    calibration disagreement.

    Greedy (not exhaustive) is deliberate: detection counts per tick are small
    (tens), this is O(n * clusters) with no allocation churn, and it runs
    inside the 10 Hz perception loop where a pathological clustering cost
    would delay the whole path. Class comes from pose.position.z, which is
    where the class id is smuggled.
    """
    if not poses:
        return poses
    r2 = radius * radius
    clusters = []          # [cls, sum_x, sum_y, n]
    for p in poses:
        cls = p.position.z
        px, py = p.position.x, p.position.y
        hit = None
        for c in clusters:
            if c[0] != cls:
                continue
            dx = px - c[1] / c[3]
            dy = py - c[2] / c[3]
            if dx * dx + dy * dy <= r2:
                hit = c
                break
        if hit is None:
            clusters.append([cls, px, py, 1])
        else:
            hit[1] += px
            hit[2] += py
            hit[3] += 1
    out = []
    for cls, sx, sy, n in clusters:
        q = Pose()
        q.position.x = float(sx / n)
        q.position.y = float(sy / n)
        q.position.z = float(cls)
        out.append(q)
    return out


@dataclass(frozen=True)
class SourceSpec:
    """Static description of one camera source."""
    name: str                              # unique label, used in topics/logs
    kind: str                              # "csi" | "tcp"
    sensor_id: int = -1                    # CSI only
    sensor_mode: int = -1                  # CSI only; -1 = let argus choose.
                                           # PIN THIS on any sensor that has a
                                           # cropped mode -- see gst_pipeline.
    fps: int = 0                           # CSI only; 0 = CSI_FPS. Must not
                                           # exceed the PINNED mode's max rate
                                           # or the pipeline fails to open.
    hosts: Tuple[str, ...] = ()            # TCP only, tried in order
    port: int = -1                         # TCP only
    enabled: bool = True                   # default; overridable via ROS param

    @property
    def h_file(self) -> Path:
        if self.kind == "csi":
            return CALIB_DIR / f"cam{self.sensor_id}_H.npy"
        return CALIB_DIR / f"cam_tcp{self.port}_H.npy"

    @property
    def key(self) -> str:
        """Calibration filename stem. Must match calibrate_homography.py."""
        return (f"cam{self.sensor_id}" if self.kind == "csi"
                else f"cam_tcp{self.port}")

    @property
    def meta_file(self) -> Path:
        """Says which space H lives in (raw pixels vs undistorted)."""
        return CALIB_DIR / f"{self.key}_H.json"

    @property
    def intrinsics_file(self) -> Path:
        return CALIB_DIR / f"{self.key}_fisheye.npz"


SOURCES: List[SourceSpec] = [
    # Names describe WHERE THE CAMERA POINTS, not which connector it uses.
    # Renaming is safe: calibration files are keyed off sensor_id/port (see
    # SourceSpec.h_file), not off this name.
    #
    # 2026-08-11: cameras are grouped by WHICH PLATFORM SUPPORTS THEM, not by
    # what they point at. An imx708 (Camera Module 3 Wide) was briefly run on
    # the Jetson and rendered badly: it binds the MAINLINE v4l2 driver
    # (kernel/drivers/media/i2c/imx708.ko) rather than an NVIDIA nv_* Tegra
    # driver, so argus gets no ISP tuning for it and never subtracts the
    # sensor's black-level pedestal. Measured with the lens fully covered it
    # produced a flat 103/255 grey instead of black, and that offset SCALED
    # WITH AE GAIN (51 uncovered vs 103 covered) -- so no fixed correction
    # could undo it. imx477 and imx219 both use NVIDIA's own drivers and
    # render correctly here; the imx708s render correctly on perc-1, where
    # libcamera ships a real imx708.json tuning file. Hence:
    #   Jetson  -- imx477 + imx219   (NVIDIA-supported)
    #   perc-1  -- both imx708       (libcamera-supported)
    #
    #   narrow  sensor_id=0 -- IMX219 (~62deg) on CAM0 / i2c bus 10 / A.
    #           Narrow field puts more pixels on distant cones, so this is
    #           the long-range one. Rectilinear enough that a plain pinhole
    #           homography holds; no fisheye intrinsics needed.
    #   wide    sensor_id=1 -- HQ IMX477 + 184.6deg-diagonal fisheye, on
    #           CAM1 / i2c bus 9 / C. Its homography MUST be solved in
    #           UNDISTORTED space (see undistort_points_fisheye); a plain
    #           pinhole homography is badly wrong off-axis.
    #   left    :5001 -- perc-1 Camera Module 3 Wide (imx708, ~120deg)
    #   right   :5002 -- perc-1 Camera Module 3 Wide (imx708, ~120deg)
    #
    # sensor_id 0/1 are the reverse of the obvious reading of CAM0/CAM1 --
    # they follow the tegra-camera-platform module order, verified from
    # /proc/device-tree/tegra-camera-platform/modules/moduleN/badge
    # (RBP194 = imx219 = module0, RBPCV3 = imx477 = module1). Do not infer
    # sensor_id from the /dev/videoN number; it has been observed to differ.
    #
    # BOTH sensor_modes are PINNED, and this is calibration-critical: argus
    # otherwise picks the smallest mode satisfying CSI_WIDTH x CSI_HEIGHT,
    # which on both of these sensors is a reduced-FOV mode (see gst_pipeline).
    #   imx219 mode1 = 3280x1848, full sensor WIDTH at 16:9. Left on auto it
    #     would take its native 1280x720 mode, a heavy centre crop. That mode
    #     caps at 28 fps, hence the explicit fps below -- asking for the
    #     default 30 makes argus refuse to open the stream entirely.
    #   imx477 mode0 = 3840x2160, the widest mode exposed. Left on auto it
    #     would take 1920x1080. 30 fps is within this mode, so no fps override.
    # Changing either mode after calibration silently invalidates that
    # camera's homography -- the FOV moves under it. Re-check with
    # `v4l2-ctl -d /dev/videoN --list-formats-ext` if a camera or the boot
    # overlay is ever changed, and re-calibrate if a mode changes.
    SourceSpec(name="narrow", kind="csi", sensor_id=0, sensor_mode=1, fps=28),
    SourceSpec(name="wide", kind="csi", sensor_id=1, sensor_mode=0),
    SourceSpec(name="left", kind="tcp", hosts=tuple(PERC1_HOSTS), port=5001),
    SourceSpec(name="right", kind="tcp", hosts=tuple(PERC1_HOSTS), port=5002),
]


def gst_pipeline(sensor_id: int, sensor_mode: int = -1, fps: int = 0) -> str:
    """nvarguscamerasrc pipeline delivering BGR frames, latest-only appsink.

    sensor_mode PINS the sensor readout mode. This matters enormously and is
    invisible if you only look at the output size: left to itself, argus picks
    the SMALLEST sensor mode that can satisfy the requested caps, and on these
    sensors that is a reduced-FOV mode. The result looks like a perfectly
    correct 1280x720 picture with a chunk of the field of view silently
    missing. Modes below are in v4l2/device-tree enumeration order, which is
    also the sensor-mode index (verified against
    /proc/device-tree/.../modeN/active_w on 2026-08-11):

        imx219 (narrow)                    imx477 (wide)
        0  3280x2464 @21  full, 4:3        0  3840x2160 @30  widest  <- pinned
        1  3280x1848 @28  full width 16:9  1  1920x1080 @60  reduced
           <- pinned
        2  1920x1080 @30  crop
        3  1640x1232 @30  binned full, 4:3
        4  1280x720  @60  heavy crop  <- what argus would auto-pick

    imx219 mode 1 is chosen over mode 0/3 because it is already 16:9 at the
    full sensor width: scaling it to CSI_WIDTH x CSI_HEIGHT preserves aspect,
    whereas scaling a 4:3 mode into a 16:9 output would stretch the image
    non-uniformly and put a distortion in front of the homography that the
    homography cannot represent.

    Same failure mode, same fix, as fleet_stream.py's --raw-width/--raw-height
    on perc-1.

    `fps` must not exceed the PINNED mode's max frame rate, or argus fails to
    create the stream and the source never opens at all (observed 2026-08-11:
    imx219 mode 1 caps at 28 fps, and the hardcoded 30 made it fail while the
    imx477 -- whose pinned mode does 30 -- came up fine). Fusion consumes at
    FUSION_PERIOD_S (10 Hz) regardless, so a lower sensor rate costs nothing.
    """
    mode = f"sensor-mode={sensor_mode} " if sensor_mode >= 0 else ""
    rate = fps if fps > 0 else CSI_FPS
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} {mode}wbmode=1 ! "
        f"video/x-raw(memory:NVMM),width={CSI_WIDTH},height={CSI_HEIGHT},"
        f"framerate={rate}/1 ! "
        "nvvidconv ! video/x-raw,format=BGRx ! "
        "videoconvert ! video/x-raw,format=BGR ! "
        "appsink drop=true max-buffers=1 sync=false"
    )


def _recv_exact(sock: socket.socket, n: int) -> bytes:
    """Read exactly n bytes or raise ConnectionError on EOF."""
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("peer closed connection")
        buf.extend(chunk)
    return bytes(buf)


# ----------------------------------------------------------- frame sources
class FrameSource:
    """Background-threaded capture with a 1-slot latest-frame buffer."""

    def __init__(self, spec: SourceSpec, logger):
        self.spec = spec
        self._log = logger
        self._lock = threading.Lock()
        self._frame: Optional[np.ndarray] = None
        self._seq = 0
        self._stop = threading.Event()
        self._alive = False
        self._thread = threading.Thread(
            target=self._run, name=f"src-{spec.name}", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self, join_timeout: float = 3.0) -> None:
        """Signal the capture loop and wait for it to release the device.

        Joining matters on CSI sources: nvargus/gstreamer must finish
        cap.release() on its own thread before the interpreter tears down,
        otherwise the native pipeline aborts (SIGABRT core dump) on exit.
        """
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=join_timeout)

    @property
    def alive(self) -> bool:
        return self._alive

    def latest(self) -> Optional[Tuple[np.ndarray, int]]:
        """Return (frame, seq) of the newest frame, or None if none yet."""
        with self._lock:
            if self._frame is None:
                return None
            return self._frame, self._seq

    def _store(self, frame: np.ndarray) -> None:
        with self._lock:
            self._frame = frame
            self._seq += 1

    def _run(self) -> None:  # pragma: no cover - overridden
        raise NotImplementedError


class CsiSource(FrameSource):
    """Jetson CSI camera via nvarguscamerasrc; re-opens every CSI_RETRY_S."""

    def _run(self) -> None:
        while not self._stop.is_set():
            cap = cv2.VideoCapture(
                gst_pipeline(self.spec.sensor_id, self.spec.sensor_mode,
                             self.spec.fps),
                cv2.CAP_GSTREAMER)
            if not cap.isOpened():
                cap.release()
                self._alive = False
                self._log.warn(
                    f"[{self.spec.name}] CSI open failed, "
                    f"retrying in {CSI_RETRY_S:.0f}s")
                if self._stop.wait(CSI_RETRY_S):
                    return
                continue
            self._log.info(f"[{self.spec.name}] CSI camera opened")
            self._alive = True
            while not self._stop.is_set():
                ok, frame = cap.read()
                if not ok:
                    self._log.warn(
                        f"[{self.spec.name}] frame read failed, re-opening")
                    break
                self._store(frame)
            cap.release()
            self._alive = False
            if not self._stop.is_set() and self._stop.wait(CSI_RETRY_S):
                return


class TcpSource(FrameSource):
    """Length-prefixed MJPEG over TCP; rotates hosts, 3 s reconnect backoff."""

    def _run(self) -> None:
        host_idx = 0
        while not self._stop.is_set():
            host = self.spec.hosts[host_idx % len(self.spec.hosts)]
            host_idx += 1
            try:
                sock = socket.create_connection(
                    (host, self.spec.port), timeout=TCP_CONNECT_TIMEOUT_S)
                sock.settimeout(TCP_RECV_TIMEOUT_S)
            except OSError:
                self._alive = False
                if self._stop.wait(TCP_BACKOFF_S):
                    return
                continue
            self._log.info(
                f"[{self.spec.name}] connected to {host}:{self.spec.port}")
            self._alive = True
            try:
                with sock:
                    self._recv_loop(sock)
            except (OSError, ConnectionError, struct.error) as exc:
                self._log.warn(
                    f"[{self.spec.name}] stream lost ({exc}), "
                    f"reconnecting in {TCP_BACKOFF_S:.0f}s")
            self._alive = False
            if self._stop.wait(TCP_BACKOFF_S):
                return

    def _recv_loop(self, sock: socket.socket) -> None:
        while not self._stop.is_set():
            size = LEN_PREFIX.unpack(_recv_exact(sock, LEN_PREFIX.size))[0]
            if not 0 < size <= TCP_MAX_JPEG_BYTES:
                raise ConnectionError(f"bad frame size {size} (desync)")
            jpeg = _recv_exact(sock, size)
            frame = cv2.imdecode(
                np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            if frame is not None:        # corrupt JPEG: skip, keep stream
                self._store(frame)


# ------------------------------------------------------------- statistics
@dataclass
class SourceStats:
    """Rolling per-source counters, reset at each housekeeping log."""
    frames: int = 0
    detections: int = 0
    window_start: float = field(default_factory=time.monotonic)

    def fps(self) -> float:
        elapsed = time.monotonic() - self.window_start
        return self.frames / elapsed if elapsed > 0.5 else 0.0

    def reset(self) -> None:
        self.frames = 0
        self.detections = 0
        self.window_start = time.monotonic()


# ------------------------------------------------------------------- node
class BirdseyeFusion(Node):
    """Multi-source YOLO fusion publishing ground-frame obstacle positions."""

    def __init__(self):
        super().__init__("birdseye_fusion")

        self._model_path = self.declare_parameter(
            "model_path", DEFAULT_MODEL_PATH).value
        # Extra cone source, UNIONED with the model's own cone class.
        # ethon_v1 is weak on this team's small flat orange cones (measured
        # 2026-08-18: its best cone scored 0.376 against the 0.45 cutoff and it
        # missed five others entirely), so on a controlled course colour is by
        # far the stronger cue -- see orange_cone.py. The two detectors fail on
        # DIFFERENT cones, so both run and their outputs merge
        # (_merge_duplicates collapses a cone found by both). Read LIVE every
        # tick rather than cached, so colour can be switched off from the
        # dashboard param editor to isolate the model without a restart.
        # /ethon/obstacles is untouched by this, so hazard classes are
        # unaffected either way.
        self.declare_parameter("orange_cones", True)
        self.get_logger().info(f"loading model {self._model_path}")
        try:
            self.model = YOLO(self._model_path, task="detect")
            # Warm-up so the first real tick doesn't stall on engine init.
            self.model(np.zeros((CSI_HEIGHT, CSI_WIDTH, 3), dtype=np.uint8),
                       verbose=False, imgsz=IMG_SIZE,
                       half=USE_HALF, conf=CONF_THRESHOLD)
        except Exception as exc:
            self.get_logger().fatal(f"model load failed: {exc}")
            raise

        self.sources: Dict[str, FrameSource] = {}
        self.homography: Dict[str, Optional[np.ndarray]] = {}
        # name -> (K, D) for sources whose H was solved in undistorted space.
        # Absent means H maps raw pixels directly (legacy / rectilinear lens).
        self._intrinsics: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
        self.stats: Dict[str, SourceStats] = {}
        self._last_seq: Dict[str, int] = {}
        self._rr_offset = 0

        for spec in SOURCES:
            enabled = self.declare_parameter(
                f"enable_{spec.name}", spec.enabled).value
            if not enabled:
                self.get_logger().info(f"[{spec.name}] disabled by parameter")
                continue
            cls = CsiSource if spec.kind == "csi" else TcpSource
            src = cls(spec, self.get_logger())
            src.start()
            self.sources[spec.name] = src
            self.stats[spec.name] = SourceStats()
            self._last_seq[spec.name] = -1
            self.homography[spec.name] = self._load_h(spec, announce=True)

        self.pub_obstacles = self.create_publisher(
            PoseArray, "/ethon/obstacles", 10)
        self.pub_cones = self.create_publisher(PoseArray, "/ethon/cones", 10)
        # v1, 2026-08-13 -- classical-CV edge/color detector, see
        # curb_detect.py. NOT YET consumed by the planner (that's a
        # deliberate follow-up, not this change): this topic exists so the
        # detector can be watched live on the dashboard and tuned against
        # real sidewalk/road footage before anything acts on it.
        self.pub_curb = self.create_publisher(PoseArray, "/ethon/curb_points", 10)
        self.pub_raw = self.create_publisher(
            String, "/ethon/detections_raw", 10)
        self.pub_status = self.create_publisher(
            String, "/ethon/fusion_status", 10)

        self.create_timer(FUSION_PERIOD_S, self._tick)
        self.create_timer(STATUS_PERIOD_S, self._publish_status)
        self.create_timer(HOUSEKEEP_PERIOD_S, self._housekeep)

    # ---------------------------------------------------------- calibration
    def _load_h(self, spec: SourceSpec, announce: bool = False
                ) -> Optional[np.ndarray]:
        """Load the source's homography, or None if absent/invalid."""
        if not spec.h_file.exists():
            if announce:
                self.get_logger().warn(
                    f"[{spec.name}] no homography ({spec.h_file}) — "
                    "detections go to /ethon/detections_raw ONLY. "
                    "Run calibrate_homography.py.")
            return None
        try:
            h = np.load(spec.h_file)
            if h.shape != (3, 3):
                raise ValueError(f"bad shape {h.shape}, expected (3, 3)")
        except (OSError, ValueError) as exc:
            self.get_logger().error(
                f"[{spec.name}] homography load failed: {exc}")
            return None
        # Does this source's H live in undistorted space? The calibrator
        # records that in the sidecar so the two can never disagree; a
        # missing sidecar means legacy raw-pixel H.
        undistort = False
        if spec.meta_file.exists():
            try:
                with open(spec.meta_file) as f:
                    undistort = bool(json.load(f).get("undistort", False))
            except (OSError, ValueError) as exc:
                self.get_logger().error(
                    f"[{spec.name}] unreadable {spec.meta_file}: {exc} — "
                    "refusing to project (H space unknown)")
                return None
        if undistort:
            try:
                z = np.load(spec.intrinsics_file)
                self._intrinsics[spec.name] = (z["K"], z["D"])
            except (OSError, ValueError, KeyError) as exc:
                self.get_logger().error(
                    f"[{spec.name}] H was solved undistorted but its "
                    f"intrinsics are missing/bad ({exc}) — refusing to "
                    "project. Re-run the fisheye calibration.")
                return None
        else:
            self._intrinsics.pop(spec.name, None)
        if announce:
            self.get_logger().info(
                f"[{spec.name}] loaded {spec.h_file}"
                + (" (fisheye-undistorted)" if undistort else " (raw pixels)"))
        return h

    # ---------------------------------------------------------- fusion loop
    def _tick(self) -> None:
        stamp = self.get_clock().now().to_msg()
        obstacles = PoseArray()
        obstacles.header.stamp = stamp
        obstacles.header.frame_id = FRAME_ID
        cones = PoseArray()
        cones.header.stamp = stamp
        cones.header.frame_id = FRAME_ID
        curb = PoseArray()
        curb.header.stamp = stamp
        curb.header.frame_id = FRAME_ID

        # Round-robin start order so no source monopolizes a slow tick.
        names = list(self.sources)
        if not names:
            return
        self._rr_offset = (self._rr_offset + 1) % len(names)
        ordered = names[self._rr_offset:] + names[:self._rr_offset]

        for name in ordered:
            latest = self.sources[name].latest()
            if latest is None:
                continue
            frame, seq = latest
            if seq == self._last_seq[name]:
                continue                      # no fresh frame this tick
            self._last_seq[name] = seq
            self.stats[name].frames += 1

            result = self.model(frame, verbose=False, imgsz=IMG_SIZE,
                                half=USE_HALF, conf=CONF_THRESHOLD)
            boxes = result[0].boxes

            # Curb-edge detection (v1, not planner-consumed yet -- see
            # pub_curb comment). Skipped for an uncalibrated source: a pixel
            # with no ground mapping isn't useful for anything, so don't
            # spend the CV time. Run once here so the preview overlay and
            # the published points are guaranteed identical, not two
            # independent detector runs that could disagree.
            left_px = right_px = []
            orange_px = []
            if self.homography[name] is not None:
                left_px, right_px = curb_detect.detect_edges(frame)
                self._process_curb(name, left_px, right_px, curb)
                # Orange-colour cones. MUST run here, above the
                # "no boxes -> continue" early-out below: the whole reason
                # this exists is frames where cones are present and the MODEL
                # sees nothing, which is exactly when that early-out fires.
                if bool(self.get_parameter("orange_cones").value):
                    orange_px = orange_cone.detect(frame)
                    self._process_orange(name, orange_px, cones)

            # Write the preview BEFORE the empty-detection early-out, or the
            # dashboard would freeze on the last frame that happened to
            # contain something — an empty road is exactly when you want to
            # see that the camera is still live.
            self._write_preview(name, frame, boxes, left_px, right_px,
                                orange_px)
            if boxes is None or len(boxes) == 0:
                continue
            self.stats[name].detections += len(boxes)
            self._process_boxes(name, boxes, obstacles, cones)

        # Cameras are mounted to OVERLAP so the corridor never falls into a
        # gap between fields of view. Without merging, one physical cone seen
        # by two cameras is published twice at two slightly different
        # positions (they differ by the two calibrations' disagreement). That
        # pulls the planner's midline toward whatever happens to be
        # double-counted, and can trip its "corridor narrower than the
        # vehicle" check on a phantom pair that is really one cone.
        obstacles.poses = _merge_duplicates(obstacles.poses, MERGE_RADIUS_M)
        cones.poses = _merge_duplicates(cones.poses, MERGE_RADIUS_M)
        self.pub_obstacles.publish(obstacles)
        self.pub_cones.publish(cones)
        # No merge_duplicates for curb: it's a dense sampled line, not
        # discrete objects -- collapsing "nearby" points would just erase
        # the shape of the boundary.
        self.pub_curb.publish(curb)

    def _write_preview(self, name: str, frame, boxes,
                       left_px=(), right_px=(), orange_px=()) -> None:
        """Draw detections on the frame and atomically publish it as JPEG.

        Best-effort by design: a preview failure must never disturb the
        perception path, so everything here is wrapped and swallowed.
        """
        try:
            img = frame
            h, w = img.shape[:2]
            if w > ANNOT_MAX_W:                       # cheap encode
                s = ANNOT_MAX_W / float(w)
                img = cv2.resize(img, (ANNOT_MAX_W, int(round(h * s))))
            else:
                img = img.copy()
                s = 1.0
            # Curb-edge points, drawn distinct from the class-detection
            # palette (cyan/magenta, neither used in ANNOT_COLORS) so this
            # v1 debug overlay is never mistaken for a real class detection.
            for u, v in left_px:
                cv2.circle(img, (int(u * s), int(v * s)), 3, (255, 255, 0), -1)
            for u, v in right_px:
                cv2.circle(img, (int(u * s), int(v * s)), 3, (255, 0, 255), -1)
            # Colour-detected cones: bright green ring at the ground-contact
            # point that was actually projected. Drawn thicker than the curb
            # dots so it reads as a real detection, and in a colour no class
            # box uses, so you can tell at a glance on the dashboard whether
            # a cone came from the model or from the colour detector.
            for u, v in orange_px:
                cv2.circle(img, (int(u * s), int(v * s)), 7, (0, 255, 0), 2)
            n = 0
            for box in (boxes if boxes is not None else []):
                x1, y1, x2, y2 = (float(v) * s for v in box.xyxy[0])
                cid = int(box.cls[0])
                conf = float(box.conf[0])
                col = ANNOT_COLORS.get(cid, (200, 200, 200))
                p1, p2 = (int(x1), int(y1)), (int(x2), int(y2))
                cv2.rectangle(img, p1, p2, col, 2)
                lbl = "%s %.2f" % (
                    ANNOT_NAMES[cid] if cid < len(ANNOT_NAMES) else cid, conf)
                (tw, th), _ = cv2.getTextSize(
                    lbl, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)
                # Keep the label inside the frame on all four edges — a box
                # against the right or top border would otherwise have its
                # class name clipped, which is exactly when you most want to
                # read it.
                iw = img.shape[1]
                xleft = min(max(0, p1[0]), max(0, iw - tw - 5))
                ytop = p1[1] - th - 4
                if ytop < 0:                 # no room above: put it inside
                    ytop = min(p1[1] + 1, img.shape[0] - th - 5)
                cv2.rectangle(img, (xleft, ytop),
                              (xleft + tw + 4, ytop + th + 4), col, -1)
                cv2.putText(img, lbl, (xleft + 2, ytop + th + 1),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1,
                            cv2.LINE_AA)
                n += 1
            cv2.putText(img, "%s  %d det" % (name, n), (6, 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3,
                        cv2.LINE_AA)
            cv2.putText(img, "%s  %d det" % (name, n), (6, 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
                        cv2.LINE_AA)
            ok, buf = cv2.imencode(
                ".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), ANNOT_QUALITY])
            if not ok:
                return
            final = os.path.join(ANNOT_DIR, ANNOT_PREFIX + name + ".jpg")
            tmp = final + ".tmp"
            with open(tmp, "wb") as f:
                f.write(buf.tobytes())
            os.replace(tmp, final)        # atomic: readers never see a partial
        except Exception:
            pass

    def _project_point(self, name: str, u: float, v: float
                       ) -> Optional[Tuple[float, float]]:
        """Map one pixel of source `name` to (x, y) base_link metres, or
        None if uncalibrated / off the representable fisheye field / on the
        horizon. Shared by box detections and curb-edge points so both get
        IDENTICAL undistortion and degenerate-projection handling -- two
        independent copies of this math would risk silently drifting apart.
        """
        h = self.homography[name]
        if h is None:
            return None     # uncalibrated: caller routes to debug-only path
        kd = self._intrinsics.get(name)
        if kd is not None:
            # H was fitted in undistorted space, so map the pixel through
            # the fisheye model first. Points too far off-axis come back
            # NaN and are dropped: a wild position is worse than none.
            und = undistort_points_fisheye([[u, v]], kd[0], kd[1])[0]
            if not np.isfinite(und).all():
                return None
            u, v = float(und[0]), float(und[1])
        gx, gy, gw = h @ np.array([u, v, 1.0])
        if abs(gw) < 1e-9:
            return None                        # degenerate projection
        return float(gx / gw), float(gy / gw)

    def _process_boxes(self, name: str, boxes, obstacles: PoseArray,
                       cones: PoseArray) -> None:
        """Project one source's detections; route by calibration state."""
        for box in boxes:
            x1, y1, x2, y2 = (float(v) for v in box.xyxy[0])
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])

            raw = String()
            raw.data = json.dumps({
                "source": name, "cls": cls_id, "conf": round(conf, 3),
                "bbox": [round(x1, 1), round(y1, 1),
                         round(x2, 1), round(y2, 1)],
            })
            self.pub_raw.publish(raw)

            xy = self._project_point(name, (x1 + x2) / 2.0, y2)  # ground contact
            if xy is None:
                continue    # uncalibrated (or off-field): debug topic only
            pose = Pose()
            pose.position.x, pose.position.y = xy
            pose.position.z = float(cls_id)    # class id smuggled in z
            obstacles.poses.append(pose)
            # Model cones are always published, and the colour detector ADDS to
            # them (union) rather than replacing them -- the two fail on
            # different cones, so together they cover more: on the 2026-08-18
            # garage frame the model got the tall cone (0.60) that colour
            # missed, while colour got the flat ones the model could not see at
            # all. Double-counting one physical cone is handled downstream by
            # _merge_duplicates: both detectors report the cone's GROUND-CONTACT
            # point (box bottom-centre / blob bottom-centre), so the two land
            # well inside MERGE_RADIUS_M and collapse to one.
            if cls_id == CONE_CLS:
                cones.poses.append(pose)

    def _process_orange(self, name: str, orange_px,
                        cones: PoseArray) -> None:
        """Project colour-detected cone ground-contact pixels onto /ethon/cones.

        Uses the same _project_point as the model path, so undistortion and
        degenerate-projection handling are identical -- the ground maths cannot
        drift between the two cone sources.
        """
        for u, v in orange_px:
            xy = self._project_point(name, u, v)
            if xy is None:
                continue
            # Ground-plausibility gate. Colour alone cannot tell a cone from
            # an orange object on a wall or shelf, but GEOMETRY can: the
            # homography assumes the pixel lies on the ground, so anything
            # that does not projects to an implausible spot -- absurdly far,
            # sideways, or behind the car. Rejecting here (rather than letting
            # the planner drop it) keeps /ethon/cones honest, so the dashboard
            # cone count means what it says.
            x, y = xy
            if not (CONE_MIN_X_M <= x <= CONE_MAX_X_M
                    and abs(y) <= CONE_MAX_ABS_Y_M):
                continue
            pose = Pose()
            pose.position.x, pose.position.y = xy
            pose.position.z = float(CONE_CLS)   # same class convention
            cones.poses.append(pose)

    def _process_curb(self, name: str, left_px, right_px,
                      curb: PoseArray) -> None:
        """Project one source's already-detected curb-edge pixels (from
        curb_detect.detect_edges, run once per frame in _tick) to ground
        positions, appended to `curb` with side encoded in position.z
        (+1.0 = left boundary, -1.0 = right) -- same class-id-in-z
        convention already used for obstacles/cones.
        """
        for side, pts in ((1.0, left_px), (-1.0, right_px)):
            for u, v in pts:
                xy = self._project_point(name, float(u), float(v))
                if xy is None:
                    continue
                pose = Pose()
                pose.position.x, pose.position.y = xy
                pose.position.z = side
                curb.poses.append(pose)

    # --------------------------------------------------------------- status
    def _publish_status(self) -> None:
        msg = String()
        msg.data = json.dumps({
            "stamp": time.time(),
            "model": self._model_path,
            "sources": {
                spec.name: self._source_status(spec) for spec in SOURCES
            },
        })
        self.pub_status.publish(msg)

    def _source_status(self, spec: SourceSpec) -> dict:
        if spec.name not in self.sources:
            return {"enabled": False}
        return {
            "enabled": True,
            "alive": self.sources[spec.name].alive,
            "calibrated": self.homography[spec.name] is not None,
            "fps": round(self.stats[spec.name].fps(), 1),
            "detections": self.stats[spec.name].detections,
        }

    def _housekeep(self) -> None:
        """Every 10 s: log per-source stats, re-check missing homographies."""
        parts = []
        for name, src in self.sources.items():
            st = self.stats[name]
            spec = src.spec
            if self.homography[name] is None:
                refreshed = self._load_h(spec)
                if refreshed is not None:
                    self.homography[name] = refreshed
                    self.get_logger().info(
                        f"[{name}] homography appeared — now feeding planner")
            calib = "cal" if self.homography[name] is not None else "RAW"
            state = "up" if src.alive else "DOWN"
            parts.append(f"{name}: {st.fps():.1f}fps "
                         f"{st.detections}det {state} {calib}")
            st.reset()
        self.get_logger().info("stats | " + " | ".join(parts))

    def shutdown(self) -> None:
        for src in self.sources.values():
            src.stop()


def _on_term(signum, _frame):
    raise SystemExit(signum)


def main():
    rclpy.init()
    node = BirdseyeFusion()

    # systemd stops this node with SIGTERM (mode switch / stack stop). Turn it
    # into SystemExit so the finally block runs and the camera threads are
    # joined+released cleanly -- otherwise the nvargus pipeline aborts on exit.
    signal.signal(signal.SIGTERM, _on_term)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.shutdown()           # joins capture threads -> releases nvargus
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass  # context may already be down (external shutdown)


if __name__ == "__main__":
    main()
