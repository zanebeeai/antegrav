#!/usr/bin/env python3
"""Ethon web debug dashboard (team 843).

A browser dashboard for the EV, served straight off the Jetson at
    http://<jetson-ip>/dashboard      (port 80)

It is a ROS2 node that subscribes to *every* topic on the graph, caches the
latest message of each, and exposes them — plus live parameter editing and the
same ARM / DISARM / E-STOP / MARK / MODE actions as the steering wheel — over a
tiny JSON API. The single-page UI (embedded below, no external files) polls that
API a few times a second.

Design notes
------------
* Pure standard-library HTTP server (http.server.ThreadingHTTPServer) so there
  is nothing to ``pip install`` on the Jetson — robust if it is offline.
* Generic topic echo via rosidl_runtime_py.message_to_ordereddict, so any
  message type renders without per-type code. Big array messages (PoseArray /
  Path) are summarised to a count to keep the payload small.
* Subscriptions are BEST_EFFORT so they are QoS-compatible with every publisher
  (a best-effort reader accepts both reliable and best-effort writers);
  /ethon/estop additionally uses TRANSIENT_LOCAL so the latched value shows up.
* Parameter get/list/set go through the standard rcl_interfaces services, so any
  node's parameters can be read and edited live (matches ``ros2 param``).
* Actions mirror wheel_bridge.py exactly (same topics, same latched E-STOP QoS,
  same sudo ethon_set_mode.sh for mode), so the wheel and the dashboard stay in
  lock-step.

Read-only by nature except for the explicit param edits and action buttons.
Runs as ethon-dashboard.service (User=jetson, CAP_NET_BIND_SERVICE for port 80).
"""

import array
import csv
import json
import os
import shutil
import threading
import time
import subprocess
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, Empty, String, Float64
from rcl_interfaces.srv import ListParameters, GetParameters, SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType, Log
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message

HTTP_PORT = int(os.environ.get("ETHON_DASH_PORT", "80"))
MODE_SCRIPT = "/home/jetson/ethon/ethon_set_mode.sh"
CLEAR_ESTOP_SCRIPT = "/home/jetson/ethon/ethon_clear_estop.sh"

SKIP_TOPICS = {"/parameter_events", "/rosout"}
# only these nodes are offered in the parameter editor (the rest have nothing
# user-tunable); discovered live, intersected with whatever is actually running.
PARAM_NODES = [
    "/cone_corridor_planner", "/lap_timer", "/ethon_drive",
    "/gps_driver", "/birdseye_fusion", "/health_monitor",
    "/race_strategist", "/corridor_warning",
]
PARAM_NOISE_PREFIX = ("qos_overrides",)
PARAM_NOISE = {"use_sim_time", "start_type_description_service"}

HIST_HZ = 4.0                 # history sampling rate (charts + track)
# Annotated camera previews written by birdseye_fusion into tmpfs.
CAM_DIR = "/dev/shm"
CAM_PREFIX = "ethon_cam_"
CAM_STALE_S = 5.0             # older than this and the source counts as dead


def _list_cams():
    """Source names with a preview frame that is actually fresh."""
    out = []
    try:
        now = time.time()
        for fn in sorted(os.listdir(CAM_DIR)):
            if not (fn.startswith(CAM_PREFIX) and fn.endswith(".jpg")):
                continue
            full = os.path.join(CAM_DIR, fn)
            try:
                if now - os.path.getmtime(full) <= CAM_STALE_S:
                    out.append(fn[len(CAM_PREFIX):-4])
            except OSError:
                continue
    except OSError:
        pass
    return out


# Matches calibrate_homography.py's CALIB_DIR/snapshot_path()/homography_path()
# exactly, so "already calibrated" here means the same thing birdseye_fusion
# checks -- this module deliberately does not import that script (kept as an
# independent CLI tool with no dependents), so the path is duplicated here.
CALIB_DIR = "/home/jetson/ethon/calib"


def _list_calib_snapshots():
    """Snapshot files ready for the /calib pixel-picker, keyed by source."""
    out = {}
    try:
        for fn in sorted(os.listdir(CALIB_DIR)):
            if not fn.endswith("_snapshot.jpg"):
                continue
            key = fn[: -len("_snapshot.jpg")]
            full = os.path.join(CALIB_DIR, fn)
            try:
                mtime = os.path.getmtime(full)
            except OSError:
                continue
            out[key] = {
                "mtime": time.strftime("%Y-%m-%d %H:%M", time.localtime(mtime)),
                "calibrated": os.path.exists(
                    os.path.join(CALIB_DIR, key + "_H.npy")),
            }
    except OSError:
        pass
    return out


HIST_LEN = int(HIST_HZ * 300)  # ~5 minutes of history
LOG_LEN = 300                 # rolling /rosout lines kept
LOG_LEVEL_NAME = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}

T_ESTOP = "/ethon/estop"
T_ARM = "/ethon/hmi/arm"
T_MODE = "/ethon/hmi/mode"
T_MARK = "/ethon/lap/mark"
T_RACE_START = "/ethon/race/start"
LOG_DIR = "/home/jetson/ethon/logs"


def _jsonable(o):
    """Coerce a message_to_ordereddict tree into something json.dumps accepts."""
    if isinstance(o, dict):
        return {k: _jsonable(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_jsonable(v) for v in o]
    if isinstance(o, (bytes, bytearray, array.array)):
        return list(o)
    if o is None or isinstance(o, (bool, int, float, str)):
        return o
    return str(o)


# ── ROS value <-> Python helpers (parameters) ───────────────────────────────
_PT = ParameterType
_TYPE_NAME = {
    _PT.PARAMETER_NOT_SET: "unset", _PT.PARAMETER_BOOL: "bool",
    _PT.PARAMETER_INTEGER: "int", _PT.PARAMETER_DOUBLE: "double",
    _PT.PARAMETER_STRING: "string", _PT.PARAMETER_BYTE_ARRAY: "byte[]",
    _PT.PARAMETER_BOOL_ARRAY: "bool[]", _PT.PARAMETER_INTEGER_ARRAY: "int[]",
    _PT.PARAMETER_DOUBLE_ARRAY: "double[]", _PT.PARAMETER_STRING_ARRAY: "string[]",
}


def _pv_to_py(pv):
    t = pv.type
    if t == _PT.PARAMETER_BOOL:
        return pv.bool_value
    if t == _PT.PARAMETER_INTEGER:
        return pv.integer_value
    if t == _PT.PARAMETER_DOUBLE:
        return pv.double_value
    if t == _PT.PARAMETER_STRING:
        return pv.string_value
    if t == _PT.PARAMETER_BOOL_ARRAY:
        return list(pv.bool_array_value)
    if t == _PT.PARAMETER_INTEGER_ARRAY:
        return list(pv.integer_array_value)
    if t == _PT.PARAMETER_DOUBLE_ARRAY:
        return list(pv.double_array_value)
    if t == _PT.PARAMETER_STRING_ARRAY:
        return list(pv.string_array_value)
    if t == _PT.PARAMETER_BYTE_ARRAY:
        return list(pv.byte_array_value)
    return None


def _py_to_pv(type_int, raw):
    """Build a ParameterValue of the given type from the user's string input."""
    pv = ParameterValue()
    pv.type = type_int
    s = str(raw).strip()
    if type_int == _PT.PARAMETER_BOOL:
        pv.bool_value = s.lower() in ("1", "true", "yes", "on")
    elif type_int == _PT.PARAMETER_INTEGER:
        pv.integer_value = int(float(s))
    elif type_int == _PT.PARAMETER_DOUBLE:
        pv.double_value = float(s)
    elif type_int == _PT.PARAMETER_STRING:
        pv.string_value = s
    elif type_int == _PT.PARAMETER_INTEGER_ARRAY:
        pv.integer_array_value = [int(x) for x in json.loads(s)]
    elif type_int == _PT.PARAMETER_DOUBLE_ARRAY:
        pv.double_array_value = [float(x) for x in json.loads(s)]
    elif type_int == _PT.PARAMETER_BOOL_ARRAY:
        pv.bool_array_value = [bool(x) for x in json.loads(s)]
    elif type_int == _PT.PARAMETER_STRING_ARRAY:
        pv.string_array_value = [str(x) for x in json.loads(s)]
    else:
        raise ValueError("unsupported parameter type %s" % type_int)
    return pv


class EthonDashboard(Node):
    def __init__(self):
        super().__init__("web_dashboard")
        self._lock = threading.Lock()
        self._topics = {}             # name -> {type, t, value, json}
        self._subscribed = {}         # name -> type_str
        self._cbg = ReentrantCallbackGroup()
        self._param_clients = {}      # (node, srv) -> client
        self._hist = deque(maxlen=HIST_LEN)   # time-series for charts + track
        self._logs = deque(maxlen=LOG_LEN)    # rolling /rosout lines
        self._log_seq = 0
        self._cones = {"t": 0.0, "pts": []}   # latest /ethon/cones (base_link x,y)
        self._path = {"t": 0.0, "pts": []}    # latest /ethon/path (base_link x,y)
        # Obstacles and curb points were previously mirrored into /api/state as
        # a bare pose_count by _summarise, so the dashboard could tell you that
        # 32 curb points existed but never where they were. Keep the geometry
        # the same way cones already are, so the bird's-eye can draw them.
        self._obstacles = {"t": 0.0, "pts": []}  # latest /ethon/obstacles
        self._curb = {"t": 0.0, "pts": []}       # latest /ethon/curb_points

        latched = QoSProfile(
            depth=1, history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._estop_pub = self.create_publisher(Bool, T_ESTOP, latched)
        self._arm_pub = self.create_publisher(Bool, T_ARM, 10)
        self._mode_pub = self.create_publisher(String, T_MODE, 10)
        self._mark_pub = self.create_publisher(Empty, T_MARK, 10)
        self._race_pub = self.create_publisher(Empty, T_RACE_START, 10)
        self._drivetest_pub = self.create_publisher(Float64, "/ethon/drive_test", 10)
        self._steertest_pub = self.create_publisher(Float64, "/ethon/steer_test_deg", 10)

        # /rosout is captured into a dedicated log ring (not the topic grid)
        self.create_subscription(
            Log, "/rosout", self._on_rosout,
            QoSProfile(depth=100, history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE),
            callback_group=self._cbg)

        self.rescan()
        self.create_timer(3.0, self.rescan)
        self.create_timer(1.0 / HIST_HZ, self._sample)
        self.get_logger().info("web_dashboard up -- serving on :%d/dashboard" % HTTP_PORT)

    # ── topic discovery / caching ──────────────────────────────────────────
    def _qos_for(self, name):
        # latched topics need a RELIABLE + TRANSIENT_LOCAL reader to actually
        # receive the last value on (re)connect -- a BEST_EFFORT reader does
        # NOT get transient-local historical samples. /ethon/estop (health
        # monitor) and /ethon/hmi/armed (planner's authoritative arm state).
        if name in (T_ESTOP, "/ethon/hmi/armed"):
            return QoSProfile(
                depth=1, history=HistoryPolicy.KEEP_LAST,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        return QoSProfile(
            depth=10, history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)

    def rescan(self):
        """Subscribe to any topic we are not already watching."""
        try:
            names_types = self.get_topic_names_and_types()
        except Exception:
            return
        for name, types in names_types:
            if name in SKIP_TOPICS or name in self._subscribed or not types:
                continue
            type_str = types[0]
            try:
                msg_cls = get_message(type_str)
            except Exception:
                continue
            self._subscribed[name] = type_str
            self.create_subscription(
                msg_cls, name,
                lambda m, n=name, t=type_str: self._on_msg(n, t, m),
                self._qos_for(name), callback_group=self._cbg)
            self.get_logger().info("watching %s [%s]" % (name, type_str))

    def _on_msg(self, name, type_str, msg):
        try:
            d = _jsonable(message_to_ordereddict(msg))
        except Exception as exc:
            d = {"_decode_error": str(exc)}
        parsed = None
        # String topics on this project carry JSON payloads -- decode for display
        if isinstance(d, dict) and set(d.keys()) == {"data"} and isinstance(d["data"], str):
            try:
                parsed = json.loads(d["data"])
            except (ValueError, TypeError):
                parsed = None
        value = self._summarise(type_str, parsed if parsed is not None else d)
        now = time.monotonic()
        with self._lock:
            self._topics[name] = {"type": type_str, "t": now, "value": value}
            if name == "/ethon/cones":
                self._cones = {"t": now, "pts": self._extract_xy(d, False)}
            elif name == "/ethon/path":
                self._path = {"t": now, "pts": self._extract_xy(d, True)}
            elif name == "/ethon/obstacles":
                self._obstacles = {"t": now, "pts": self._extract_xy(d, False)}
            elif name == "/ethon/curb_points":
                self._curb = {"t": now, "pts": self._extract_xy(d, False)}

    @staticmethod
    def _extract_xy(d, nested):
        """Pull [x, y] (base_link metres) from a PoseArray (nested=False) or a
        Path (nested=True, where each entry wraps the pose under 'pose')."""
        out = []
        for p in (d.get("poses") or []):
            pose = p.get("pose") if nested else p
            pos = (pose or {}).get("position") or {}
            x, y = pos.get("x"), pos.get("y")
            if isinstance(x, (int, float)) and isinstance(y, (int, float)):
                out.append([round(x, 2), round(y, 2)])
        return out

    @staticmethod
    def _summarise(type_str, d):
        """Trim huge geometry messages so /api/state stays light."""
        if isinstance(d, dict) and "poses" in d and isinstance(d["poses"], list):
            head = d.get("header", {})
            return {"frame_id": head.get("frame_id"),
                    "pose_count": len(d["poses"])}
        return d

    # ── /rosout log ring ───────────────────────────────────────────────────
    def _on_rosout(self, m):
        self._log_seq += 1
        entry = {"i": self._log_seq, "lvl": int(m.level),
                 "lvl_name": LOG_LEVEL_NAME.get(int(m.level), str(m.level)),
                 "name": m.name, "msg": m.msg}
        with self._lock:
            self._logs.append(entry)

    def logs(self, since=0):
        with self._lock:
            rows = [r for r in self._logs if r["i"] > since]
        return {"logs": rows, "last": (rows[-1]["i"] if rows else since)}

    # ── time-series history (charts + GPS track) ───────────────────────────
    def _sample(self):
        with self._lock:
            drive = (self._topics.get("/ethon/drive_status") or {}).get("value")
            gps = (self._topics.get("/gps/fix") or {}).get("value")
        sp = en = wh = tp = lat = lon = None
        if isinstance(drive, dict):
            ws = drive.get("wheel_speed_ms")
            if isinstance(ws, (int, float)):
                sp = round(abs(ws) * 3.6, 2)
            en = drive.get("energy_wh")
            wh = drive.get("wh_per_km")
            motors = drive.get("motors") or {}
            temps = [v.get("temp_c") for v in motors.values()
                     if isinstance(v, dict) and isinstance(v.get("temp_c"), (int, float))]
            tp = max(temps) if temps else None
        if isinstance(gps, dict):
            la, lo = gps.get("latitude"), gps.get("longitude")
            if (isinstance(la, (int, float)) and isinstance(lo, (int, float))
                    and not (la == 0.0 and lo == 0.0)):
                lat, lon = la, lo
        with self._lock:
            self._hist.append({"t": round(time.monotonic(), 2), "speed": sp,
                               "energy": en, "whkm": wh, "temp": tp,
                               "lat": lat, "lon": lon})

    def history(self):
        with self._lock:
            h = list(self._hist)
            lap = (self._topics.get("/ethon/lap") or {}).get("value")
            cones = list(self._cones["pts"])
            path = list(self._path["pts"])
            obstacles = list(self._obstacles["pts"])
            curb = list(self._curb["pts"])
        line = None
        if isinstance(lap, dict) and lap.get("line_lat") is not None:
            line = {"lat": lap["line_lat"], "lon": lap["line_lon"],
                    "r": lap.get("geofence_m") or 20.0}
        base = {"cones": cones, "path": path, "line": line,
                "obstacles": obstacles, "curb": curb}
        if not h:
            base.update({"t": [], "speed": [], "energy": [], "whkm": [],
                         "temp": [], "track": [], "track_speed": []})
            return base
        t0 = h[0]["t"]
        base.update({
            "t": [round(x["t"] - t0, 2) for x in h],
            "speed": [x["speed"] for x in h],
            "energy": [x["energy"] for x in h],
            "whkm": [x["whkm"] for x in h],
            "temp": [x["temp"] for x in h],
            "track": [[x["lat"], x["lon"]] for x in h if x["lat"] is not None],
            # Same filter as `track`, so the two stay index-aligned: the map
            # colours each trace segment by the speed recorded at that point.
            # Filtering on a different predicate would shift the colours along
            # the lap, which looks plausible and is completely wrong.
            "track_speed": [x["speed"] for x in h if x["lat"] is not None],
        })
        return base

    # ── derived high-level status ──────────────────────────────────────────
    def _status(self):
        with self._lock:
            snap = {n: v["value"] for n, v in self._topics.items()}
        st = {"mode": None, "armed": None, "estop": None, "gps_fix": None,
              "lat": None, "lon": None, "lap": None, "speed_kmh": None,
              "line_set": None, "drive_enabled": None, "config_hold": None,
              "estop_latched": None, "can_ok": None, "arm_requested": None,
              "arm_block": None, "arm_ready": False, "battery_v": None}
        drive = snap.get("/ethon/drive_status")
        if isinstance(drive, dict):
            # NOTE: drive "enabled" is /cmd_vel freshness, NOT arm state -- it
            # stays true while disarmed (planner sends zero cmd_vel). Arm state
            # comes from the planner's latched /ethon/hmi/armed below.
            if isinstance(drive.get("enabled"), bool):
                st["drive_enabled"] = drive["enabled"]
            if drive.get("config_hold") is not None:
                st["config_hold"] = bool(drive.get("config_hold"))
            if drive.get("estop_latched") is not None:
                st["estop_latched"] = bool(drive.get("estop_latched"))
            ws = drive.get("wheel_speed_ms")
            if isinstance(ws, (int, float)):
                st["speed_kmh"] = round(abs(ws) * 3.6, 1)
            sv = drive.get("supply_v")
            if isinstance(sv, (int, float)):
                st["battery_v"] = sv
            motors = drive.get("motors") or {}
            if motors:
                avail = [v for v in motors.values() if isinstance(v, dict)
                         and "unavailable" not in [str(f) for f in (v.get("faults") or [])]]
                st["can_ok"] = len(avail) > 0
        mode = snap.get("/ethon/hmi/mode")
        if isinstance(mode, str):
            st["mode"] = mode
        arm = snap.get("/ethon/hmi/arm")
        if isinstance(arm, dict) and "data" in arm:
            st["arm_requested"] = bool(arm["data"])
        # authoritative armed state from the planner (latched). Falls back to
        # the last arm COMMAND if the planner hasn't published yet, else None.
        armed = snap.get("/ethon/hmi/armed")
        if isinstance(armed, dict) and "data" in armed:
            st["armed"] = bool(armed["data"])
        elif st["arm_requested"] is not None:
            st["armed"] = st["arm_requested"]
        estop = snap.get("/ethon/estop")
        if isinstance(estop, dict) and "data" in estop:
            st["estop"] = bool(estop["data"])
        gps = snap.get("/gps/fix")
        if isinstance(gps, dict):
            status = (gps.get("status") or {}).get("status")
            lat, lon = gps.get("latitude"), gps.get("longitude")
            if isinstance(status, int):
                st["gps_fix"] = (status >= 0 and not (lat == 0.0 and lon == 0.0))
                st["lat"], st["lon"] = lat, lon
        lap = snap.get("/ethon/lap")
        if isinstance(lap, dict):
            st["lap"] = lap.get("lap")
            st["line_set"] = lap.get("line_set")
            if st["gps_fix"] is None:
                st["gps_fix"] = lap.get("fix")

        # why won't it arm? — single human-readable reason for the UI
        if st["drive_enabled"]:
            reason = None
        elif st["estop"] or st["estop_latched"]:
            reason = "E-STOP latched — clear it (restarts ethon-stack)"
        elif st["config_hold"]:
            reason = "CONFIG HOLD — geometry_measured is false in vehicle.yaml"
        elif st["can_ok"] is False:
            reason = "no CAN — motors unavailable (check power / wiring)"
        elif not st["arm_requested"]:
            reason = "not armed — press ARM"
        else:
            reason = "armed — waiting for fresh /cmd_vel from planner"
        st["arm_block"] = reason
        st["arm_ready"] = reason is None
        return st

    def state(self):
        now = time.monotonic()
        with self._lock:
            topics = {n: {"type": v["type"], "age_s": round(now - v["t"], 2),
                          "value": v["value"]}
                      for n, v in self._topics.items()}
        live_nodes = set()
        try:
            live_nodes = {("/" + n.lstrip("/")) for n in self.get_node_names()}
        except Exception:
            pass
        nodes = [n for n in PARAM_NODES if n in live_nodes]
        return {"t": round(now, 2), "status": self._status(),
                "topics": topics, "param_nodes": nodes}

    # ── parameter services ─────────────────────────────────────────────────
    def _param_client(self, node, srv_name, srv_type):
        key = (node, srv_name)
        cli = self._param_clients.get(key)
        if cli is None:
            cli = self.create_client(
                srv_type, "%s/%s" % (node, srv_name), callback_group=self._cbg)
            self._param_clients[key] = cli
        return cli

    def _call(self, cli, req, timeout=4.0):
        if not cli.wait_for_service(timeout_sec=2.0):
            return None
        fut = cli.call_async(req)
        done = threading.Event()
        fut.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout):
            return None
        return fut.result()

    def list_params(self, node):
        lst = self._param_client(node, "list_parameters", ListParameters)
        res = self._call(lst, ListParameters.Request())
        if res is None:
            return None
        names = [n for n in res.result.names
                 if n not in PARAM_NOISE
                 and not n.startswith(PARAM_NOISE_PREFIX)]
        names.sort()
        if not names:
            return []
        get = self._param_client(node, "get_parameters", GetParameters)
        gres = self._call(get, GetParameters.Request(names=names))
        if gres is None:
            return None
        out = []
        for name, pv in zip(names, gres.values):
            out.append({"name": name, "type": int(pv.type),
                        "type_name": _TYPE_NAME.get(pv.type, str(pv.type)),
                        "value": _jsonable(_pv_to_py(pv))})
        return out

    def set_param(self, node, name, type_int, value):
        try:
            pv = _py_to_pv(int(type_int), value)
        except Exception as exc:
            return False, "bad value: %s" % exc
        cli = self._param_client(node, "set_parameters", SetParameters)
        res = self._call(cli, SetParameters.Request(
            parameters=[Parameter(name=name, value=pv)]))
        if res is None or not res.results:
            return False, "no response from %s" % node
        r = res.results[0]
        return bool(r.successful), (r.reason or "")

    # ── pre-race self-test ─────────────────────────────────────────────────
    def selftest(self):
        now = time.monotonic()
        with self._lock:
            ages = {n: now - v["t"] for n, v in self._topics.items()}
            snap = {n: v["value"] for n, v in self._topics.items()}
        checks = []

        def chk(name, ok, detail=""):
            checks.append({"name": name, "ok": bool(ok), "detail": detail})

        for topic, limit, label in (
                ("/ethon/drive_status", 3.0, "drive node publishing"),
                ("/ethon/health", 3.0, "health monitor publishing"),
                ("/ethon/cones", 3.0, "perception (cones) publishing"),
                ("/ethon/lap", 2.0, "lap timer publishing"),
                ("/gps/fix", 3.0, "GPS driver publishing"),
                ("/ethon/strategy", 4.0, "race strategist publishing"),
                ("/ethon/corridor", 2.0, "corridor warning publishing")):
            age = ages.get(topic)
            chk(label, age is not None and age < limit,
                "no data yet" if age is None else "last msg %.1fs ago" % age)

        gps = snap.get("/gps/fix") or {}
        gst = (gps.get("status") or {}).get("status")
        has_fix = (isinstance(gst, int) and gst >= 0
                   and not (gps.get("latitude") == 0.0
                            and gps.get("longitude") == 0.0))
        _la, _lo = gps.get("latitude"), gps.get("longitude")
        chk("GPS satellite fix", has_fix,
            ("lat %.5f lon %.5f" % (_la, _lo)
             if has_fix and isinstance(_la, (int, float))
             and isinstance(_lo, (int, float))
             else ("has fix" if has_fix else "no fix -- needs sky view")))

        drive = snap.get("/ethon/drive_status") or {}
        motors = drive.get("motors") or {}
        avail = [k for k, v in motors.items() if isinstance(v, dict)
                 and "unavailable" not in [str(f) for f in (v.get("faults") or [])]]
        chk("CAN / motors reachable", bool(avail),
            ("%d/%d motors" % (len(avail), len(motors))) if motors
            else "no motor data")
        chk("E-STOP clear", not drive.get("estop_latched")
            and not (snap.get("/ethon/estop") or {}).get("data"),
            "latched -- clear + restart stack"
            if drive.get("estop_latched") else "clear")
        chk("vehicle.yaml geometry measured", not drive.get("config_hold"),
            "CONFIG HOLD -- fill vehicle.yaml, set geometry_measured true"
            if drive.get("config_hold") else "ok")

        chk("Pico wheel hub USB present", os.path.exists("/dev/ethon-wheel"),
            "/dev/ethon-wheel")
        try:
            nodes = {"/" + n.lstrip("/") for n in self.get_node_names()}
        except Exception:
            nodes = set()
        chk("wheel_bridge node alive", "/wheel_bridge" in nodes, "")
        try:
            free_gb = shutil.disk_usage("/home/jetson").free / 1e9
            chk("disk space for logs", free_gb > 1.0, "%.1f GB free" % free_gb)
        except OSError:
            chk("disk space for logs", False, "statvfs failed")

        ready = all(c["ok"] for c in checks)
        return {"ready": ready, "checks": checks,
                "verdict": "READY TO RACE" if ready else "NOT READY"}

    # ── session logs (written by race_services SessionLogger) ─────────────
    def list_sessions(self):
        out = []
        try:
            for fn in sorted(os.listdir(LOG_DIR), reverse=True):
                if not fn.endswith(".csv"):
                    continue
                st = os.stat(os.path.join(LOG_DIR, fn))
                out.append({"name": fn, "kb": round(st.st_size / 1024.0, 1),
                            "mtime": time.strftime(
                                "%Y-%m-%d %H:%M",
                                time.localtime(st.st_mtime))})
        except OSError:
            pass
        return out

    def read_session(self, fname):
        if "/" in fname or "\\" in fname or ".." in fname \
                or not fname.endswith(".csv"):
            return None
        path = os.path.join(LOG_DIR, fname)
        try:
            with open(path, newline="") as fh:
                rows = list(csv.DictReader(fh))
        except OSError:
            return None

        def _f(row, key):
            v = row.get(key)
            try:
                return float(v) if v not in (None, "", "None") else None
            except ValueError:
                return None

        # per-lap table from lap-counter increments
        laps = []
        prev_lap, prev_wh = None, None
        for r in rows:
            lap = _f(r, "lap")
            wh = _f(r, "energy_wh")
            if lap is None:
                continue
            if prev_lap is not None and lap > prev_lap:
                laps.append({"lap": int(lap), "lap_s": _f(r, "last_s"),
                             "wh": None if (wh is None or prev_wh is None)
                             else round(wh - prev_wh, 1)})
                prev_wh = wh
            if prev_lap is None:
                prev_wh = wh
            prev_lap = lap

        step = max(1, len(rows) // 2000)     # decimate for the browser
        rows = rows[::step]
        t0 = _f(rows[0], "t") if rows else 0.0
        return {
            "name": fname, "n": len(rows), "laps": laps,
            "t": [round((_f(r, "t") or t0) - t0, 1) for r in rows],
            "speed": [_f(r, "speed_ms") for r in rows],
            "energy": [_f(r, "energy_wh") for r in rows],
            "temp": [_f(r, "temp_c") for r in rows],
            "track": [[_f(r, "lat"), _f(r, "lon")] for r in rows
                      if _f(r, "lat") is not None],
        }

    # ── bench test: direct duty-cycle command to the drive motors ──────────
    def drive_test(self, duty):
        """Publish a duty-cycle (-1..1) to /ethon/drive_test. ethon_drive caps
        it to test_max_duty and stops the motors if this stops being re-posted
        (watchdog). The dashboard posts continuously while the test is on."""
        try:
            d = max(-1.0, min(1.0, float(duty)))
        except (TypeError, ValueError):
            return False, "bad duty"
        self._drivetest_pub.publish(Float64(data=d))
        return True, ""

    def steer_test(self, deg):
        """Publish a target road-wheel angle (deg) to /ethon/steer_test_deg.
        ethon_drive clamps it to the homed steering limit and stops holding
        the position if this stops being re-posted (watchdog), same shape as
        drive_test. Bypasses the normal armed/hand-back rule -- wheels OFF
        the ground."""
        try:
            d = float(deg)
        except (TypeError, ValueError):
            return False, "bad angle"
        self._steertest_pub.publish(Float64(data=d))
        return True, ""

    # ── actions (mirror wheel_bridge) ──────────────────────────────────────
    def action(self, what, arg=None):
        if what == "arm":
            self._arm_pub.publish(Bool(data=True))
        elif what == "disarm":
            self._arm_pub.publish(Bool(data=False))
        elif what == "estop":
            self._estop_pub.publish(Bool(data=True))
            self.get_logger().error("dashboard E-STOP -> /ethon/estop true")
        elif what == "clear_estop":
            # drive latches estop until restart -> publish false then bounce
            # ethon-stack (same mechanism as the wheel ARM+DISARM gesture).
            self._estop_pub.publish(Bool(data=False))
            try:
                subprocess.Popen(["sudo", "-n", CLEAR_ESTOP_SCRIPT],
                                 stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL)
            except OSError as exc:
                return False, "clear exec failed: %s" % exc
            self.get_logger().warning("dashboard CLEAR E-STOP -> restart ethon-stack")
        elif what == "mark":
            self._mark_pub.publish(Empty())
        elif what == "race_start":
            self._race_pub.publish(Empty())
            self.get_logger().warning("dashboard RACE START -> race clock running")
        elif what == "mode":
            target = arg if arg in ("capture", "autonomy") else None
            if target is None:
                return False, "mode must be capture|autonomy"
            self._mode_pub.publish(String(data=target))
            try:
                subprocess.Popen(["sudo", "-n", MODE_SCRIPT, target],
                                 stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL)
            except OSError as exc:
                return False, "mode exec failed: %s" % exc
        else:
            return False, "unknown action %s" % what
        return True, ""


# ── HTTP layer ───────────────────────────────────────────────────────────────
PAGE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Ethon Dashboard</title>
<style>
:root{--bg:#0d1117;--card:#161b22;--edge:#30363d;--fg:#e6edf3;--mut:#8b949e;
--grn:#3fb950;--red:#f85149;--amb:#d29922;--blu:#58a6ff;--cy:#39c5cf;}
*{box-sizing:border-box}
body{margin:0;font:14px/1.4 ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;
background:var(--bg);color:var(--fg)}
header{display:flex;align-items:center;gap:12px 16px;flex-wrap:wrap;padding:9px 16px;
border-bottom:1px solid var(--edge);position:sticky;top:0;background:rgba(13,17,23,.92);
backdrop-filter:blur(6px);z-index:5}
header h1{font-size:16px;margin:0;letter-spacing:.5px}
#conn{color:var(--mut);font-size:12px}
.dot{display:inline-block;width:9px;height:9px;border-radius:50%;margin-right:5px;
vertical-align:middle;background:var(--mut)}
.hdrstat{display:flex;gap:6px;flex-wrap:wrap;align-items:center;margin-left:auto}
.pill{font-size:11px;font-weight:700;padding:3px 10px;border-radius:20px;
border:1px solid var(--edge);letter-spacing:.4px;white-space:nowrap;color:var(--mut)}
.pill.on{background:rgba(63,185,80,.14);border-color:var(--grn);color:var(--grn)}
.pill.bad{background:var(--red);border-color:var(--red);color:#fff}
.pill.warn{background:rgba(210,153,34,.14);border-color:var(--amb);color:var(--amb)}
.pill.info{border-color:var(--edge);color:var(--fg)}
main{padding:16px;max-width:1200px;margin:0 auto}
.cards{display:flex;flex-wrap:wrap;gap:10px;margin-bottom:18px}
.stat{background:var(--card);border:1px solid var(--edge);border-radius:8px;
padding:10px 14px;min-width:120px;flex:1}
.stat .k{color:var(--mut);font-size:11px;text-transform:uppercase;letter-spacing:.5px}
.stat .v{font-size:20px;font-weight:600;margin-top:3px}
h2{font-size:13px;text-transform:uppercase;letter-spacing:1px;color:var(--mut);
border-bottom:1px solid var(--edge);padding-bottom:6px;margin:22px 0 12px}
details.sec{margin:22px 0 12px}
details.sec>summary{font-size:13px;text-transform:uppercase;letter-spacing:1px;
color:var(--mut);border-bottom:1px solid var(--edge);padding-bottom:6px;cursor:pointer;
list-style:none;user-select:none}
details.sec>summary::before{content:"\25B8  ";color:var(--mut)}
details.sec[open]>summary::before{content:"\25BE  "}
details.sec>summary::-webkit-details-marker{display:none}
details.sec>*:not(summary){margin-top:12px}
.btns{display:flex;flex-wrap:wrap;gap:8px}
button{font:inherit;cursor:pointer;border:1px solid var(--edge);background:var(--card);
color:var(--fg);border-radius:6px;padding:9px 16px;transition:border-color .12s,background .12s}
button:hover{border-color:var(--blu)}
button:active{background:#1c2330}
button.g{border-color:var(--grn);color:var(--grn)}
button.r{border-color:var(--red);color:var(--red)}
button.a{border-color:var(--amb);color:var(--amb)}
button.estop{background:var(--red);color:#fff;border-color:var(--red);font-weight:700}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(300px,1fr));gap:10px}
.topic{background:var(--card);border:1px solid var(--edge);border-radius:8px;padding:10px 12px;overflow:hidden}
.topic .th{display:flex;justify-content:space-between;align-items:baseline;gap:8px}
.topic .tn{color:var(--blu);font-weight:600;word-break:break-all}
.topic .ty{color:var(--mut);font-size:11px}
.topic .age{font-size:11px;color:var(--mut)}
.topic.stale .age{color:var(--amb)}
pre{margin:8px 0 0;white-space:pre-wrap;word-break:break-word;font-size:12px;color:#c9d1d9;max-height:220px;overflow:auto}
select{font:inherit;background:var(--card);color:var(--fg);border:1px solid var(--edge);border-radius:6px;padding:7px 10px}
table{width:100%;border-collapse:collapse;margin-top:10px}
td,th{text-align:left;padding:6px 8px;border-bottom:1px solid var(--edge);vertical-align:middle}
th{color:var(--mut);font-size:11px;text-transform:uppercase}
td .pt{color:var(--mut);font-size:11px}
input.pv{font:inherit;background:#0d1117;color:var(--fg);border:1px solid var(--edge);
border-radius:5px;padding:5px 8px;width:140px}
.ok{color:var(--grn)} .bad{color:var(--red)}
.muted{color:var(--mut)}
#toast{position:fixed;bottom:16px;right:16px;background:var(--card);border:1px solid var(--edge);
border-radius:8px;padding:10px 14px;opacity:0;transition:opacity .2s;pointer-events:none;max-width:340px}
#toast.show{opacity:1}
.armbar{padding:10px 14px;border-radius:8px;margin-bottom:14px;border-left:5px solid var(--mut);
background:var(--card);font-weight:600}
.armbar.ok{border-color:var(--grn);color:var(--grn)}
.armbar.warn{border-color:var(--amb);color:var(--amb)}
.armbar.bad{border-color:var(--red);color:var(--red)}
.panel{background:var(--card);border:1px solid var(--edge);border-radius:8px;padding:8px}
#map{width:100%;height:320px;display:block}
#bird{width:100%;height:340px;display:block}
.charts{display:grid;grid-template-columns:repeat(auto-fill,minmax(260px,1fr));gap:10px}
.chart{background:var(--card);border:1px solid var(--edge);border-radius:8px;padding:4px}
.chart canvas{width:100%;height:150px;display:block}
.camstat{display:flex;flex-wrap:wrap;gap:6px;margin-bottom:8px}
.srcchip{display:flex;align-items:center;gap:6px;font-size:11px;
  background:var(--card);border:1px solid var(--edge);border-radius:6px;
  padding:3px 8px}
.srcchip .nm{font-weight:600}
.srcchip .dot2{width:7px;height:7px;border-radius:50%;display:inline-block}
/* Three columns so the tiles sit the way the cameras actually sit on the car:
   left | wide | right across the top, and narrow directly beneath wide
   (both front-biased cameras stacked in the middle column). Names changed
   2026-08-11 when camera roles were redistributed across both boards
   (old side_left/front_wide/side_right/front_far -> left/wide/right/narrow);
   the grid position each name maps to is unchanged.
   A source that is down simply leaves its cell empty, which is useful -- the
   gap shows you which camera is missing rather than silently reflowing. */
.camwrap{display:grid;grid-template-columns:repeat(3,1fr);gap:10px}
@media (max-width:900px){ /* stack on narrow screens rather than squashing */
  .camwrap{grid-template-columns:1fr}
  .camcell{grid-column:auto !important;grid-row:auto !important}
}
.camcell{position:relative;background:#0d1117;border:1px solid var(--edge);
  border-radius:8px;overflow:hidden}
.camcell img{width:100%;display:block}
.camcell .camlbl{position:absolute;top:6px;left:8px;font-size:11px;
  background:rgba(0,0,0,.55);padding:2px 6px;border-radius:4px}
.steerwrap{display:flex;gap:14px;align-items:center;flex-wrap:wrap}
.steerwrap canvas{width:400px;height:210px;flex:0 0 auto;max-width:100%}
.steerinfo{flex:1 1 200px;min-width:190px;font-size:13px}
.steerrow{display:flex;justify-content:space-between;padding:3px 0;
  border-bottom:1px solid var(--edge)}
/* Big live road-wheel angle. tabular-nums keeps the number from shifting
   sideways as digit widths change -- at 4 Hz that jitter makes a tracking
   angle genuinely hard to read. */
.steerbig{font-size:40px;font-weight:700;line-height:1;letter-spacing:-1px;
  font-variant-numeric:tabular-nums}
.steerbigsub{font-size:11px;font-weight:400;color:var(--mut);letter-spacing:0;
  padding:4px 0 6px;border-bottom:1px solid var(--edge);margin-bottom:4px}
.steerbigsub b{font-variant-numeric:tabular-nums;color:var(--fg)}
.logbar{display:flex;gap:10px;align-items:center;margin-bottom:8px;color:var(--mut);font-size:12px}
.log{height:240px;overflow:auto;background:#0d1117;border:1px solid var(--edge);border-radius:8px;padding:8px;font-size:12px}
.log .lg{padding:1px 0;white-space:pre-wrap;word-break:break-word}
.log .l30{color:var(--amb)} .log .l40{color:var(--red)} .log .l50{color:#fff;background:#5a1111}
.log .l10{color:var(--mut)}
</style>
</head>
<body>
<header>
  <h1>ETHON&nbsp;//&nbsp;DEBUG</h1>
  <nav style="display:flex;gap:12px;font-size:12px">
    <a href="/dashboard" style="color:var(--blu)">DASH</a>
    <a href="/pit" style="color:var(--mut)">PIT</a>
    <a href="/replay" style="color:var(--mut)">REPLAY</a>
    <a href="/calib" style="color:var(--mut)">CALIB</a>
  </nav>
  <div id="hdrstat" class="hdrstat"></div>
  <span id="conn"><span class="dot" id="dot"></span><span id="conntxt">connecting…</span></span>
</header>
<main>
  <div id="armbar" class="armbar">checking drive…</div>
  <div class="cards" id="cards"></div>

  <h2>Actions</h2>
  <div class="btns">
    <button class="g" onclick="act('arm')">ARM</button>
    <button onclick="act('disarm')">DISARM</button>
    <button class="estop" onclick="act('estop')">E-STOP</button>
    <button class="a" onclick="act('clear_estop')">CLEAR E-STOP</button>
    <button class="g" onclick="act('mark')">MARK LINE</button>
    <button onclick="act('mode','autonomy')">MODE: AUTONOMY</button>
    <button onclick="act('mode','capture')">MODE: CAPTURE</button>
    <button class="g" onclick="if(confirm('Start the 70-min race clock?'))act('race_start')">START RACE</button>
    <button class="a" onclick="selftest()">SELF-TEST</button>
  </div>
  <div id="selftest" style="margin-top:10px"></div>

  <h2>Motor bench test <span class="muted">wheels OFF the ground</span></h2>
  <div class="panel" style="padding:12px">
    <div class="armbar warn" style="margin-bottom:10px">
      &#9888; Spins the drive Krakens directly (open-loop duty, bypasses the
      planner). Rear wheels MUST be off the ground. Release / STOP and the
      motors coast within half a second.
    </div>
    <div class="btns" style="align-items:center;margin-bottom:8px">
      <span class="muted">Control mode:</span>
      <button id="mode-duty" onclick="setFoc(false)">DUTY (no license)</button>
      <button id="mode-foc" onclick="setFoc(true)">FOC (needs Pro)</button>
      <span id="modestate" class="muted"></span>
    </div>
    <div class="btns" style="align-items:center;margin-bottom:8px">
      <span class="muted">Duty</span>
      <input id="dutyslider" type="range" min="0" max="30" value="10" step="1"
             oninput="document.getElementById('dutyval').textContent=this.value+'%'"
             style="width:220px">
      <span id="dutyval" style="min-width:44px;display:inline-block">10%</span>
      <label class="muted"><input type="radio" name="dir" id="dir-fwd" checked> fwd</label>
      <label class="muted"><input type="radio" name="dir" id="dir-rev"> rev</label>
    </div>
    <div class="btns">
      <button class="g" id="test-start" onclick="startTest()">START TEST</button>
      <button class="estop" id="test-stop" onclick="stopTest()">STOP</button>
      <span id="teststate" class="muted"></span>
    </div>
    <div id="testreadout" class="muted" style="margin-top:8px;font-size:12px"></div>
  </div>

  <h2>Steering bench test <span class="muted">wheels OFF the ground</span></h2>
  <div class="panel" style="padding:12px">
    <div class="armbar warn" style="margin-bottom:10px">
      &#9888; Commands the steering column directly to a target road-wheel
      angle, bypassing the planner AND the armed hand-back rule. Wheels MUST
      be off the ground. Release / STOP and the column freewheels within
      half a second.
    </div>
    <div class="btns" style="align-items:center;margin-bottom:8px">
      <span class="muted">Target angle</span>
      <input id="steerslider" type="range" min="-30" max="30" value="0" step="0.5"
             oninput="document.getElementById('steerval').textContent=this.value+'&deg;'"
             style="width:220px">
      <span id="steerval" style="min-width:44px;display:inline-block">0&deg;</span>
    </div>
    <div class="btns">
      <button class="g" id="steertest-start" onclick="startSteerTest()">START TEST</button>
      <button class="estop" id="steertest-stop" onclick="stopSteerTest()">STOP</button>
      <span id="steerteststate" class="muted"></span>
    </div>
    <div id="steertestreadout" class="muted" style="margin-top:8px;font-size:12px"></div>
  </div>

  <h2>Track map <span class="muted">GPS / world</span></h2>
  <div class="panel"><canvas id="map"></canvas></div>

  <h2>Cones &amp; plan <span class="muted">robot frame, forward = up</span></h2>
  <h2>Cameras <span class="muted" style="font-size:12px">what the model sees</span></h2>
  <div class="panel">
    <div id="camstat" class="camstat"></div>
    <div id="camwrap" class="camwrap"></div>
    <div id="camnone" class="muted" style="font-size:12px">
      no source is delivering frames yet — a preview appears here as soon as
      one does.</div>
  </div>

  <div class="panel"><canvas id="bird"></canvas></div>

  <h2>Telemetry</h2>
  <div class="charts">
    <div class="chart"><canvas id="c_speed"></canvas></div>
    <div class="chart"><canvas id="c_energy"></canvas></div>
    <div class="chart"><canvas id="c_whkm"></canvas></div>
    <div class="chart"><canvas id="c_temp"></canvas></div>
  </div>

  <h2>Steering</h2>
  <div class="panel">
    <div class="steerwrap">
      <canvas id="steerviz"></canvas>
      <div class="steerinfo">
        <div class="steerbig" id="sv_big">—</div>
        <div class="steerbigsub">STEERING WHEEL (column)
          &nbsp;·&nbsp; target <b id="sv_cmd">—</b>
          &nbsp;·&nbsp; err <b id="sv_err">—</b>
          &nbsp;·&nbsp; road wheel <b id="sv_rw">—</b></div>
        <div class="steerrow"><span class="muted">state</span>
          <b id="sv_state">—</b></div>
        <div class="steerrow"><span class="muted">road wheel</span>
          <b id="sv_deg">—</b></div>
        <div class="steerrow"><span class="muted">column</span>
          <b id="sv_col">—</b></div>
        <div class="steerrow"><span class="muted">lock (measured)</span>
          <b id="sv_lim">—</b></div>
        <div class="steerrow"><span class="muted">travel used</span>
          <b id="sv_pct">—</b></div>
        <div class="steerrow"><span class="muted">control</span>
          <b id="sv_mode">—</b></div>
        <div class="steerrow"><span class="muted">motor</span>
          <b id="sv_mot">—</b></div>
      </div>
    </div>
  </div>

  <h2>Parameters &mdash; edit live</h2>
  <div class="btns" style="margin-bottom:6px">
    <select id="pnode" onchange="loadParams()"><option value="">— pick a node —</option></select>
    <button onclick="loadParams()">↻ reload</button>
  </div>
  <div id="params" class="muted">Pick a node to view and edit its parameters.</div>

  <details class="sec">
    <summary>Log <span class="muted">/rosout</span></summary>
    <div class="logbar">
      <label><input type="checkbox" id="logpause"> pause</label>
      <span>level</span>
      <select id="loglevel" onchange="renderLog()">
        <option value="0">ALL</option><option value="10">DEBUG+</option>
        <option value="20" selected>INFO+</option><option value="30">WARN+</option>
        <option value="40">ERROR+</option>
      </select>
      <button onclick="clearLog()">clear</button>
    </div>
    <pre id="log" class="log"></pre>
  </details>

  <details class="sec">
    <summary>Topics <span class="muted">live &mdash; all ROS topics</span></summary>
    <div class="grid" id="topics"></div>
  </details>
</main>
<div id="toast"></div>

<script>
let lastNodes = "";
let lastHist = null;    // last /api/history payload (cones + path)
let lastDrive = {};     // last /ethon/drive_status value
function el(t,c,txt){const e=document.createElement(t);if(c)e.className=c;if(txt!=null)e.textContent=txt;return e;}
function toast(msg,bad){const t=document.getElementById('toast');t.textContent=msg;
  t.className=bad?'show bad':'show';setTimeout(()=>t.className='',2600);}

function fmtHeader(s){
  const box=document.getElementById('hdrstat');if(!box)return;box.innerHTML='';
  const pill=(t,c)=>{box.appendChild(el('span','pill'+(c?' '+c:''),t));};
  if(s.estop){pill('E-STOP','bad');}
  else pill(s.armed===true?'ARMED':(s.armed===false?'DISARMED':'ARM ?'),s.armed?'on':'');
  pill('GPS '+(s.gps_fix===true?'OK':(s.gps_fix===false?'NO FIX':'?')),
       s.gps_fix===true?'on':(s.gps_fix===false?'warn':''));
  if(s.battery_v!=null)pill(s.battery_v.toFixed(1)+' V',
       s.battery_v<10.5?'bad':(s.battery_v<11.5?'warn':'on'));
  if(s.speed_kmh!=null)pill(s.speed_kmh+' km/h','info');
}

function fmtStat(s){
  const cards=[
    ['State', s.estop?'E-STOP':(s.armed===true?'ARMED':(s.armed===false?'DISARMED':'—')),
       s.estop?'var(--red)':(s.armed?'var(--grn)':'var(--mut)')],
    ['Mode', s.mode?s.mode.toUpperCase():'—', 'var(--fg)'],
    ['GPS', s.gps_fix===true?'FIX':(s.gps_fix===false?'NO FIX':'—'),
       s.gps_fix?'var(--grn)':'var(--red)'],
    ['Lap', s.lap==null?'—':s.lap, 'var(--fg)'],
    ['Speed', s.speed_kmh==null?'—':s.speed_kmh+' km/h', 'var(--fg)'],
    ['Line', s.line_set===true?'set':(s.line_set===false?'unset':'—'),
       s.line_set?'var(--grn)':'var(--mut)'],
    ['Battery', s.battery_v==null?'—':s.battery_v.toFixed(1)+' V',
       s.battery_v==null?'var(--mut)':(s.battery_v<10.5?'var(--red)':(s.battery_v<11.5?'var(--amb)':'var(--grn)'))],
  ];
  const box=document.getElementById('cards');box.innerHTML='';
  for(const [k,v,col] of cards){
    const c=el('div','stat');c.appendChild(el('div','k',k));
    const ve=el('div','v',v);ve.style.color=col;c.appendChild(ve);box.appendChild(c);
  }
}

function fmtTopics(topics){
  const box=document.getElementById('topics');
  const names=Object.keys(topics).sort();
  box.innerHTML='';
  for(const n of names){
    const t=topics[n];const card=el('div','topic'+(t.age_s>3?' stale':''));
    const th=el('div','th');th.appendChild(el('span','tn',n));
    th.appendChild(el('span','age',t.age_s+'s'));card.appendChild(th);
    card.appendChild(el('div','ty',t.type));
    card.appendChild(el('pre',null,JSON.stringify(t.value,null,1)));
    box.appendChild(card);
  }
}

async function tick(){
  try{
    const r=await fetch('/api/state',{cache:'no-store'});const s=await r.json();
    document.getElementById('dot').style.background='var(--grn)';
    document.getElementById('conntxt').textContent='live';
    fmtHeader(s.status);fmtStat(s.status);fmtArm(s.status);fmtTopics(s.topics);
    const dst=(s.topics['/ethon/drive_status']||{}).value||{};
    updateTestPanel(s.status,dst);
    drawSteer(dst,(s.topics['/cmd_vel']||{}).value);
    // Redraw the bird's-eye here too (not just on the slow history tick) so
    // the predicted-trajectory arc tracks the wheel without lagging a second
    // behind. Cones/path come from the cached history payload.
    lastDrive=dst;
    if(lastHist) drawBird('bird',lastHist.cones,lastHist.path,dst);
    updateCamStatus((s.topics['/ethon/fusion_status']||{}).value||{});
    const nodes=s.param_nodes.join(',');
    if(nodes!==lastNodes){lastNodes=nodes;const sel=document.getElementById('pnode');
      const cur=sel.value;sel.innerHTML='<option value="">— pick a node —</option>';
      for(const n of s.param_nodes){const o=el('option',null,n);o.value=n;sel.appendChild(o);}
      sel.value=cur;}
  }catch(e){
    document.getElementById('dot').style.background='var(--red)';
    document.getElementById('conntxt').textContent='disconnected';
  }
}

async function act(what,arg){
  try{
    const r=await fetch('/api/action',{method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify({action:what,arg:arg})});
    const j=await r.json();
    toast(j.ok?('✓ '+what+(arg?' '+arg:'')):('✗ '+(j.reason||what)),!j.ok);
  }catch(e){toast('✗ '+what+' failed',true);}
}

async function loadParams(){
  const node=document.getElementById('pnode').value;
  const box=document.getElementById('params');
  if(!node){box.className='muted';box.textContent='Pick a node to view and edit its parameters.';return;}
  box.className='';box.textContent='loading…';
  try{
    const r=await fetch('/api/params?node='+encodeURIComponent(node),{cache:'no-store'});
    const j=await r.json();
    if(!j.ok){box.textContent='error: '+(j.reason||'unreachable');return;}
    if(!j.params.length){box.className='muted';box.textContent='(no tunable parameters)';return;}
    const tbl=el('table');const hdr=el('tr');
    for(const h of ['Parameter','Type','Value','']){hdr.appendChild(el('th',null,h));}
    tbl.appendChild(hdr);
    for(const p of j.params){
      const tr=el('tr');
      tr.appendChild(el('td',null,p.name));
      tr.appendChild(el('td','pt',p.type_name));
      const tdv=el('td');const inp=el('input','pv');inp.value=JSON.stringify(p.value).replace(/^"|"$/g,'');
      inp.dataset.type=p.type;inp.dataset.name=p.name;tdv.appendChild(inp);tr.appendChild(tdv);
      const tdb=el('td');const b=el('button',null,'Set');
      b.onclick=()=>setParam(node,p.name,p.type,inp.value,b);tdb.appendChild(b);tr.appendChild(tdb);
      tbl.appendChild(tr);
    }
    box.innerHTML='';box.appendChild(tbl);
  }catch(e){box.textContent='request failed';}
}

async function setParam(node,name,type,value,btn){
  btn.disabled=true;btn.textContent='…';
  try{
    const r=await fetch('/api/param',{method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify({node:node,name:name,type:type,value:value})});
    const j=await r.json();
    toast(j.ok?('✓ '+name+' = '+value):('✗ '+name+': '+(j.reason||'rejected')),!j.ok);
  }catch(e){toast('✗ '+name+' failed',true);}
  btn.disabled=false;btn.textContent='Set';
}

function fmtArm(s){
  const b=document.getElementById('armbar');
  if(s.arm_ready){b.className='armbar ok';b.textContent='✓ DRIVE ENABLED — motors live';return;}
  const sev=(s.estop||s.estop_latched)?'bad':'warn';
  b.className='armbar '+sev;b.textContent='⚠ ARM BLOCKED: '+(s.arm_block||'unknown');
}

function setupCanvas(id){
  const cv=document.getElementById(id);if(!cv)return null;
  const dpr=window.devicePixelRatio||1;
  const w=cv.clientWidth||300,h=cv.clientHeight||150;
  cv.width=Math.round(w*dpr);cv.height=Math.round(h*dpr);
  const ctx=cv.getContext('2d');ctx.setTransform(dpr,0,0,dpr,0,0);
  return {ctx:ctx,w:w,h:h};
}

// ── steering visualiser ───────────────────────────────────────────────────
// Left: the steering wheel as the driver sees it (column rotation, so it can
// exceed 360 deg). Right: the road wheels from above. The arc shows the
// measured lock-to-lock travel; it turns amber near the stops.
function drawSteer(d,cv){
  const s=setupCanvas('steerviz');if(!s)return;
  const ctx=s.ctx,w=s.w,h=s.h;
  ctx.clearRect(0,0,w,h);ctx.fillStyle='#161b22';ctx.fillRect(0,0,w,h);
  // Rendering sign. Reported position is positive in the direction the motor
  // calls forward, which on this car is physically COUNTER-clockwise while
  // canvas rotate() is positive clockwise — so the raw value renders mirrored.
  // Follow steer_inverted instead of hardcoding, so flipping that config keeps
  // the picture matching the real wheel.
  const dsign=(d.steer_inverted===true)?1:-1;
  const col=(d.steer_col_rot==null)?null:dsign*(+d.steer_col_rot);
  const deg=(d.road_wheel_deg==null)?null:dsign*(+d.road_wheel_deg);
  const lim=(d.steer_limit_col_rot==null)?null:Math.abs(+d.steer_limit_col_rot);
  const homed=!!d.steer_homed, dead=(col==null);
  const frac=(col!=null&&lim)?Math.max(-1,Math.min(1,col/lim)):0;
  const near=Math.abs(frac)>0.9;
  const live=homed&&!dead;
  const accent=dead?'#6e7681':(near?'#d29922':(live?'#3fb950':'#6e7681'));

  // ── steering wheel (left) — formula style, matching the real wheel ──
  const cx=w*0.30,cy=h*0.52,R=Math.min(w*0.155,h*0.40);
  const rr=(x,y,ww,hh,r)=>{ctx.beginPath();
    ctx.moveTo(x+r,y);ctx.arcTo(x+ww,y,x+ww,y+hh,r);
    ctx.arcTo(x+ww,y+hh,x,y+hh,r);ctx.arcTo(x,y+hh,x,y,r);
    ctx.arcTo(x,y,x+ww,y,r);ctx.closePath();};
  // travel arc sits behind the wheel: -lim..+lim over the top
  if(lim){
    const ar=R*1.42;
    ctx.lineWidth=4;ctx.strokeStyle='#21262d';
    ctx.beginPath();ctx.arc(cx,cy,ar,Math.PI*0.78,Math.PI*2.22);ctx.stroke();
    const a0=Math.PI*1.5,aN=a0+frac*(Math.PI*0.72);
    ctx.strokeStyle=accent;ctx.beginPath();
    ctx.arc(cx,cy,ar,Math.min(a0,aN),Math.max(a0,aN));ctx.stroke();
  }
  // Verified 2026-08-16 against a live bench-test command: the wheel
  // diagram below (right side) is ground truth and matches physical
  // wheel direction. This icon needs the opposite sign to agree with it --
  // ctx.rotate() reads canvas-clockwise as "steering right" to a viewer,
  // but col>0 (after dsign) is physically LEFT on this car.
  const rot=(col==null)?0:-col*Math.PI*2;  // column rotations -> radians
  ctx.save();ctx.translate(cx,cy);ctx.rotate(rot);
  const carbon='#22272e',edge=dead?'#444c56':accent;
  ctx.lineWidth=1.5;ctx.strokeStyle=edge;ctx.fillStyle=carbon;
  // grips (angled, lower left/right)
  for(const sx of [-1,1]){
    ctx.save();ctx.translate(sx*R*0.92,R*0.34);ctx.rotate(sx*0.20);
    rr(-R*0.20,-R*0.30,R*0.40,R*1.02,R*0.16);ctx.fill();ctx.stroke();
    ctx.restore();
  }
  // upper wings (button pods) + top bar, drawn as one open-top body
  for(const sx of [-1,1]){
    ctx.save();ctx.scale(sx,1);
    rr(R*0.30,-R*0.80,R*0.98,R*0.92,R*0.14);ctx.fill();ctx.stroke();
    ctx.restore();
  }
  rr(-R*0.62,-R*0.80,R*1.24,R*0.30,R*0.08);ctx.fill();ctx.stroke();  // LED bar
  rr(-R*0.66,-R*0.50,R*1.32,R*0.86,R*0.10);ctx.fill();ctx.stroke();  // centre
  rr(-R*0.52,R*0.36,R*1.04,R*0.40,R*0.12);ctx.fill();ctx.stroke();   // lower
  // rev / shift LEDs across the top bar (same map as the Nextion HMI)
  const kmh=Math.abs(+(d.wheel_speed_ms||0))*3.6, sf=Math.min(1,kmh/50);
  for(let i=0;i<15;i++){
    const on=live&&(i/15)<sf;
    ctx.fillStyle=on?(i<9?'#3fb950':(i<12?'#d29922':'#f85149')):'#161b22';
    ctx.beginPath();
    ctx.arc(-R*0.55+i*(R*1.10/14),-R*0.65,R*0.045,0,Math.PI*2);ctx.fill();
  }
  // centre screen
  ctx.fillStyle='#0d1117';ctx.strokeStyle='#30363d';
  rr(-R*0.46,-R*0.40,R*0.92,R*0.64,R*0.05);ctx.fill();ctx.stroke();
  ctx.fillStyle=dead?'#6e7681':'#e6edf3';
  ctx.font='bold '+Math.round(R*0.34)+'px system-ui';
  ctx.textAlign='center';ctx.textBaseline='middle';
  ctx.fillText(dead?'--':kmh.toFixed(0),0,-R*0.16);
  ctx.font=Math.round(R*0.16)+'px system-ui';ctx.fillStyle='#8b949e';
  ctx.fillText('km/h',0,R*0.10);
  // button pods: a few coloured buttons per side, like the real wheel
  const pods=[[-0.95,-0.62,'#d29922'],[-0.62,-0.60,'#f85149'],
              [-0.95,-0.30,'#f85149'],[-0.60,-0.26,'#8b949e'],
              [ 0.95,-0.62,'#388bfd'],[ 0.62,-0.60,'#3fb950'],
              [ 0.95,-0.30,'#3fb950'],[ 0.60,-0.26,'#8b949e']];
  for(const [px,py,c] of pods){
    ctx.fillStyle=live?c:'#30363d';
    ctx.beginPath();ctx.arc(px*R,py*R,R*0.075,0,Math.PI*2);ctx.fill();
  }
  // centre hub + straight-ahead marker
  ctx.fillStyle='#30363d';ctx.beginPath();
  ctx.arc(0,R*0.56,R*0.10,0,Math.PI*2);ctx.fill();
  ctx.fillStyle=edge;ctx.beginPath();
  ctx.arc(0,-R*0.86,R*0.05,0,Math.PI*2);ctx.fill();
  ctx.textBaseline='alphabetic';
  ctx.restore();

  // ── road wheels from above (right) ──
  const bx=w*0.68,by=h*0.5,L=Math.min(w*0.12,h*0.30);
  ctx.strokeStyle='#30363d';ctx.lineWidth=2;
  ctx.beginPath();ctx.moveTo(bx,by-L*1.15);ctx.lineTo(bx,by+L*1.15);ctx.stroke();
  ctx.beginPath();ctx.moveTo(bx-L*0.85,by-L*1.15);   // front axle
  ctx.lineTo(bx+L*0.85,by-L*1.15);ctx.stroke();
  const wr=(deg==null)?0:(-deg*Math.PI/180);  // +deg = right = clockwise
  for(const sx of [-1,1]){
    const px=bx+sx*L*0.85, py=by-L*1.15;
    ctx.save();ctx.translate(px,py);ctx.rotate(wr);
    ctx.fillStyle=accent;ctx.fillRect(-2.5,-L*0.42,5,L*0.84);ctx.restore();
  }
  for(const sx of [-1,1]){                     // rear wheels, fixed
    ctx.fillStyle='#30363d';
    ctx.fillRect(bx+sx*L*0.85-2.5,by+L*1.15-L*0.42,5,L*0.84);
  }
  ctx.fillStyle='#6e7681';ctx.font='10px system-ui';ctx.textAlign='center';
  ctx.fillText('front',bx,by-L*1.6);

  if(dead){ctx.fillStyle='#6e7681';ctx.font='12px system-ui';
    ctx.textAlign='center';ctx.fillText('no steering data',w/2,h-8);}

  const S=(id,v)=>{const e=document.getElementById(id);if(e)e.textContent=v;};
  // ── big live STEERING-WHEEL angle ──────────────────────────────────────
  // Reported in COLUMN degrees, not rotations and not road-wheel degrees:
  // this is the number that matches the wheel the driver is looking at.
  // col is already sign-corrected (dsign) above, so positive = LEFT, the
  // same convention the diagram renders. 1 column rot = 360 deg; the road
  // wheel is steer_col_ratio (12:1) smaller, which is why 90 deg of wheel
  // is only ~7.5 deg of tyre.
  const fmtd=(v)=>(v>0?'+':'')+v.toFixed(1)+'°';
  const colDeg=(col==null)?null:col*360;
  // Target road-wheel angle the drive node is chasing, reconstructed from
  // /cmd_vel exactly as drive/steering.py does it: road = atan(wheelbase *
  // omega / v). Only valid above ACKERMANN_MIN_SPEED_MS (0.3) -- below that
  // the drive node commands centre regardless of omega, so showing a target
  // there would be a lie. dsign matches the display convention used above.
  let cmdDeg=null;
  if(cv){
    const vx=+(((cv.linear||{}).x)||0), wz=+(((cv.angular||{}).z)||0);
    const wb=(d.wheelbase_m!=null)?+d.wheelbase_m:1.524;
    if(Math.abs(vx)>0.3) cmdDeg=dsign*(Math.atan(wb*wz/vx)*180/Math.PI);
  }
  S('sv_big', colDeg==null?'—':fmtd(colDeg));
  const bigEl=document.getElementById('sv_big');
  if(bigEl) bigEl.style.color = dead ? 'var(--mut)'
    : (near ? 'var(--amb)' : 'var(--fg)');
  // Target the drive node is actually chasing, in the same units, so target
  // vs actual vs error read off one line without unit conversion in your head.
  const cmdCol=(cmdDeg==null)?null:cmdDeg*(d.steer_col_ratio||12.0);
  S('sv_cmd', cmdCol==null?'—':fmtd(cmdCol));
  S('sv_err', (cmdCol==null||colDeg==null)?'—':fmtd(colDeg-cmdCol));
  S('sv_rw', deg==null?'—':fmtd(deg));
  S('sv_state', dead?'no data':(d.steering||(homed?'homed':'—')));
  S('sv_deg', deg==null?'—':(deg>0?'+':'')+deg.toFixed(1)+'°');
  S('sv_col', col==null?'—':(col>0?'+':'')+col.toFixed(3)+' rot');
  S('sv_lim', lim?('±'+lim.toFixed(3)+' rot ('
    +(lim*360).toFixed(0)+'° col)'):'—');
  S('sv_pct', lim&&col!=null?(Math.abs(frac)*100).toFixed(0)+'%'
    +(near?'  ⚠ near lock':''):'—');
  S('sv_mode', d.steer_mode
    ? (d.steer_mode==='foc' ? 'FOC (Pro licence)' : 'duty (no licence)') : '—');
  const m=(d.motors||{}).steer||{};
  S('sv_mot', m.faults&&m.faults.length
    ? (m.faults.join(',')) : (m.temp_c!=null?(m.temp_c+'°C'):'—'));
}

function drawChart(id,t,vals,o){
  const s=setupCanvas(id);if(!s)return;const ctx=s.ctx,w=s.w,h=s.h;
  ctx.clearRect(0,0,w,h);ctx.fillStyle='#161b22';ctx.fillRect(0,0,w,h);
  ctx.font='11px ui-monospace,monospace';ctx.textAlign='left';
  ctx.fillStyle='#8b949e';ctx.fillText(o.label,8,14);
  const pts=[];for(let i=0;i<vals.length;i++){const v=vals[i];if(v!=null&&isFinite(v))pts.push([t[i],v]);}
  const cur=pts.length?pts[pts.length-1][1]:null;
  ctx.fillStyle=o.color;ctx.textAlign='right';
  ctx.fillText(cur==null?'—':cur.toFixed(o.dec)+' '+o.unit,w-8,14);ctx.textAlign='left';
  if(pts.length<2){ctx.fillStyle='#8b949e';ctx.fillText('waiting…',8,Math.round(h/2));return;}
  const padL=8,padR=8,padT=22,padB=12;
  let xmin=pts[0][0],xmax=pts[pts.length-1][0];
  let ymin=Math.min.apply(null,pts.map(p=>p[1])),ymax=Math.max.apply(null,pts.map(p=>p[1]));
  if(ymin===ymax){ymin-=1;ymax+=1;}
  const yr=ymax-ymin;ymin-=yr*0.1;ymax+=yr*0.1;const xr=(xmax-xmin)||1;
  const X=x=>padL+(x-xmin)/xr*(w-padL-padR);
  const Y=y=>padT+(1-(y-ymin)/(ymax-ymin))*(h-padT-padB);
  ctx.strokeStyle='#30363d';ctx.lineWidth=1;ctx.beginPath();
  ctx.moveTo(padL,Y(ymin));ctx.lineTo(w-padR,Y(ymin));ctx.stroke();
  ctx.strokeStyle=o.color;ctx.lineWidth=1.5;ctx.beginPath();
  pts.forEach((p,i)=>{const x=X(p[0]),y=Y(p[1]);i?ctx.lineTo(x,y):ctx.moveTo(x,y);});ctx.stroke();
}

function drawMap(id,track,line){
  const s=setupCanvas(id);if(!s)return;const ctx=s.ctx,w=s.w,h=s.h;
  ctx.clearRect(0,0,w,h);ctx.fillStyle='#161b22';ctx.fillRect(0,0,w,h);
  ctx.font='11px ui-monospace,monospace';ctx.fillStyle='#8b949e';ctx.textAlign='left';
  const hasTrack=track&&track.length;
  if(!hasTrack&&!line){ctx.fillText('waiting for GPS track…',8,Math.round(h/2));return;}
  const ref=hasTrack?track[Math.floor(track.length/2)]:[line.lat,line.lon];
  const lat0=ref[0]*Math.PI/180;
  const toXY=(la,lo)=>[(lo-ref[1])*Math.cos(lat0)*111320,(la-ref[0])*111320];
  const pts=(track||[]).map(p=>toXY(p[0],p[1]));
  const lineXY=line?toXY(line.lat,line.lon):null;
  let xs=pts.map(p=>p[0]),ys=pts.map(p=>p[1]);
  if(lineXY){xs=xs.concat([lineXY[0]-line.r,lineXY[0]+line.r]);ys=ys.concat([lineXY[1]-line.r,lineXY[1]+line.r]);}
  if(!xs.length){ctx.fillText('waiting for GPS track…',8,Math.round(h/2));return;}
  const xmn=Math.min.apply(null,xs),xmx=Math.max.apply(null,xs);
  const ymn=Math.min.apply(null,ys),ymx=Math.max.apply(null,ys);
  const dx=(xmx-xmn)||10,dy=(ymx-ymn)||10,pad=26;
  const sc=Math.min((w-2*pad)/dx,(h-2*pad)/dy);
  const cx=(xmn+xmx)/2,cy=(ymn+ymx)/2;
  const X=x=>w/2+(x-cx)*sc,Y=y=>h/2-(y-cy)*sc;
  if(lineXY){
    ctx.strokeStyle='#3fb950';ctx.setLineDash([4,4]);ctx.lineWidth=1.5;ctx.beginPath();
    ctx.arc(X(lineXY[0]),Y(lineXY[1]),line.r*sc,0,2*Math.PI);ctx.stroke();ctx.setLineDash([]);
    ctx.fillStyle='#3fb950';ctx.beginPath();ctx.arc(X(lineXY[0]),Y(lineXY[1]),4,0,2*Math.PI);ctx.fill();
    ctx.fillText('S/F',X(lineXY[0])+6,Y(lineXY[1])-6);
  }
  if(pts.length>1){ctx.strokeStyle='#39c5cf';ctx.lineWidth=2;ctx.beginPath();
    pts.forEach((p,i)=>{const x=X(p[0]),y=Y(p[1]);i?ctx.lineTo(x,y):ctx.moveTo(x,y);});ctx.stroke();}
  if(pts.length){const c=pts[pts.length-1];ctx.fillStyle='#fff';ctx.strokeStyle='#000';ctx.lineWidth=1;
    ctx.beginPath();ctx.arc(X(c[0]),Y(c[1]),5,0,2*Math.PI);ctx.fill();ctx.stroke();}
  ctx.fillStyle='#8b949e';ctx.fillText('~'+Math.round(dx)+' m wide',8,h-8);
}

// Predicted vehicle trajectory from the CURRENT steering angle (bicycle
// model). This is where the car actually goes if you hold this lock — as
// opposed to /ethon/path, which is where the planner WANTS to go. Comparing
// the two is the whole point. Works with no cameras: turn the wheel on the
// bench and the arc sweeps.
//   turn radius R = wheelbase / tan(delta),  +delta = left
//   x = R sin(t),  y = R (1 - cos(t)),  t = arclen / R
function arcPts(degRoadWheel, wheelbase, reach){
  const d=(degRoadWheel||0)*Math.PI/180;
  const pts=[];
  if(!wheelbase||Math.abs(d)<0.0035){                 // ~0.2 deg = straight
    for(let i=0;i<=12;i++) pts.push([reach*i/12,0]);
    return pts;
  }
  const R=wheelbase/Math.tan(d);
  // t is HEADING CHANGE in radians, not distance. reach/|R| is unbounded, so
  // at full lock (R ~ 4.7 m here) a 12 m arc sweeps ~147 deg and curls back
  // across the view. Cap it at a quarter turn: past that the preview stops
  // being a useful "where am I pointed" cue.
  const tmax=Math.min(reach/Math.abs(R), Math.PI/2);
  // x uses |R| and y uses signed R. Using signed R for x too makes a
  // right-hand turn run BACKWARDS through the car (negative forward
  // distance), which draws as a diagonal across the view.
  const aR=Math.abs(R);
  for(let i=0;i<=24;i++){
    const t=tmax*i/24;
    pts.push([aR*Math.sin(t), R*(1-Math.cos(t))]);
  }
  return pts;
}

function drawBird(id,cones,path,drive){
  const s=setupCanvas(id);if(!s)return;const ctx=s.ctx,w=s.w,h=s.h;
  ctx.clearRect(0,0,w,h);ctx.fillStyle='#161b22';ctx.fillRect(0,0,w,h);
  ctx.font='11px ui-monospace,monospace';ctx.textAlign='left';
  const cx=w/2,cy=h/2,pad=18;
  let m=8;
  (cones||[]).forEach(p=>{m=Math.max(m,Math.abs(p[0]),Math.abs(p[1]));});
  (path||[]).forEach(p=>{m=Math.max(m,Math.abs(p[0]),Math.abs(p[1]));});
  const R=Math.max(10,m*1.1);const sc=(Math.min(w,h)/2-pad)/R;
  const PX=(x,y)=>cx-y*sc,PY=(x,y)=>cy-x*sc;   // forward(+x)=up, left(+y)=left
  ctx.strokeStyle='#30363d';ctx.lineWidth=1;ctx.fillStyle='#8b949e';
  for(let r=5;r<=R;r+=5){ctx.beginPath();ctx.arc(cx,cy,r*sc,0,2*Math.PI);ctx.stroke();
    ctx.fillText(r+'m',cx+3,cy-r*sc+11);}
  ctx.strokeStyle='#262c36';ctx.beginPath();ctx.moveTo(cx,pad);ctx.lineTo(cx,h-pad);
  ctx.moveTo(pad,cy);ctx.lineTo(w-pad,cy);ctx.stroke();
  if(path&&path.length>1){ctx.strokeStyle='#58a6ff';ctx.lineWidth=2;ctx.beginPath();
    path.forEach((p,i)=>{const x=PX(p[0],p[1]),y=PY(p[0],p[1]);i?ctx.lineTo(x,y):ctx.moveTo(x,y);});ctx.stroke();}
  // ── predicted trajectory from the live steering angle ──
  const d=drive||{};
  const wb=+d.wheelbase_m||0;
  // rw is in the SAME convention as the wheel panel: positive = LEFT,
  // matching the vehicle frame used below (+y is LEFT) -- verified
  // 2026-08-16 against a live bench-test command, no negation needed.
  const dsg=(d.steer_inverted===true)?1:-1;
  // Verified 2026-08-16: dsg*road_wheel_deg is positive = LEFT (not
  // RIGHT as previously assumed below), matching arcPts' own convention
  // (positive input -> positive y -> LEFT in the +y=left frame). No sign
  // flip is needed at the call site any more -- see arcPts(rw,...) below.
  const rw=(d.road_wheel_deg==null)?null:dsg*(+d.road_wheel_deg);
  const rwMax=(d.road_wheel_max_deg==null)?null:Math.abs(+d.road_wheel_max_deg);
  const poly=(pts,style,width,dash)=>{
    ctx.save();ctx.setLineDash(dash||[]);ctx.strokeStyle=style;ctx.lineWidth=width;
    ctx.beginPath();
    pts.forEach((p,i)=>{const x=PX(p[0],p[1]),y=PY(p[0],p[1]);i?ctx.lineTo(x,y):ctx.moveTo(x,y);});
    ctx.stroke();ctx.restore();
  };
  const reach=Math.min(R*0.95,12);
  if(wb&&rwMax){                                    // steering envelope at full lock
    poly(arcPts( rwMax,wb,reach),'#30363d',1,[4,4]);
    poly(arcPts(-rwMax,wb,reach),'#30363d',1,[4,4]);
  }
  if(wb&&rw!=null&&d.steer_homed){                  // where it will actually go
    poly(arcPts(rw,wb,reach),'#f778ba',2.5);        // rw already +y-left convention
  }
  ctx.fillStyle='#d29922';
  (cones||[]).forEach(p=>{ctx.beginPath();ctx.arc(PX(p[0],p[1]),PY(p[0],p[1]),4,0,2*Math.PI);ctx.fill();});
  ctx.fillStyle='#3fb950';ctx.beginPath();
  ctx.moveTo(cx,cy-9);ctx.lineTo(cx-6,cy+6);ctx.lineTo(cx+6,cy+6);ctx.closePath();ctx.fill();
  ctx.fillStyle='#8b949e';ctx.textAlign='right';
  ctx.fillText('cones '+((cones||[]).length)+' · path '+((path||[]).length)+' pts',w-8,14);
  ctx.textAlign='left';ctx.fillText('▲ forward',8,14);
  // legend so the two curves are never confused
  ctx.font='10px system-ui';
  ctx.fillStyle='#58a6ff';ctx.fillText('— planner path',8,h-20);
  ctx.fillStyle='#f778ba';
  ctx.fillText('— steering now'+(rw==null?'':' ('+rw.toFixed(1)+'°)'),8,h-8);
}

// ── camera previews ───────────────────────────────────────────────────────
// Cheap polling of one JPEG per source rather than MJPEG: the dashboard is a
// stdlib HTTP server with no streaming, and a stuck MJPEG connection would
// hold a thread. Cache-buster on each request so the browser never reuses a
// stale frame.
// Every configured source, not just the ones delivering frames. A source that
// is configured but dead has no preview image, so without this row it is
// simply invisible — you cannot tell "no camera" from "camera broken".
// Also surfaces calibrated=false, which is otherwise silent and is the reason
// detections never reach the planner.
function updateCamStatus(fs){
  const box=document.getElementById('camstat');if(!box)return;
  const src=fs.sources||{};
  const names=Object.keys(src);
  if(!names.length){box.innerHTML='';return;}
  box.innerHTML='';
  for(const n of names){
    const s=src[n]||{};
    const alive=!!s.alive, cal=!!s.calibrated;
    const chip=el('div');chip.className='srcchip';
    const dot=el('span');dot.className='dot2';
    dot.style.background=alive?'var(--grn)':'var(--red)';
    chip.appendChild(dot);
    const nm=el('span',null,n);nm.className='nm';chip.appendChild(nm);
    chip.appendChild(el('span',null,alive?((+s.fps||0).toFixed(1)+' fps'):'down'));
    if(alive) chip.appendChild(el('span',null,(s.detections||0)+' det'));
    const c=el('span',null,cal?'cal':'UNCALIBRATED');
    c.style.color=cal?'var(--grn)':'var(--amb)';
    c.title=cal?'homography loaded — detections reach the planner'
               :'no homography — detections go to /ethon/detections_raw only '
               +'and are NEVER used by the planner. Run calibrate_homography.py.';
    chip.appendChild(c);
    box.appendChild(chip);
  }
}

let camList=[], camSeq=0;
async function camTick(){
  try{
    const r=await fetch('/api/cams',{cache:'no-store'});
    const j=await r.json();
    const cams=j.cams||[];
    if(cams.join(',')!==camList.join(',')){        // rebuild only on change
      camList=cams;
      const wrap=document.getElementById('camwrap');
      const none=document.getElementById('camnone');
      wrap.innerHTML='';
      none.style.display=cams.length?'none':'';
      // Place each tile where its camera physically sits, rather than in
      // whatever order the sources happened to be discovered.
      const LAYOUT={left:[1,1], wide:[2,1], right:[3,1], narrow:[2,2]};
      const ordered=[...cams].sort((a,b)=>{
        const A=LAYOUT[a], B=LAYOUT[b];
        if(A&&B) return (A[1]-B[1])||(A[0]-B[0]);   // row, then column
        if(A) return -1;                            // known cameras first
        if(B) return 1;
        return a.localeCompare(b);                  // anything new: by name
      });
      for(const c of ordered){
        const cell=el('div');cell.className='camcell';
        const pos=LAYOUT[c];
        if(pos){ cell.style.gridColumn=pos[0]; cell.style.gridRow=pos[1]; }
        const img=el('img');img.id='cam_'+c;img.alt=c;
        const lbl=el('div',null,c);lbl.className='camlbl';
        cell.appendChild(img);cell.appendChild(lbl);wrap.appendChild(cell);
      }
    }
    camSeq++;
    for(const c of camList){
      const img=document.getElementById('cam_'+c);
      if(img) img.src='/api/cam?src='+encodeURIComponent(c)+'&_='+camSeq;
    }
  }catch(e){}
}

async function histTick(){
  try{const r=await fetch('/api/history',{cache:'no-store'});const hh=await r.json();
    drawMap('map',hh.track,hh.line);
    lastHist=hh;                       // cached so the faster tick can redraw
    drawBird('bird',hh.cones,hh.path,lastDrive);
    drawChart('c_speed',hh.t,hh.speed,{color:'#58a6ff',label:'Speed',unit:'km/h',dec:1});
    drawChart('c_energy',hh.t,hh.energy,{color:'#3fb950',label:'Energy',unit:'Wh',dec:0});
    drawChart('c_whkm',hh.t,hh.whkm,{color:'#d29922',label:'Efficiency',unit:'Wh/km',dec:0});
    drawChart('c_temp',hh.t,hh.temp,{color:'#f85149',label:'Motor temp',unit:'°C',dec:0});
  }catch(e){}
}

let logSince=0,logRows=[];
async function logTick(){
  if(document.getElementById('logpause').checked)return;
  try{const r=await fetch('/api/logs?since='+logSince,{cache:'no-store'});const j=await r.json();
    if(j.logs&&j.logs.length){logRows=logRows.concat(j.logs);
      if(logRows.length>600)logRows=logRows.slice(-600);logSince=j.last;renderLog();}
  }catch(e){}
}
function renderLog(){
  const lvl=parseInt(document.getElementById('loglevel').value);
  const pre=document.getElementById('log');
  const bottom=pre.scrollTop+pre.clientHeight>=pre.scrollHeight-30;
  pre.innerHTML='';
  for(const r of logRows){if(r.lvl<lvl)continue;
    pre.appendChild(el('div','lg l'+r.lvl,'['+r.lvl_name+'] '+r.name+': '+r.msg));}
  if(bottom)pre.scrollTop=pre.scrollHeight;
}
function clearLog(){logRows=[];renderLog();}

async function selftest(){
  const box=document.getElementById('selftest');
  box.innerHTML='<span class="muted">running checks…</span>';
  try{
    const r=await fetch('/api/selftest',{cache:'no-store'});const j=await r.json();
    box.innerHTML='';
    const v=el('div','armbar '+(j.ready?'ok':'bad'),
      (j.ready?'✓ ':'✗ ')+j.verdict);box.appendChild(v);
    const tbl=el('table');
    for(const c of j.checks){
      const tr=el('tr');
      tr.appendChild(el('td',c.ok?'ok':'bad',c.ok?'PASS':'FAIL'));
      tr.appendChild(el('td',null,c.name));
      tr.appendChild(el('td','pt',c.detail||''));
      tbl.appendChild(tr);
    }
    box.appendChild(tbl);
  }catch(e){box.textContent='self-test request failed';}
}

// ── motor bench test ──
let testTimer=null;
async function setFoc(on){
  try{
    const r=await fetch('/api/param',{method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify({node:'/ethon_drive',name:'use_foc',type:1,value:on?'true':'false'})});
    const j=await r.json();
    toast(j.ok?('control mode -> '+(on?'FOC':'DUTY')):('mode set failed: '+(j.reason||'')),!j.ok);
  }catch(e){toast('mode set failed',true);}
}
function testDuty(){
  const mag=parseInt(document.getElementById('dutyslider').value)/100.0;
  const rev=document.getElementById('dir-rev').checked;
  return rev?-mag:mag;
}
async function postDuty(d){
  try{await fetch('/api/drivetest',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({duty:d})});}catch(e){}
}
function startTest(){
  if(testTimer)return;
  postDuty(testDuty());                       // fire immediately
  testTimer=setInterval(()=>postDuty(testDuty()),200);  // re-post < watchdog
  document.getElementById('test-start').disabled=true;
  document.getElementById('teststate').textContent=' RUNNING';
  document.getElementById('teststate').style.color='var(--grn)';
}
function stopTest(){
  if(testTimer){clearInterval(testTimer);testTimer=null;}
  postDuty(0.0);                              // explicit stop + watchdog backstop
  document.getElementById('test-start').disabled=false;
  document.getElementById('teststate').textContent=' stopped';
  document.getElementById('teststate').style.color='var(--mut)';
}
// safety: stop posting if the page is hidden/closed (watchdog also covers this)
window.addEventListener('beforeunload',()=>{if(testTimer)postDuty(0.0);});
document.addEventListener('visibilitychange',()=>{if(document.hidden)stopTest();});

// ── steering bench test ──
let steerTestTimer=null;
async function postSteerDeg(d){
  try{await fetch('/api/steertest',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({deg:d})});}catch(e){}
}
function startSteerTest(){
  if(steerTestTimer)return;
  const deg=()=>parseFloat(document.getElementById('steerslider').value);
  postSteerDeg(deg());                                   // fire immediately
  steerTestTimer=setInterval(()=>postSteerDeg(deg()),200); // re-post < watchdog
  document.getElementById('steertest-start').disabled=true;
  document.getElementById('steerteststate').textContent=' RUNNING';
  document.getElementById('steerteststate').style.color='var(--grn)';
}
function stopSteerTest(){
  if(steerTestTimer){clearInterval(steerTestTimer);steerTestTimer=null;}
  document.getElementById('steertest-start').disabled=false;
  document.getElementById('steerteststate').textContent=' stopped';
  document.getElementById('steerteststate').style.color='var(--mut)';
  // no explicit zero-post here: unlike drive duty, 0 deg is a real target
  // (centre) -- just stop re-posting and let the watchdog release it.
}
window.addEventListener('beforeunload',()=>{if(steerTestTimer)stopSteerTest();});
document.addEventListener('visibilitychange',()=>{if(document.hidden)stopSteerTest();});

function updateTestPanel(s,drive){
  const foc=drive.use_foc===true;
  const md=document.getElementById('mode-duty'), mf=document.getElementById('mode-foc');
  md.className=foc?'':'g'; mf.className=foc?'g':'';
  document.getElementById('modestate').textContent=
    drive.use_foc==null?'':(' active: '+(foc?'FOC':'DUTY'));
  const dm=drive.motors||{};
  const rows=Object.keys(dm).filter(k=>k!=='steer').map(k=>{
    const m=dm[k]||{};
    return k+': '+(m.vel_rps!=null?m.vel_rps+' rps':'—')+' / '
      +(m.supply_a!=null?m.supply_a+'A':'—')
      +(m.faults&&m.faults.length?' ['+m.faults.join(',')+']':'');
  });
  document.getElementById('testreadout').innerHTML=
    'test_active: '+(drive.test_active?('<span style="color:var(--grn)">YES</span> duty '+drive.test_duty):'no')
    +'<br>'+rows.join('<br>');
  document.getElementById('steertestreadout').innerHTML=
    'steer_test_active: '+(drive.steer_test_active?('<span style="color:var(--grn)">YES</span> target '+drive.steer_test_deg+'&deg;'):'no')
    +'<br>current road-wheel angle: '+(drive.road_wheel_deg!=null?drive.road_wheel_deg+'&deg;':'—')
    +'<br>homed limit: &plusmn;'+(drive.steer_limit_deg!=null?drive.steer_limit_deg+'&deg;':'—');
}

window.addEventListener('resize',()=>{histTick();});
tick();setInterval(tick,600);
histTick();setInterval(histTick,1200);
logTick();setInterval(logTick,1200);
camTick();setInterval(camTick,200);   // ~5 fps preview
</script>
</body>
</html>
"""


PIT_PAGE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Ethon Pit</title>
<style>
:root{--bg:#0d1117;--card:#161b22;--edge:#30363d;--fg:#e6edf3;--mut:#8b949e;
--grn:#3fb950;--red:#f85149;--amb:#d29922;--blu:#58a6ff;}
*{box-sizing:border-box}
body{margin:0;font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;
background:var(--bg);color:var(--fg)}
header{display:flex;align-items:center;gap:14px;padding:10px 16px;border-bottom:1px solid var(--edge)}
header h1{font-size:16px;margin:0}
nav a{margin-right:10px;font-size:12px;text-decoration:none}
main{padding:14px;max-width:1100px;margin:0 auto}
#pace{border-radius:10px;padding:18px;text-align:center;font-size:44px;font-weight:800;
letter-spacing:1px;background:var(--card);border:2px solid var(--edge);margin-bottom:12px}
.tiles{display:grid;grid-template-columns:repeat(auto-fit,minmax(210px,1fr));gap:10px}
.tile{background:var(--card);border:1px solid var(--edge);border-radius:10px;padding:14px;text-align:center}
.tile .k{color:var(--mut);font-size:12px;text-transform:uppercase;letter-spacing:1px}
.tile .v{font-size:40px;font-weight:700;margin-top:4px}
.tile .s{color:var(--mut);font-size:13px;margin-top:2px}
.alerts{display:flex;gap:8px;flex-wrap:wrap;margin-top:12px}
.al{padding:8px 14px;border-radius:8px;font-weight:700;background:var(--card);
border:1px solid var(--edge);color:var(--mut)}
.al.bad{background:var(--red);color:#fff;border-color:var(--red)}
.al.warn{background:var(--amb);color:#000;border-color:var(--amb)}
button{font:inherit;cursor:pointer;border:1px solid var(--grn);background:var(--card);
color:var(--grn);border-radius:8px;padding:10px 18px;font-weight:700}
</style>
</head>
<body>
<header>
  <h1>ETHON&nbsp;//&nbsp;PIT</h1>
  <nav><a href="/dashboard" style="color:var(--mut)">DASH</a>
  <a href="/pit" style="color:var(--blu)">PIT</a>
  <a href="/replay" style="color:var(--mut)">REPLAY</a>
  <a href="/calib" style="color:var(--mut)">CALIB</a></nav>
  <span id="conn" style="margin-left:auto;color:var(--mut);font-size:12px">…</span>
</header>
<main>
  <div id="pace">—</div>
  <div class="tiles" id="tiles"></div>
  <div class="alerts" id="alerts"></div>
  <div style="margin-top:14px"><button onclick="if(confirm('Start the race clock?'))start()">START RACE</button></div>
</main>
<script>
function el(t,c,x){const e=document.createElement(t);if(c)e.className=c;if(x!=null)e.textContent=x;return e;}
function mmss(s){if(s==null)return '—';s=Math.max(0,Math.round(s));
  return Math.floor(s/60)+':'+String(s%60).padStart(2,'0');}
function fmtlap(s){if(s==null)return '—';const m=Math.floor(s/60);
  return m+':'+(s-60*m).toFixed(1).padStart(4,'0');}
async function start(){await fetch('/api/action',{method:'POST',
  headers:{'Content-Type':'application/json'},body:JSON.stringify({action:'race_start'})});}
function tile(k,v,s){const t=el('div','tile');t.appendChild(el('div','k',k));
  t.appendChild(el('div','v',v));if(s)t.appendChild(el('div','s',s));return t;}
async function tick(){
  try{
    const r=await fetch('/api/state',{cache:'no-store'});const st=await r.json();
    document.getElementById('conn').textContent='live';
    const tp=st.topics||{};
    const strat=(tp['/ethon/strategy']||{}).value||{};
    const lap=(tp['/ethon/lap']||{}).value||{};
    const corr=(tp['/ethon/corridor']||{}).value||{};
    const drive=(tp['/ethon/drive_status']||{}).value||{};
    const s=st.status||{};
    const pace=document.getElementById('pace');
    if(!strat.race_on){pace.textContent='RACE NOT STARTED';
      pace.style.borderColor='var(--edge)';pace.style.color='var(--mut)';}
    else{pace.textContent=strat.pace+'  ·  '+mmss(strat.remaining_s)+' left';
      const c=strat.pace_n<0?'var(--red)':(strat.pace_n>0?'var(--grn)':'var(--blu)');
      pace.style.borderColor=c;pace.style.color=c;}
    const tiles=document.getElementById('tiles');tiles.innerHTML='';
    tiles.appendChild(tile('Battery',(strat.battery_pct!=null?strat.battery_pct+'%':'—'),
      (strat.wh_remaining!=null?strat.wh_remaining+' Wh left of '+strat.wh_budget:'')+' (estimate)'));
    tiles.appendChild(tile('Battery V',(drive.supply_v!=null?drive.supply_v.toFixed(1):'—'),
      'measured at the Krakens'));
    tiles.appendChild(tile('Burn rate',(strat.rate_wh_min!=null?strat.rate_wh_min:'—'),
      'Wh/min · budget '+(strat.budget_wh_min!=null?strat.budget_wh_min:'—')));
    tiles.appendChild(tile('Speed',(s.speed_kmh!=null?s.speed_kmh:'—'),'km/h'));
    tiles.appendChild(tile('Lap',(lap.lap!=null?lap.lap:'—'),
      'last '+fmtlap(lap.last_s)+' · best '+fmtlap(lap.best_s)));
    tiles.appendChild(tile('Wh / lap',(strat.wh_per_lap!=null?strat.wh_per_lap:'—'),
      'last lap '+(strat.last_lap_wh!=null?strat.last_lap_wh+' Wh':'—')));
    const mt=Object.values(drive.motors||{}).map(m=>m&&m.temp_c).filter(x=>x!=null);
    tiles.appendChild(tile('Motor temp',(mt.length?Math.max.apply(null,mt):'—'),'°C hottest'));
    tiles.appendChild(tile('Regen',(drive.regen_strength!=null?Math.round(drive.regen_strength*100)+'%':'—'),'e-brake strength'));
    tiles.appendChild(tile('Projected',(strat.projected_wh!=null?strat.projected_wh:'—'),
      'Wh at finish vs '+(strat.wh_budget||'—')));
    const al=document.getElementById('alerts');al.innerHTML='';
    al.appendChild(el('div','al'+(s.estop?' bad':''),s.estop?'E-STOP':'estop clear'));
    al.appendChild(el('div','al'+(s.gps_fix===false?' bad':''),
      s.gps_fix?'GPS OK':'GPS NO FIX'));
    al.appendChild(el('div','al'+((corr.state==='warn'||corr.state==='off')?' warn':''),
      'corridor: '+(corr.state||'—')));
    al.appendChild(el('div','al'+(s.armed?'':''),(s.armed?'ARMED':'disarmed')));
  }catch(e){document.getElementById('conn').textContent='disconnected';}
}
tick();setInterval(tick,1000);
</script>
</body>
</html>
"""


REPLAY_PAGE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Ethon Replay</title>
<style>
:root{--bg:#0d1117;--card:#161b22;--edge:#30363d;--fg:#e6edf3;--mut:#8b949e;
--grn:#3fb950;--red:#f85149;--amb:#d29922;--blu:#58a6ff;--cy:#39c5cf;}
*{box-sizing:border-box}
body{margin:0;font:14px/1.4 ui-monospace,Menlo,Consolas,monospace;background:var(--bg);color:var(--fg)}
header{display:flex;align-items:center;gap:14px;padding:10px 16px;border-bottom:1px solid var(--edge)}
header h1{font-size:16px;margin:0}
nav a{margin-right:10px;font-size:12px;text-decoration:none}
main{padding:16px;max-width:1100px;margin:0 auto}
select{font:inherit;background:var(--card);color:var(--fg);border:1px solid var(--edge);
border-radius:6px;padding:8px 10px;min-width:280px}
.panel{background:var(--card);border:1px solid var(--edge);border-radius:8px;padding:6px;margin-top:12px}
canvas{width:100%;display:block}
#c_speed,#c_energy{height:170px}
#map{height:300px}
table{width:100%;border-collapse:collapse;margin-top:12px}
td,th{text-align:left;padding:6px 8px;border-bottom:1px solid var(--edge)}
th{color:var(--mut);font-size:11px;text-transform:uppercase}
h2{font-size:13px;text-transform:uppercase;letter-spacing:1px;color:var(--mut);margin:18px 0 6px}
.muted{color:var(--mut)}
</style>
</head>
<body>
<header>
  <h1>ETHON&nbsp;//&nbsp;REPLAY</h1>
  <nav><a href="/dashboard" style="color:var(--mut)">DASH</a>
  <a href="/pit" style="color:var(--mut)">PIT</a>
  <a href="/replay" style="color:var(--blu)">REPLAY</a>
  <a href="/calib" style="color:var(--mut)">CALIB</a></nav>
</header>
<main>
  <select id="sess" onchange="load()"><option>loading sessions…</option></select>
  <span class="muted" id="meta"></span>
  <h2>Speed (m/s)</h2><div class="panel"><canvas id="c_speed"></canvas></div>
  <h2>Energy (Wh)</h2><div class="panel"><canvas id="c_energy"></canvas></div>
  <h2>Track</h2><div class="panel"><canvas id="map"></canvas></div>
  <h2>Laps</h2><div id="laps" class="muted">—</div>
</main>
<script>
function el(t,c,x){const e=document.createElement(t);if(c)e.className=c;if(x!=null)e.textContent=x;return e;}
function cv(id){const c=document.getElementById(id);const d=window.devicePixelRatio||1;
  const w=c.clientWidth||600,h=c.clientHeight||160;c.width=w*d;c.height=h*d;
  const x=c.getContext('2d');x.setTransform(d,0,0,d,0,0);return{x:x,w:w,h:h};}
function line(id,t,vals,color){
  const s=cv(id),x=s.x,w=s.w,h=s.h;x.fillStyle='#161b22';x.fillRect(0,0,w,h);
  const pts=[];for(let i=0;i<vals.length;i++)if(vals[i]!=null)pts.push([t[i],vals[i]]);
  if(pts.length<2){x.fillStyle='#8b949e';x.fillText('no data',10,h/2);return;}
  const xm=pts[0][0],xM=pts[pts.length-1][0]||1;
  let ym=Math.min.apply(null,pts.map(p=>p[1])),yM=Math.max.apply(null,pts.map(p=>p[1]));
  if(ym===yM){ym-=1;yM+=1;}
  const X=v=>10+(v-xm)/(xM-xm||1)*(w-20),Y=v=>h-14-(v-ym)/(yM-ym)*(h-30);
  x.strokeStyle=color;x.lineWidth=1.6;x.beginPath();
  pts.forEach((p,i)=>{i?x.lineTo(X(p[0]),Y(p[1])):x.moveTo(X(p[0]),Y(p[1]));});x.stroke();
  x.fillStyle='#8b949e';x.font='11px monospace';
  x.fillText(yM.toFixed(1),4,12);x.fillText(ym.toFixed(1),4,h-4);
}
function track(id,tr){
  const s=cv(id),x=s.x,w=s.w,h=s.h;x.fillStyle='#161b22';x.fillRect(0,0,w,h);
  if(!tr||tr.length<2){x.fillStyle='#8b949e';x.fillText('no GPS in this session',10,h/2);return;}
  const ref=tr[Math.floor(tr.length/2)],la0=ref[0]*Math.PI/180;
  const pts=tr.map(p=>[(p[1]-ref[1])*Math.cos(la0)*111320,(p[0]-ref[0])*111320]);
  const xs=pts.map(p=>p[0]),ys=pts.map(p=>p[1]);
  const xm=Math.min.apply(null,xs),xM=Math.max.apply(null,xs);
  const ym=Math.min.apply(null,ys),yM=Math.max.apply(null,ys);
  const sc=Math.min((w-40)/((xM-xm)||10),(h-40)/((yM-ym)||10));
  const cx=(xm+xM)/2,cy=(ym+yM)/2;
  x.strokeStyle='#39c5cf';x.lineWidth=2;x.beginPath();
  pts.forEach((p,i)=>{const px=w/2+(p[0]-cx)*sc,py=h/2-(p[1]-cy)*sc;
    i?x.lineTo(px,py):x.moveTo(px,py);});x.stroke();
  x.fillStyle='#8b949e';x.fillText('~'+Math.round(xM-xm)+' m wide',8,h-8);
}
async function init(){
  const r=await fetch('/api/sessions',{cache:'no-store'});const j=await r.json();
  const sel=document.getElementById('sess');sel.innerHTML='';
  if(!j.sessions.length){sel.appendChild(el('option',null,'no sessions logged yet'));return;}
  for(const s of j.sessions){const o=el('option',null,s.name+'  ('+s.mtime+', '+s.kb+' kB)');
    o.value=s.name;sel.appendChild(o);}
  load();
}
async function load(){
  const f=document.getElementById('sess').value;if(!f)return;
  const r=await fetch('/api/session?f='+encodeURIComponent(f),{cache:'no-store'});
  const d=await r.json();if(d.ok===false)return;
  document.getElementById('meta').textContent=' '+d.n+' samples';
  line('c_speed',d.t,d.speed,'#58a6ff');
  line('c_energy',d.t,d.energy,'#3fb950');
  track('map',d.track);
  const box=document.getElementById('laps');
  if(!d.laps.length){box.textContent='no completed laps in this session';return;}
  box.className='';const tbl=el('table');const hr=el('tr');
  for(const hh of ['Lap','Time','Wh'])hr.appendChild(el('th',null,hh));tbl.appendChild(hr);
  for(const L of d.laps){const tr=el('tr');
    tr.appendChild(el('td',null,L.lap));
    tr.appendChild(el('td',null,L.lap_s==null?'—':(Math.floor(L.lap_s/60)+':'+(L.lap_s%60).toFixed(1).padStart(4,'0'))));
    tr.appendChild(el('td',null,L.wh==null?'—':L.wh));
    tbl.appendChild(tr);}
  box.innerHTML='';box.appendChild(tbl);
}
init();
</script>
</body>
</html>
"""


CALIB_PAGE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Ethon Calibration</title>
<style>
:root{--bg:#0d1117;--card:#161b22;--edge:#30363d;--fg:#e6edf3;--mut:#8b949e;
--grn:#3fb950;--red:#f85149;--amb:#d29922;--blu:#58a6ff;--pk:#f778ba;}
*{box-sizing:border-box}
body{margin:0;font:14px/1.4 ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;
background:var(--bg);color:var(--fg)}
header{display:flex;align-items:center;gap:14px;padding:10px 16px;border-bottom:1px solid var(--edge)}
header h1{font-size:16px;margin:0}
nav a{margin-right:10px;font-size:12px;text-decoration:none}
main{padding:16px;max-width:1300px;margin:0 auto}
.muted{color:var(--mut)}
.row{display:flex;gap:16px;flex-wrap:wrap;align-items:flex-start}
.panel{background:var(--card);border:1px solid var(--edge);border-radius:8px;padding:10px}
select{font:inherit;background:var(--card);color:var(--fg);border:1px solid var(--edge);
border-radius:6px;padding:7px 10px;min-width:260px}
button{font:inherit;cursor:pointer;border:1px solid var(--edge);background:var(--card);
color:var(--fg);border-radius:6px;padding:7px 14px}
button:hover{border-color:var(--blu)}
.imgwrap{position:relative;line-height:0}
#shot{max-width:100%;height:auto;cursor:crosshair;border-radius:4px;display:block}
#loupe{position:absolute;top:8px;right:8px;border:2px solid var(--pk);border-radius:6px;
background:#000;pointer-events:none}
#loupecoord{position:absolute;top:8px;left:8px;background:rgba(0,0,0,.65);
padding:2px 8px;border-radius:4px;font-size:12px;pointer-events:none}
.side{flex:0 0 340px;min-width:280px}
table{width:100%;border-collapse:collapse;margin-top:6px}
td,th{text-align:left;padding:4px 6px;border-bottom:1px solid var(--edge);font-size:12px}
th{color:var(--mut);text-transform:uppercase;font-size:10px}
input.pv{font:inherit;background:#0d1117;color:var(--fg);border:1px solid var(--edge);
border-radius:5px;padding:4px 6px;width:64px;font-size:12px}
textarea{font:inherit;font-size:12px;width:100%;background:#0d1117;color:var(--grn);
border:1px solid var(--edge);border-radius:6px;padding:8px;resize:vertical;min-height:64px}
#toast2{position:fixed;bottom:16px;right:16px;background:var(--card);border:1px solid var(--edge);
border-radius:8px;padding:10px 14px;opacity:0;transition:opacity .2s;pointer-events:none}
#toast2.show{opacity:1}
.hint{font-size:12px;color:var(--mut);margin:6px 0 12px;line-height:1.5}
.badge{display:inline-block;font-size:10px;padding:1px 6px;border-radius:10px;
border:1px solid var(--edge);margin-left:6px}
.badge.cal{color:var(--grn);border-color:var(--grn)}
</style>
</head>
<body>
<header>
  <h1>ETHON&nbsp;//&nbsp;CALIB</h1>
  <nav><a href="/dashboard" style="color:var(--mut)">DASH</a>
  <a href="/pit" style="color:var(--mut)">PIT</a>
  <a href="/replay" style="color:var(--mut)">REPLAY</a>
  <a href="/calib" style="color:var(--blu)">CALIB</a></nav>
</header>
<main>
  <div class="hint">
    Pixel-picker for <code>calibrate_homography.py --solve</code>. This page only reads
    coordinates off a snapshot already on disk and builds the command for you to run over
    SSH &mdash; it does not capture frames or solve the homography itself, so
    <code>calibrate_homography.py</code> stays the one place that math happens.
    Click a marker's exact centre on the image below (use the magnifier, top-right, for
    precision near the fisheye edges), then fill in its measured ground position in
    <b>metres</b> &mdash; x = forward from the rear axle, y = left (right is negative).
  </div>
  <div class="row" style="margin-bottom:10px">
    <select id="src"></select>
    <button id="reload">&#8635; reload sources</button>
    <button id="clearpts">clear points</button>
    <span class="muted" id="dims"></span>
  </div>
  <div class="row">
    <div class="panel imgwrap" style="flex:1 1 600px">
      <canvas id="shot" width="1" height="1"></canvas>
      <canvas id="loupe" width="200" height="200"></canvas>
      <div id="loupecoord"></div>
    </div>
    <div class="side panel">
      <div id="points" class="muted">click the image to add markers</div>
      <div style="margin-top:12px">
        <div class="muted" style="margin-bottom:4px">generated command (run on the Jetson):</div>
        <textarea id="cmd" readonly></textarea>
        <button id="copybtn" style="margin-top:6px">copy</button>
      </div>
    </div>
  </div>
</main>
<div id="toast2"></div>
<script>
function el(t,c,txt){const e=document.createElement(t);if(c)e.className=c;if(txt!=null)e.textContent=txt;return e;}
let img=new Image(), natW=0, natH=0, points=[], currentSrc=null;

async function loadSources(){
  const r=await fetch('/api/calib_snapshots',{cache:'no-store'});
  const j=await r.json();
  const sel=document.getElementById('src');
  const cur=sel.value;
  sel.innerHTML='';
  const keys=Object.keys(j.snapshots).sort();
  if(!keys.length){sel.appendChild(el('option',null,'no snapshots yet -- run calibrate_homography.py first'));return;}
  for(const k of keys){
    const info=j.snapshots[k];
    const o=el('option',null,k+'   ('+info.mtime+')'+(info.calibrated?'  [already calibrated]':''));
    o.value=k;sel.appendChild(o);
  }
  if(keys.includes(cur))sel.value=cur;
  loadImage(sel.value);
}

function loadImage(src){
  if(!src)return;
  currentSrc=src;points=[];renderPoints();updateCmd();
  img=new Image();
  img.onload=()=>{
    natW=img.naturalWidth;natH=img.naturalHeight;
    const cv=document.getElementById('shot');
    cv.width=natW;cv.height=natH;
    redraw();
    document.getElementById('dims').textContent=natW+' x '+natH+' px';
  };
  img.src='/api/calib_snapshot?src='+encodeURIComponent(src)+'&_='+Date.now();
}

function redraw(){
  const cv=document.getElementById('shot');const ctx=cv.getContext('2d');
  if(!natW)return;
  ctx.drawImage(img,0,0);
  const r=Math.max(4,natW/150),lw=Math.max(1,natW/500);
  points.forEach((p,i)=>{
    ctx.strokeStyle='#f778ba';ctx.fillStyle='#f778ba';ctx.lineWidth=lw;
    ctx.beginPath();ctx.moveTo(p.u-r,p.v);ctx.lineTo(p.u+r,p.v);
    ctx.moveTo(p.u,p.v-r);ctx.lineTo(p.u,p.v+r);ctx.stroke();
    ctx.beginPath();ctx.arc(p.u,p.v,r*1.6,0,2*Math.PI);ctx.stroke();
    ctx.font=Math.max(12,natW/60)+'px system-ui';
    ctx.fillText(String(i+1),p.u+r*1.8,p.v-r*0.5);
  });
}

function canvasCoords(ev){
  const cv=document.getElementById('shot');const rect=cv.getBoundingClientRect();
  const sx=cv.width/rect.width,sy=cv.height/rect.height;
  let u=(ev.clientX-rect.left)*sx,v=(ev.clientY-rect.top)*sy;
  u=Math.max(0,Math.min(natW-1,u));v=Math.max(0,Math.min(natH-1,v));
  return {u:u,v:v};
}

function updateLoupe(ev){
  if(!natW)return;
  const {u,v}=canvasCoords(ev);
  const lp=document.getElementById('loupe');const lctx=lp.getContext('2d');
  const zoom=6,half=lp.width/(2*zoom);
  lctx.imageSmoothingEnabled=false;
  lctx.clearRect(0,0,lp.width,lp.height);
  lctx.drawImage(img,u-half,v-half,half*2,half*2,0,0,lp.width,lp.height);
  lctx.strokeStyle='#f778ba';lctx.lineWidth=1;
  lctx.beginPath();
  lctx.moveTo(lp.width/2,0);lctx.lineTo(lp.width/2,lp.height);
  lctx.moveTo(0,lp.height/2);lctx.lineTo(lp.width,lp.height/2);
  lctx.stroke();
  document.getElementById('loupecoord').textContent='u='+u.toFixed(0)+'  v='+v.toFixed(0);
}

function onClick(ev){
  if(!natW)return;
  const {u,v}=canvasCoords(ev);
  points.push({u:Math.round(u),v:Math.round(v),x:'',y:''});
  redraw();renderPoints();updateCmd();
}

function renderPoints(){
  const box=document.getElementById('points');box.innerHTML='';
  if(!points.length){box.appendChild(el('div','muted','click the image to add markers'));return;}
  const tbl=el('table');const hdr=el('tr');
  for(const h of ['#','u','v','x (fwd, m)','y (left, m)',''])hdr.appendChild(el('th',null,h));
  tbl.appendChild(hdr);
  points.forEach((p,i)=>{
    const tr=el('tr');
    tr.appendChild(el('td',null,String(i+1)));
    tr.appendChild(el('td',null,String(p.u)));
    tr.appendChild(el('td',null,String(p.v)));
    const tx=el('td');const ix=el('input','pv');ix.value=p.x;ix.placeholder='e.g. 1.5';
    ix.oninput=()=>{p.x=ix.value;updateCmd();};tx.appendChild(ix);tr.appendChild(tx);
    const ty=el('td');const iy=el('input','pv');iy.value=p.y;iy.placeholder='e.g. 0.9';
    iy.oninput=()=>{p.y=iy.value;updateCmd();};ty.appendChild(iy);tr.appendChild(ty);
    const td=el('td');const rb=el('button',null,'×');
    rb.onclick=()=>{points.splice(i,1);redraw();renderPoints();updateCmd();};
    td.appendChild(rb);tr.appendChild(td);
    tbl.appendChild(tr);
  });
  box.appendChild(tbl);
}

function srcFlag(){
  if(!currentSrc)return'';
  if(currentSrc.indexOf('cam_tcp')===0)return'--tcp '+currentSrc.slice('cam_tcp'.length);
  if(currentSrc.indexOf('cam')===0)return'--camera '+currentSrc.slice('cam'.length);
  return'';
}

function updateCmd(){
  const box=document.getElementById('cmd');
  const ready=points.filter(p=>p.x!==''&&p.y!==''&&!isNaN(parseFloat(p.x))&&!isNaN(parseFloat(p.y)));
  if(!currentSrc){box.value='# pick a source above';return;}
  if(ready.length<4){box.value='# need >= 4 points with x,y filled in ('+ready.length+' ready)';return;}
  const pts=ready.map(p=>p.u+','+p.v+','+parseFloat(p.x)+','+parseFloat(p.y)).join('  ');
  box.value='python3 calibrate_homography.py '+srcFlag()+' --solve \\\n    "'+pts+'"';
}

function copyCmd(){
  const box=document.getElementById('cmd');
  box.select();box.setSelectionRange(0,99999);
  try{document.execCommand('copy');toast2('copied');}catch(e){toast2('select + copy manually');}
}
function toast2(msg){
  const t=document.getElementById('toast2');
  t.textContent=msg;t.className='show';setTimeout(()=>t.className='',1500);
}

document.getElementById('src').addEventListener('change',e=>loadImage(e.target.value));
document.getElementById('shot').addEventListener('click',onClick);
document.getElementById('shot').addEventListener('mousemove',updateLoupe);
document.getElementById('reload').addEventListener('click',loadSources);
document.getElementById('clearpts').addEventListener('click',()=>{points=[];redraw();renderPoints();updateCmd();});
document.getElementById('copybtn').addEventListener('click',copyCmd);
loadSources();
</script>
</body>
</html>
"""


PAGE_V2 = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<meta name="color-scheme" content="dark">
<title>Ethon Console</title>
<style>
/* ── Ethon console design system ──────────────────────────────────────────
   Ground rules, so later edits don't fight the cascade:
     * Layout is done with grid/flex + gap. Elements do NOT carry their own
       outer margins; a parent's gap owns the space between siblings. If you
       need more room, change the container, not the child.
     * Selectors stay single-class and flat. There is deliberately no rule
       that reaches across the tree (no `.panel .row span`), so a new element
       can be dropped anywhere without inheriting spacing it didn't ask for.
     * Semantic colour (ok/warn/crit) means VEHICLE STATE and nothing else.
       Teal is "commanded or planned", magenta is "actually measured", orange
       is cones. Never borrow one for the other -- the whole point is that a
       glance at the colour tells you which of those four things you're
       looking at.
   ------------------------------------------------------------------------ */
:root{
  /* ground: near-black, biased blue-green so it reads as instrument glass
     rather than as an unconsidered neutral grey */
  --ground:#090B0F;
  --bg2:#0E1218;
  --panel:#131923;
  --panel2:#182030;
  --rule:#212B3A;
  --rule2:#2C3949;

  --ink:#EAF0F7;
  --ink2:#9AA9BC;   /* blue-grey, picked to sit with the ground */
  --ink3:#64748B;

  --ok:#2ED47A;
  --warn:#F5A524;
  --crit:#FF4D4F;

  --accent:#4CE0D2;  /* commanded / planned / interactive */
  --actual:#FF5CA8;  /* actual / measured */
  --cone:#FF8A3D;    /* literal traffic-cone orange */

  --sans:system-ui,-apple-system,"Segoe UI",Roboto,"Helvetica Neue",Arial,sans-serif;
  --mono:ui-monospace,"SF Mono","Cascadia Mono",Menlo,Consolas,"DejaVu Sans Mono",monospace;

  --gap:14px;
  --pad:14px;
}
*{box-sizing:border-box}
html,body{height:100%}
body{
  margin:0;background:var(--ground);color:var(--ink);
  font:400 13px/1.45 var(--sans);
  -webkit-font-smoothing:antialiased;
  font-variant-numeric:tabular-nums;
}
:focus-visible{outline:2px solid var(--accent);outline-offset:2px;border-radius:3px}
@media (prefers-reduced-motion:reduce){
  *{animation-duration:.001ms !important;transition-duration:.001ms !important}
}

/* ── command bar ─────────────────────────────────────────────────────────
   Square-edged and full-bleed on purpose: it is the frame of the instrument,
   not another card floating inside it. */
.cmd{
  position:sticky;top:0;z-index:40;
  display:flex;align-items:center;gap:18px;flex-wrap:wrap;
  padding:0 16px;min-height:52px;
  background:rgba(9,11,15,.88);backdrop-filter:blur(10px);
  border-bottom:1px solid var(--rule);
}
.brand{display:flex;align-items:baseline;gap:9px;font-weight:700;letter-spacing:.14em;font-size:13px}
.brand .mk{color:var(--ink)}
.brand .sub{color:var(--ink3);font-weight:600;letter-spacing:.18em;font-size:10px}

.live{display:flex;align-items:center;gap:7px;font-size:11px;color:var(--ink2);letter-spacing:.06em}
.beat{width:7px;height:7px;border-radius:50%;background:var(--ink3);flex:0 0 auto}
.beat.up{background:var(--ok);box-shadow:0 0 0 3px rgba(46,212,122,.16)}
.beat.down{background:var(--crit);box-shadow:0 0 0 3px rgba(255,77,79,.16)}

/* ── view tabs ── */
.views{display:flex;gap:2px;background:var(--bg2);border:1px solid var(--rule);border-radius:7px;padding:3px}
.view{
  appearance:none;border:0;cursor:pointer;
  font:600 11px/1 var(--sans);letter-spacing:.14em;
  color:var(--ink3);background:transparent;
  padding:8px 13px;border-radius:5px;
  transition:color .12s,background .12s;
}
.view:hover{color:var(--ink2)}
.view[aria-selected="true"]{background:var(--panel2);color:var(--ink);box-shadow:inset 0 1px 0 rgba(255,255,255,.05)}
.view .cnt{margin-left:6px;color:var(--crit);font-size:10px}

/* links to the other pages: quieter than the view tabs, because switching
   document is a rarer act than switching view */
.pages{display:flex;gap:13px;align-items:center}
.pages a{font:600 10px/1 var(--sans);letter-spacing:.14em;color:var(--ink3);text-decoration:none}
.pages a:hover{color:var(--accent)}

/* ── status chips in the bar ── */
.chips{display:flex;gap:6px;flex-wrap:wrap;align-items:center;margin-left:auto}
.chip{
  display:inline-flex;align-items:center;gap:6px;
  font:600 10.5px/1 var(--sans);letter-spacing:.1em;
  color:var(--ink2);background:var(--bg2);
  border:1px solid var(--rule);border-radius:4px;padding:6px 9px;white-space:nowrap;
}
.chip b{font-weight:700;color:var(--ink);letter-spacing:0;font-family:var(--mono);font-size:11px}
.chip.ok{color:var(--ok);border-color:rgba(46,212,122,.35);background:rgba(46,212,122,.08)}
.chip.warn{color:var(--warn);border-color:rgba(245,165,36,.35);background:rgba(245,165,36,.08)}
.chip.crit{color:#fff;background:var(--crit);border-color:var(--crit)}
.chip.crit b{color:#fff}

/* ── the two controls that must never be hunted for ── */
.cmdacts{display:flex;gap:8px;align-items:center}
.estop{
  appearance:none;cursor:pointer;
  font:800 12px/1 var(--sans);letter-spacing:.16em;
  color:#fff;background:var(--crit);border:1px solid #FF6B6D;
  border-radius:5px;padding:11px 18px;
  box-shadow:0 1px 0 rgba(255,255,255,.18) inset,0 0 18px rgba(255,77,79,.28);
}
.estop:hover{background:#FF6265}
.estop:active{transform:translateY(1px)}

/* ── status ribbon ── */
.ribbon{
  display:flex;align-items:center;gap:11px;
  padding:11px 16px;font-weight:600;font-size:12.5px;letter-spacing:.02em;
  border-bottom:1px solid var(--rule);
  background:var(--bg2);color:var(--ink2);
  border-left:3px solid var(--ink3);
}
.ribbon.ok{border-left-color:var(--ok);color:var(--ok);background:rgba(46,212,122,.05)}
.ribbon.warn{border-left-color:var(--warn);color:var(--warn);background:rgba(245,165,36,.05)}
.ribbon.crit{border-left-color:var(--crit);color:var(--crit);background:rgba(255,77,79,.07)}

/* ── page frame ──
   `.pane[hidden]` outranks `.pane` on specificity, so a hidden pane stays
   hidden no matter where these two rules sit relative to each other. */
.wrap{max-width:1680px;margin:0 auto;padding:var(--gap) 16px 56px}
.pane{display:grid;gap:var(--gap)}
.pane[hidden]{display:none}
/* a grid cell that holds several stacked panels rather than one */
.stack{display:grid;gap:var(--gap);align-content:start}

/* ── instrument cluster ─────────────────────────────────────────────────
   Deliberately NOT a row of cards. One recessed strip divided by hairlines,
   the way a cluster is one piece of glass with printed dividers. */
.cluster{
  display:grid;grid-template-columns:repeat(auto-fit,minmax(132px,1fr));
  background:var(--bg2);border:1px solid var(--rule);border-radius:9px;
  overflow:hidden;
}
.cell{padding:13px 16px;border-left:1px solid var(--rule);display:flex;flex-direction:column;gap:3px}
.cell:first-child{border-left:0}
.cell .k{font:600 10px/1 var(--sans);letter-spacing:.16em;color:var(--ink3);text-transform:uppercase}
.cell .v{font:700 27px/1.05 var(--sans);letter-spacing:-.02em;color:var(--ink);font-variant-numeric:tabular-nums}
.cell .u{font:600 11px/1 var(--sans);color:var(--ink3);letter-spacing:.06em}
.cell .foot{font:500 11px/1.2 var(--mono);color:var(--ink3)}
.cell.lead{background:var(--panel)}
.cell.lead .v{font-size:44px;letter-spacing:-.035em}
.cell .v.ok{color:var(--ok)} .cell .v.warn{color:var(--warn)} .cell .v.crit{color:var(--crit)}
.cell .v.dim{color:var(--ink3)}

/* ── panels ── */
.panel{background:var(--panel);border:1px solid var(--rule);border-radius:9px;
  box-shadow:inset 0 1px 0 rgba(255,255,255,.035);display:flex;flex-direction:column;min-width:0}
.phead{display:flex;align-items:center;gap:10px;padding:10px 14px;border-bottom:1px solid var(--rule)}
.phead h2{margin:0;font:600 11px/1 var(--sans);letter-spacing:.16em;text-transform:uppercase;color:var(--ink2)}
.phead .hint{font:400 11px/1 var(--sans);color:var(--ink3);letter-spacing:0}
.phead .right{margin-left:auto;display:flex;gap:7px;align-items:center}
.pbody{padding:var(--pad);display:flex;flex-direction:column;gap:11px;min-width:0}
.pbody.flush{padding:0}

/* ── grid columns ── */
.cols{display:grid;gap:var(--gap);grid-template-columns:repeat(12,1fr)}
.c12{grid-column:span 12} .c8{grid-column:span 8} .c7{grid-column:span 7}
.c6{grid-column:span 6}   .c5{grid-column:span 5} .c4{grid-column:span 4}
@media (max-width:1180px){
  .c8,.c7,.c6,.c5,.c4{grid-column:span 12}
}

/* canvases fill their panel; heights are set per-canvas so the aspect the
   drawing code assumes is the aspect it actually gets */
canvas{display:block;width:100%}
#bird{height:520px} #steerviz{height:220px}
#motors,#motors_b{height:158px}
#map{height:262px} #energy{height:220px}
.chartgrid{display:grid;grid-template-columns:repeat(auto-fit,minmax(258px,1fr));gap:var(--gap)}
.chartgrid canvas{height:158px}
@media (max-width:1180px){ #bird{height:420px} }

/* ── buttons ── */
.btns{display:flex;flex-wrap:wrap;gap:8px;align-items:center}
button.b{
  appearance:none;cursor:pointer;
  font:600 12px/1 var(--sans);letter-spacing:.04em;
  color:var(--ink);background:var(--panel2);
  border:1px solid var(--rule2);border-radius:5px;padding:10px 15px;
  transition:border-color .12s,background .12s,color .12s;
}
button.b:hover{border-color:var(--ink3);background:#1E2839}
button.b:active{transform:translateY(1px)}
button.b:disabled{opacity:.42;cursor:not-allowed;transform:none}
button.b.go{color:var(--ok);border-color:rgba(46,212,122,.4)}
button.b.go:hover{background:rgba(46,212,122,.1);border-color:var(--ok)}
button.b.cau{color:var(--warn);border-color:rgba(245,165,36,.4)}
button.b.cau:hover{background:rgba(245,165,36,.1);border-color:var(--warn)}
button.b.dgr{color:var(--crit);border-color:rgba(255,77,79,.42)}
button.b.dgr:hover{background:rgba(255,77,79,.12);border-color:var(--crit)}
button.b.on{background:rgba(76,224,210,.12);border-color:var(--accent);color:var(--accent)}

/* ── form controls ── */
select,input[type=text],input.pv{
  font:500 12px/1 var(--mono);color:var(--ink);background:var(--bg2);
  border:1px solid var(--rule2);border-radius:5px;padding:9px 10px;
}
input.pv{width:150px}
label.opt{display:inline-flex;align-items:center;gap:6px;color:var(--ink2);font-size:12px;cursor:pointer}
input[type=range]{
  -webkit-appearance:none;appearance:none;height:4px;border-radius:2px;
  background:var(--rule2);outline:none;width:230px;
}
input[type=range]::-webkit-slider-thumb{
  -webkit-appearance:none;width:16px;height:16px;border-radius:50%;
  background:var(--accent);border:2px solid var(--ground);cursor:grab;
}
input[type=range]::-moz-range-thumb{
  width:16px;height:16px;border-radius:50%;border:2px solid var(--ground);
  background:var(--accent);cursor:grab;
}
.val{font:700 14px/1 var(--mono);color:var(--ink);min-width:52px;display:inline-block}

/* ── hazard notice: for the panels that can physically move the car ── */
.hazard{
  display:flex;gap:11px;align-items:flex-start;
  padding:11px 13px;border-radius:6px;font-size:12px;line-height:1.5;
  background:rgba(245,165,36,.07);border:1px solid rgba(245,165,36,.3);color:var(--warn);
}
.hazard .mark{font-weight:800;letter-spacing:.1em;flex:0 0 auto}
.hazard p{margin:0;color:#E8D2A8}

/* ── readouts / key-value rows ── */
.kv{display:flex;flex-direction:column;gap:0}
/* two columns of readouts, so a tall list of pairs doesn't stretch its panel
   past the ones beside it in the grid row */
.kv2{display:grid;grid-template-columns:1fr 1fr;gap:0 20px}
@media (max-width:520px){.kv2{grid-template-columns:1fr}}
.kvrow{display:flex;justify-content:space-between;gap:12px;align-items:baseline;
  padding:7px 0;border-bottom:1px solid var(--rule)}
.kvrow:last-child{border-bottom:0}
.kvrow .k{color:var(--ink3);font-size:11.5px;letter-spacing:.04em}
.kvrow .v{font:600 12.5px/1 var(--mono);color:var(--ink);text-align:right}

/* ── tables ── */
.tbl{width:100%;border-collapse:collapse}
.tbl th{text-align:left;padding:8px 10px;border-bottom:1px solid var(--rule2);
  font:600 10px/1 var(--sans);letter-spacing:.14em;text-transform:uppercase;color:var(--ink3)}
.tbl td{padding:8px 10px;border-bottom:1px solid var(--rule);vertical-align:middle;font-size:12.5px}
.tbl tr:last-child td{border-bottom:0}
.tbl td.mono{font-family:var(--mono)}
.tbl td.pass{color:var(--ok);font-weight:700} .tbl td.fail{color:var(--crit);font-weight:700}
.scrollx{overflow-x:auto}

/* ── camera tiles ── */
.camgrid{display:grid;grid-template-columns:repeat(3,1fr);gap:10px}
@media (max-width:900px){.camgrid{grid-template-columns:1fr}
  .camcell{grid-column:auto !important;grid-row:auto !important}}
.camcell{position:relative;background:#05070A;border:1px solid var(--rule);
  border-radius:7px;overflow:hidden;min-height:96px}
.camcell img{width:100%;display:block}
.camlbl{position:absolute;top:7px;left:8px;font:600 10px/1 var(--sans);letter-spacing:.14em;
  text-transform:uppercase;background:rgba(5,7,10,.72);color:var(--ink);padding:4px 7px;border-radius:3px}
.srcs{display:flex;flex-wrap:wrap;gap:7px}
.src{display:inline-flex;align-items:center;gap:7px;font:500 11px/1 var(--sans);
  background:var(--bg2);border:1px solid var(--rule);border-radius:5px;padding:6px 9px;color:var(--ink2)}
.src .nm{font-weight:700;color:var(--ink);letter-spacing:.06em;text-transform:uppercase;font-size:10px}
.src .d{width:6px;height:6px;border-radius:50%}
.src .fps{font-family:var(--mono)}

/* ── log ── */
.log{height:340px;overflow:auto;background:#05070A;border:1px solid var(--rule);
  border-radius:7px;padding:10px;font:500 11.5px/1.55 var(--mono);margin:0}
.lg{padding:1px 0;white-space:pre-wrap;word-break:break-word;color:var(--ink2)}
.l10{color:var(--ink3)} .l30{color:var(--warn)} .l40{color:var(--crit)}
.l50{color:#fff;background:rgba(255,77,79,.24);border-radius:2px}

/* ── topics ── */
.topics{display:grid;grid-template-columns:repeat(auto-fill,minmax(320px,1fr));gap:10px}
.topic{background:var(--bg2);border:1px solid var(--rule);border-radius:7px;padding:11px 12px;
  overflow:hidden;display:flex;flex-direction:column;gap:5px}
.topic .tn{color:var(--accent);font:600 12px/1.3 var(--mono);word-break:break-all}
.topic .ty{color:var(--ink3);font-size:10.5px;font-family:var(--mono)}
.topic .age{font:600 10.5px/1 var(--mono);color:var(--ink3);white-space:nowrap}
.topic.stale{border-color:rgba(245,165,36,.35)}
.topic.stale .age{color:var(--warn)}
.topic pre{margin:0;white-space:pre-wrap;word-break:break-word;
  font:500 11px/1.5 var(--mono);color:var(--ink2);max-height:210px;overflow:auto}
.throw{display:flex;justify-content:space-between;gap:8px;align-items:baseline}

/* ── system health bars ── */
.meters{display:grid;grid-template-columns:repeat(auto-fit,minmax(190px,1fr));gap:11px}
.meter{display:flex;flex-direction:column;gap:6px}
.meter .top{display:flex;justify-content:space-between;align-items:baseline;gap:8px}
.meter .k{font:600 10px/1 var(--sans);letter-spacing:.14em;text-transform:uppercase;color:var(--ink3)}
.meter .v{font:700 13px/1 var(--mono);color:var(--ink)}
.bar{height:5px;border-radius:3px;background:var(--rule);overflow:hidden}
.bar i{display:block;height:100%;border-radius:3px;background:var(--accent);transition:width .3s}
.bar i.ok{background:var(--ok)} .bar i.warn{background:var(--warn)} .bar i.crit{background:var(--crit)}

/* ── toast ── */
.toast{
  position:fixed;bottom:18px;right:18px;z-index:80;max-width:360px;
  background:var(--panel2);border:1px solid var(--rule2);border-left:3px solid var(--accent);
  border-radius:6px;padding:12px 15px;font-size:12.5px;color:var(--ink);
  opacity:0;transform:translateY(6px);pointer-events:none;
  transition:opacity .18s,transform .18s;
  box-shadow:0 12px 32px rgba(0,0,0,.55);
}
.toast.show{opacity:1;transform:none}
.toast.err{border-left-color:var(--crit)}

.muted{color:var(--ink3)}
.empty{color:var(--ink3);font-size:12px;padding:6px 0}
</style>
</head>
<body>
<header class="cmd">
  <div class="brand"><span class="mk">ETHON</span><span class="sub">CONSOLE</span></div>

  <div class="views" role="tablist" aria-label="Console views">
    <button class="view" role="tab" id="tab-drive" aria-selected="true"  aria-controls="pane-drive" onclick="showView('drive')">DRIVE</button>
    <button class="view" role="tab" id="tab-bench" aria-selected="false" aria-controls="pane-bench" onclick="showView('bench')">BENCH</button>
    <button class="view" role="tab" id="tab-tune"  aria-selected="false" aria-controls="pane-tune"  onclick="showView('tune')">TUNE</button>
    <button class="view" role="tab" id="tab-diag"  aria-selected="false" aria-controls="pane-diag"  onclick="showView('diag')">DIAG<span class="cnt" id="diagcnt"></span></button>
  </div>

  <div class="chips" id="chips"></div>

  <nav class="pages" aria-label="Other pages">
    <a href="/pit2">PIT</a><a href="/replay">REPLAY</a><a href="/calib">CALIB</a>
    <a href="/dashboard" title="the previous dashboard, unchanged">OLD</a>
  </nav>

  <div class="live"><span class="beat" id="beat"></span><span id="beattxt">connecting</span></div>

  <div class="cmdacts">
    <button class="b go" id="armbtn" onclick="act('arm')">ARM</button>
    <button class="estop" onclick="act('estop')">E&#8209;STOP</button>
  </div>
</header>

<div class="ribbon" id="ribbon">checking drive&hellip;</div>

<div class="wrap">

<!-- ══ DRIVE ═══════════════════════════════════════════════════════════ -->
<section class="pane" id="pane-drive" role="tabpanel" aria-labelledby="tab-drive">

  <div class="cluster" id="cluster"></div>

  <div class="cols">
    <div class="c8">
      <div class="panel">
        <div class="phead">
          <h2>Situation</h2>
          <span class="hint">robot frame &middot; forward is up</span>
          <div class="right"><span class="hint" id="birdcount"></span></div>
        </div>
        <div class="pbody flush"><canvas id="bird"></canvas></div>
      </div>
    </div>

    <div class="c4 stack">
      <div class="panel">
        <div class="phead"><h2>Command</h2></div>
        <div class="pbody">
          <div class="btns">
            <button class="b" onclick="act('disarm')">DISARM</button>
            <button class="b cau" onclick="act('clear_estop')">CLEAR E&#8209;STOP</button>
            <button class="b go" onclick="act('mark')">MARK LINE</button>
          </div>
          <div class="btns">
            <button class="b" id="md-autonomy" onclick="act('mode','autonomy')">AUTONOMY</button>
            <button class="b" id="md-capture" onclick="act('mode','capture')">CAPTURE</button>
            <button class="b go" onclick="confirmRace()">START RACE</button>
          </div>
        </div>
      </div>

      <div class="panel">
        <div class="phead"><h2>Motors</h2><span class="hint">4&times; Kraken on can0</span></div>
        <div class="pbody flush"><canvas id="motors"></canvas></div>
      </div>
    </div>

    <div class="c12">
      <div class="panel">
        <div class="phead">
          <h2>Cameras</h2><span class="hint">what the model sees</span>
          <div class="right"><span class="hint" id="camnote"></span></div>
        </div>
        <div class="pbody">
          <div class="srcs" id="srcs"></div>
          <div class="camgrid" id="camgrid"></div>
          <div class="empty" id="camnone">No source is delivering frames yet &mdash; a preview appears here as soon as one does.</div>
        </div>
      </div>
    </div>

    <div class="c4">
      <div class="panel">
        <div class="phead"><h2>Steering</h2><span class="hint" id="steerhint"></span></div>
        <div class="pbody">
          <canvas id="steerviz"></canvas>
          <div class="kv2">
            <div class="kvrow"><span class="k">state</span><span class="v" id="sv_state">&mdash;</span></div>
            <div class="kvrow"><span class="k">road wheel</span><span class="v" id="sv_deg">&mdash;</span></div>
            <div class="kvrow"><span class="k">column</span><span class="v" id="sv_col">&mdash;</span></div>
            <div class="kvrow"><span class="k">lock</span><span class="v" id="sv_lim">&mdash;</span></div>
            <div class="kvrow"><span class="k">travel used</span><span class="v" id="sv_pct">&mdash;</span></div>
            <div class="kvrow"><span class="k">control</span><span class="v" id="sv_mode">&mdash;</span></div>
            <div class="kvrow"><span class="k">motor</span><span class="v" id="sv_mot">&mdash;</span></div>
          </div>
        </div>
      </div>
    </div>

    <div class="c4">
      <div class="panel">
        <div class="phead"><h2>Track</h2><span class="hint">GPS &middot; world frame</span></div>
        <div class="pbody flush"><canvas id="map"></canvas></div>
      </div>
    </div>

    <div class="c4">
      <div class="panel">
        <div class="phead"><h2>Energy</h2><span class="hint">race budget</span></div>
        <div class="pbody flush"><canvas id="energy"></canvas></div>
      </div>
    </div>

    <div class="c12">
      <div class="panel">
        <div class="phead"><h2>Telemetry</h2><span class="hint">last 20 minutes &middot; hover to scrub</span></div>
        <div class="pbody">
          <div class="chartgrid">
            <canvas id="c_speed"></canvas>
            <canvas id="c_energy"></canvas>
            <canvas id="c_whkm"></canvas>
            <canvas id="c_temp"></canvas>
          </div>
        </div>
      </div>
    </div>
  </div>
</section>

<!-- ══ BENCH ═══════════════════════════════════════════════════════════ -->
<section class="pane" id="pane-bench" role="tabpanel" aria-labelledby="tab-bench" hidden>

  <div class="hazard">
    <span class="mark">HAZARD</span>
    <p>Everything on this tab drives the hardware directly, bypassing the planner &mdash; and the
    steering test also bypasses the armed hand&#8209;back rule. <b>All four wheels must be off the
    ground.</b> Every test re&#8209;posts its command 5&times; a second; stop posting and the vehicle
    watchdog releases the motors within half a second.</p>
  </div>

  <div class="cols">
    <div class="c12">
      <div class="panel">
        <div class="phead"><h2>Pre&#8209;flight self test</h2>
          <div class="right"><button class="b cau" onclick="selftest()">RUN SELF&#8209;TEST</button></div>
        </div>
        <div class="pbody"><div id="selftest" class="empty">Not run yet.</div></div>
      </div>
    </div>

    <div class="c6">
      <div class="panel">
        <div class="phead"><h2>Drive motors</h2><span class="hint">open&#8209;loop duty</span></div>
        <div class="pbody">
          <div class="btns">
            <span class="muted">Control mode</span>
            <button class="b" id="mode-duty" onclick="setFoc(false)">DUTY</button>
            <button class="b" id="mode-foc" onclick="setFoc(true)">FOC</button>
            <span class="muted" id="modestate"></span>
          </div>
          <div class="btns">
            <span class="muted">Duty</span>
            <input id="dutyslider" type="range" min="0" max="30" value="10" step="1"
                   aria-label="Test duty cycle percent"
                   oninput="document.getElementById('dutyval').textContent=this.value+'%'">
            <span class="val" id="dutyval">10%</span>
            <label class="opt"><input type="radio" name="dir" id="dir-fwd" checked> forward</label>
            <label class="opt"><input type="radio" name="dir" id="dir-rev"> reverse</label>
          </div>
          <div class="btns">
            <button class="b go" id="test-start" onclick="startTest()">START</button>
            <button class="estop" id="test-stop" onclick="stopTest()">STOP</button>
            <span class="muted" id="teststate"></span>
          </div>
          <div class="kv" id="testreadout"></div>
        </div>
      </div>
    </div>

    <div class="c6">
      <div class="panel">
        <div class="phead"><h2>Steering column</h2><span class="hint">direct angle command</span></div>
        <div class="pbody">
          <div class="btns">
            <span class="muted">Target</span>
            <input id="steerslider" type="range" min="-30" max="30" value="0" step="0.5"
                   aria-label="Target road wheel angle in degrees"
                   oninput="document.getElementById('steerval').textContent=(this.value>0?'+':'')+this.value+'°'">
            <span class="val" id="steerval">0&deg;</span>
          </div>
          <div class="btns">
            <button class="b go" id="steertest-start" onclick="startSteerTest()">START</button>
            <button class="estop" id="steertest-stop" onclick="stopSteerTest()">STOP</button>
            <span class="muted" id="steerteststate"></span>
          </div>
          <div class="kv" id="steertestreadout"></div>
        </div>
      </div>
    </div>

    <div class="c12">
      <div class="panel">
        <div class="phead"><h2>Motors</h2><span class="hint">live while testing</span></div>
        <div class="pbody flush"><canvas id="motors_b"></canvas></div>
      </div>
    </div>
  </div>
</section>

<!-- ══ TUNE ════════════════════════════════════════════════════════════ -->
<section class="pane" id="pane-tune" role="tabpanel" aria-labelledby="tab-tune" hidden>
  <div class="panel">
    <div class="phead">
      <h2>Parameters</h2><span class="hint">edits apply live, and are lost on node restart</span>
      <div class="right">
        <select id="pnode" onchange="loadParams()" aria-label="Node"><option value="">&mdash; pick a node &mdash;</option></select>
        <button class="b" onclick="loadParams()">RELOAD</button>
      </div>
    </div>
    <div class="pbody">
      <div class="btns" id="nodeshort"></div>
      <div id="params" class="empty">Pick a node to view and edit its parameters.</div>
    </div>
  </div>
</section>

<!-- ══ DIAG ════════════════════════════════════════════════════════════ -->
<section class="pane" id="pane-diag" role="tabpanel" aria-labelledby="tab-diag" hidden>
  <div class="cols">
    <div class="c6">
      <div class="panel">
        <div class="phead"><h2>Jetson</h2><span class="hint" id="throt"></span></div>
        <div class="pbody"><div class="meters" id="sysmeters"></div></div>
      </div>
    </div>

    <div class="c6">
      <div class="panel">
        <div class="phead"><h2>Topic rates</h2><span class="hint">measured vs required</span></div>
        <div class="pbody flush"><div class="scrollx"><table class="tbl" id="hztbl"></table></div></div>
      </div>
    </div>

    <div class="c12">
      <div class="panel">
        <div class="phead">
          <h2>Log</h2><span class="hint">/rosout</span>
          <div class="right">
            <label class="opt"><input type="checkbox" id="logpause"> pause</label>
            <select id="loglevel" onchange="renderLog()" aria-label="Minimum log level">
              <option value="0">ALL</option><option value="10">DEBUG+</option>
              <option value="20" selected>INFO+</option><option value="30">WARN+</option>
              <option value="40">ERROR+</option>
            </select>
            <button class="b" onclick="clearLog()">CLEAR</button>
          </div>
        </div>
        <div class="pbody"><pre id="log" class="log"></pre></div>
      </div>
    </div>

    <div class="c12">
      <div class="panel">
        <div class="phead"><h2>Topics</h2><span class="hint">every ROS topic the dashboard mirrors</span></div>
        <div class="pbody"><div class="topics" id="topics"></div></div>
      </div>
    </div>
  </div>
</section>

</div>
<div class="toast" id="toast" role="status" aria-live="polite"></div>
<script>
// ── gfx_chart.js ──
// ── telemetry chart ────────────────────────────────────────────────────────
// drawChart(id, t, vals, o) — one series over the ~5 minute history window.
// Replaces the original flat 1.5 px polyline. Call signature is unchanged so
// the four existing call sites keep working verbatim:
//   drawChart('c_speed', hh.t, hh.speed, {color:'#…',label:'Speed',unit:'km/h',dec:1})
//
// Facts about the data this has to survive (see EthonDashboard.history()):
//   • t and vals are parallel arrays of up to 1200 samples taken at 4 Hz.
//     t is SECONDS SINCE THE OLDEST SAMPLE, not epoch, and it restarts from 0
//     as the deque rolls — so nothing here may cache an absolute time.
//   • ANY element of vals may be null. whkm is null the whole time the car is
//     stationary, temp is null until a motor reports a temperature, speed is
//     null before /ethon/drive_status ever arrives. All-null is normal, not an
//     error, and gets a legible "no data" state rather than a blank panel.
//   • The chart is redrawn from histTick() every 1200 ms, from the window
//     resize handler, and from its own mousemove listener. It therefore has to
//     be safe to call at any time in any order, and must not accumulate state.
//
// This module renders. It never fetches, posts, sets a timer, or touches any
// element other than the canvas it was handed. Nothing it does can reach the
// vehicle. Nothing here animates either: redraws are externally driven at
// 1.2 s, so any time-based motion would alias into a flicker rather than a
// pulse. _chReduceMotion() exists and is honoured for the one piece of
// non-essential visual emphasis, so a future pulse inherits the gate for free.

function drawChart(id, t, vals, o){
  var s = setupCanvas(id); if(!s) return;
  var ctx = s.ctx, w = s.w, h = s.h;
  var cv = document.getElementById(id);
  o = o || {};
  // Cache the last arguments ON THE ELEMENT, not in a module-level map: the
  // hover listener needs them to repaint, and per-canvas storage is what keeps
  // four live charts from stealing each other's crosshair.
  if(cv){ cv._chT = t; cv._chV = vals; cv._chO = o; }
  _chAttachHover(cv);
  if(cv) cv._chBusy = true;
  try{
    _chPaint(ctx, w, h, cv, t, vals, o);
  }catch(e){
    // A chart is a diagnostic, not a control. If it throws, the panel says so
    // and the rest of the page keeps updating — histTick() swallows errors,
    // but the mousemove path has no such net, so the net lives here.
    try{
      ctx.setLineDash([]);
      ctx.fillStyle = '#131923'; ctx.fillRect(0, 0, w, h);
      ctx.textAlign = 'left'; ctx.textBaseline = 'alphabetic';
      ctx.font = _chFont(11, 600, false); ctx.fillStyle = '#F5A524';
      ctx.fillText('chart render error', 10, Math.round(h / 2));
    }catch(e2){}
  }
  if(cv) cv._chBusy = false;
}

// ── hover crosshair ────────────────────────────────────────────────────────
// Attached ONCE per canvas, guarded by a dataset flag. drawChart runs every
// 1.2 s; adding a listener per draw would pile up 3000 of them an hour on four
// canvases and every one of them would keep a stale closure alive.
function _chAttachHover(cv){
  if(!cv || !cv.dataset || cv.dataset.chHover === '1') return;
  cv.dataset.chHover = '1';
  cv.addEventListener('mousemove', function(ev){
    try{
      var r = cv.getBoundingClientRect();
      if(!r.width) return;
      // setupCanvas works in CSS pixels; scale through clientWidth so page
      // zoom or a CSS transform cannot desync the crosshair from the trace.
      var x = (ev.clientX - r.left) * ((cv.clientWidth || r.width) / r.width);
      if(cv._chHoverX != null && Math.abs(cv._chHoverX - x) < 0.5) return;
      cv._chHoverX = x;
      _chRedraw(cv);
    }catch(e){}
  });
  cv.addEventListener('mouseleave', function(){
    try{
      if(cv._chHoverX == null) return;
      cv._chHoverX = null;
      _chRedraw(cv);
    }catch(e){}
  });
}

// Repaint from the cached arguments. _chBusy makes the module re-entrant: a
// repaint triggered from inside a paint is dropped rather than recursing.
function _chRedraw(cv){
  if(!cv || cv._chBusy || !cv._chO) return;
  drawChart(cv.id, cv._chT, cv._chV, cv._chO);
}

function _chReduceMotion(){
  try{
    return !!(window.matchMedia
      && window.matchMedia('(prefers-reduced-motion: reduce)').matches);
  }catch(e){ return false; }
}

// ── small helpers ──────────────────────────────────────────────────────────
// No webfonts exist on this box, so the stacks are spelled out in full. All
// numbers are set in mono on purpose: canvas 2d on Chrome 90 cannot do
// font-variant-numeric:tabular-nums, and a proportional readout jitters
// sideways every time a digit changes — unreadable at a glance in a pit lane.
function _chFont(px, weight, mono){
  var fam = mono
    ? 'ui-monospace,"SF Mono","Cascadia Mono",Menlo,Consolas,"DejaVu Sans Mono",monospace'
    : 'system-ui,-apple-system,"Segoe UI",Roboto,"Helvetica Neue",Arial,sans-serif';
  return (weight || 400) + ' ' + px + 'px ' + fam;
}

// Manual letter tracking. ctx.letterSpacing landed in Chrome 99 and the target
// here is 90, so uppercase labels get spaced a character at a time.
function _chSpacedText(ctx, text, x, y, sp){
  for(var i = 0; i < text.length; i++){
    var c = text.charAt(i);
    ctx.fillText(c, x, y);
    x += ctx.measureText(c).width + sp;
  }
  return x - sp;
}
function _chSpacedWidth(ctx, text, sp){
  if(!text.length) return 0;
  return ctx.measureText(text).width + sp * (text.length - 1);
}

function _chColor(c){
  // Honour whatever the caller passed, verbatim. The series colour is the
  // caller's semantic decision, not this module's, and silently remapping it
  // is exactly how a chart ends up lying about which quantity it is showing.
  return (typeof c === 'string' && c.length) ? c : '#4CE0D2';
}

function _chDec(d){
  d = +d;
  if(!isFinite(d)) return 1;
  return Math.max(0, Math.min(6, Math.round(d)));
}

function _chNum(v, dec){
  if(v == null) return '—';
  v = +v;
  if(!isFinite(v)) return '—';
  var a = Math.abs(v);
  if(a >= 1e6) return v.toExponential(1);
  if(a >= 1e4 && dec > 0) dec = 0;   // stop a big value from running into the label
  return v.toFixed(dec);
}

// Alpha variants of the series colour for the area gradient and the endpoint
// halo. Returns null for anything that is not a hex or rgb() string; callers
// fall back to globalAlpha so an exotic colour still renders, just flatter.
function _chRGBA(c, a){
  if(typeof c !== 'string') return null;
  var s = c.trim(), r, g, b;
  if(s.charAt(0) === '#'){
    var hex = s.slice(1);
    if(hex.length === 3){
      r = parseInt(hex.charAt(0) + hex.charAt(0), 16);
      g = parseInt(hex.charAt(1) + hex.charAt(1), 16);
      b = parseInt(hex.charAt(2) + hex.charAt(2), 16);
    }else if(hex.length === 6){
      r = parseInt(hex.slice(0, 2), 16);
      g = parseInt(hex.slice(2, 4), 16);
      b = parseInt(hex.slice(4, 6), 16);
    }else return null;
    if(!isFinite(r) || !isFinite(g) || !isFinite(b)) return null;
    return 'rgba(' + r + ',' + g + ',' + b + ',' + a + ')';
  }
  if(s.indexOf('rgb') === 0){
    var m = s.replace(/[^0-9.,]/g, '').split(',');
    if(m.length < 3) return null;
    return 'rgba(' + (+m[0] || 0) + ',' + (+m[1] || 0) + ',' + (+m[2] || 0) + ',' + a + ')';
  }
  return null;
}

// "Nice number" gridlines: 1/2/5 x 10^n. Dividing the raw min/max into equal
// slices gives labels like 37.416, which nobody can read off a moving chart —
// the point of a gridline is that you can name the value it sits on.
function _chNiceStep(span, want){
  if(!isFinite(span) || span <= 0) return 1;
  var raw = span / Math.max(1, want);
  var mag = Math.pow(10, Math.floor(Math.log(raw) / Math.LN10));
  if(!isFinite(mag) || mag <= 0) return 1;
  var n = raw / mag;
  var step = (n <= 1) ? 1 : ((n <= 2) ? 2 : ((n <= 5) ? 5 : 10));
  return step * mag;
}

function _chTicks(lo, hi, want){
  if(!isFinite(lo) || !isFinite(hi)){ lo = 0; hi = 1; }
  if(hi <= lo) hi = lo + 1;
  want = Math.max(2, Math.min(8, Math.round(want) || 3));
  var step = _chNiceStep(hi - lo, want);
  var l = Math.floor(lo / step) * step;
  var t = Math.ceil(hi / step) * step;
  var cnt = Math.round((t - l) / step);
  if(!isFinite(cnt) || cnt < 1) cnt = 1;
  if(cnt > 24) cnt = 24;             // never loop away on a pathological range
  // Decimals implied by the step, so 0.5-spaced lines read 12.0/12.5 and
  // 5-spaced lines read 10/15 rather than 10.00000000001.
  var dec = Math.max(0, Math.min(4, Math.ceil(-Math.log(step) / Math.LN10)));
  var vals = [];
  for(var i = 0; i <= cnt; i++) vals.push(+(l + i * step).toFixed(6));
  return {lo: l, hi: +(l + cnt * step).toFixed(6), step: step, dec: dec, vals: vals};
}

function _chRound(ctx, x, y, w, h, r){
  ctx.beginPath();
  ctx.moveTo(x + r, y);
  ctx.arcTo(x + w, y, x + w, y + h, r);
  ctx.arcTo(x + w, y + h, x, y + h, r);
  ctx.arcTo(x, y + h, x, y, r);
  ctx.arcTo(x, y, x + w, y, r);
  ctx.closePath();
}

// A min or max marker: hollow ring plus its value, nudged so the label cannot
// escape the plot. dir -1 puts the label above the point, +1 below.
function _chMark(ctx, px, py, dir, txt, l, r, bt, skipX){
  if(!isFinite(px) || !isFinite(py)) return;
  if(isFinite(skipX) && Math.abs(px - skipX) < 14) return;  // would sit under the endpoint
  ctx.strokeStyle = '#64748B'; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.arc(px, py, 2.6, 0, Math.PI * 2); ctx.stroke();
  var tw = ctx.measureText(txt).width, tx = px, al = 'center';
  if(px - tw / 2 < l){ al = 'left'; tx = l; }
  else if(px + tw / 2 > r){ al = 'right'; tx = r; }
  // A low minimum would otherwise put its label on top of the footer line.
  var ly = py + (dir < 0 ? -7 : 13);
  if(dir > 0 && ly > bt - 2) ly = py - 7;
  ctx.textAlign = al; ctx.fillStyle = '#64748B';
  ctx.fillText(txt, tx, ly);
}

// ── the actual paint ───────────────────────────────────────────────────────
function _chPaint(ctx, w, h, cv, t, vals, o){
  var i, v, x, y;
  var color = _chColor(o.color);
  var label = (o.label == null ? '' : String(o.label)).toUpperCase();
  var unit  = (o.unit == null ? '' : String(o.unit));
  var dec   = _chDec(o.dec);

  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = '#131923'; ctx.fillRect(0, 0, w, h);   // panel
  ctx.textBaseline = 'alphabetic';
  ctx.lineJoin = 'round'; ctx.lineCap = 'round';
  ctx.setLineDash([]);

  // ── normalise input ──
  var V = (vals && vals.length != null) ? vals : [];
  var n = V.length | 0;
  // If t is short or missing, fall back to sample index for EVERY point rather
  // than for some of them: a half-indexed x axis draws a plausible-looking
  // chart with the wrong time base, which is worse than an unlabelled one.
  var T = (t && t.length != null && t.length >= n) ? t : null;
  var xs = new Array(n), pts = [];
  for(i = 0; i < n; i++){
    x = T ? +T[i] : i;
    xs[i] = isFinite(x) ? x : NaN;
    v = V[i];
    if(v == null) continue;
    v = +v;
    if(!isFinite(v) || !isFinite(xs[i])) continue;
    pts.push([xs[i], v, i]);
  }

  // ── layout ──
  var padL = 10;
  var padT = (h < 110) ? Math.max(18, h * 0.19) : 28;
  var padB = (h < 110) ? 12 : 16;
  // Right gutter holds the gridline value labels. It is wider than the text
  // needs because the endpoint dot sits hard against the plot edge and paints
  // a panel-coloured knock-out disc around itself — too narrow a gutter and
  // that disc eats the first character of a six-digit tick label.
  var padR = Math.max(20, Math.min(54, w * 0.24));
  var pw = w - padL - padR, ph = h - padT - padB;
  var base = Math.max(12, Math.round(padT - 9));     // shared header baseline
  var right = w - 8;

  // ── header: label left, big current value right ──
  var cur = pts.length ? pts[pts.length - 1][1] : null;
  var first = pts.length ? pts[0][1] : null;
  var uw = 0;
  if(unit){
    ctx.font = _chFont(10, 600, false); ctx.textAlign = 'right';
    ctx.fillStyle = '#64748B';
    uw = ctx.measureText(unit).width + 4;
    ctx.fillText(unit, right, base);
  }
  // The readout shrinks on a short canvas so it cannot clip out of the top.
  ctx.font = _chFont(h < 110 ? 13 : 16, 700, true);
  var curTxt = _chNum(cur, dec);
  var curW = ctx.measureText(curTxt).width;
  ctx.textAlign = 'right';
  ctx.fillStyle = (cur == null) ? '#64748B' : color;
  ctx.fillText(curTxt, right - uw, base);
  var blockL = right - uw - curW;

  // Delta against the start of the window. The arrow is deliberately NOT
  // coloured: green/amber/red are reserved for vehicle STATE, and direction is
  // not state — a rising speed is good, a rising motor temp is not, so a green
  // "up" would be telling the engineer something untrue about half the charts.
  if(cur != null && first != null && pts.length > 1){
    var d = cur - first;
    var tol = Math.pow(10, -dec) / 2;
    var glyph = (d > tol) ? '▲' : ((d < -tol) ? '▼' : '·');
    var dTxt = glyph + ' ' + ((d > 0 ? '+' : '') + _chNum(d, dec));
    ctx.font = _chFont(10, 600, true); ctx.fillStyle = '#9AA9BC';
    ctx.textAlign = 'right';
    ctx.fillText(dTxt, blockL - 8, base);
    blockL -= 8 + ctx.measureText(dTxt).width;
  }

  if(label){
    ctx.font = _chFont(10, 600, false);
    ctx.textAlign = 'left'; ctx.fillStyle = '#9AA9BC';
    var avail = blockL - 8 - padL, lab = label;
    while(lab.length > 1 && _chSpacedWidth(ctx, lab, 0.6) > avail) {
      lab = lab.slice(0, lab.length - 2) + '…';
    }
    if(_chSpacedWidth(ctx, lab, 0.6) <= avail) _chSpacedText(ctx, lab, padL, base, 0.6);
  }

  // Too small to plot into (a collapsed or mid-transition panel). The header
  // above already carries the live number, so bail out rather than draw
  // anything that would have to be squeezed into two pixels.
  if(pw < 24 || ph < 20) return;

  // ── no data ──
  // All-null is a normal boot state (and the normal state of Wh/km while the
  // car is stationary), so it gets a real answer instead of an empty box —
  // and it says WHICH kind of nothing, because "no samples at all" and "1200
  // samples that are all null" point at completely different faults.
  if(!pts.length){
    ctx.setLineDash([3, 4]); ctx.strokeStyle = '#212B3A'; ctx.lineWidth = 1;
    y = Math.round(padT + Math.max(0, ph) * 0.62) + 0.5;
    ctx.beginPath(); ctx.moveTo(padL, y); ctx.lineTo(Math.max(padL, w - padR), y);
    ctx.stroke(); ctx.setLineDash([]);
    ctx.textAlign = 'center'; ctx.fillStyle = '#64748B';
    ctx.font = _chFont(11, 600, false);
    _chCentred(ctx, 'NO DATA', padL + Math.max(0, pw) / 2, padT + Math.max(0, ph) * 0.45, 1.2);
    ctx.font = _chFont(9.5, 400, false);
    ctx.fillText(n ? 'series is all null' : 'waiting for samples',
                 padL + Math.max(0, pw) / 2, padT + Math.max(0, ph) * 0.45 + 14);
    return;
  }

  // ── domains ──
  var ymin = pts[0][1], ymax = ymin, minI = 0, maxI = 0;
  for(i = 1; i < pts.length; i++){
    v = pts[i][1];
    if(v < ymin){ ymin = v; minI = i; }
    if(v > ymax){ ymax = v; maxI = i; }
  }
  var flat = (ymin === ymax);
  if(flat){ ymin -= 1; ymax += 1; }        // flat series: keep the original's +/-1 box
  else { var yp = (ymax - ymin) * 0.06; ymin -= yp; ymax += yp; }
  var tk = _chTicks(ymin, ymax, Math.max(2, Math.min(6, Math.round(ph / 34))));

  // The x domain is the WHOLE window from t, not just the span that happens to
  // hold finite values. That keeps the four charts on a common time axis (the
  // same x is the same instant on every one of them) and makes a series that
  // only just started reading draw as a short trace on the right, which is the
  // honest picture.
  var xmin = Infinity, xmax = -Infinity;
  for(i = 0; i < n; i++){
    if(!isFinite(xs[i])) continue;
    if(xs[i] < xmin) xmin = xs[i];
    if(xs[i] > xmax) xmax = xs[i];
  }
  if(!isFinite(xmin) || !isFinite(xmax)){ xmin = pts[0][0]; xmax = pts[pts.length - 1][0]; }
  var xr = xmax - xmin;
  // Single sample (or a degenerate window): pin it to the right edge, where the
  // latest sample always lives, so the endpoint emphasis still means "now".
  var X = function(xv){ return (xr > 0) ? (padL + (xv - xmin) / xr * pw) : (padL + pw); };
  var Y = function(yv){ return padT + (1 - (yv - tk.lo) / (tk.hi - tk.lo)) * ph; };

  // ── gridlines + right-gutter labels ──
  ctx.lineWidth = 1;
  for(i = 0; i < tk.vals.length; i++){
    v = tk.vals[i];
    y = Math.round(Y(v)) + 0.5;          // half-pixel so a 1 px rule stays 1 px
    if(y < padT - 1 || y > h - padB + 1) continue;
    ctx.strokeStyle = (v === 0 && tk.lo < 0 && tk.hi > 0) ? '#2C3949' : '#212B3A';
    ctx.beginPath(); ctx.moveTo(padL, y); ctx.lineTo(w - padR, y); ctx.stroke();
    ctx.font = _chFont(10, 600, true);
    ctx.textAlign = 'right'; ctx.fillStyle = '#64748B';
    ctx.fillText(_chNum(v, tk.dec), right, Math.min(h - 4, Math.max(padT + 4, y + 3.5)));
  }

  // ── trace + area fill ──
  // Split at nulls instead of drawing straight through them. whkm is null for
  // every sample the car is stationary; a connecting line there would invent an
  // efficiency figure that was never measured, and that is a number someone
  // would then go and act on.
  var segs = [], seg = [pts[0]];
  for(i = 1; i < pts.length; i++){
    if(pts[i][2] - pts[i - 1][2] > 1){ segs.push(seg); seg = []; }
    seg.push(pts[i]);
  }
  segs.push(seg);

  var grad = null;
  var c22 = _chRGBA(color, 0.22), c00 = _chRGBA(color, 0);
  if(c22 && c00){
    grad = ctx.createLinearGradient(0, padT, 0, h - padB);
    grad.addColorStop(0, c22);
    grad.addColorStop(1, c00);
  }

  ctx.save();
  ctx.beginPath(); ctx.rect(padL, padT - 2, pw, ph + 4); ctx.clip();
  var bottom = h - padB;
  for(var si = 0; si < segs.length; si++){
    var sp = segs[si];
    if(sp.length === 1){
      // An isolated sample between two null runs would otherwise be invisible.
      ctx.fillStyle = color;
      ctx.beginPath(); ctx.arc(X(sp[0][0]), Y(sp[0][1]), 1.8, 0, Math.PI * 2); ctx.fill();
      continue;
    }
    ctx.beginPath();
    ctx.moveTo(X(sp[0][0]), bottom);
    for(i = 0; i < sp.length; i++) ctx.lineTo(X(sp[i][0]), Y(sp[i][1]));
    ctx.lineTo(X(sp[sp.length - 1][0]), bottom);
    ctx.closePath();
    if(grad){ ctx.fillStyle = grad; ctx.fill(); }
    else { ctx.globalAlpha = 0.14; ctx.fillStyle = color; ctx.fill(); ctx.globalAlpha = 1; }
    ctx.beginPath();
    for(i = 0; i < sp.length; i++){
      x = X(sp[i][0]); y = Y(sp[i][1]);
      if(i) ctx.lineTo(x, y); else ctx.moveTo(x, y);
    }
    ctx.strokeStyle = color; ctx.lineWidth = 2; ctx.stroke();
  }
  ctx.restore();

  var last = pts[pts.length - 1];
  var epx = X(last[0]), epy = Y(last[1]);

  // ── min / max markers ──
  if(pts.length > 2 && !flat){
    ctx.font = _chFont(9.5, 600, true);
    _chMark(ctx, X(pts[maxI][0]), Y(pts[maxI][1]), -1, _chNum(pts[maxI][1], dec),
            padL, w - padR, h - padB, epx);
    _chMark(ctx, X(pts[minI][0]), Y(pts[minI][1]), 1, _chNum(pts[minI][1], dec),
            padL, w - padR, h - padB, epx);
  }

  // ── endpoint: the only sample that is "now" ──
  var halo = _chRGBA(color, _chReduceMotion() ? 0.16 : 0.30);
  if(halo){
    var rg = ctx.createRadialGradient(epx, epy, 0, epx, epy, 10);
    rg.addColorStop(0, halo);
    rg.addColorStop(1, _chRGBA(color, 0));
    ctx.fillStyle = rg;
    ctx.beginPath(); ctx.arc(epx, epy, 10, 0, Math.PI * 2); ctx.fill();
  }
  ctx.fillStyle = '#131923';                       // knock the trace out behind the dot
  ctx.beginPath(); ctx.arc(epx, epy, 5, 0, Math.PI * 2); ctx.fill();
  ctx.fillStyle = color;
  ctx.beginPath(); ctx.arc(epx, epy, 3.2, 0, Math.PI * 2); ctx.fill();

  // ── footer ──
  ctx.font = _chFont(9.5, 400, true);
  ctx.textAlign = 'left'; ctx.fillStyle = '#64748B';
  ctx.fillText((T ? (xr >= 1 ? Math.round(xr) + ' s window · ' : '') : '')
               + pts.length + ' pts', padL, h - 5);

  // ── hover crosshair ──
  var hx = cv ? cv._chHoverX : null;
  if(hx == null || !isFinite(hx) || hx < padL - 8 || hx > w - padR + 8) return;
  var bi = -1, bd = 1e9;
  for(i = 0; i < n; i++){
    if(!isFinite(xs[i])) continue;
    var dx = Math.abs(X(xs[i]) - hx);
    if(dx < bd){ bd = dx; bi = i; }
  }
  if(bi < 0) return;
  var hpx = Math.round(X(xs[bi])) + 0.5;
  ctx.strokeStyle = '#2C3949'; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(hpx, padT); ctx.lineTo(hpx, h - padB); ctx.stroke();

  var hv = V[bi];
  hv = (hv == null) ? null : +hv;
  if(hv != null && isFinite(hv)){
    var hpy = Y(hv);
    ctx.fillStyle = '#131923';
    ctx.beginPath(); ctx.arc(hpx, hpy, 4.5, 0, Math.PI * 2); ctx.fill();
    ctx.strokeStyle = color; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(hpx, hpy, 3.4, 0, Math.PI * 2); ctx.stroke();
  }
  // Time is shown relative to the newest sample, because "12.4 s ago" is what
  // an engineer is actually asking; t itself is an arbitrary window offset.
  var l1 = (hv != null && isFinite(hv)) ? (_chNum(hv, dec) + (unit ? ' ' + unit : ''))
                                        : 'no sample';
  var l2 = T ? ('-' + Math.max(0, xmax - xs[bi]).toFixed(1) + ' s') : ('#' + bi);
  ctx.font = _chFont(11, 700, true); var w1 = ctx.measureText(l1).width;
  ctx.font = _chFont(9.5, 400, true); var w2 = ctx.measureText(l2).width;
  var bw = Math.max(w1, w2) + 14, bh = 30;
  var bx = hpx + 10, by = padT + 6;
  if(bx + bw > w - padR - 2) bx = hpx - 10 - bw;      // flip before it clips
  if(bx < padL) bx = padL;
  if(by + bh > h - padB) by = Math.max(padT, h - padB - bh);
  ctx.fillStyle = '#182030'; _chRound(ctx, bx, by, bw, bh, 5); ctx.fill();
  ctx.strokeStyle = '#2C3949'; ctx.lineWidth = 1;
  _chRound(ctx, bx + 0.5, by + 0.5, bw - 1, bh - 1, 5); ctx.stroke();
  ctx.textAlign = 'left';
  ctx.font = _chFont(11, 700, true);
  ctx.fillStyle = (hv != null && isFinite(hv)) ? '#EAF0F7' : '#64748B';
  ctx.fillText(l1, bx + 7, by + 14);
  ctx.font = _chFont(9.5, 400, true); ctx.fillStyle = '#64748B';
  ctx.fillText(l2, bx + 7, by + 25);
}

// Centred text with manual tracking, for the one uppercase string that is not
// left-aligned. Kept separate so _chSpacedText stays a simple left-to-left run.
function _chCentred(ctx, text, cx, y, sp){
  var prev = ctx.textAlign;
  ctx.textAlign = 'left';
  _chSpacedText(ctx, text, cx - _chSpacedWidth(ctx, text, sp) / 2, y, sp);
  ctx.textAlign = prev;
}

// ── gfx_steer.js ──
// ═══ steering visualiser ══════════════════════════════════════════════════
// Left: the steering wheel as the driver sees it (column rotation, so it can
// exceed 360 deg), sitting inside a lock-to-lock travel gauge. Right: the road
// wheels from above, now with Ackermann geometry so the inner wheel visibly
// turns more than the outer one.
//
// Colour discipline in this panel:
//   magenta  = MEASURED   — travel fill, needle, road wheels, rim index
//   teal     = COMMANDED  — the bench-test target, drawn only while one is live
//   grn/amb/red = vehicle STATE only — homed, near lock, the red zone
// The button caps keep their real-world colours: they depict physical coloured
// plastic on the actual wheel, the same way cone orange depicts a real cone.
// They are never driven by telemetry, so they cannot be read as status.
//
// Nothing in here fetches, posts, mutates shared state or sets a timer. The
// only writes outside its own canvas are the seven sv_* readout spans.

// Front track width, centre-to-centre of the contact patches. ethon_drive
// publishes wheelbase_m but NOT the track, so this is the one number here that
// is not live telemetry — named so it can be corrected after a tape measure.
// The Ackermann split is directly proportional to it.
const _SV_TRACK_M = 1.30;

// Design tokens. canvas 2d cannot read CSS custom properties, so the palette
// is duplicated here; keep it in step with :root.
const _SV_C = {
  panel:'#131923', panel2:'#182030', rule:'#212B3A', rule2:'#2C3949',
  ink:'#EAF0F7', ink2:'#9AA9BC', ink3:'#64748B',
  ok:'#2ED47A', warn:'#F5A524', crit:'#FF4D4F',
  accent:'#4CE0D2', actual:'#FF5CA8', ground:'#090B0F', bg2:'#0E1218'
};
const _SV_SANS = 'system-ui,-apple-system,"Segoe UI",Roboto,"Helvetica Neue",Arial,sans-serif';
const _SV_MONO = 'ui-monospace,"SF Mono","Cascadia Mono",Menlo,Consolas,"DejaVu Sans Mono",monospace';

const _SV_TOP  = -Math.PI/2;      // canvas angle of straight-up
const _SV_SPAN = Math.PI*0.72;    // half-sweep of the travel gauge (matches
                                  // the old flat arc's 0.78pi..2.22pi span)

// ── small helpers, all prefixed so they cannot collide with the other
// ── canvas modules that share this script block ───────────────────────────

// Every field in drive_status can be null, missing, or (after a bad JSON round
// trip) a string. Anything that is not a finite number is "no reading" — a NaN
// that reaches ctx.rotate() silently blanks the whole wheel instead of saying
// so, which is worse than an honest dash.
function _svNum(v){ if(v==null) return null; const n=+v; return isFinite(n)?n:null; }

function _svRR(ctx,x,y,ww,hh,r){
  ctx.beginPath();
  ctx.moveTo(x+r,y);ctx.arcTo(x+ww,y,x+ww,y+hh,r);
  ctx.arcTo(x+ww,y+hh,x,y+hh,r);ctx.arcTo(x,y+hh,x,y,r);
  ctx.arcTo(x,y,x+ww,y,r);ctx.closePath();
}

// prefers-reduced-motion, asked fresh each draw so a mid-session OS change is
// picked up. Wrapped because matchMedia is absent in some embedded webviews.
function _svReduced(){
  try{ return !!(window.matchMedia && window.matchMedia('(prefers-reduced-motion: reduce)').matches); }
  catch(e){ return false; }
}

// Manual letter-spacing: ctx.letterSpacing only landed in Chrome 99 and the
// pit laptop is not guaranteed to be newer than 90.
function _svTracked(ctx,txt,x,y,sp,align){
  txt = String(txt==null?'':txt);
  const ws=[]; let total=0;
  for(let i=0;i<txt.length;i++){
    const cw=ctx.measureText(txt.charAt(i)).width;
    ws.push(cw); total+=cw+(i<txt.length-1?sp:0);
  }
  let px=x;
  if(align==='center') px=x-total/2; else if(align==='right') px=x-total;
  const old=ctx.textAlign; ctx.textAlign='left';
  for(let i=0;i<txt.length;i++){ ctx.fillText(txt.charAt(i),px,y); px+=ws[i]+sp; }
  ctx.textAlign=old;
}

// Take a screen-space offset and express it in the wheel's ROTATED frame, so a
// gradient's light source can stay pinned to the top-left of the screen while
// the wheel spins under it. local = Rot(-rot) * screen.
function _svUnrotate(ox,oy,rot){
  const c=Math.cos(rot), s=Math.sin(rot);
  return [ox*c+oy*s, -ox*s+oy*c];
}

// Ackermann split. delta is the bicycle-model (centreline) road-wheel angle in
// the SAME convention as everything else here: positive = LEFT. Both returned
// angles keep that sign; only their magnitudes differ.
//   R  = wheelbase / tan(delta)        radius to the centreline at the rear axle
//   inner: tan(a) = wheelbase / (|R| - track/2)      -> larger angle
//   outer: tan(a) = wheelbase / (|R| + track/2)      -> smaller angle
// Returns [innerDeg, outerDeg]. With no wheelbase there is no geometry to
// solve, so it degrades to parallel steering rather than inventing a split.
function _svAckermann(deltaDeg,wheelbase,track){
  const dd=_svNum(deltaDeg);
  if(dd==null) return [null,null];
  const wb=_svNum(wheelbase);
  const rad=dd*Math.PI/180;
  if(!wb||wb<=0||Math.abs(rad)<0.0017) return [dd,dd];   // ~0.1 deg = straight
  const aR=Math.abs(wb/Math.tan(rad));
  const half=Math.abs(track)/2;
  // Guard the degenerate case where the turn centre falls inside the track:
  // this car cannot physically do it, but a bad wheelbase param could ask for
  // it and atan of a negative denominator would flip the inner wheel around.
  const din=Math.max(aR-half, wb*0.05);
  const sgn=rad<0?-1:1;
  return [sgn*Math.atan(wb/din)*180/Math.PI, sgn*Math.atan(wb/(aR+half))*180/Math.PI];
}

// ── the travel gauge ──────────────────────────────────────────────────────
// A ring behind the wheel showing where the column sits inside its MEASURED
// lock-to-lock, with a tick every 90 deg of column rotation and a red zone
// over the last 10% of travel each side (the same 0.9 threshold the readout
// calls "near lock").
//
// DIRECTION — deliberate change from the original flat arc, which filled
// CLOCKWISE for positive column (a0 + frac*span, and canvas angles grow
// clockwise). Positive column, after dsign, is physically LEFT, and both of
// the bench-verified elements on this canvas render it that way: the wheel
// icon rotates by -col*2pi (counter-clockwise) and the road wheels by
// -deg (pointing left). A needle that swung right while the wheel and the
// tyres both swung left is a mirrored reading on a safety panel, so the gauge
// now runs counter-clockwise for positive col: angle = TOP - frac*SPAN.
// No sign on any *quantity* changed — only this ring's screen mapping.
function _svGauge(ctx,cx,cy,gr,g){
  const tw=Math.max(5,gr*0.115);            // track width
  const a0=_SV_TOP-_SV_SPAN, a1=_SV_TOP+_SV_SPAN;
  const scaled=!!g.lim;                     // is there a scale to plot against?

  ctx.save();
  ctx.lineCap='butt';

  // track
  ctx.lineWidth=tw; ctx.strokeStyle=_SV_C.rule;
  if(!g.live) ctx.setLineDash([3,5]);       // unhomed / dead reads as provisional
  ctx.beginPath(); ctx.arc(cx,cy,gr,a0,a1); ctx.stroke();
  ctx.setLineDash([]);

  // red zone: the outer 10% of travel each side. Drawn dim rather than solid
  // crit so it reads as a zone, not an active alarm.
  if(scaled){
    ctx.save(); ctx.globalAlpha=0.30; ctx.strokeStyle=_SV_C.crit; ctx.lineWidth=tw;
    ctx.beginPath(); ctx.arc(cx,cy,gr,_SV_TOP-_SV_SPAN,_SV_TOP-_SV_SPAN*0.9); ctx.stroke();
    ctx.beginPath(); ctx.arc(cx,cy,gr,_SV_TOP+_SV_SPAN*0.9,_SV_TOP+_SV_SPAN); ctx.stroke();
    ctx.restore();
  }

  // Ticks every 90 deg of COLUMN rotation, majors on whole turns. Only drawn
  // when there is a measured lock to scale against — ticking an unknown range
  // would invent precision.
  // This car's usable travel is well under one turn (road-wheel lock / column
  // ratio ~= 0.4 rot = 144 deg), which at a flat 90 deg step would leave one
  // tick per side, so the step drops to 30 and the in-between marks are drawn
  // as hairlines. The 90s stay visually dominant. The step is chosen to keep
  // the tick count bounded rather than fixed, because steer_limit_col_rot is a
  // live parameter: a mis-set lock of 20 rotations must not either carpet the
  // ring with 240 ticks or (worse) tick only part of it and look truncated.
  if(scaled&&g.live){
    const rIn=gr+tw/2+2;
    const totalDeg=g.lim*360;
    const steps=[30,90,180,360,720,1440,3600];
    let step=steps[steps.length-1];
    for(let i=0;i<steps.length;i++){ if(totalDeg/steps[i]<=24){ step=steps[i]; break; } }
    const n=Math.min(24,Math.floor(totalDeg/step+1e-6));
    ctx.font='600 8px '+_SV_MONO; ctx.textBaseline='middle';
    for(let k=-n;k<=n;k++){
      if(k===0) continue;                  // centre is the fixed notch below
      const cd=k*step, f=cd/totalDeg;
      if(f<-1||f>1) continue;
      const is90=(Math.abs(cd)%90===0), isTurn=(Math.abs(cd)%360===0);
      const a=_SV_TOP-f*_SV_SPAN;
      const len=isTurn?8:(is90?6:3);
      ctx.strokeStyle=is90?_SV_C.rule2:_SV_C.rule;
      ctx.lineWidth=isTurn?1.8:(is90?1.4:1);
      ctx.beginPath();
      ctx.moveTo(cx+Math.cos(a)*rIn, cy+Math.sin(a)*rIn);
      ctx.lineTo(cx+Math.cos(a)*(rIn+len), cy+Math.sin(a)*(rIn+len));
      ctx.stroke();
      // Whole-turn labels only, and never right on top of the L/R end caps.
      if(isTurn&&gr>46&&Math.abs(f)<0.97){
        const lr=rIn+len+7;
        ctx.fillStyle=_SV_C.ink3; ctx.textAlign='center';
        ctx.fillText(String(Math.abs(cd)/360),cx+Math.cos(a)*lr,cy+Math.sin(a)*lr);
      }
    }
  }

  // fixed straight-ahead reference notch — neither commanded nor measured, so
  // it stays neutral grey.
  ctx.strokeStyle=_SV_C.ink3; ctx.lineWidth=1.5;
  ctx.beginPath();
  ctx.moveTo(cx,cy-gr-tw/2-2); ctx.lineTo(cx,cy-gr-tw/2-9); ctx.stroke();

  if(g.dead){ ctx.restore(); return; }

  const na=_SV_TOP-g.frac*_SV_SPAN;

  // Near-lock halo, UNDER the travel fill so it spreads around it instead of
  // tinting the measured magenta amber — the two colours mean different things
  // and must not blend into each other.
  // The panel is redrawn by the dashboard's 600 ms poll and this module is
  // forbidden from owning a timer, so the "pulse" is sampled at that rate:
  // hence a deliberately slow ~2.4 s period and a shallow alpha swing, which
  // breathes instead of strobing. Frozen under prefers-reduced-motion.
  if(g.near&&g.live){
    const ph=_svReduced()?0.5:(0.34+0.22*(0.5+0.5*Math.sin(Date.now()/382)));
    ctx.save();
    ctx.globalAlpha=ph; ctx.strokeStyle=_SV_C.warn; ctx.lineWidth=tw+6;
    ctx.beginPath();
    ctx.arc(cx,cy,gr,Math.min(_SV_TOP,na),Math.max(_SV_TOP,na));
    ctx.stroke(); ctx.restore();
  }

  // travel fill, centre -> needle. MEASURED, so magenta.
  if(scaled&&g.live){
    ctx.save();
    ctx.globalAlpha=0.75; ctx.strokeStyle=_SV_C.actual; ctx.lineWidth=Math.max(2,tw-4);
    ctx.beginPath(); ctx.arc(cx,cy,gr,Math.min(_SV_TOP,na),Math.max(_SV_TOP,na)); ctx.stroke();
    ctx.restore();
  }

  // commanded target from a live bench test — teal, and only ever a thin
  // outline so it can never be mistaken for the measured position.
  if(g.cmdFrac!=null){
    const ca=_SV_TOP-g.cmdFrac*_SV_SPAN, cr=gr;
    ctx.save();
    ctx.translate(cx+Math.cos(ca)*cr, cy+Math.sin(ca)*cr);
    ctx.rotate(ca+Math.PI/2);
    ctx.strokeStyle=_SV_C.accent; ctx.lineWidth=1.6;
    ctx.beginPath(); ctx.moveTo(0,-tw/2-3); ctx.lineTo(0,tw/2+3); ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(-3.5,-tw/2-3); ctx.lineTo(3.5,-tw/2-3); ctx.lineTo(0,-tw/2+2);
    ctx.closePath(); ctx.stroke();
    ctx.restore();
  }

  // needle. Filled magenta when the column is homed (a real position on a real
  // scale); hollow grey when it is not, so an unhomed column reads as inert
  // even in a screenshot with no colour.
  ctx.save();
  ctx.translate(cx+Math.cos(na)*gr, cy+Math.sin(na)*gr);
  ctx.rotate(na+Math.PI/2);
  ctx.beginPath();
  ctx.moveTo(0,-tw/2-7); ctx.lineTo(4.5,-tw/2-1); ctx.lineTo(0,tw/2+3);
  ctx.lineTo(-4.5,-tw/2-1); ctx.closePath();
  if(g.live){ ctx.fillStyle=g.near?_SV_C.warn:_SV_C.actual; ctx.fill(); }
  else { ctx.strokeStyle=_SV_C.ink3; ctx.lineWidth=1.2; ctx.setLineDash([2,2]); ctx.stroke(); }
  ctx.restore();

  // end labels: which way is which. Cheap, and it kills the "is this mirrored?"
  // question that costs an engineer ten minutes at the bench.
  ctx.font='600 9px '+_SV_SANS; ctx.fillStyle=_SV_C.ink3; ctx.textBaseline='middle';
  const lr=gr+tw/2+17;
  _svTracked(ctx,'L',cx+Math.cos(a0)*lr,cy+Math.sin(a0)*lr,0.8,'center');
  _svTracked(ctx,'R',cx+Math.cos(a1)*lr,cy+Math.sin(a1)*lr,0.8,'center');
  ctx.restore();
}

// ── the wheel itself ──────────────────────────────────────────────────────
// Same body proportions as the original — this is a portrait of the car's real
// formula wheel (angled grips, upper button pods, LED bar, centre screen) and
// the shapes are what make it recognisable. What changed is the finish.
function _svWheel(ctx,cx,cy,R,g){
  // Verified 2026-08-16 against a live bench-test command: the wheel
  // diagram below (right side) is ground truth and matches physical
  // wheel direction. This icon needs the opposite sign to agree with it --
  // ctx.rotate() reads canvas-clockwise as "steering right" to a viewer,
  // but col>0 (after dsign) is physically LEFT on this car.
  const rot=(g.col==null)?0:-g.col*Math.PI*2;  // column rotations -> radians

  // state underglow behind the whole wheel, so homed/near-lock reads from
  // across the garage without staring at the needle
  ctx.save();
  const ug=ctx.createRadialGradient(cx,cy,Math.max(0.01,R*0.55),cx,cy,R*1.72);
  ug.addColorStop(0,g.dead?'rgba(100,116,139,0.10)'
    :(g.near?'rgba(245,165,36,0.20)':(g.live?'rgba(46,212,122,0.14)':'rgba(100,116,139,0.10)')));
  ug.addColorStop(1,'rgba(0,0,0,0)');
  ctx.fillStyle=ug; ctx.beginPath(); ctx.arc(cx,cy,R*1.72,0,Math.PI*2); ctx.fill();
  ctx.restore();

  ctx.save(); ctx.translate(cx,cy); ctx.rotate(rot);

  // Body fill and specular are built in the wheel's LOCAL frame but aimed
  // using screen-space offsets pushed back through the rotation, so the
  // highlight stays where the room light is while the wheel turns under it.
  const lp=_svUnrotate(-R*0.38,-R*0.62,rot);
  const body=ctx.createRadialGradient(lp[0],lp[1],Math.max(0.01,R*0.06),0,0,R*1.65);
  if(g.inert){
    // Unhomed or no data: flat, unlit, hollow. The absence of the gradient is
    // the tell — this must look obviously dead in a greyscale screenshot too.
    body.addColorStop(0,'#161C27'); body.addColorStop(1,'#111621');
  }else{
    body.addColorStop(0,'#2C3646'); body.addColorStop(0.55,'#1B2432'); body.addColorStop(1,'#0D1219');
  }
  const s0=_svUnrotate(0,-R*1.05,rot), s1=_svUnrotate(0,R*0.85,rot);
  const spec=ctx.createLinearGradient(s0[0],s0[1],s1[0],s1[1]);
  spec.addColorStop(0,'rgba(234,240,247,0.30)');
  spec.addColorStop(0.42,'rgba(234,240,247,0.06)');
  spec.addColorStop(1,'rgba(9,11,15,0.55)');

  const piece=(setup,path)=>{
    ctx.save();
    if(setup) setup();
    path();
    ctx.fillStyle=body; ctx.fill();
    ctx.lineWidth=1.2;
    if(g.inert){ ctx.setLineDash([4,3]); ctx.strokeStyle=_SV_C.rule2; }
    else ctx.strokeStyle=spec;
    ctx.stroke();
    ctx.restore();
  };

  // grips (angled, lower left/right)
  for(const sx of [-1,1]){
    piece(()=>{ ctx.translate(sx*R*0.92,R*0.34); ctx.rotate(sx*0.20); },
          ()=>{ _svRR(ctx,-R*0.20,-R*0.30,R*0.40,R*1.02,R*0.16); });
  }
  // upper wings (button pods) + top bar, drawn as one open-top body
  for(const sx of [-1,1]){
    piece(()=>{ ctx.scale(sx,1); },
          ()=>{ _svRR(ctx,R*0.30,-R*0.80,R*0.98,R*0.92,R*0.14); });
  }
  piece(null,()=>{ _svRR(ctx,-R*0.62,-R*0.80,R*1.24,R*0.30,R*0.08); });  // LED bar
  piece(null,()=>{ _svRR(ctx,-R*0.66,-R*0.50,R*1.32,R*0.86,R*0.10); });  // centre
  piece(null,()=>{ _svRR(ctx,-R*0.52,R*0.36,R*1.04,R*0.40,R*0.12); });   // lower

  // rev / shift LEDs across the top bar (same map as the Nextion HMI):
  // 15 LEDs over 0-50 km/h, green to nine, amber to twelve, red above.
  const sf=Math.min(1,g.kmh/50);
  for(let i=0;i<15;i++){
    const on=g.live&&(i/15)<sf;
    const lx=-R*0.55+i*(R*1.10/14), ly=-R*0.65, lrad=Math.max(1,R*0.045);
    const c=i<9?_SV_C.ok:(i<12?_SV_C.warn:_SV_C.crit);
    ctx.save();
    if(on){ ctx.shadowColor=c; ctx.shadowBlur=R*0.16; }
    ctx.fillStyle=on?c:_SV_C.ground;
    ctx.beginPath(); ctx.arc(lx,ly,lrad,0,Math.PI*2); ctx.fill();
    ctx.restore();
    if(!on){                                   // unlit lens still has a bezel
      ctx.strokeStyle='rgba(44,57,73,0.9)'; ctx.lineWidth=0.8;
      ctx.beginPath(); ctx.arc(lx,ly,lrad,0,Math.PI*2); ctx.stroke();
    }
  }

  // centre screen — recessed bezel, near-black glass, mono digits
  ctx.save();
  _svRR(ctx,-R*0.48,-R*0.42,R*0.96,R*0.68,R*0.06);
  ctx.fillStyle=_SV_C.ground; ctx.fill();
  ctx.strokeStyle=g.inert?_SV_C.rule:'rgba(234,240,247,0.10)'; ctx.lineWidth=1; ctx.stroke();
  ctx.clip();
  if(!g.inert){                                 // faint glass sheen
    const sg=ctx.createLinearGradient(0,-R*0.42,0,R*0.26);
    sg.addColorStop(0,'rgba(234,240,247,0.07)'); sg.addColorStop(1,'rgba(234,240,247,0)');
    ctx.fillStyle=sg; ctx.fillRect(-R*0.48,-R*0.42,R*0.96,R*0.68);
  }
  ctx.textAlign='center'; ctx.textBaseline='middle';
  ctx.fillStyle=g.inert?_SV_C.ink3:_SV_C.ink;
  ctx.font='700 '+Math.round(R*0.36)+'px '+_SV_MONO;
  ctx.fillText(g.dead?'--':String(Math.round(g.kmh)),0,-R*0.13);
  ctx.font='600 '+Math.max(7,Math.round(R*0.15))+'px '+_SV_SANS;
  ctx.fillStyle=_SV_C.ink3;
  _svTracked(ctx,'KM/H',0,R*0.06,1.1,'center');
  // second line: the road-wheel angle, MEASURED, so magenta
  ctx.font='650 '+Math.max(8,Math.round(R*0.17))+'px '+_SV_MONO;
  ctx.fillStyle=(g.deg==null||g.inert)?_SV_C.ink3:_SV_C.actual;
  ctx.fillText(g.deg==null?'--.-°':((g.deg>0?'+':'')+g.deg.toFixed(1)+'°'),0,R*0.20);
  ctx.restore();

  // button pods: a few coloured buttons per side, like the real wheel.
  // Positions unchanged from the original photo-match.
  const pods=[[-0.95,-0.62,_SV_C.warn],[-0.62,-0.60,_SV_C.crit],
              [-0.95,-0.30,_SV_C.crit],[-0.60,-0.26,_SV_C.ink2],
              [ 0.95,-0.62,'#3B82F6'],[ 0.62,-0.60,_SV_C.ok],
              [ 0.95,-0.30,_SV_C.ok],[ 0.60,-0.26,_SV_C.ink2]];
  for(const p of pods){
    const px=p[0]*R, py=p[1]*R, br=Math.max(1.5,R*0.078);
    ctx.save();
    ctx.beginPath(); ctx.arc(px,py,br+1.2,0,Math.PI*2);
    ctx.fillStyle='#0B0F16'; ctx.fill();                    // bezel well
    const cap=ctx.createRadialGradient(px-br*0.35,py-br*0.45,Math.max(0.01,br*0.1),px,py,br);
    if(g.inert){ cap.addColorStop(0,'#252E3C'); cap.addColorStop(1,'#161C27'); }
    else { cap.addColorStop(0,'rgba(255,255,255,0.35)'); cap.addColorStop(0.35,p[2]); cap.addColorStop(1,'#0E1218'); }
    ctx.beginPath(); ctx.arc(px,py,br,0,Math.PI*2);
    ctx.fillStyle=cap; ctx.fill();
    if(!g.inert){
      ctx.globalAlpha=0.55; ctx.fillStyle=p[2];
      ctx.beginPath(); ctx.arc(px,py,br*0.55,0,Math.PI*2); ctx.fill();
    }
    ctx.restore();
  }

  // centre hub
  ctx.save();
  const hub=ctx.createRadialGradient(-R*0.03,R*0.52,Math.max(0.01,R*0.01),0,R*0.56,Math.max(0.02,R*0.12));
  hub.addColorStop(0,'#39445A'); hub.addColorStop(1,'#161C27');
  ctx.fillStyle=hub; ctx.beginPath(); ctx.arc(0,R*0.56,Math.max(1,R*0.10),0,Math.PI*2); ctx.fill();
  ctx.strokeStyle=g.inert?_SV_C.rule2:'rgba(234,240,247,0.12)'; ctx.lineWidth=1; ctx.stroke();
  ctx.restore();

  // straight-ahead marker on the rim. This rotates WITH the wheel, so it is a
  // measured indication of where the column sits -> magenta when it means
  // something, hollow grey when the column has no reference.
  ctx.save();
  const mr=Math.max(1.5,R*0.055);
  if(g.live){
    ctx.shadowColor=_SV_C.actual; ctx.shadowBlur=R*0.18;
    ctx.fillStyle=_SV_C.actual;
    ctx.beginPath(); ctx.arc(0,-R*0.90,mr,0,Math.PI*2); ctx.fill();
  }else{
    ctx.strokeStyle=_SV_C.ink3; ctx.lineWidth=1.2; ctx.setLineDash([2,2]);
    ctx.beginPath(); ctx.arc(0,-R*0.90,mr,0,Math.PI*2); ctx.stroke();
  }
  ctx.restore();

  ctx.textBaseline='alphabetic';
  ctx.restore();
}

// ── road wheels from above ────────────────────────────────────────────────
// Drawn to the car's real proportions (wheelbase_m x _SV_TRACK_M) with true
// Ackermann angles. The split is small at this car's lock (~2 deg at 12), so
// the numbers under the sketch are the readable channel and the drawing is the
// sanity check. Angles are NEVER exaggerated for legibility: an engineer must
// be able to trust that what is drawn is what the geometry says.
function _svTopView(ctx,bx,by,availW,availH,g){
  // wheelbase is only used for the sketch's aspect ratio here; when it is
  // missing the Ackermann helper degrades to parallel steering, so nothing is
  // ever inferred from this fallback.
  const wbM=g.wb||1.60;
  const sc=Math.max(4,Math.min(availW/_SV_TRACK_M, availH/wbM));
  const halfT=_SV_TRACK_M*sc/2, wbPx=wbM*sc;
  const yF=by-wbPx/2, yR=by+wbPx/2;
  const wl=Math.max(8,wbPx*0.24), ww=Math.max(4,wbPx*0.055);

  ctx.save();
  ctx.lineCap='butt';

  // chassis: centreline plus both axles
  ctx.strokeStyle=_SV_C.rule; ctx.lineWidth=1;
  ctx.beginPath(); ctx.moveTo(bx,yF); ctx.lineTo(bx,yR); ctx.stroke();
  ctx.strokeStyle=_SV_C.rule2; ctx.lineWidth=2;
  ctx.beginPath(); ctx.moveTo(bx-halfT,yF); ctx.lineTo(bx+halfT,yF); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(bx-halfT,yR); ctx.lineTo(bx+halfT,yR); ctx.stroke();

  // rear wheels, fixed
  for(const sx of [-1,1]){
    _svRR(ctx,bx+sx*halfT-ww/2,yR-wl/2,ww,wl,ww*0.35);
    ctx.fillStyle=_SV_C.rule2; ctx.fill();
  }

  const ack=_svAckermann(g.deg,g.wb,_SV_TRACK_M);
  const cmdAck=(g.cmdDeg==null)?[null,null]:_svAckermann(g.cmdDeg,g.wb,_SV_TRACK_M);

  // front wheels. sx=-1 is screen-left, which in this top view (front up) is
  // vehicle LEFT — the same frame drawBird uses (+y = left = screen-left).
  // A wheel is the INNER one when the turn goes its way: deg>0 is LEFT, so the
  // left wheel (sx<0) is inner exactly when deg*sx < 0.
  for(const sx of [-1,1]){
    const px=bx+sx*halfT, py=yF;
    const inner=(g.deg!=null)&&(g.deg*sx<0);
    const dw=(g.deg==null)?null:(inner?ack[0]:ack[1]);
    const cw=(g.cmdDeg==null)?null:((g.cmdDeg*sx<0)?cmdAck[0]:cmdAck[1]);

    // commanded ghost first, underneath: teal, dashed outline only
    if(cw!=null){
      ctx.save();
      ctx.translate(px,py); ctx.rotate(-cw*Math.PI/180);   // same convention as below
      _svRR(ctx,-ww/2,-wl/2,ww,wl,ww*0.35);
      ctx.strokeStyle=_SV_C.accent; ctx.lineWidth=1.2; ctx.setLineDash([3,2]); ctx.stroke();
      ctx.restore();
    }

    const wr=(dw==null)?0:(-dw*Math.PI/180);  // +deg = right = clockwise
    ctx.save();
    ctx.translate(px,py); ctx.rotate(wr);
    _svRR(ctx,-ww/2,-wl/2,ww,wl,ww*0.35);
    if(g.live){
      const tg=ctx.createLinearGradient(-ww/2,0,ww/2,0);
      tg.addColorStop(0,'#B33B74'); tg.addColorStop(0.45,_SV_C.actual); tg.addColorStop(1,'#7E2A52');
      ctx.fillStyle=tg; ctx.fill();
    }else{
      // unhomed or dead: hollow outline, so the tyre reads as "position not
      // trustworthy" without relying on the colour alone
      ctx.strokeStyle=_SV_C.ink3; ctx.lineWidth=1.2; ctx.setLineDash([3,2]); ctx.stroke();
    }
    ctx.restore();
  }

  ctx.font='600 9px '+_SV_SANS; ctx.fillStyle=_SV_C.ink3;
  ctx.textAlign='center'; ctx.textBaseline='alphabetic';
  if(yF-wl/2-8>8) _svTracked(ctx,'FRONT',bx,yF-wl/2-8,1.4,'center');

  // The Ackermann split, spelled out. At this car's lock the inner/outer
  // difference is only about 2 deg, which is real but nearly invisible at this
  // scale — so the numbers carry it and the sketch is the sanity check. Do not
  // be tempted to scale the drawn angles up to "show" the split.
  // Captions are measured before they are drawn, then shortened or dropped:
  // this canvas is fluid and a caption that runs off the panel edge looks like
  // a broken readout rather than a tight layout.
  // Suppressed entirely with no data: they would read as a row of em-dashes
  // sitting under the "no steering data" strip, which is just noise.
  const room=Math.max(0,Math.min(bx-g.leftW,g.w-bx)*2-6);
  const yTxt=yR+wl/2+16;
  if(g.dead){ ctx.restore(); return; }
  if(yTxt<g.h-6){
    ctx.font='650 10px '+_SV_MONO;
    ctx.fillStyle=(ack[0]==null||!g.live)?_SV_C.ink3:_SV_C.actual;
    let txt=(ack[0]==null)?'in —   out —'
      :('in '+Math.abs(ack[0]).toFixed(1)+'°  out '+Math.abs(ack[1]).toFixed(1)+'°');
    if(ctx.measureText(txt).width>room&&ack[0]!=null)
      txt=Math.abs(ack[0]).toFixed(1)+'/'+Math.abs(ack[1]).toFixed(1)+'°';
    if(ctx.measureText(txt).width<=room) ctx.fillText(txt,bx,yTxt);
  }
  if(yTxt+12<g.h-4){
    ctx.font='600 8px '+_SV_SANS; ctx.fillStyle=_SV_C.ink3;
    const cap='TRACK '+_SV_TRACK_M.toFixed(2)+'M · WB '+(g.wb?g.wb.toFixed(2):'—')+'M';
    if(ctx.measureText(cap).width+cap.length*0.6<=room)
      _svTracked(ctx,cap,bx,yTxt+12,0.6,'center');
  }
  ctx.restore();
}

// ── main entry point ──────────────────────────────────────────────────────
function drawSteer(d){
  d=d||{};

  // Rendering sign. Reported position is positive in the direction the motor
  // calls forward, which on this car is physically COUNTER-clockwise while
  // canvas rotate() is positive clockwise — so the raw value renders mirrored.
  // Follow steer_inverted instead of hardcoding, so flipping that config keeps
  // the picture matching the real wheel.
  const dsign=(d.steer_inverted===true)?1:-1;
  // The !isFinite guards are the only addition to these four lines: a
  // non-numeric reading is treated as "no data" instead of being multiplied
  // through into a NaN that would silently blank the canvas. No sign changed.
  const col=(d.steer_col_rot==null||!isFinite(+d.steer_col_rot))?null:dsign*(+d.steer_col_rot);
  const deg=(d.road_wheel_deg==null||!isFinite(+d.road_wheel_deg))?null:dsign*(+d.road_wheel_deg);
  const lim=(d.steer_limit_col_rot==null||!isFinite(+d.steer_limit_col_rot))?null:Math.abs(+d.steer_limit_col_rot);
  const homed=!!d.steer_homed, dead=(col==null);
  const frac=(col!=null&&lim)?Math.max(-1,Math.min(1,col/lim)):0;
  const near=Math.abs(frac)>0.9;
  const live=homed&&!dead;

  // Commanded overlay: only while the steering bench test's 0.5 s watchdog
  // says a target is actually live. ethon_drive maps steer_test_deg through
  // exactly the inverse of road_wheel_deg (col_rot = deg * col_ratio / 360),
  // so it shares this convention and takes the same dsign.
  const rwMaxRaw=_svNum(d.road_wheel_max_deg);
  const rwMax=(rwMaxRaw==null)?null:Math.abs(rwMaxRaw);
  const tRaw=_svNum(d.steer_test_deg);
  const cmdDeg=(d.steer_test_active===true&&tRaw!=null)?dsign*tRaw:null;
  const cmdFrac=(cmdDeg!=null&&rwMax)?Math.max(-1,Math.min(1,cmdDeg/rwMax)):null;

  const wms=_svNum(d.wheel_speed_ms);
  const kmh=Math.abs(wms==null?0:wms)*3.6;

  const g={
    col:col, deg:deg, lim:lim, frac:frac, near:near, live:live, dead:dead,
    homed:homed, inert:(!live), kmh:kmh, cmdDeg:cmdDeg, cmdFrac:cmdFrac,
    wb:_svNum(d.wheelbase_m)
  };

  // Drawing and the readouts are isolated from each other: a canvas that
  // cannot paint must not also cost us the numbers, and neither may throw out
  // of drawSteer into tick() and take the rest of the page's poll with it.
  try{ _svPaint(g); }catch(e){}

  try{
    const S=(id,v)=>{const e=document.getElementById(id);if(e)e.textContent=v;};
    S('sv_state', dead?'no data':(d.steering||(homed?'homed':'—')));
    S('sv_deg', deg==null?'—':(deg>0?'+':'')+deg.toFixed(1)+'°');
    S('sv_col', col==null?'—':(col>0?'+':'')+col.toFixed(3)+' rot');
    S('sv_lim', lim?('±'+lim.toFixed(3)+' rot ('
      +(lim*360).toFixed(0)+'° col)'):'—');
    S('sv_pct', lim&&col!=null?(Math.abs(frac)*100).toFixed(0)+'%'
      +(near?'  ⚠ near lock':''):'—');
    S('sv_mode', d.steer_mode
      ? (d.steer_mode==='foc' ? 'FOC (Pro licence)' : 'duty (no licence)') : '—');
    const m=(d.motors||{}).steer||{};
    S('sv_mot', m.faults&&m.faults.length
      ? (m.faults.join(',')) : (m.temp_c!=null?(m.temp_c+'°C'):'—'));
  }catch(e){}
}

function _svPaint(g){
  const s=setupCanvas('steerviz'); if(!s) return;
  const ctx=s.ctx, w=s.w, h=s.h;
  if(!(w>0)||!(h>0)) return;
  g.w=w; g.h=h;                       // so the sub-draws can clip their own
                                      // captions against the canvas edge
  ctx.clearRect(0,0,w,h);
  ctx.fillStyle=_SV_C.panel; ctx.fillRect(0,0,w,h);

  // a hair of depth on the panel itself so the gauge does not float
  const bgg=ctx.createLinearGradient(0,0,0,h);
  bgg.addColorStop(0,'rgba(24,32,48,0.55)'); bgg.addColorStop(1,'rgba(9,11,15,0.35)');
  ctx.fillStyle=bgg; ctx.fillRect(0,0,w,h);

  ctx.textBaseline='alphabetic';

  // Layout: wheel + gauge on the left, top view on the right. Sized off the
  // canvas rather than the 400x210 it happens to be today, because the
  // dashboard's panels are fluid.
  //
  // The 1.70 ring factor is not cosmetic: the outer bottom corner of a grip
  // sits at ~1.46 R from the wheel centre once it is rotated, so anything
  // tighter has the wheel chewing through the gauge track at part-lock.
  const leftW=w*0.575;
  const R=Math.max(14,Math.min(leftW*0.215,(h-52)*0.29));
  const cx=leftW*0.50, cy=h*0.50;
  const gr=R*1.70;
  g.leftW=leftW;

  _svGauge(ctx,cx,cy,gr,g);
  _svWheel(ctx,cx,cy,R,g);

  // section caption under the gauge (suppressed when the dead-state strip
  // below would paint straight over it)
  if(!g.dead){
    ctx.font='600 9px '+_SV_SANS; ctx.fillStyle=_SV_C.ink3; ctx.textBaseline='alphabetic';
    _svTracked(ctx,g.lim?'LOCK-TO-LOCK TRAVEL':'NO MEASURED LOCK',cx,h-8,1.4,'center');
  }

  const bx=leftW+(w-leftW)*0.48, by=h*0.47;
  const availW=Math.max(24,Math.min((w-leftW)*0.78,(w-bx-10)*1.9));
  const availH=Math.max(24,h*0.50);
  _svTopView(ctx,bx,by,availW,availH,g);

  // Status strip. Homing state gets words, not just a tint: "unhomed" is the
  // difference between a real angle and a number relative to wherever the
  // column happened to sit at boot, and that must never be a colour-only cue.
  ctx.textBaseline='alphabetic';
  if(g.dead){
    ctx.save();
    ctx.fillStyle='rgba(9,11,15,0.72)'; ctx.fillRect(0,h-24,w,24);
    ctx.strokeStyle=_SV_C.rule; ctx.lineWidth=1;
    ctx.beginPath(); ctx.moveTo(0,h-24.5); ctx.lineTo(w,h-24.5); ctx.stroke();
    ctx.font='600 11px '+_SV_SANS; ctx.fillStyle=_SV_C.ink3;
    _svTracked(ctx,'NO STEERING DATA',w/2,h-8,1.6,'center');
    ctx.restore();
  }else if(!g.homed){
    ctx.save();
    ctx.font='600 9px '+_SV_SANS; ctx.fillStyle=_SV_C.ink3;
    _svTracked(ctx,'UNHOMED · SCALE UNVERIFIED',w/2,14,1.4,'center');
    ctx.restore();
  }
  if(g.cmdDeg!=null){
    ctx.save();
    ctx.font='600 9px '+_SV_SANS; ctx.fillStyle=_SV_C.accent;
    ctx.textAlign='right';
    _svTracked(ctx,'BENCH TARGET '+((g.cmdDeg>0?'+':'')+g.cmdDeg.toFixed(1))+'°',w-8,14,1.2,'right');
    ctx.restore();
  }
  ctx.textAlign='left'; ctx.textBaseline='alphabetic';
}

// ── gfx_motors.js ──
// ── motor bank ────────────────────────────────────────────────────────────
// The four Krakens (drive_0/1/2 + steer) currently reach the operator as a
// single line of text buried in the bench-test readout, so a fault bit or a
// motor climbing through its derate window is effectively invisible until
// something stops working. This strip gives every motor in
// /ethon/drive_status one row: temperature against a labelled scale, supply
// current against the live thermal limit, rotor velocity, and faults.
//
// Pure rendering. No fetch, no timers, no globals, no DOM outside the canvas
// it is handed — the caller owns the polling. Nothing in here can command the
// car, which is the point: this panel is read during a bench test while the
// motors are live.
//
// Colour discipline (see the dashboard token table):
//   green/amber/red  = vehicle STATE  -> temperature, faults, over-limit
//   magenta          = actually MEASURED -> supply current bars
//   teal             = commanded/planned -> deliberately unused here, because
//                      nothing on this strip is a command. If you ever add a
//                      commanded-current overlay, teal is the colour for it.
function drawMotors(id, drive){
  var s = null;
  try{ s = setupCanvas(id); }catch(e){ return; }
  if(!s) return;
  try{
    _motPaint(s.ctx, s.w, s.h, drive);
  }catch(e){
    // A malformed payload must never take the page down with it — every other
    // draw call on this dashboard runs after this one. Fail to a visible
    // marker rather than a silently stale canvas, so nobody reads an old
    // temperature as a current one.
    try{
      var T = _motTokens();
      s.ctx.fillStyle = T.panel; s.ctx.fillRect(0, 0, s.w, s.h);
      s.ctx.fillStyle = T.crit;
      s.ctx.font = '600 11px ' + T.sans;
      s.ctx.textAlign = 'center'; s.ctx.textBaseline = 'middle';
      s.ctx.fillText('MOTOR PANEL RENDER ERROR', s.w / 2, s.h / 2);
    }catch(e2){ /* canvas itself is gone; nothing sane left to do */ }
  }
}

// Design tokens, hardcoded because canvas2d cannot read CSS custom properties.
// Returned fresh per call rather than held in a top-level const: this file is
// pasted into a shared <script> alongside five sibling modules, and a
// duplicated top-level const is a SyntaxError that kills every one of them.
function _motTokens(){
  return {
    panel:  '#131923',
    panel2: '#182030',
    rule:   '#212B3A',
    rule2:  '#2C3949',
    ink:    '#EAF0F7',
    ink2:   '#9AA9BC',
    ink3:   '#64748B',
    ok:     '#2ED47A',
    warn:   '#F5A524',
    crit:   '#FF4D4F',
    actual: '#FF5CA8',
    sans: 'system-ui,-apple-system,"Segoe UI",Roboto,"Helvetica Neue",Arial,sans-serif',
    mono: 'ui-monospace,"SF Mono","Cascadia Mono",Menlo,Consolas,"DejaVu Sans Mono",monospace'
  };
}

// Temperature anchors. These are DISPLAY thresholds, not the control law:
// ethon_drive derates torque current between derate_lo_c 55 C and derate_hi_c
// 70 C (vehicle.yaml), and that derate is already baked into the
// thermal_limit_a we scale the current bar against. 40 C is "warm but
// nominal", 70 C is where the motor has given up its last amp of headroom,
// 90 C is where you stop the test. Full scale is 0..100 C so the bar position
// is absolute and comparable between rows and between sessions.
function _motTempMax(){ return 100; }

// Anything can be null: the car boots with no CAN, no homing and no fix.
// Coerce to a finite number or give up honestly.
function _motF(v){
  if(v === null || v === undefined || v === '') return null;
  var n = +v;
  return isFinite(n) ? n : null;
}

function _motNum(v, dec, suffix){
  if(v === null || v === undefined) return '—';
  return v.toFixed(dec) + (suffix || '');
}

function _motMix(a, b, t){
  t = Math.max(0, Math.min(1, t));
  var pa = parseInt(a.slice(1), 16), pb = parseInt(b.slice(1), 16);
  var r = Math.round(((pa >> 16) & 255) + (((pb >> 16) & 255) - ((pa >> 16) & 255)) * t);
  var g = Math.round(((pa >> 8) & 255) + (((pb >> 8) & 255) - ((pa >> 8) & 255)) * t);
  var bl = Math.round((pa & 255) + ((pb & 255) - (pa & 255)) * t);
  return 'rgb(' + r + ',' + g + ',' + bl + ')';
}

// Colour for a temperature, on the same green -> amber -> red ramp the bar
// gradient uses, so the numeric label and the bar tip always agree.
function _motTempColor(c, T){
  if(c === null) return T.ink3;
  if(c <= 40) return T.ok;
  if(c <= 70) return _motMix(T.ok, T.warn, (c - 40) / 30);
  if(c <= 90) return _motMix(T.warn, T.crit, (c - 70) / 20);
  return T.crit;
}

function _motRR(ctx, x, y, w, h, r){
  r = Math.min(r, w / 2, h / 2);
  ctx.beginPath();
  ctx.moveTo(x + r, y);
  ctx.arcTo(x + w, y, x + w, y + h, r);
  ctx.arcTo(x + w, y + h, x, y + h, r);
  ctx.arcTo(x, y + h, x, y, r);
  ctx.arcTo(x, y, x + w, y, r);
  ctx.closePath();
}

// Stable row order. JSON object order already matches ethon_drive's
// _motor_table (drive master, followers, then steer), but a reordered payload
// would make rows swap places between polls and an operator would read the
// wrong row. Sort explicitly: drive_* by device id, steer last, anything new
// alphabetically in between — steer stays at the bottom because it is the odd
// one out (different duty cycle, different current envelope entirely).
function _motSortNames(names){
  return names.slice().sort(function(a, b){
    var ad = /^drive_(\d+)$/.exec(a), bd = /^drive_(\d+)$/.exec(b);
    if(ad && bd) return (+ad[1]) - (+bd[1]);
    if(ad) return -1;
    if(bd) return 1;
    if(a === 'steer') return 1;
    if(b === 'steer') return -1;
    return a < b ? -1 : (a > b ? 1 : 0);
  });
}

// Fault presentation. read_faults() in ethon_drive returns [] for a healthy
// motor, a list of phoenix6 fault names, "raw:0x..." when only the bitfield is
// readable, or exactly ["unavailable"] when the device does not answer at all.
// That last one is NOT a motor fault — /api/state's arm-block and the selftest
// both read it as "this motor is not on the CAN bus". Painting it crit red
// sends an engineer hunting a blown Kraken when the real answer is a dead bus
// or an unpowered motor, so it gets its own amber OFFLINE state.
function _motFaultChip(faults){
  if(!faults) return null;                                    // silence is good
  // Tolerate a bare string: iterating one would spell it out character by
  // character, which is how you end up with a chip reading "u · n · a · v".
  var arr = Array.isArray(faults) ? faults : [faults];
  if(!arr.length) return null;
  var list = [];
  for(var i = 0; i < arr.length; i++){
    var f = arr[i];
    if(f === null || f === undefined) continue;
    list.push(String(f));
  }
  if(!list.length) return null;
  if(list.length === 1 && list[0] === 'unavailable'){
    return { text: 'OFFLINE — no CAN reply', offline: true };
  }
  return { text: list.join(' · '), offline: false };
}

// Honour the OS motion preference for the fault-chip pulse. If the query
// itself throws (very old WebView) assume the calmer branch.
function _motReducedMotion(){
  try{
    return !!(window.matchMedia
      && window.matchMedia('(prefers-reduced-motion: reduce)').matches);
  }catch(e){ return true; }
}

function _motPaint(ctx, w, h, drive){
  var T = _motTokens();
  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = T.panel; ctx.fillRect(0, 0, w, h);
  ctx.textBaseline = 'alphabetic';

  var d = drive || {};
  var motors = d.motors || {};
  var names = _motSortNames(Object.keys(motors));

  var padX = 10, padY = 6;
  var headH = (h >= 92) ? 22 : 0;          // drop the header before the rows
  // How many rows actually FIT at the minimum legible height. A row clipped by
  // the canvas edge shows a temperature bar with no number, or half a fault
  // chip, which is worse than admitting the panel is short — so overflow is
  // counted and stated instead of drawn.
  var ROW_MIN = 18;
  var overflow = 0;
  var fits = Math.max(1, Math.floor((h - padY * 2 - headH) / ROW_MIN));
  var cap = Math.min(8, fits);             // 8 is the design ceiling either way
  if(names.length > cap){
    // Re-fit with room reserved for the "+N more" footer.
    fits = Math.max(1, Math.floor((h - padY * 2 - headH - 12) / ROW_MIN));
    cap = Math.min(8, fits);
    overflow = names.length - cap;
    names = names.slice(0, cap);
  }

  if(!names.length){ _motEmpty(ctx, w, h, T, d); return; }

  // ── read the payload once, defensively ──
  var rows = [], i, hottest = -1, hottestT = null;
  for(i = 0; i < names.length; i++){
    var m = motors[names[i]];
    if(!m || typeof m !== 'object') m = {};
    var tc = _motF(m.temp_c);
    rows.push({
      name: names[i],
      temp: tc,
      amps: _motF(m.supply_a),
      vel:  _motF(m.vel_rps),
      chip: _motFaultChip(m.faults)
    });
    if(tc !== null && (hottestT === null || tc > hottestT)){ hottestT = tc; hottest = i; }
  }
  // "Hottest" is hottest among the rows actually drawn. With this car's four
  // motors nothing is ever hidden; if a future bank overflows the panel, grow
  // the canvas rather than trusting the highlight.
  // Highlighting the only motor that reports a temperature is noise, not
  // information — the highlight has to mean "this one, not those ones".
  var tempCount = 0;
  for(i = 0; i < rows.length; i++) if(rows[i].temp !== null) tempCount++;
  if(tempCount < 2) hottest = -1;

  // Current full scale. thermal_limit_a is AMPS, not degrees, and it is not a
  // constant: ethon_drive derates it from max_drive_a down to min_drive_a as
  // the hottest drive motor heats up, so this bar can grow while the current
  // is unchanged. The header prints the number for exactly that reason.
  var lim = _motF(d.thermal_limit_a);
  var limKnown = (lim !== null && lim > 0);
  if(!limKnown){
    // No published limit (drive node down / old payload). Derive a scale from
    // the data instead of inventing a constant, and say the scale is auto.
    var peak = 0;
    for(i = 0; i < rows.length; i++)
      if(rows[i].amps !== null) peak = Math.max(peak, Math.abs(rows[i].amps));
    lim = Math.max(10, Math.ceil(peak * 1.25));
  }

  // ── geometry ──
  var avail = h - padY * 2 - headH - (overflow ? 12 : 0);
  // rows.length was capped to what fits, so the lower clamp is a floor for the
  // degenerate tiny-canvas case only and cannot push a row off the bottom.
  var rowH = Math.max(ROW_MIN, Math.min(46, avail / rows.length));
  var fsName = Math.max(9, Math.min(12, rowH * 0.44));
  var fsSmall = Math.max(8, Math.min(10.5, rowH * 0.37));

  // Widest motor name drives the label column so drive_10 does not clip.
  ctx.font = '600 ' + fsName.toFixed(1) + 'px ' + T.mono;
  var nameW = 0;
  for(i = 0; i < rows.length; i++) nameW = Math.max(nameW, ctx.measureText(rows[i].name).width);
  nameW = Math.max(46, Math.min(96, nameW + 6));

  // Value gutter holds the two numeric labels, right-aligned, one per bar.
  ctx.font = '650 ' + fsSmall.toFixed(1) + 'px ' + T.mono;
  var valW = Math.max(34, Math.min(56, ctx.measureText('-188.8A').width + 4));

  // Fault chips are measured across ALL rows and the widest one reserves the
  // column for everybody. Per-row chip widths would give each row a different
  // bar length, and bars of different lengths cannot be compared at a glance —
  // which is the only thing this strip is for.
  ctx.font = '600 ' + fsSmall.toFixed(1) + 'px ' + T.sans;
  var chipW = 0;
  for(i = 0; i < rows.length; i++){
    if(!rows[i].chip) continue;
    chipW = Math.max(chipW, ctx.measureText(rows[i].chip.text).width + 12);
  }
  chipW = Math.min(chipW, w * 0.34);
  var chipGap = chipW > 0 ? 8 : 0;

  // Narrow-panel degradation ladder, most expendable column first. Fixed
  // order so the layout is predictable as the browser is resized rather than
  // rearranging itself: chips (the fault text is also in the bench readout) ->
  // name column -> the numeric gutter (last, because the numbers are the part
  // an engineer writes down).
  var barX = padX + nameW + 6;
  var gutter = valW + 4;
  var chipCol = (chipW > 0) ? (chipW + chipGap) : 0;
  var barW = w - barX - gutter - chipCol - padX;
  if(barW < 40 && chipCol > 0){
    chipW = 0; chipGap = 0; chipCol = 0;
    barW = w - barX - gutter - padX;
  }
  if(barW < 40 && nameW > 46){
    nameW = 46; barX = padX + nameW + 6;
    barW = w - barX - gutter - padX;
  }
  if(barW < 28){
    valW = 0; gutter = 0;
    barW = w - barX - padX;
  }
  if(barW < 8) barW = 8;                   // never negative; may clip off-canvas
  var showBars = (barX + barW) <= (w - 2);
  var chipX = barX + barW + gutter + chipGap;

  var TMAX = _motTempMax();

  // ── header: title, live current limit, and the temperature scale ──
  if(headH){
    ctx.textAlign = 'left'; ctx.textBaseline = 'alphabetic';
    ctx.fillStyle = T.ink2;
    ctx.font = '600 10px ' + T.sans;
    ctx.fillText('MOTORS', padX, 12);
    var titleEnd = padX + ctx.measureText('MOTORS').width;

    // Ticks sit over the shared bar column, labelled once instead of per row.
    // The 0 and full-scale end labels are deliberately omitted: they would
    // collide with the title on the left and the current limit on the right at
    // narrow widths, and the ramp is unambiguous from three anchors.
    ctx.font = '650 9px ' + T.mono;
    var limTxt = limKnown ? ('LIMIT ' + lim.toFixed(0) + 'A')
                          : ('SCALE ~' + lim.toFixed(0) + 'A AUTO');
    var limLeft = w - padX - ctx.measureText(limTxt).width;

    ctx.font = '600 9px ' + T.mono;
    var ticks = [[40, '40', T.ok], [70, '70', T.warn], [90, '90°C', T.crit]];
    ctx.textAlign = 'center';
    for(i = 0; showBars && i < ticks.length; i++){
      var tx = barX + (ticks[i][0] / TMAX) * barW;
      var tHalf = ctx.measureText(ticks[i][1]).width / 2;
      // Drop a label rather than overprint the title or the limit readout.
      // The per-row tick lines still mark the position, so nothing is lost
      // that the eye needs.
      if(tx - tHalf < titleEnd + 6 || tx + tHalf > limLeft - 6) continue;
      ctx.fillStyle = ticks[i][2];
      ctx.fillText(ticks[i][1], tx, 12);
    }

    ctx.fillStyle = T.ink3;
    ctx.font = '650 9px ' + T.mono;
    ctx.textAlign = 'right';
    ctx.fillText(limTxt, w - padX, 12);

    ctx.strokeStyle = T.rule; ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(padX, headH - 4.5); ctx.lineTo(w - padX, headH - 4.5);
    ctx.stroke();
  }

  var pulse = _motReducedMotion() ? 1 : (0.80 + 0.20 * (0.5 + 0.5 * Math.sin(Date.now() / 700)));

  // ── rows ──
  for(i = 0; i < rows.length; i++){
    var r = rows[i];
    var y = padY + headH + i * rowH;
    var tCol = _motTempColor(r.temp, T);

    if(i === hottest){
      ctx.fillStyle = T.panel2;
      ctx.fillRect(padX - 4, y, w - (padX - 4) * 2, rowH - 1);
      ctx.fillStyle = tCol;                       // edge carries the severity
      ctx.fillRect(padX - 4, y, 2, rowH - 1);
    }else if(i){
      ctx.strokeStyle = T.rule; ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(padX, y + 0.5); ctx.lineTo(w - padX, y + 0.5);
      ctx.stroke();
    }

    // Name + velocity stacked in the label column.
    ctx.textAlign = 'left'; ctx.textBaseline = 'alphabetic';
    ctx.font = '600 ' + fsName.toFixed(1) + 'px ' + T.mono;
    ctx.fillStyle = (i === hottest) ? T.ink : T.ink2;
    ctx.fillText(r.name, padX, y + rowH * 0.46);
    // vel_rps is the ROTOR velocity the Kraken reports, straight from
    // get_velocity — it is NOT wheel rps and NOT m/s. Dividing by gear_ratio
    // is the drive node's job (_wheel_speed_ms), and doing it here would put
    // two different "speeds" on one page with no way to tell them apart.
    // The sign is the motor's own forward convention and is deliberately left
    // untouched: this row is raw device telemetry, and re-signing it would
    // make it disagree with the bench readout and with `ros2 topic echo`.
    // (steer_inverted / dsign is a RENDERING transform for the steering
    //  geometry only — see drawSteer — never for reported motor numbers.)
    ctx.font = '650 ' + fsSmall.toFixed(1) + 'px ' + T.mono;
    ctx.fillStyle = T.ink3;
    ctx.fillText(_motNum(r.vel, 1) + (r.vel === null ? '' : ' rps'), padX, y + rowH * 0.88);

    var offline = !!(r.chip && r.chip.offline);
    var barGap = Math.max(2, rowH * 0.10);
    var tH = Math.max(4, Math.min(9, rowH * 0.26));
    var aH = Math.max(3, Math.min(7, rowH * 0.20));
    var tY = y + rowH * 0.5 - tH - barGap * 0.5;
    var aY = y + rowH * 0.5 + barGap * 0.5;

    if(showBars){
      _motTempBar(ctx, T, barX, tY, barW, tH, r.temp, TMAX, offline);
      _motAmpBar(ctx, T, barX, aY, barW, aH, r.amps, lim, offline);
    }

    // Numeric gutter, one label per bar, right-aligned on a mono stack so the
    // digits line up down the strip. The two labels share the name/velocity
    // baselines rather than hanging off their bars — anchoring to the bars
    // pushes the lower number past the row edge once rowH drops to ~20 px,
    // which is exactly the four-motors-in-120-px case this has to survive.
    if(valW > 0){
      var vx = barX + barW + gutter;
      ctx.textAlign = 'right';
      ctx.font = '650 ' + fsSmall.toFixed(1) + 'px ' + T.mono;
      ctx.fillStyle = (r.temp === null) ? T.ink3 : ((i === hottest) ? tCol : T.ink);
      ctx.fillText(_motNum(r.temp, 1, '°'), vx, y + rowH * 0.46);
      var over = (r.amps !== null && Math.abs(r.amps) > lim);
      ctx.fillStyle = (r.amps === null) ? T.ink3 : (over ? T.crit : T.actual);
      ctx.fillText(_motNum(r.amps, 1, 'A'), vx, y + rowH * 0.88);
      ctx.textAlign = 'left';
    }

    if(r.chip && chipW > 0){
      var tone = r.chip.offline ? T.warn : T.crit;
      ctx.save();
      ctx.globalAlpha = r.chip.offline ? 1 : pulse;   // steady amber, pulsing red
      ctx.fillStyle = tone;
      ctx.globalAlpha *= 0.16;
      _motRR(ctx, chipX, y + rowH * 0.5 - fsSmall * 0.95,
             chipW, fsSmall * 1.9, fsSmall * 0.6);
      ctx.fill();
      ctx.globalAlpha = r.chip.offline ? 1 : pulse;
      ctx.strokeStyle = tone; ctx.lineWidth = 1;
      _motRR(ctx, chipX + 0.5, y + rowH * 0.5 - fsSmall * 0.95 + 0.5,
             chipW - 1, fsSmall * 1.9 - 1, fsSmall * 0.6);
      ctx.stroke();
      ctx.beginPath();
      ctx.rect(chipX + 5, y, chipW - 10, rowH);       // belt and braces
      ctx.clip();
      ctx.fillStyle = tone;
      ctx.font = '600 ' + fsSmall.toFixed(1) + 'px ' + T.sans;
      ctx.textBaseline = 'middle';
      // Ellipsise rather than hard-clip: a chip reading "device_temp, stator_c"
      // looks like the fault name, an ellipsis reads as "there is more, open
      // the Topics dump". The full list is always in /ethon/drive_status.
      var label = r.chip.text;
      if(ctx.measureText(label).width > chipW - 12){
        while(label.length > 1 && ctx.measureText(label + '…').width > chipW - 12)
          label = label.slice(0, label.length - 1);
        label = label + '…';
      }
      ctx.fillText(label, chipX + 6, y + rowH * 0.5);
      ctx.textBaseline = 'alphabetic';
      ctx.restore();
    }
  }

  if(overflow){
    // Never silently drop a motor: say how many are hidden, on an opaque
    // backing so it stays readable over whatever row it lands on.
    ctx.font = '600 9px ' + T.sans;
    ctx.textAlign = 'right';
    var oTxt = '+' + overflow + ' more', oW = ctx.measureText(oTxt).width;
    ctx.fillStyle = T.panel;
    ctx.fillRect(w - padX - oW - 4, h - 13, oW + 8, 13);
    ctx.fillStyle = T.warn;
    ctx.fillText(oTxt, w - padX, h - 3);
    ctx.textAlign = 'left';
  }
}

// Temperature bar. The gradient is anchored to the TRACK, not to the fill, so
// a given colour always sits at the same temperature: 40 C is the last green
// pixel, 70 C amber, 90 C red. Fill the sub-rect with the full-width gradient
// and the colours stay absolute.
function _motTempBar(ctx, T, x, y, w, h, temp, tmax, offline){
  ctx.save();
  var g = ctx.createLinearGradient(x, 0, x + w, 0);
  g.addColorStop(0, T.ok);
  g.addColorStop(40 / tmax, T.ok);
  g.addColorStop(70 / tmax, T.warn);
  g.addColorStop(90 / tmax, T.crit);
  g.addColorStop(1, T.crit);

  ctx.globalAlpha = 0.14;                       // ghost of the full scale
  ctx.fillStyle = g;
  _motRR(ctx, x, y, w, h, h / 2); ctx.fill();
  ctx.globalAlpha = 1;

  // Scale ticks repeated per row: reading a bar against a header 100 px away
  // is guesswork, and this is the number an engineer aborts a test on.
  ctx.fillStyle = T.rule2;
  var marks = [40, 70, 90], i;
  for(i = 0; i < marks.length; i++)
    ctx.fillRect(Math.round(x + (marks[i] / tmax) * w), y, 1, h);

  if(temp === null){
    ctx.fillStyle = T.ink3;
    ctx.globalAlpha = 0.5;
    ctx.fillRect(x, y + h / 2 - 0.5, w, 1);     // flatline = no reading
    ctx.restore();
    return;
  }
  var f = Math.max(0, Math.min(1, temp / tmax));
  var fw = Math.max(h, f * w);                  // never narrower than a dot
  ctx.globalAlpha = offline ? 0.35 : 1;         // dim a motor that isn't answering
  ctx.fillStyle = g;
  _motRR(ctx, x, y, fw, h, h / 2); ctx.fill();
  ctx.restore();
}

// Supply current against the live thermal limit.
//
// supply_a goes NEGATIVE under regen — ethon_drive integrates exactly that
// sign to net recovered charge out of the energy total — so the sign is
// information, not noise. The track is therefore bipolar on ONE linear scale:
// zero sits 20% in, giving -0.25*limit to the left and +limit to the right at
// identical amps-per-pixel. A unipolar |a| bar would render regen and drive
// identically, which is the one thing you must not do while tuning
// regen_strength.
//
// The steer motor shares the drive scale on purpose: its bar is nearly always
// a stub, and that IS the reading — if it ever grows to look like a drive
// motor's, something is binding. The numeric gutter carries the precision.
function _motAmpBar(ctx, T, x, y, w, h, amps, lim, offline){
  ctx.save();
  var zf = 0.20;                                 // zero position along the track
  var zx = x + zf * w;
  var pxPerA = (w * (1 - zf)) / lim;

  ctx.globalAlpha = 0.10;
  ctx.fillStyle = T.ink2;
  _motRR(ctx, x, y, w, h, h / 2); ctx.fill();
  ctx.globalAlpha = 1;

  ctx.fillStyle = T.rule2;                       // zero mark
  ctx.fillRect(Math.round(zx), y - 1, 1, h + 2);

  if(amps === null){
    ctx.fillStyle = T.ink3;
    ctx.globalAlpha = 0.5;
    ctx.fillRect(x, y + h / 2 - 0.5, w, 1);
    ctx.restore();
    return;
  }
  var over = Math.abs(amps) > lim;
  ctx.globalAlpha = offline ? 0.35 : 1;
  // Magenta = measured. Over the derated limit it becomes a state alarm, so it
  // hands over to crit red.
  ctx.fillStyle = over ? T.crit : T.actual;
  if(amps >= 0){
    var wR = Math.min(w - zf * w, amps * pxPerA);
    _motRR(ctx, zx, y, Math.max(1.5, wR), h, h / 2); ctx.fill();
  }else{
    var wL = Math.min(zf * w, -amps * pxPerA);
    _motRR(ctx, zx - Math.max(1.5, wL), y, Math.max(1.5, wL), h, h / 2); ctx.fill();
    if(-amps * pxPerA > zf * w){                 // regen off the left of scale
      ctx.fillStyle = T.actual;
      ctx.fillRect(x - 3, y, 2, h);
    }
  }
  ctx.restore();
}

// Calm empty state. The car boots with the CAN bus down and an unhomed
// column, so "no data" is a normal condition on this panel, not an error —
// it must not look like one. The second line separates "the drive node never
// spoke" from "it spoke and reported no motors", because those send you to
// two completely different places.
function _motEmpty(ctx, w, h, T, d){
  var spoke = false;
  try{ spoke = !!(d && typeof d === 'object' && Object.keys(d).length); }catch(e){ spoke = false; }
  ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
  ctx.fillStyle = T.ink2;
  ctx.font = '600 11px ' + T.sans;
  ctx.fillText('NO MOTOR DATA', w / 2, h / 2 - 8);
  ctx.fillStyle = T.ink3;
  ctx.font = '10px ' + T.mono;
  ctx.fillText(spoke ? 'drive_status live, motor table empty — check CAN'
                     : 'waiting for /ethon/drive_status', w / 2, h / 2 + 9);
  ctx.textAlign = 'left'; ctx.textBaseline = 'alphabetic';
}

// ── gfx_energy.js ──
// ═══ race energy budget ═══════════════════════════════════════════════════
// /ethon/strategy has been reaching the browser since the strategist was
// written but has only ever been rendered on the pit board, so the person
// sitting at the console — the one who can actually change how the car is
// driven — has never seen it. This panel answers exactly one question at a
// glance: AM I GOING TO MAKE IT.
//
// The race is an energy budget, not a speed contest: a fixed usable Wh
// (battery_usable_wh, 480 Wh today) over a fixed clock (race_minutes, 70).
// Everything here is a comparison of two numbers — what you have spent
// against what the clock says you should have spent.
//
// Colour discipline in this panel:
//   teal     = PLANNED   — the budget line, the target-now marker, target rate
//   magenta  = MEASURED  — Wh actually spent, actual burn rate, the projection
//                          (a projection is an extrapolation of a MEASUREMENT,
//                           so it stays magenta and is drawn dashed/hollow to
//                           say "not banked yet")
//   grn/amb/red = the VERDICT only — on budget / marginal / will not finish.
//                 Nothing else in this panel is allowed to use them, so a red
//                 pixel anywhere in the frame always means the same thing.
//
// Nothing in here fetches, posts, mutates shared state or sets a timer. The
// only thing it touches is the canvas it is handed; there are no readout
// element ids for this panel. It is repainted by the page's own poll, which
// is also the only clock any animation in here gets.

// Design tokens. canvas 2d cannot read CSS custom properties, so the palette
// is duplicated here; keep it in step with :root.
const _EN_C = {
  ground:'#090B0F', bg2:'#0E1218', panel:'#131923', panel2:'#182030',
  rule:'#212B3A', rule2:'#2C3949',
  ink:'#EAF0F7', ink2:'#9AA9BC', ink3:'#64748B',
  ok:'#2ED47A', warn:'#F5A524', crit:'#FF4D4F',
  accent:'#4CE0D2', actual:'#FF5CA8'
};
const _EN_SANS = 'system-ui,-apple-system,"Segoe UI",Roboto,"Helvetica Neue",Arial,sans-serif';
const _EN_MONO = 'ui-monospace,"SF Mono","Cascadia Mono",Menlo,Consolas,"DejaVu Sans Mono",monospace';

// Mirror of RaceStrategist.PACE_BAND in race_services.py (line 125): the
// strategist treats +/-8% of the budget rate as "on target". We reuse the same
// number so this panel can never call something a disaster while the pit board
// is still calling it ON TARGET — outside the band we go red exactly when the
// strategist says SLOW DOWN, and the amber "marginal" grade lives strictly
// INSIDE the band, on the overspending side of it. If that constant is ever
// retuned in race_services.py, retune it here too.
const _EN_PACE_BAND = 0.08;

// How much of the bar is kept to the right of the budget line for the
// over-budget zone. Fixed rather than data-driven on purpose: the budget line
// must not slide left and right between frames, or the eye loses the one
// reference point the whole panel is built around.
const _EN_OVER_ZONE = 0.16;

// ── small helpers, all prefixed so they cannot collide with the other canvas
// ── modules sharing this script block ─────────────────────────────────────

// Every field of /ethon/strategy can be null (the strategist publishes null
// for rate/projection/per-lap until it has enough race to mean anything) and
// after a bad JSON round trip a number can arrive as a string. Anything that
// is not a finite number is "no reading". A NaN that reaches a canvas
// coordinate silently draws nothing at all, which is far worse on a race
// dashboard than an honest em dash.
function _enNum(v){ if(v==null) return null; const n=+v; return isFinite(n)?n:null; }

function _enFix(v,dec){ return v==null ? '—' : (+v).toFixed(dec==null?0:dec); }

function _enMmss(s){
  const n=_enNum(s);
  if(n==null) return '—:—';
  const t=Math.max(0,Math.round(n));
  return Math.floor(t/60)+':'+String(t%60).padStart(2,'0');
}

function _enRGBA(hex,a){
  const h=String(hex).replace('#','');
  const r=parseInt(h.substring(0,2),16), g=parseInt(h.substring(2,4),16), b=parseInt(h.substring(4,6),16);
  return 'rgba('+r+','+g+','+b+','+a+')';
}

function _enRR(ctx,x,y,w,h,r){
  const rr=Math.max(0,Math.min(r,w/2,h/2));
  ctx.beginPath();
  ctx.moveTo(x+rr,y);ctx.arcTo(x+w,y,x+w,y+h,rr);
  ctx.arcTo(x+w,y+h,x,y+h,rr);ctx.arcTo(x,y+h,x,y,rr);
  ctx.arcTo(x,y,x+w,y,rr);ctx.closePath();
}

// Manual letter-spacing: ctx.letterSpacing only landed in Chrome 99 and the
// pit laptop is not guaranteed to be newer than 90.
function _enTracked(ctx,txt,x,y,sp,align){
  txt=String(txt==null?'':txt);
  const ws=[]; let total=0;
  for(let i=0;i<txt.length;i++){
    const cw=ctx.measureText(txt.charAt(i)).width;
    ws.push(cw); total+=cw+(i<txt.length-1?sp:0);
  }
  let px=x;
  if(align==='center') px=x-total/2; else if(align==='right') px=x-total;
  const old=ctx.textAlign; ctx.textAlign='left';
  for(let i=0;i<txt.length;i++){ ctx.fillText(txt.charAt(i),px,y); px+=ws[i]+sp; }
  ctx.textAlign=old;
  return total;
}

function _enTrackedWidth(ctx,txt,sp){
  txt=String(txt==null?'':txt);
  let total=0;
  for(let i=0;i<txt.length;i++) total+=ctx.measureText(txt.charAt(i)).width+(i<txt.length-1?sp:0);
  return total;
}

// Ellipsise to a width. This panel is span-4 on a 12-column grid and goes
// full-width under 1000 px, so the same string has to survive both a 300 px
// and a 900 px canvas. A sentence that runs off the edge of a race dashboard
// looks like a rendering bug and gets the whole panel distrusted.
function _enClip(ctx,txt,maxW){
  txt=String(txt==null?'':txt);
  if(maxW<=0) return '';
  if(ctx.measureText(txt).width<=maxW) return txt;
  let lo=0, hi=txt.length;
  while(lo<hi){
    const mid=(lo+hi+1)>>1;
    if(ctx.measureText(txt.substring(0,mid)+'…').width<=maxW) lo=mid; else hi=mid-1;
  }
  return lo>0?(txt.substring(0,lo)+'…'):'';
}

// The one uppercase micro-label style used throughout the panel.
function _enLabel(ctx,txt,x,y,color,align){
  ctx.font='600 10px '+_EN_SANS;
  ctx.fillStyle=color||_EN_C.ink3;
  ctx.textBaseline='alphabetic';
  return _enTracked(ctx,String(txt).toUpperCase(),x,y,0.7,align||'left');
}

// prefers-reduced-motion, asked fresh on every draw so a mid-session OS change
// is picked up. Wrapped because matchMedia is absent in some embedded webviews.
function _enReduced(){
  try{ return !!(window.matchMedia && window.matchMedia('(prefers-reduced-motion: reduce)').matches); }
  catch(e){ return false; }
}

// A slow breathe for the "will not finish" pill, and nothing else. This module
// owns no timer — it only ever samples the wall clock at whatever rate the
// page happens to repaint (600 ms on the console, 1 s on the pit board), so
// the motion has to survive coarse, irregular sampling. A low-amplitude alpha
// ramp does; a blink would just look like a rendering fault. Under
// prefers-reduced-motion it is pinned fully opaque, never dimmed, so the
// warning is if anything MORE readable.
function _enPulse(){
  if(_enReduced()) return 1;
  const ph=(Date.now()%1800)/1800;
  return 0.80+0.20*(0.5+0.5*Math.cos(ph*2*Math.PI));
}

// Diagonal hatch, used only for the over-budget zone. Clipped rather than
// computed per line so it cannot bleed past the budget line.
function _enHatch(ctx,x,y,w,h,color,alpha,step){
  if(w<=0||h<=0) return;
  ctx.save();
  ctx.beginPath();ctx.rect(x,y,w,h);ctx.clip();
  ctx.strokeStyle=_enRGBA(color,alpha);ctx.lineWidth=1;
  const s=step||7;
  ctx.beginPath();
  for(let i=-h;i<w+h;i+=s){ ctx.moveTo(x+i,y+h); ctx.lineTo(x+i+h,y); }
  ctx.stroke();
  ctx.restore();
}

// ── the model ─────────────────────────────────────────────────────────────
// Everything the painter needs, normalised once, so no drawing code ever has
// to think about nulls or units again.
function _enModel(strategy,drive,status){
  const s=(strategy&&typeof strategy==='object')?strategy:{};
  const d=(drive&&typeof drive==='object')?drive:{};
  const st=(status&&typeof status==='object')?status:{};

  const g={};
  g.err     = (typeof s.error==='string'&&s.error)?s.error:null;
  g.raceOn  = (s.race_on===true);
  g.budget  = _enNum(s.wh_budget);
  g.used    = _enNum(s.wh_used);
  g.remain  = _enNum(s.wh_remaining);
  g.elapsed = _enNum(s.elapsed_s);
  g.left_s  = _enNum(s.remaining_s);
  g.rate    = _enNum(s.rate_wh_min);      // null for the first 30 s of race
  g.target  = _enNum(s.budget_wh_min);
  g.proj    = _enNum(s.projected_wh);     // null while rate is null
  g.pace    = (typeof s.pace==='string'&&s.pace!=='-')?s.pace:null;
  g.paceN   = _enNum(s.pace_n);
  g.whLap   = _enNum(s.wh_per_lap);
  g.lastLap = _enNum(s.last_lap_wh);
  g.laps    = _enNum(s.laps_done);
  g.pct     = _enNum(s.battery_pct);
  g.boot    = _enNum(s.wh_total_since_boot);

  // Pre-race fallbacks. drive_status is published at 2 Hz whether or not the
  // strategist is alive, so the calm state can still show something true even
  // if /ethon/strategy is missing entirely.
  if(g.boot==null) g.boot=_enNum(d.energy_wh);
  // supply_v is measured at the Krakens; status.battery_v is the same number
  // relayed through the dashboard's status block, so it is the fallback rather
  // than a second opinion.
  g.volts = _enNum(d.supply_v);
  if(g.volts==null) g.volts=_enNum(st.battery_v);

  // Total race length, reconstructed. The strategist publishes elapsed_s and
  // remaining_s but not race_minutes, and remaining_s is clamped at 0 — so
  // once the clock expires this sum collapses to elapsed and the target marker
  // correctly parks on the full budget rather than running off the end.
  const total=(g.elapsed!=null&&g.left_s!=null)?(g.elapsed+g.left_s):null;
  g.raceS=(total!=null&&total>0)?total:null;

  // Where the schedule says you should be RIGHT NOW. Straight-line pacing is
  // what the strategist itself budgets against (budget_wh_min is a flat rate),
  // so anything cleverer here would disagree with the number next to it.
  g.targetNow=null;
  if(g.raceOn&&g.budget!=null&&g.elapsed!=null&&g.raceS!=null){
    g.targetNow=g.budget*Math.max(0,Math.min(1,g.elapsed/g.raceS));
  }
  // Positive = spent less than the schedule = energy in hand.
  g.slack=(g.targetNow!=null&&g.used!=null)?(g.targetNow-g.used):null;
  // Positive = projected to finish over budget.
  g.over=(g.proj!=null&&g.budget!=null)?(g.proj-g.budget):null;

  return g;
}

// The verdict. Three grades, and they are deliberately pinned to the
// strategist's own arithmetic so the console and the pit board can never tell
// two different stories:
//   projected <= budget                  -> ok    "ON BUDGET"
//   0 < overrun <= PACE_BAND of budget   -> warn  "MARGINAL"
//   overrun  >  PACE_BAND of budget      -> crit  "WILL NOT FINISH"
// Because projected = rate * race_minutes and budget = budget_rate *
// race_minutes, projected/budget is identically rate/budget_rate — so the crit
// grade fires on exactly the same condition as the strategist's "SLOW DOWN",
// and amber only ever appears inside the band it still calls ON TARGET. The
// strategist's own word is shown verbatim further down the panel; this pill
// grades severity, it does not form a second opinion.
function _enVerdict(g){
  if(g.err) return {lvl:'crit', txt:'STRATEGIST ERROR', col:_EN_C.crit};
  if(!g.raceOn) return {lvl:'idle', txt:'RACE NOT STARTED', col:_EN_C.ink3};
  // Already spent it. A rate extrapolation is irrelevant once the tank is
  // empty, so this outranks the projection.
  if(g.budget!=null&&g.used!=null&&g.used>=g.budget) return {lvl:'crit', txt:'OVER BUDGET', col:_EN_C.crit};
  if(g.left_s!=null&&g.left_s<=0) return {lvl:'idle', txt:'TIME UP', col:_EN_C.ink2};
  // The strategist withholds rate/projection until 30 s of race have run,
  // because a 5 s sample of a standing start projects nonsense. Say so rather
  // than inventing a verdict.
  if(g.proj==null||g.budget==null||g.budget<=0) return {lvl:'idle', txt:'MEASURING PACE', col:_EN_C.ink2};
  const r=g.proj/g.budget;
  if(r<=1.0) return {lvl:'ok', txt:'ON BUDGET', col:_EN_C.ok};
  if(r<=1.0+_EN_PACE_BAND) return {lvl:'warn', txt:'MARGINAL', col:_EN_C.warn};
  return {lvl:'crit', txt:'WILL NOT FINISH', col:_EN_C.crit};
}

// ── header: section label + verdict pill ──────────────────────────────────
function _enHeader(ctx,x,y,w,v){
  ctx.font='700 10px '+_EN_SANS;
  const tw=_enTrackedWidth(ctx,v.txt,0.8);
  const pw=tw+18, ph=18, px=x+w-pw, py=y-2;

  // The verdict outranks the section label: on a narrow canvas the label is
  // dropped rather than allowed to run underneath the pill. The panel heading
  // already says "Energy" in the DOM above the canvas, so nothing is lost.
  ctx.font='600 10px '+_EN_SANS;
  if(_enTrackedWidth(ctx,'ENERGY BUDGET',0.7)+14<=w-pw){
    _enLabel(ctx,'Energy budget',x,y+10,_EN_C.ink2,'left');
  }

  const alpha=(v.lvl==='crit')?_enPulse():1;

  ctx.save();
  ctx.globalAlpha=alpha;
  _enRR(ctx,px,py,pw,ph,9);
  ctx.fillStyle=(v.lvl==='idle')?_EN_C.panel2:_enRGBA(v.col,0.16);
  ctx.fill();
  ctx.strokeStyle=(v.lvl==='idle')?_EN_C.rule2:_enRGBA(v.col,0.55);
  ctx.lineWidth=1;ctx.stroke();
  ctx.restore();

  ctx.fillStyle=v.col;ctx.textBaseline='alphabetic';
  _enTracked(ctx,v.txt,px+pw/2,py+12.5,0.8,'center');
}

// ── the three-up metric strip ─────────────────────────────────────────────
// Cell value sizes track the panel width so the numbers stay large on the
// console's 1/3-width column and do not collide on a narrow pit-board window.
function _enCell(ctx,x,w,yTop,hh,label,value,unit,col,sub,subCol){
  // Bounded by the cell WIDTH so three cells never collide, and by the cell
  // HEIGHT so the sub-line underneath is not sitting in the digits' descenders
  // when the panel is short.
  const big=Math.max(16,Math.min(27,Math.round(w*0.30),hh-24));
  ctx.font='600 10px '+_EN_SANS;
  // clipped a little short of the cell: _enLabel adds manual letter-spacing on
  // top of whatever measureText reported here.
  _enLabel(ctx,_enClip(ctx,String(label).toUpperCase(),Math.max(12,w-9)),x,yTop+9,_EN_C.ink3,'left');

  ctx.textBaseline='alphabetic';
  ctx.textAlign='left';
  ctx.font='700 '+big+'px '+_EN_MONO;
  const vy=yTop+9+big+2;
  ctx.fillStyle=col||_EN_C.ink;
  ctx.fillText(value,x,vy);
  if(unit){
    const vw=ctx.measureText(value).width;
    ctx.font='600 10px '+_EN_SANS;
    ctx.fillStyle=_EN_C.ink3;
    ctx.fillText(unit,x+vw+4,vy);
  }
  if(sub){
    ctx.font='500 10px '+_EN_SANS;
    ctx.fillStyle=subCol||_EN_C.ink3;
    ctx.fillText(_enClip(ctx,sub,w),x,Math.min(yTop+hh-1,vy+12));
  }
}

function _enStrip(ctx,x,y,w,h,g,v){
  const cw=w/3;
  ctx.save();
  ctx.strokeStyle=_EN_C.rule;ctx.lineWidth=1;
  ctx.beginPath();
  // Hairlines are drawn on the half pixel so they stay 1 device pixel wide
  // after the devicePixelRatio transform instead of blurring across two.
  ctx.moveTo(Math.round(x+cw)+0.5,y+2);ctx.lineTo(Math.round(x+cw)+0.5,y+h-2);
  ctx.moveTo(Math.round(x+2*cw)+0.5,y+2);ctx.lineTo(Math.round(x+2*cw)+0.5,y+h-2);
  ctx.stroke();
  ctx.restore();

  const pad=10, cellW=cw-pad;

  if(!g.raceOn){
    // Calm state. Everything shown here is real and available before the
    // clock starts: the configured budget, the pack voltage at the Krakens,
    // and the full race length the strategist is holding ready.
    _enCell(ctx,x,cellW,y,h,'Budget',_enFix(g.budget,0),'Wh',_EN_C.accent,
            g.pct!=null?(_enFix(g.pct,0)+'% pack estimate'):null,_EN_C.ink3);
    _enCell(ctx,x+cw+pad,cellW,y,h,'Battery',_enFix(g.volts,1),'V',_EN_C.actual,
            g.boot!=null?(_enFix(g.boot,1)+' Wh since boot'):null,_EN_C.ink3);
    _enCell(ctx,x+2*cw+pad,cellW,y,h,'Race clock',_enMmss(g.left_s),null,_EN_C.ink2,
            'ready',_EN_C.ink3);
    return;
  }

  // Projected finish. Magenta would be the honest colour for an extrapolated
  // measurement, but this is THE number the verdict is made of, so it carries
  // the verdict colour and the bar below carries the magenta.
  _enCell(ctx,x,cellW,y,h,'Projected',_enFix(g.proj,0),'Wh',
          g.proj==null?_EN_C.ink3:v.col,
          g.proj==null?'needs 30 s of race':('budget '+_enFix(g.budget,0)+' Wh'),_EN_C.ink3);

  // The gap, signed, because "480 vs 512" makes you do arithmetic while
  // "+32 OVER" does not.
  let gapTxt='—', gapCol=_EN_C.ink3, gapLbl='vs budget', gapSub=null;
  if(g.over!=null){
    const o=g.over;
    if(Math.abs(o)<0.5){ gapTxt='0'; }
    else gapTxt=(o>0?'+':'')+_enFix(o,0);
    gapCol=(o>0)?v.col:_EN_C.ok;
    gapLbl=(o>0)?'over budget':'under budget';
    gapSub=(o>0)?'at this burn rate':'spare at the flag';
  }
  _enCell(ctx,x+cw+pad,cellW,y,h,gapLbl,gapTxt,'Wh',gapCol,gapSub,_EN_C.ink3);

  _enCell(ctx,x+2*cw+pad,cellW,y,h,'Time left',_enMmss(g.left_s),null,_EN_C.ink,
          g.elapsed!=null?(_enMmss(g.elapsed)+' elapsed'):null,_EN_C.ink3);
}

// ── the budget bar ────────────────────────────────────────────────────────
// One rail, 0 Wh on the left, the budget line fixed near the right with the
// over-budget zone hatched beyond it. Magenta fill = spent. Teal marker =
// where the clock says you should be. Hollow magenta caret = where this burn
// rate lands you. If the caret is left of the budget line you finish.
function _enBudgetBar(ctx,x,y,w,h,g){
  const barH=Math.max(16,Math.min(22,Math.round(h*0.40)));
  const barY=y+14;
  const budget=(g.budget!=null&&g.budget>0)?g.budget:null;

  // Header line of the block: spent on the left in magenta, budget on the
  // right in teal, so the bar underneath needs no legend.
  ctx.textBaseline='alphabetic';ctx.textAlign='left';
  const lw=_enLabel(ctx,g.raceOn?'Spent':'Since boot',x,y+9,_EN_C.ink3,'left');
  ctx.font='700 11px '+_EN_MONO;ctx.fillStyle=_EN_C.actual;
  ctx.fillText(_enFix(g.raceOn?g.used:g.boot,1)+' Wh',x+lw+7,y+9);

  ctx.textAlign='right';
  ctx.font='700 11px '+_EN_MONO;ctx.fillStyle=_EN_C.accent;
  const bTxt=_enFix(budget,0)+' Wh';
  ctx.fillText(bTxt,x+w,y+9);
  const bw=ctx.measureText(bTxt).width;
  ctx.textAlign='left';
  _enLabel(ctx,'Budget',x+w-bw-7,y+9,_EN_C.ink3,'right');

  // Track.
  ctx.save();
  _enRR(ctx,x,barY,w,barH,5);
  ctx.fillStyle=_EN_C.panel2;ctx.fill();
  ctx.strokeStyle=_EN_C.rule;ctx.lineWidth=1;ctx.stroke();
  ctx.restore();

  if(budget==null){
    ctx.font='500 11px '+_EN_SANS;ctx.fillStyle=_EN_C.ink3;ctx.textAlign='center';
    ctx.fillText('no budget published',x+w/2,barY+barH/2+4);
    ctx.textAlign='left';
    return;
  }

  // Domain. The budget line sits at a FIXED fraction of the rail so it never
  // moves between frames; everything past it is the over-budget zone.
  const dmax=budget*(1+_EN_OVER_ZONE);
  const X=function(wh){ return x+Math.max(0,Math.min(1,wh/dmax))*w; };
  const xb=X(budget);

  // Over-budget zone.
  ctx.save();
  _enRR(ctx,x,barY,w,barH,5);ctx.clip();
  ctx.fillStyle=_enRGBA(_EN_C.crit,0.07);
  ctx.fillRect(xb,barY,x+w-xb,barH);
  _enHatch(ctx,xb,barY,x+w-xb,barH,_EN_C.crit,0.16,7);
  ctx.restore();

  // Spent fill. Before the race the meter shown is wh_total_since_boot, which
  // the strategist EXCLUDES from the race total (it re-zeros at START RACE —
  // see _on_start in race_services.py), so it is drawn dimmed to say "this
  // does not count against the budget yet".
  const spent=g.raceOn?g.used:g.boot;
  if(spent!=null&&spent>0){
    ctx.save();
    _enRR(ctx,x,barY,w,barH,5);ctx.clip();
    ctx.globalAlpha=g.raceOn?1:0.40;
    const xs=X(spent);
    const grd=ctx.createLinearGradient(x,0,xs,0);
    grd.addColorStop(0,_enRGBA(_EN_C.actual,0.55));
    grd.addColorStop(1,_EN_C.actual);
    ctx.fillStyle=grd;
    ctx.fillRect(x,barY,Math.max(2,xs-x),barH);
    ctx.restore();
  }

  // Budget line: the wall. Drawn over the fill, under the markers.
  ctx.save();
  ctx.strokeStyle=_EN_C.accent;ctx.lineWidth=2;
  ctx.beginPath();ctx.moveTo(Math.round(xb),barY-3);ctx.lineTo(Math.round(xb),barY+barH+3);ctx.stroke();
  ctx.restore();

  // Target-now marker: teal, planned. A filled down-triangle above the rail
  // plus a dashed line through it, so it reads even where it overlaps the
  // magenta fill.
  if(g.targetNow!=null){
    const xt=X(g.targetNow);
    ctx.save();
    ctx.setLineDash([3,3]);ctx.strokeStyle=_enRGBA(_EN_C.accent,0.85);ctx.lineWidth=1.5;
    ctx.beginPath();ctx.moveTo(xt,barY);ctx.lineTo(xt,barY+barH);ctx.stroke();
    ctx.restore();
    ctx.fillStyle=_EN_C.accent;
    ctx.beginPath();
    ctx.moveTo(xt,barY-1);ctx.lineTo(xt-4.5,barY-7);ctx.lineTo(xt+4.5,barY-7);
    ctx.closePath();ctx.fill();
  }

  // Projection caret: hollow magenta, below the rail, with a dashed run from
  // where you are now to where you end up. Hollow and dashed on purpose —
  // this Wh has not been spent yet.
  if(g.proj!=null&&g.used!=null&&g.raceOn){
    const xu=X(g.used), xp=X(g.proj);
    ctx.save();
    ctx.setLineDash([4,3]);ctx.strokeStyle=_enRGBA(_EN_C.actual,0.65);ctx.lineWidth=1.5;
    ctx.beginPath();ctx.moveTo(xu,barY+barH+6);ctx.lineTo(xp,barY+barH+6);ctx.stroke();
    ctx.restore();
    ctx.save();
    ctx.strokeStyle=_EN_C.actual;ctx.lineWidth=1.5;
    ctx.beginPath();
    ctx.moveTo(xp,barY+barH+2);ctx.lineTo(xp-4.5,barY+barH+9);ctx.lineTo(xp+4.5,barY+barH+9);
    ctx.closePath();ctx.stroke();
    ctx.restore();
    // Clamped projections must say so, or a caret parked on the right-hand
    // edge reads as "just barely over" when it may be double the budget.
    if(g.proj>dmax){
      ctx.fillStyle=_EN_C.crit;ctx.font='700 10px '+_EN_MONO;ctx.textAlign='right';
      ctx.fillText('»',x+w-1,barY+barH+10);ctx.textAlign='left';
    }
  }

  // Footer line of the block: the slack, in words, because "in hand" and
  // "overspent" need no interpretation at 60 km/h.
  const fy=y+h-1;
  ctx.font='500 10px '+_EN_SANS;
  if(!g.raceOn){
    ctx.fillStyle=_EN_C.ink3;
    ctx.fillText(_enClip(ctx,'not counted — START RACE re-zeros this meter',w),x,fy);
  }else if(g.slack!=null){
    const ahead=g.slack>=0;
    ctx.fillStyle=_EN_C.ink3;
    const head='▲ target now  ';
    ctx.fillText(head,x,fy);
    const hw=ctx.measureText(head).width;
    ctx.font='700 10px '+_EN_MONO;
    ctx.fillStyle=ahead?_EN_C.ok:_EN_C.crit;
    const num=_enFix(Math.abs(g.slack),1)+' Wh';
    // Measured while the mono face is still selected: mono is the wider of the
    // two, so measuring after the switch back to sans would tuck the tail of
    // the sentence under the digits.
    const nw=ctx.measureText(num).width;
    ctx.fillText(num,x+hw,fy);
    ctx.font='500 10px '+_EN_SANS;
    ctx.fillStyle=_EN_C.ink3;
    ctx.fillText(ahead?' in hand':' overspent',x+hw+nw+1,fy);
  }
}

// ── burn rate rail ────────────────────────────────────────────────────────
// Actual Wh/min against the flat budget rate. One rail rather than two so the
// comparison is a distance, not a subtraction.
function _enRateRail(ctx,x,y,w,h,g){
  ctx.textBaseline='alphabetic';ctx.textAlign='left';
  const lw=_enLabel(ctx,'Burn rate',x,y+9,_EN_C.ink3,'left');
  ctx.font='700 12px '+_EN_MONO;ctx.fillStyle=(g.rate==null)?_EN_C.ink3:_EN_C.actual;
  ctx.fillText(_enFix(g.rate,1),x+lw+7,y+9);
  const nw=ctx.measureText(_enFix(g.rate,1)).width;
  ctx.font='600 10px '+_EN_SANS;ctx.fillStyle=_EN_C.ink3;
  ctx.fillText('Wh/min',x+lw+11+nw,y+9);

  ctx.textAlign='right';
  ctx.font='700 12px '+_EN_MONO;ctx.fillStyle=_EN_C.accent;
  const tTxt=_enFix(g.target,1);
  ctx.fillText(tTxt,x+w,y+9);
  const tw=ctx.measureText(tTxt).width;
  ctx.textAlign='left';
  _enLabel(ctx,'Target',x+w-tw-7,y+9,_EN_C.ink3,'right');

  const railY=y+15, railH=8;
  ctx.save();
  _enRR(ctx,x,railY,w,railH,4);
  ctx.fillStyle=_EN_C.panel2;ctx.fill();
  ctx.strokeStyle=_EN_C.rule;ctx.lineWidth=1;ctx.stroke();
  ctx.restore();

  // Scale on the larger of the two so neither can leave the rail, with
  // headroom so a rate exactly on target does not paint the whole thing.
  const hi=Math.max(g.rate||0,g.target||0);
  if(hi>0){
    const dmax=hi*1.35;
    const X=function(v){ return x+Math.max(0,Math.min(1,v/dmax))*w; };
    if(g.rate!=null&&g.rate>0){
      ctx.save();
      _enRR(ctx,x,railY,w,railH,4);ctx.clip();
      ctx.fillStyle=_EN_C.actual;
      ctx.fillRect(x,railY,Math.max(2,X(g.rate)-x),railH);
      ctx.restore();
    }
    if(g.target!=null&&g.target>0){
      const xt=X(g.target);
      ctx.save();
      ctx.strokeStyle=_EN_C.accent;ctx.lineWidth=2;
      ctx.beginPath();ctx.moveTo(Math.round(xt),railY-3);ctx.lineTo(Math.round(xt),railY+railH+3);ctx.stroke();
      ctx.restore();
    }
  }

  // Sub-line. The strategist's own verdict word is echoed verbatim so the
  // console and the pit board are quoting one source; the pill above grades
  // it, this line attributes it.
  const fy=y+h-1;
  ctx.font='500 10px '+_EN_SANS;ctx.fillStyle=_EN_C.ink3;ctx.textAlign='left';

  // Per-lap energy on the right when there is any, because it is the number
  // you actually act on: it turns "12 Wh over" into "one slower lap". Measured
  // first so the attribution line on the left can be clipped around it.
  let right='';
  if(g.whLap!=null){
    right=_enFix(g.whLap,1)+' Wh/lap';
    if(g.laps!=null&&g.laps>0) right+='  ·  '+_enFix(g.laps,0)+(g.laps===1?' lap':' laps');
  }
  const rw=right?ctx.measureText(right).width+12:0;

  let left;
  if(g.err) left='strategist: '+g.err;
  else if(g.pace) left='strategist: '+g.pace;
  else if(g.raceOn) left='strategist: measuring';
  else left='flat pacing — '+_enFix(g.target,1)+' Wh/min all race';
  ctx.fillText(_enClip(ctx,left,w-rw),x,fy);

  if(right){
    ctx.textAlign='right';
    ctx.fillText(right,x+w,fy);
    ctx.textAlign='left';
  }
}

// ── layout + paint ────────────────────────────────────────────────────────
// The panel is 220 CSS px tall on the console and 230 on the pit board, and
// both pages drop it to full width under 1000 px. Rather than hardcode those,
// the blocks state what they need and the leftover is spread between them;
// blocks fall away from the bottom up if the canvas is ever made shorter, so a
// squeezed panel loses detail instead of overprinting itself.
function _enPaint(ctx,w,h,g,v){
  ctx.clearRect(0,0,w,h);
  ctx.fillStyle=_EN_C.panel;ctx.fillRect(0,0,w,h);
  ctx.textBaseline='alphabetic';ctx.textAlign='left';
  ctx.lineJoin='round';

  const pad=Math.max(10,Math.min(16,Math.round(w*0.034)));
  const x=pad, iw=Math.max(40,w-2*pad);
  const avail=h-2*pad;

  const headH=16, stripH=46, barH=56, railH=40;
  let showBar=true, showRail=true;
  if(avail<headH+stripH+barH+railH+16) showRail=false;
  if(avail<headH+stripH+barH+8){ showBar=false; showRail=false; }

  const need=headH+stripH+(showBar?barH:0)+(showRail?railH:0);
  const slots=1+(showBar?1:0)+(showRail?1:0);
  // Gaps are capped low on purpose: slack is worth more as a bigger number in
  // the strip than as more air between blocks. Those three figures are what
  // gets read from across the pit box.
  const gap=Math.max(6,Math.min(12,Math.floor((avail-need)/Math.max(1,slots))));
  const extra=Math.max(0,avail-need-gap*slots);
  const sH=stripH+Math.min(16,extra);

  let y=pad;
  _enHeader(ctx,x,y,iw,v);
  y+=headH+gap;
  _enStrip(ctx,x,y,iw,sH,g,v);
  y+=sH+gap;
  if(showBar){
    ctx.save();
    ctx.strokeStyle=_EN_C.rule;ctx.lineWidth=1;
    ctx.beginPath();ctx.moveTo(x,Math.round(y-gap/2)+0.5);ctx.lineTo(x+iw,Math.round(y-gap/2)+0.5);ctx.stroke();
    ctx.restore();
    _enBudgetBar(ctx,x,y,iw,barH,g);
    y+=barH+gap;
  }
  if(showRail) _enRateRail(ctx,x,y,iw,railH,g);
}

// A canvas that has never been laid out (hidden tab, panel not yet in the
// flow) reports 0x0; painting into it would throw on the gradient and leave a
// blank rectangle behind when the tab is shown. The page redraws on tab
// change, so returning quietly is correct.
function drawEnergy(id,strategy,drive,status){
  const s=setupCanvas(id);
  if(!s||!s.ctx||!s.w||!s.h) return;
  const ctx=s.ctx, w=s.w, h=s.h;
  try{
    const g=_enModel(strategy,drive,status);
    _enPaint(ctx,w,h,g,_enVerdict(g));
  }catch(e){
    // Last-ditch: the caller already wraps draws, but a half-painted energy
    // panel showing a stale bar is worse than one that admits it is broken,
    // so repaint the surface and say so. Everything in here is primitive
    // enough that it cannot throw in turn.
    try{
      ctx.clearRect(0,0,w,h);
      ctx.fillStyle=_EN_C.panel;ctx.fillRect(0,0,w,h);
      ctx.font='600 11px '+_EN_SANS;ctx.fillStyle=_EN_C.crit;ctx.textAlign='left';
      ctx.fillText('energy panel failed to draw — see console',12,Math.round(h/2));
    }catch(e2){}
    if(!drawEnergy._logged){ drawEnergy._logged=1; try{ console.error('drawEnergy',e); }catch(e3){} }
  }
}
drawEnergy._logged=0;

// ── gfx_map.js ──
// ═══════════════════════════════════════════════════════════════════════════
// GPS / world track map  —  drawMap(id, track, line, speed)
// ═══════════════════════════════════════════════════════════════════════════
// Pure render. This module reads the arrays it is handed and writes nothing
// but pixels into the canvas it was named. It never fetches, never POSTs,
// never sets a timer, never touches a DOM node other than that canvas, and
// writes no readout element ids. Nothing here can move the car.
//
// Everything downstream of the projection lives in a local metre frame with
// +x = east and +y = north. The screen mapping negates y (see Y() below),
// which is the one sign in this file that matters: canvas y grows DOWNWARD,
// so north renders UP only because Y() flips it. The north indicator and the
// heading chevron both depend on that single flip and nothing else.

const _MAP_C = {
  panel:'#131923', panel2:'#182030', ground:'#090B0F',
  rule:'#212B3A', rule2:'#2C3949',
  ink:'#EAF0F7', ink2:'#9AA9BC', ink3:'#64748B',
  actual:'#FF5CA8', accent:'#4CE0D2'
};
const _MAP_SANS = 'system-ui,-apple-system,"Segoe UI",Roboto,"Helvetica Neue",Arial,sans-serif';
const _MAP_MONO = 'ui-monospace,"SF Mono","Cascadia Mono",Menlo,Consolas,"DejaVu Sans Mono",monospace';

// ── entry point ────────────────────────────────────────────────────────────
// speed is OPTIONAL and must be index-parallel to track. It is deliberately
// treated as untrusted: /api/history builds `track` by dropping every sample
// that had no lat, while `speed` keeps every sample, so the two arrays only
// line up when the car held a fix for the whole 5-minute window. If the
// lengths disagree we drop to a flat trace rather than shift the colours
// along the lap — a mis-attributed colour would draw a braking point where
// there wasn't one, and someone would go re-tune a corner that was fine.
function drawMap(id,track,line,speed){
  let s=null;
  try{ s=setupCanvas(id); }catch(e){ return; }
  if(!s)return;
  const ctx=s.ctx,w=s.w,h=s.h;
  // Background first and outside the main try: even a total failure below
  // leaves a clean panel rather than whatever the previous frame drew.
  try{
    _mapResetState(ctx);
    ctx.clearRect(0,0,w,h);
    ctx.fillStyle=_MAP_C.panel;
    ctx.fillRect(0,0,w,h);
  }catch(e){ return; }
  try{
    _mapBody(ctx,w,h,track,line,speed);
  }catch(e){
    // A thrown draw must never take the page down, and a silently blank
    // canvas must never be mistaken for "no GPS" — say which it is.
    // getContext('2d') hands back the SAME context object every frame, so an
    // exception that escaped between save() and restore() leaves the save
    // stack one deep forever and bleeds globalAlpha/lineDash into every later
    // frame. restore() on an empty stack is a spec'd no-op, so draining it is
    // safe and costs nothing on the happy path (we never get here).
    try{ _mapUnwind(ctx); }catch(e2){}
    try{ _mapNote(ctx,w,h,'MAP RENDER ERROR','telemetry is unaffected — see console'); }catch(e3){}
    try{ if(window.console&&console.warn)console.warn('drawMap failed:',e); }catch(e4){}
  }
}

// Put the shared context back to known defaults. Cheap insurance against a
// previous frame (this module's or a sibling canvas module's) having died
// mid-draw and left alpha, dash or alignment set.
function _mapResetState(ctx){
  ctx.globalAlpha=1;
  ctx.lineWidth=1;
  ctx.lineCap='butt';
  ctx.lineJoin='miter';
  ctx.textAlign='left';
  ctx.textBaseline='alphabetic';
  if(ctx.setLineDash)ctx.setLineDash([]);
}

function _mapUnwind(ctx){
  for(let i=0;i<16;i++)ctx.restore();
  _mapResetState(ctx);
}

// ── main body ──────────────────────────────────────────────────────────────
function _mapBody(ctx,w,h,track,line,speed){
  const clean=_mapClean(track,speed);
  const pts0=clean.track, spd=clean.speed;
  const hasTrack=pts0.length>0;

  // The lap line is only usable if it actually carries a position. lap_timer
  // publishes the block before the line is marked, so line can exist with
  // null coordinates — and a bare isFinite(+line.lat) does NOT catch that,
  // because +null and +'' are both 0. An unmarked line read as 0,0 anchors
  // the extent to the Gulf of Guinea and squashes the real track to a pixel,
  // so null and the null island are both rejected explicitly.
  const llat=(line&&line.lat!=null&&line.lat!=='')?+line.lat:NaN;
  const llon=(line&&line.lon!=null&&line.lon!=='')?+line.lon:NaN;
  const hasLine=isFinite(llat)&&isFinite(llon)&&!(llat===0&&llon===0)
                &&Math.abs(llat)<=90&&Math.abs(llon)<=180;
  const lr=(hasLine&&isFinite(+line.r)&&+line.r>0)?+line.r:20;

  if(!hasTrack&&!hasLine){ _mapWaiting(ctx,w,h); return; }

  // ── projection ──────────────────────────────────────────────────────────
  // VERBATIM from the original implementation — equirectangular about a
  // reference point taken from the MIDDLE of the track (not the ends, which
  // are the most likely to be a stale or first-fix outlier). cos(lat0) scales
  // longitude degrees to metres at this latitude; 111320 m/deg is the
  // meridian constant. Over a race circuit the distortion is negligible and
  // the maths stays cheap enough to run every 1.2 s poll.
  const ref=hasTrack?pts0[Math.floor(pts0.length/2)]:[llat,llon];
  const lat0=ref[0]*Math.PI/180;
  const toXY=(la,lo)=>[(lo-ref[1])*Math.cos(lat0)*111320,(la-ref[0])*111320];

  const pts=pts0.map(p=>toXY(p[0],p[1]));
  const lineXY=hasLine?toXY(llat,llon):null;

  // ── framing ─────────────────────────────────────────────────────────────
  // VERBATIM extent/scale logic. The geofence corners are folded into the
  // bounds so the whole circle stays on screen even when the car has only
  // driven one end of the track.
  let xs=pts.map(p=>p[0]),ys=pts.map(p=>p[1]);
  if(lineXY){
    xs=xs.concat([lineXY[0]-lr,lineXY[0]+lr]);
    ys=ys.concat([lineXY[1]-lr,lineXY[1]+lr]);
  }
  if(!xs.length){ _mapWaiting(ctx,w,h); return; }
  const xmn=Math.min.apply(null,xs),xmx=Math.max.apply(null,xs);
  const ymn=Math.min.apply(null,ys),ymx=Math.max.apply(null,ys);
  // pad raised from the original 26 to 32 to keep the chrome (scale bar,
  // north, ramp legend) out of the trace. Same symmetric formula otherwise,
  // so the framing behaviour is unchanged.
  const dx=(xmx-xmn)||10,dy=(ymx-ymn)||10,pad=32;
  const sc=Math.min((w-2*pad)/dx,(h-2*pad)/dy);
  const cx=(xmn+xmx)/2,cy=(ymn+ymx)/2;
  // Y NEGATES: +y is north in the world frame, canvas y grows downward.
  const X=x=>w/2+(x-cx)*sc,Y=y=>h/2-(y-cy)*sc;
  if(!isFinite(sc)||sc<=0){ _mapWaiting(ctx,w,h); return; }

  // ── layers, back to front ───────────────────────────────────────────────
  _mapGrid(ctx,w,h,cx,cy,sc,X,Y);
  if(lineXY)_mapGeofence(ctx,X(lineXY[0]),Y(lineXY[1]),lr*sc);
  const rng=_mapSpeedRange(spd);
  _mapTrace(ctx,pts,spd,rng,X,Y);
  if(lineXY)_mapGate(ctx,pts,lineXY,lr,sc,X,Y);
  _mapHere(ctx,pts,X,Y);

  // ── chrome ──────────────────────────────────────────────────────────────
  _mapScaleBar(ctx,w,h,sc,dx);
  _mapNorth(ctx,w);
  if(rng)_mapRampKey(ctx,rng);
  ctx.font='11px '+_MAP_MONO;
  ctx.textAlign='right';ctx.textBaseline='alphabetic';
  ctx.fillStyle=_MAP_C.ink3;
  ctx.fillText(pts.length+' pts',w-10,h-12);
  ctx.textAlign='left';
}

// ── empty / degenerate states ──────────────────────────────────────────────
// Calm, not alarming: booting with no fix is the normal cold-start condition,
// not a fault. No colour, no icon that reads as an error.
function _mapWaiting(ctx,w,h){
  _mapNote(ctx,w,h,'WAITING FOR GPS','no position samples yet');
}

function _mapNote(ctx,w,h,title,sub){
  const cx=w/2,cy=h/2;
  const r=Math.max(16,Math.min(w,h)*0.13);
  // Two faint rings and a crosshair — reads as "a map with nothing on it"
  // rather than as a broken canvas.
  ctx.save();
  ctx.strokeStyle=_MAP_C.rule;ctx.lineWidth=1;
  ctx.setLineDash([3,5]);
  ctx.beginPath();ctx.arc(cx,cy-14,r,0,Math.PI*2);ctx.stroke();
  ctx.beginPath();ctx.arc(cx,cy-14,r*0.45,0,Math.PI*2);ctx.stroke();
  ctx.setLineDash([]);
  ctx.strokeStyle=_MAP_C.rule2;
  ctx.beginPath();
  ctx.moveTo(cx-r*1.25,cy-14);ctx.lineTo(cx+r*1.25,cy-14);
  ctx.moveTo(cx,cy-14-r*1.25);ctx.lineTo(cx,cy-14+r*1.25);
  ctx.stroke();
  ctx.restore();

  ctx.textAlign='center';ctx.textBaseline='alphabetic';
  ctx.font='600 11px '+_MAP_SANS;ctx.fillStyle=_MAP_C.ink2;
  _mapSpaced(ctx,title,cx,cy+r+16,0.9,'center');
  ctx.font='10px '+_MAP_SANS;ctx.fillStyle=_MAP_C.ink3;
  ctx.fillText(sub,cx,cy+r+31);
  ctx.textAlign='left';
}

// ── input hygiene ──────────────────────────────────────────────────────────
// Every field can be null: the car boots with no fix, and replay dumps carry
// empty strings for missing columns. Track and speed are filtered TOGETHER so
// dropping a bad sample can never slide the two out of alignment.
function _mapClean(track,speed){
  const raw=(track&&track.length)?track:[];
  // The length comparison is made against the RAW arrays, before any
  // filtering — that is the only moment at which "parallel" is knowable.
  const parallel=!!(speed&&speed.length===raw.length);
  const tr=[],sp=[];
  for(let i=0;i<raw.length;i++){
    const p=raw[i];
    if(!p||p.length<2)continue;
    const la=+p[0],lo=+p[1];
    if(!isFinite(la)||!isFinite(lo))continue;
    // Null island. The live sampler already rejects 0,0 but replay CSVs and
    // older logs do not, and a single 0,0 blows the map extent up to
    // continental scale and squashes the real track to a dot.
    if(la===0&&lo===0)continue;
    if(Math.abs(la)>90||Math.abs(lo)>180)continue;
    tr.push([la,lo]);
    if(parallel){
      const v=speed[i];
      sp.push((v==null||!isFinite(+v))?null:+v);
    }
  }
  return {track:tr,speed:parallel?sp:null};
}

// Window-relative normalisation bounds for the speed ramp, or null to mean
// "don't colour by speed".
function _mapSpeedRange(sp){
  if(!sp)return null;
  let mn=Infinity,mx=-Infinity,n=0;
  for(let i=0;i<sp.length;i++){
    const v=sp[i];
    if(v==null)continue;
    if(v<mn)mn=v;
    if(v>mx)mx=v;
    n++;
  }
  if(n<2||!isFinite(mn)||!isFinite(mx))return null;
  // A parked car still emits GPS jitter. Stretching the ramp across a 0.2
  // spread paints a full rainbow onto sensor noise and invites someone to
  // read a "fast section" into a car sitting in the paddock. Demand a real
  // spread before the colours are allowed to mean anything.
  // The "is it moving" half of the test uses MAGNITUDE, not mx: /api/history
  // pre-abs()es speed but the replay endpoint hands through signed speed_ms,
  // so a reverse-only window has mx < 0 and would otherwise lose its colours.
  if(!(mx-mn>=1.0)||!(Math.max(Math.abs(mn),Math.abs(mx))>=1.0))return null;
  return {mn:mn,mx:mx};
}

// ── colour ramp ────────────────────────────────────────────────────────────
// Continuous cold->warm colormap, deliberately NOT built from ok/warn/crit:
// on this dashboard green/amber/red mean vehicle STATE, and a state colour
// appearing on a position trace would be read as a fault at that point on the
// lap. The cold end sits near the teal accent and the warm end stops short of
// the cone orange, so neither reads as "commanded" nor as "cone". The numeric
// endpoints are always printed alongside (see _mapRampKey) so the ramp can
// never be mistaken for a threshold.
function _mapRamp(t){
  const stops=[
    [0.00, 14, 60, 82],    // deep blue-teal   — slowest in window
    [0.30, 33,150,168],
    [0.55, 76,224,210],    // accent teal      — mid
    [0.80,214,214,138],    // pale warm
    [1.00,255,160,107]     // warm apricot     — fastest in window
  ];
  let u=t;
  if(!isFinite(u))u=0;
  if(u<0)u=0;
  if(u>1)u=1;
  for(let i=0;i<stops.length-1;i++){
    const a=stops[i],b=stops[i+1];
    if(u<=b[0]||i===stops.length-2){
      const span=(b[0]-a[0])||1;
      const k=Math.max(0,Math.min(1,(u-a[0])/span));
      const r=Math.round(a[1]+(b[1]-a[1])*k);
      const g=Math.round(a[2]+(b[2]-a[2])*k);
      const bl=Math.round(a[3]+(b[3]-a[3])*k);
      return 'rgb('+r+','+g+','+bl+')';
    }
  }
  return _MAP_C.accent;
}

// ── faint metric grid ──────────────────────────────────────────────────────
// Anchored to the PROJECTION ORIGIN (the reference GPS point), not to the
// screen, so the lines stay pinned to the ground as the extent grows. That
// makes the grid a distance reference rather than decoration.
function _mapGrid(ctx,w,h,cx,cy,sc,X,Y){
  const step=_mapNiceStep(64/sc);
  if(!isFinite(step)||step<=0)return;
  const x0=cx-(w/2)/sc,x1=cx+(w/2)/sc;
  const y0=cy-(h/2)/sc,y1=cy+(h/2)/sc;
  const nx=Math.ceil((x1-x0)/step),ny=Math.ceil((y1-y0)/step);
  if(nx+ny>240)return;               // degenerate scale — grid would be a wash
  ctx.save();
  ctx.strokeStyle=_MAP_C.rule;ctx.lineWidth=1;ctx.globalAlpha=0.55;
  ctx.beginPath();
  for(let gx=Math.ceil(x0/step)*step;gx<=x1;gx+=step){
    const px=Math.round(X(gx))+0.5;   // half-pixel so hairlines stay hairlines
    ctx.moveTo(px,0);ctx.lineTo(px,h);
  }
  for(let gy=Math.ceil(y0/step)*step;gy<=y1;gy+=step){
    const py=Math.round(Y(gy))+0.5;
    ctx.moveTo(0,py);ctx.lineTo(w,py);
  }
  ctx.stroke();
  ctx.restore();
}

// 1/2/5-per-decade sequence, so grid and scale-bar values are always numbers
// a human reads without thinking (1, 2, 5, 10, 20, 50, 100 m …).
function _mapNiceStep(v){
  if(!isFinite(v)||v<=0)return 0;
  const p=Math.pow(10,Math.floor(Math.log10(v)));
  const n=v/p;
  let m=10;
  if(n<1.5)m=1;else if(n<3.5)m=2;else if(n<7.5)m=5;
  return m*p;
}

// ── traced track ───────────────────────────────────────────────────────────
function _mapTrace(ctx,pts,spd,rng,X,Y){
  if(pts.length<2){
    return;                            // single fix: only the marker is honest
  }
  ctx.save();
  ctx.lineJoin='round';ctx.lineCap='round';
  // Dark casing under the trace so it separates from the grid without having
  // to brighten the trace itself.
  ctx.strokeStyle=_MAP_C.ground;ctx.lineWidth=5;ctx.globalAlpha=0.75;
  ctx.beginPath();
  for(let i=0;i<pts.length;i++){
    const x=X(pts[i][0]),y=Y(pts[i][1]);
    if(i)ctx.lineTo(x,y);else ctx.moveTo(x,y);
  }
  ctx.stroke();
  ctx.globalAlpha=1;

  if(!rng||!spd){
    // Flat fallback. Teal, because with no speed channel this is just "the
    // shape of the lap" — geometry, not a measurement gradient.
    ctx.strokeStyle=_MAP_C.accent;ctx.lineWidth=2.4;
    ctx.beginPath();
    for(let i=0;i<pts.length;i++){
      const x=X(pts[i][0]),y=Y(pts[i][1]);
      if(i)ctx.lineTo(x,y);else ctx.moveTo(x,y);
    }
    ctx.stroke();ctx.restore();return;
  }

  // Per-segment stroke. HIST_LEN caps the buffer at ~1200 samples and this
  // redraws on the 1.2 s history poll, so a stroke per segment is affordable
  // and avoids the seams a gradient-per-chunk approach leaves behind.
  const span=(rng.mx-rng.mn)||1;
  ctx.lineWidth=2.6;
  let lastCol=null;
  for(let i=0;i<pts.length-1;i++){
    const a=spd[i],b=spd[i+1];
    let v=null;
    if(a!=null&&b!=null)v=(a+b)/2;
    else if(a!=null)v=a;
    else if(b!=null)v=b;
    // Samples with no speed keep the previous segment's colour rather than
    // snapping to the cold end — a dropout is not a stop.
    const col=(v==null)?(lastCol||_MAP_C.accent):_mapRamp((v-rng.mn)/span);
    lastCol=col;
    ctx.strokeStyle=col;
    ctx.beginPath();
    ctx.moveTo(X(pts[i][0]),Y(pts[i][1]));
    ctx.lineTo(X(pts[i+1][0]),Y(pts[i+1][1]));
    ctx.stroke();
  }
  ctx.restore();
}

// ── start/finish geofence ──────────────────────────────────────────────────
// Translucent disc, not just an outline: the lap timer fires on ENTERING this
// area, so it is a region, and drawing it as a region stops people reading it
// as a ring you have to thread.
function _mapGeofence(ctx,px,py,rpx){
  if(!isFinite(px)||!isFinite(py)||!isFinite(rpx)||rpx<=0)return;
  ctx.save();
  ctx.fillStyle='rgba(234,240,247,0.045)';
  ctx.beginPath();ctx.arc(px,py,rpx,0,Math.PI*2);ctx.fill();
  ctx.strokeStyle=_MAP_C.rule2;ctx.lineWidth=1;
  ctx.setLineDash([5,5]);
  ctx.beginPath();ctx.arc(px,py,rpx,0,Math.PI*2);ctx.stroke();
  ctx.setLineDash([]);
  ctx.restore();
}

// ── start/finish gate ──────────────────────────────────────────────────────
// A chequered line ACROSS the track, perpendicular to the local direction of
// travel. Chequered black-and-white by racing convention, and specifically
// not green: on this dashboard green means vehicle state, and a fixed
// landmark is not a state.
function _mapGate(ctx,pts,lineXY,lr,sc,X,Y){
  const gx=X(lineXY[0]),gy=Y(lineXY[1]);
  if(!isFinite(gx)||!isFinite(gy))return;

  // Local direction of travel at the line, taken from the nearest recorded
  // track point. Computed in SCREEN space on purpose: X and Y share one
  // isotropic scale factor, so a right angle survives the mapping intact and
  // there is no world->screen sign to get wrong. (The Y flip mirrors
  // handedness, but a perpendicular line has no handedness to lose.)
  let tx=0,ty=-1;                      // default: heading north => gate runs E-W
  if(pts.length>1){
    let best=-1,bd=Infinity;
    for(let i=0;i<pts.length;i++){
      const ddx=pts[i][0]-lineXY[0],ddy=pts[i][1]-lineXY[1];
      const d=ddx*ddx+ddy*ddy;
      if(d<bd){bd=d;best=i;}
    }
    const i0=Math.max(0,best-2),i1=Math.min(pts.length-1,best+2);
    if(i1>i0){
      const sx=X(pts[i1][0])-X(pts[i0][0]);
      const sy=Y(pts[i1][1])-Y(pts[i0][1]);
      const m=Math.sqrt(sx*sx+sy*sy);
      if(m>0.5){tx=sx/m;ty=sy/m;}
    }
  }
  const nx=-ty,ny=tx;                  // 90 deg rotation: the gate itself

  const half=Math.max(14,Math.min(lr*sc*0.85,44));
  const blk=Math.max(3.5,half/5);
  const n=Math.max(4,Math.round(half/blk));

  ctx.save();
  ctx.lineCap='butt';
  // dark backing so white chequers read over a bright trace underneath
  ctx.strokeStyle=_MAP_C.ground;ctx.lineWidth=9;
  ctx.beginPath();
  ctx.moveTo(gx-nx*half,gy-ny*half);ctx.lineTo(gx+nx*half,gy+ny*half);
  ctx.stroke();
  ctx.lineWidth=7;
  for(let i=0;i<n*2;i++){
    const a=-half+(i/(n*2))*half*2;
    const b=-half+((i+1)/(n*2))*half*2;
    ctx.strokeStyle=(i%2===0)?_MAP_C.ink:_MAP_C.panel2;
    ctx.beginPath();
    ctx.moveTo(gx+nx*a,gy+ny*a);ctx.lineTo(gx+nx*b,gy+ny*b);
    ctx.stroke();
  }
  ctx.strokeStyle=_MAP_C.rule2;ctx.lineWidth=1;
  ctx.beginPath();
  ctx.moveTo(gx-nx*half,gy-ny*half);ctx.lineTo(gx+nx*half,gy+ny*half);
  ctx.stroke();
  ctx.restore();

  // Label offset along the direction of travel so it never lands on the gate.
  const lx=gx+tx*14+nx*10,ly=gy+ty*14+ny*10;
  ctx.save();
  ctx.textAlign='left';ctx.textBaseline='middle';
  ctx.font='600 10px '+_MAP_SANS;ctx.fillStyle=_MAP_C.ink;
  _mapSpaced(ctx,'S/F',lx,ly,0.8,'left');
  if(lr*sc>26){
    ctx.font='10px '+_MAP_MONO;ctx.fillStyle=_MAP_C.ink3;
    ctx.fillText(Math.round(lr)+' m',lx,ly+12);
  }
  ctx.restore();
  ctx.textAlign='left';ctx.textBaseline='alphabetic';
}

// ── current position ───────────────────────────────────────────────────────
// Magenta, because this is the MEASURED position of the physical car — the
// same convention the bird's-eye view uses for the actual steering arc.
function _mapHere(ctx,pts,X,Y){
  if(!pts.length)return;
  const last=pts[pts.length-1];
  const px=X(last[0]),py=Y(last[1]);
  if(!isFinite(px)||!isFinite(py))return;

  // Soft glow. A radial gradient rather than shadowBlur: shadow state leaks
  // into later strokes if an exception unwinds before restore().
  // The catch here is LOCAL — a failed glow must not lose the marker — which
  // means the outer unwind in drawMap never sees the throw. So the restore has
  // to be in a finally: without it a throw between save() and restore() leaks
  // one stack entry per frame, forever, on a context that lives as long as the
  // page. `saved` gates it so we never restore a save we did not make.
  const gr=22;
  let saved=false;
  try{
    const g=ctx.createRadialGradient(px,py,0,px,py,gr);
    g.addColorStop(0,'rgba(255,92,168,0.34)');
    g.addColorStop(0.45,'rgba(255,92,168,0.13)');
    g.addColorStop(1,'rgba(255,92,168,0)');
    ctx.save();saved=true;ctx.fillStyle=g;
    ctx.beginPath();ctx.arc(px,py,gr,0,Math.PI*2);ctx.fill();
  }catch(e){}
  finally{ if(saved){ try{ ctx.restore(); }catch(e2){} } }

  // Heading from the last two DISTINCT screen points. Walking backwards past
  // near-duplicates matters: standing still, consecutive fixes differ only by
  // GPS jitter and a chevron built from those spins on the spot, which looks
  // exactly like the car yawing. Below the threshold we refuse to claim a
  // heading at all and fall back to a plain marker.
  let ang=null;
  for(let i=pts.length-2;i>=0;i--){
    const sx=px-X(pts[i][0]),sy=py-Y(pts[i][1]);
    if(sx*sx+sy*sy>=9){ang=Math.atan2(sy,sx);break;}   // >= 3 px of travel
  }

  ctx.save();
  if(ang==null){
    // No trustworthy heading: a ring, not an arrow pointing somewhere made up.
    ctx.fillStyle=_MAP_C.actual;
    ctx.beginPath();ctx.arc(px,py,4,0,Math.PI*2);ctx.fill();
    ctx.strokeStyle=_MAP_C.ground;ctx.lineWidth=1.5;ctx.stroke();
    ctx.strokeStyle='rgba(255,92,168,0.55)';ctx.lineWidth=1;
    ctx.beginPath();ctx.arc(px,py,8.5,0,Math.PI*2);ctx.stroke();
  }else{
    // ang is a SCREEN angle taken straight from screen-space deltas, so
    // ctx.rotate(ang) aims the chevron's +x nose along the direction of
    // travel with no world->screen sign correction anywhere. Deriving it in
    // world coordinates instead would need a manual y negation to cancel the
    // one in Y(), and that is precisely the sign that gets flipped by
    // accident.
    const sz=9;
    ctx.translate(px,py);ctx.rotate(ang);
    ctx.beginPath();
    ctx.moveTo(sz,0);
    ctx.lineTo(-sz*0.78, sz*0.70);
    ctx.lineTo(-sz*0.34,0);
    ctx.lineTo(-sz*0.78,-sz*0.70);
    ctx.closePath();
    ctx.fillStyle=_MAP_C.actual;ctx.fill();
    ctx.strokeStyle=_MAP_C.ground;ctx.lineWidth=1.4;ctx.lineJoin='round';ctx.stroke();
  }
  ctx.restore();
}

// ── scale bar ──────────────────────────────────────────────────────────────
// Replaces the original bare "~N m wide" hint with a measurable bar, but the
// extent note is kept next to it: the bar answers "how far is that", the note
// answers "how big is the whole view", and people used the old number.
function _mapScaleBar(ctx,w,h,sc,dx){
  let m=_mapNiceStep(110/sc);
  if(!isFinite(m)||m<=0)return;
  // Step down until the bar fits comfortably; never let it eat the canvas.
  let guard=0;
  while(m*sc>w*0.42&&guard<12){
    const p=Math.pow(10,Math.floor(Math.log10(m)));
    const n=Math.round(m/p);
    m=(n===1)?p/2:(n===2?p:(n===5?p*2:p*5));
    guard++;
  }
  const px=m*sc;
  if(!isFinite(px)||px<8)return;
  const x0=12,y=h-14;
  ctx.save();
  ctx.strokeStyle=_MAP_C.ink3;ctx.lineWidth=1.5;ctx.lineCap='butt';
  ctx.beginPath();
  ctx.moveTo(x0,y);ctx.lineTo(x0+px,y);
  ctx.moveTo(x0,y-4);ctx.lineTo(x0,y+4);
  ctx.moveTo(x0+px,y-4);ctx.lineTo(x0+px,y+4);
  ctx.stroke();
  // half-way tick, so the bar can be read at a glance without halving in head
  ctx.globalAlpha=0.6;
  ctx.beginPath();ctx.moveTo(x0+px/2,y-2.5);ctx.lineTo(x0+px/2,y+2.5);ctx.stroke();
  ctx.globalAlpha=1;
  ctx.restore();

  ctx.textAlign='left';ctx.textBaseline='alphabetic';
  ctx.font='650 10px '+_MAP_MONO;ctx.fillStyle=_MAP_C.ink2;
  ctx.fillText(_mapMetres(m),x0,y-8);
  ctx.font='10px '+_MAP_MONO;ctx.fillStyle=_MAP_C.ink3;
  ctx.fillText('~'+Math.round(dx)+' m wide',x0+px+10,y+3);
}

function _mapMetres(m){
  if(m>=1000)return (m/1000)+' km';
  if(m>=1)return Math.round(m)+' m';
  // Sub-metre bars are reachable: an RTK fix on a parked car jitters a couple
  // of centimetres, the extent collapses to that, and a fixed one decimal
  // prints the bar as "0.0 m" — a scale bar that states the scale is zero.
  // _mapNiceStep only ever returns 1/2/5 x 10^k, so the decade sets exactly
  // how many decimals are needed and no more.
  const d=Math.min(6,Math.max(1,-Math.floor(Math.log10(m))));
  return m.toFixed(d)+' m';
}

// ── north indicator ────────────────────────────────────────────────────────
// The projection is axis-aligned to lat/lon and this view is NEVER rotated to
// vehicle heading, so north is up by construction. The arrow is a reminder of
// that fact, not a compass that moves.
function _mapNorth(ctx,w){
  const x=w-20,y=14;
  ctx.save();
  ctx.fillStyle=_MAP_C.ink2;
  ctx.beginPath();
  ctx.moveTo(x,y);
  ctx.lineTo(x+4.5,y+11);
  ctx.lineTo(x,y+8.5);
  ctx.lineTo(x-4.5,y+11);
  ctx.closePath();ctx.fill();
  ctx.restore();
  ctx.textAlign='center';ctx.textBaseline='alphabetic';
  ctx.font='600 10px '+_MAP_SANS;ctx.fillStyle=_MAP_C.ink3;
  ctx.fillText('N',x,y+22);
  ctx.textAlign='left';
}

// ── speed ramp key ─────────────────────────────────────────────────────────
// Endpoints are printed as bare numbers with no unit. /api/history reports
// km/h but the replay endpoint reports m/s through the same shape, and
// asserting the wrong unit on a race dashboard is worse than asserting none.
// The ramp is normalised over THIS WINDOW, so colours compare within one view
// and not between two — the printed endpoints are what make that legible.
function _mapRampKey(ctx,rng){
  const x=12,y=12,bw=76,bh=6;
  ctx.save();
  for(let i=0;i<bw;i++){
    ctx.fillStyle=_mapRamp(i/(bw-1));
    ctx.fillRect(x+i,y,1.5,bh);
  }
  ctx.strokeStyle=_MAP_C.rule2;ctx.lineWidth=1;
  ctx.strokeRect(x+0.5,y+0.5,bw,bh);
  ctx.restore();
  ctx.textBaseline='alphabetic';
  ctx.font='10px '+_MAP_MONO;ctx.fillStyle=_MAP_C.ink3;
  ctx.textAlign='left';
  ctx.fillText(_mapNum(rng.mn),x,y+bh+11);
  ctx.textAlign='right';
  ctx.fillText(_mapNum(rng.mx),x+bw,y+bh+11);
  ctx.textAlign='left';
  ctx.font='600 10px '+_MAP_SANS;ctx.fillStyle=_MAP_C.ink3;
  _mapSpaced(ctx,'SPEED',x+bw+10,y+bh,0.9,'left');
}

function _mapNum(v){
  if(v==null||!isFinite(v))return '—';
  // Keep the two endpoint labels short enough that they cannot collide under
  // the 76 px ramp strip, whatever unit the caller happened to pass.
  // 'k' alone is not enough: a garbage speed channel reading 1e9 renders as
  // "1000000k", which is 8 glyphs and walks straight over the other endpoint.
  const a=Math.abs(v);
  if(a>=1e7)return Math.round(v/1e6)+'M';
  if(a>=1e4)return Math.round(v/1e3)+'k';
  return (a>=100)?String(Math.round(v)):v.toFixed(1);
}

// Chrome 90's canvas 2d has no ctx.letterSpacing, so tracked-out uppercase
// labels have to be laid out a glyph at a time.
function _mapSpaced(ctx,txt,x,y,sp,align){
  const chars=String(txt).split('');
  let total=0;
  for(let i=0;i<chars.length;i++)total+=ctx.measureText(chars[i]).width+sp;
  total-=sp;
  let px=x;
  if(align==='center')px=x-total/2;
  else if(align==='right')px=x-total;
  const prev=ctx.textAlign;
  ctx.textAlign='left';
  for(let i=0;i<chars.length;i++){
    ctx.fillText(chars[i],px,y);
    px+=ctx.measureText(chars[i]).width+sp;
  }
  ctx.textAlign=prev;
}

// ── gfx_bird.js ──
// ── bird's-eye (robot frame) ──────────────────────────────────────────────
// The hero graphic: everything the car can see and everything it intends to
// do, in one frame, forward = up. Two curves matter and must never be
// confused — the TEAL planner path (/ethon/path, where the planner WANTS to
// go) and the MAGENTA prediction (where the car ACTUALLY goes if you hold the
// current lock). Comparing them is the whole point of this panel, and it
// works with no cameras at all: turn the wheel on the bench and the magenta
// ribbon sweeps.
//
// Pure render. No fetch, no timers, no global mutation, no element writes —
// it touches the one canvas it is handed and nothing else. Every field of
// every argument is assumed to be missing, null or garbage until proven
// otherwise: the car boots with no GPS fix, no cones and unhomed steering,
// and that state has to render legibly rather than throw.

function _birdFont(px, weight, mono){
  // No webfonts are available on the car, so both stacks are pure system.
  var sans = 'system-ui,-apple-system,"Segoe UI",Roboto,"Helvetica Neue",Arial,sans-serif';
  var mn   = 'ui-monospace,"SF Mono","Cascadia Mono",Menlo,Consolas,"DejaVu Sans Mono",monospace';
  return (weight ? (weight + ' ') : '') + px + 'px ' + (mono ? mn : sans);
}

// Accepts any of the shapes this data arrives in. /api/history pre-flattens
// cones and path to [x,y] pairs, but /ethon/obstacles and /ethon/curb_points
// are raw PoseArrays, and whichever endpoint ends up feeding them may hand us
// {x,y}, {position:{x,y}} or a Path-style {pose:{position:{x,y}}}. Anything
// that is not a finite pair of numbers is dropped rather than drawn at NaN,
// which silently poisons a whole canvas path.
function _birdXY(p){
  if(p == null) return null;
  var x, y;
  if(Object.prototype.toString.call(p) === '[object Array]'){
    x = p[0]; y = p[1];
  }else if(typeof p === 'object'){
    var q = p;
    if(q.pose && typeof q.pose === 'object') q = q.pose;
    if(q.position && typeof q.position === 'object') q = q.position;
    x = q.x; y = q.y;
  }else{
    return null;
  }
  x = +x; y = +y;
  if(!isFinite(x) || !isFinite(y)) return null;
  return [x, y];
}

// Normalise a whole collection: array, or a message-ish object that still has
// its poses/points/pts wrapper on it.
function _birdList(v){
  var arr = v;
  if(arr && typeof arr === 'object' && Object.prototype.toString.call(arr) !== '[object Array]'){
    arr = arr.poses || arr.points || arr.pts || arr.data || null;
  }
  if(Object.prototype.toString.call(arr) !== '[object Array]') return [];
  var out = [];
  for(var i = 0; i < arr.length; i++){
    var q = _birdXY(arr[i]);
    if(q) out.push(q);
  }
  return out;
}

// Any motion in here is derived from Date.now() at draw time, never from a
// timer — this module is not allowed to schedule anything. Failing closed
// (treat as "reduce") means a browser without matchMedia gets a still frame.
function _birdReduced(){
  try{
    return !!(window.matchMedia && window.matchMedia('(prefers-reduced-motion: reduce)').matches);
  }catch(e){
    return true;
  }
}

// Manual letter spacing: ctx.letterSpacing only landed in Chrome 99 and the
// pit laptop is older than that, so tracked caps are drawn a glyph at a time.
function _birdTrackW(ctx, txt, sp){
  var wsum = 0;
  for(var i = 0; i < txt.length; i++) wsum += ctx.measureText(txt.charAt(i)).width + sp;
  return wsum > 0 ? wsum - sp : 0;
}
function _birdTrack(ctx, txt, x, y, sp){
  var cur = x;
  for(var i = 0; i < txt.length; i++){
    var ch = txt.charAt(i);
    ctx.fillText(ch, cur, y);
    cur += ctx.measureText(ch).width + sp;
  }
  return cur - x - sp;
}

// Ring spacing stays at the original 5 m for every realistic view (R <= 40 m).
// The step only opens up if a bogus far-field point blows the extent out —
// without this the 5 m loop would try to stroke tens of thousands of circles
// and lock the tab, which is a worse failure than a coarse grid.
function _birdRingStep(R){
  var steps = [5, 10, 20, 25, 50, 100, 200, 500, 1000];
  for(var i = 0; i < steps.length; i++){ if(R / steps[i] <= 8) return steps[i]; }
  // Past the end of the table only garbage telemetry can reach, and returning
  // the last entry there does NOT bound anything: one cone at 1e12 m still
  // asks for a billion 1000 m rings and hard-locks the tab, which is the exact
  // failure this function exists to prevent. Fall back to a computed decade so
  // the ring count stays <= 10 for any finite R.
  var s = Math.pow(10, Math.ceil(Math.log(R / 8) / Math.LN10));
  return (isFinite(s) && s > 0) ? s : R;
}

// Catmull-Rom through the raw waypoints, expressed as cubic beziers. The
// planner emits a sparse polyline; drawing it as straight segments makes a
// smooth plan look like it has corners in it, which reads as planner chatter
// when there is none. The curve passes through every original point, so no
// waypoint is misrepresented.
function _birdSpline(ctx, p){
  ctx.moveTo(p[0][0], p[0][1]);
  if(p.length === 2){ ctx.lineTo(p[1][0], p[1][1]); return; }
  for(var i = 0; i < p.length - 1; i++){
    var p0 = p[i > 0 ? i - 1 : 0];
    var p1 = p[i];
    var p2 = p[i + 1];
    var p3 = p[i + 2 < p.length ? i + 2 : p.length - 1];
    ctx.bezierCurveTo(p1[0] + (p2[0] - p0[0]) / 6, p1[1] + (p2[1] - p0[1]) / 6,
                      p2[0] - (p3[0] - p1[0]) / 6, p2[1] - (p3[1] - p1[1]) / 6,
                      p2[0], p2[1]);
  }
}

// Turn a screen-space centreline into a closed tapered outline. Width falls
// off with distance because confidence does: the near end of the prediction
// is a measurement, the far end is an extrapolation of a lock the driver has
// not held yet.
function _birdRibbon(pts, w0, w1){
  var left = [], right = [];
  for(var i = 0; i < pts.length; i++){
    var a = pts[i > 0 ? i - 1 : 0];
    var b = pts[i + 1 < pts.length ? i + 1 : pts.length - 1];
    var dx = b[0] - a[0], dy = b[1] - a[1];
    var len = Math.sqrt(dx * dx + dy * dy);
    if(!(len > 0)){ dx = 0; dy = -1; len = 1; }
    var nx = -dy / len, ny = dx / len;
    var t = (pts.length < 2) ? 0 : i / (pts.length - 1);
    var hw = (w0 + (w1 - w0) * t) / 2;
    left.push([pts[i][0] + nx * hw, pts[i][1] + ny * hw]);
    right.push([pts[i][0] - nx * hw, pts[i][1] - ny * hw]);
  }
  return left.concat(right.reverse());
}

function _birdPoly(ctx, pts){
  ctx.beginPath();
  for(var i = 0; i < pts.length; i++){
    if(i) ctx.lineTo(pts[i][0], pts[i][1]); else ctx.moveTo(pts[i][0], pts[i][1]);
  }
  ctx.closePath();
}

// Ego seen from above, drawn to the real wheelbase so that the car's own
// footprint is directly comparable to the cone spacing around it. Coordinates
// are metres in the vehicle frame and go through the same +x=forward,
// +y=left projection as every other datum on this canvas.
function _birdCar(ctx, cx, cy, csc, L, rwDeg, live){
  // This helper is handed someone else's context and must give it back exactly
  // as it found it. Without this fence it returned with fillStyle, strokeStyle
  // and lineWidth still set to the ego's own values.
  ctx.save();
  try{
  var P = function(x, y){ return [cx - y * csc, cy - x * csc]; };
  var halfTrack = 0.36 * L;      // ~1.10 m track at the 1.524 m wheelbase
  var frontOH   = 0.28 * L;
  var rearOH    = 0.24 * L;
  var wheelHalf = 0.14 * L;      // tyre radius seen from above
  var wheelW    = 0.055 * L;
  var xf = L / 2 + frontOH, xr = -(L / 2 + rearOH);

  // Ground shadow first so the body sits on the plane rather than floating.
  ctx.save();
  ctx.globalAlpha = 0.55;
  ctx.fillStyle = '#090B0F';
  var sh = P(0, 0);
  ctx.beginPath();
  ctx.ellipse(sh[0], sh[1] + 2, halfTrack * csc * 1.15, (xf - xr) * csc * 0.55, 0, 0, Math.PI * 2);
  ctx.fill();
  ctx.restore();

  // Wheels. Rears are fixed to the chassis; only the fronts carry an angle.
  var rearC = '#3A4657';
  var frontC = live ? '#FF5CA8' : '#64748B';   // magenta = actually measured
  // Copied verbatim from the road-wheel diagram in drawSteer(), which is the
  // bench ground truth for this car:
  //   const wr=(deg==null)?0:(-deg*Math.PI/180);  // +deg = right = clockwise
  // The dated 2026-08-16 note in this file resolves what that line means in
  // practice: after the steer_inverted sign is applied, positive is LEFT, and
  // the negation here is what makes canvas' clockwise-positive rotate() agree
  // with the physical wheel. Do not fold the minus away.
  var wr = (rwDeg == null) ? 0 : (-rwDeg * Math.PI / 180);
  var side, wc, i;
  for(i = 0; i < 2; i++){
    side = i ? 1 : -1;
    wc = P(-L / 2, side * halfTrack);
    ctx.fillStyle = rearC;
    ctx.fillRect(wc[0] - wheelW * csc, wc[1] - wheelHalf * csc,
                 wheelW * 2 * csc, wheelHalf * 2 * csc);
  }
  for(i = 0; i < 2; i++){
    side = i ? 1 : -1;
    wc = P(L / 2, side * halfTrack);
    ctx.save();
    ctx.translate(wc[0], wc[1]);
    ctx.rotate(wr);
    ctx.fillStyle = frontC;
    ctx.fillRect(-wheelW * csc, -wheelHalf * csc, wheelW * 2 * csc, wheelHalf * 2 * csc);
    ctx.restore();
  }

  // Monocoque: widest at the cockpit, tapering to a narrow nose so that
  // "which way is forward" survives even at small scale.
  var a = P(xr,  0.30 * L), b = P(-0.10 * L,  0.34 * L);
  var c = P(xf * 0.75,  0.30 * L), d = P(xf,  0.10 * L);
  var e = P(xf, -0.10 * L), f = P(xf * 0.75, -0.30 * L);
  var g = P(-0.10 * L, -0.34 * L), hh = P(xr, -0.30 * L);
  ctx.beginPath();
  ctx.moveTo(a[0], a[1]);
  ctx.lineTo(b[0], b[1]);
  ctx.quadraticCurveTo(c[0], c[1], d[0], d[1]);
  ctx.lineTo(e[0], e[1]);
  ctx.quadraticCurveTo(f[0], f[1], g[0], g[1]);
  ctx.lineTo(hh[0], hh[1]);
  ctx.closePath();
  ctx.fillStyle = '#182030';
  ctx.fill();
  ctx.lineWidth = 1.2;
  ctx.strokeStyle = live ? '#2ED47A' : '#64748B';   // green = vehicle STATE
  ctx.stroke();

  // Cockpit opening, purely so the silhouette reads as a car and not a slab.
  var k0 = P(0.16 * L, 0.16 * L), k1 = P(-0.26 * L, -0.16 * L);
  ctx.fillStyle = '#0E1218';
  ctx.beginPath();
  ctx.rect(Math.min(k0[0], k1[0]), Math.min(k0[1], k1[1]),
           Math.abs(k1[0] - k0[0]), Math.abs(k1[1] - k0[1]));
  ctx.fill();
  }finally{
    ctx.restore();
  }
}

function drawBird(id, cones, path, drive, obstacles, curb){
  var s = setupCanvas(id);
  if(!s) return;
  var ctx = s.ctx, w = s.w, h = s.h;
  // One bad frame of telemetry must not be able to take the page down, so the
  // whole body is fenced. The canvas is cleared first: a half-drawn frame is
  // more dangerous than a blank one, because it looks authoritative.
  try{
    ctx.clearRect(0, 0, w, h);
    ctx.fillStyle = '#131923';
    ctx.fillRect(0, 0, w, h);
    if(!(w > 40) || !(h > 40)) return;

    var C  = _birdList(cones);
    var PA = _birdList(path);
    var OB = _birdList(obstacles);
    var CB = _birdList(curb);

    var cx = w / 2, cy = h / 2, pad = 18;
    // Extent, exactly as before, plus obstacles — those are near-field hazards
    // of the same class as cones and must never be cropped out of frame.
    // Curb points are deliberately NOT included: a kerb line can run tens of
    // metres up the track and would zoom the whole scene out to nothing.
    // Off-frame curb points simply clip at the canvas edge.
    var m = 8, i, p;
    for(i = 0; i < C.length;  i++) m = Math.max(m, Math.abs(C[i][0]),  Math.abs(C[i][1]));
    for(i = 0; i < PA.length; i++) m = Math.max(m, Math.abs(PA[i][0]), Math.abs(PA[i][1]));
    for(i = 0; i < OB.length; i++) m = Math.max(m, Math.abs(OB[i][0]), Math.abs(OB[i][1]));
    var R = Math.max(10, m * 1.1);
    // m is finite (_birdXY guarantees it) but m*1.1 can still overflow to
    // Infinity near the top of the double range, and an infinite R poisons sc,
    // Rpx and every ring below it.
    if(!isFinite(R)) R = m;
    var sc = (Math.min(w, h) / 2 - pad) / R;
    if(!isFinite(sc) || sc <= 0) sc = 1;
    // forward(+x)=up, left(+y)=left — the frame every other sign in this
    // function is expressed in.
    var PX = function(x, y){ return cx - y * sc; };
    var PY = function(x, y){ return cy - x * sc; };
    var Rpx = R * sc;

    // ── ground plane ────────────────────────────────────────────────────
    // A soft pool of light under the car: the eye reads depth from it, and it
    // separates the near field (where perception is trustworthy) from the far
    // field (where it is not).
    var glowR = Math.max(1, Math.min(w, h) * 0.62);
    var gp = ctx.createRadialGradient(cx, cy, 1, cx, cy, glowR);
    gp.addColorStop(0, '#182030');
    gp.addColorStop(1, '#131923');
    ctx.save();                          // don't leave a gradient object as fillStyle
    ctx.fillStyle = gp;
    ctx.fillRect(0, 0, w, h);
    ctx.restore();

    // Radial spokes, very faint — they give the rings a sense of rotation
    // without competing with any datum.
    ctx.save();
    ctx.strokeStyle = '#212B3A';
    ctx.globalAlpha = 0.45;
    ctx.lineWidth = 1;
    for(i = 0; i < 12; i++){
      var ang = i * Math.PI / 6;
      ctx.beginPath();
      ctx.moveTo(cx + Math.cos(ang) * Rpx * 0.14, cy + Math.sin(ang) * Rpx * 0.14);
      ctx.lineTo(cx + Math.cos(ang) * Rpx, cy + Math.sin(ang) * Rpx);
      ctx.stroke();
    }
    ctx.restore();

    // Range rings, fading with distance.
    var step = _birdRingStep(R);
    ctx.save();
    ctx.lineWidth = 1;
    for(var r = step; r <= R; r += step){
      var t = Math.min(1, r / R);
      ctx.globalAlpha = 0.55 - 0.34 * t;
      ctx.strokeStyle = (r === step) ? '#2C3949' : '#212B3A';
      ctx.beginPath();
      ctx.arc(cx, cy, r * sc, 0, Math.PI * 2);
      ctx.stroke();
    }
    ctx.restore();

    // Crosshair axes.
    ctx.save();
    ctx.strokeStyle = '#2C3949';
    ctx.globalAlpha = 0.7;
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(cx, pad); ctx.lineTo(cx, h - pad);
    ctx.moveTo(pad, cy); ctx.lineTo(w - pad, cy);
    ctx.stroke();
    ctx.restore();

    // Range labels ride the rear-right diagonal. The original stacked them up
    // the +forward axis, which is exactly where the path and the prediction
    // arc live — they collided constantly. Behind-and-right is the one
    // quadrant that is reliably empty on a forward-looking car.
    ctx.save();
    ctx.font = _birdFont(10, 600, true);
    ctx.textAlign = 'left';
    ctx.textBaseline = 'middle';
    var dg = Math.SQRT1_2;
    for(r = step; r <= R; r += step){
      var lx = cx + r * sc * dg + 4, ly = cy + r * sc * dg + 4;
      if(lx > w - 6 || ly > h - 6) continue;
      var txt = r + ' m';
      var tw = ctx.measureText(txt).width;
      ctx.globalAlpha = 0.85;
      ctx.fillStyle = '#131923';
      ctx.fillRect(lx - 2, ly - 7, tw + 4, 14);
      ctx.globalAlpha = 1;
      ctx.fillStyle = '#64748B';
      ctx.fillText(txt, lx, ly);
    }
    ctx.restore();

    // ── steering geometry ───────────────────────────────────────────────
    var d = drive || {};
    var wb = +d.wheelbase_m;
    // wheelbase_m gates BOTH arcs and the ego, and the original `+x||0` idiom
    // let two bad values through. Infinity stays truthy and turns the whole
    // silhouette into NaN geometry that silently disappears; a NEGATIVE
    // wheelbase is worse -- arcPts' R = wb/tan(delta) flips sign, so the
    // magenta prediction and the full-lock envelope render MIRRORED while the
    // ego (which tests wb>0 separately and falls back to 1.524) still draws
    // its front wheels the right way round. Reject anything outside a
    // physically plausible band instead of trusting the truthiness test.
    if(!(isFinite(wb) && wb > 0.1 && wb < 20)) wb = 0;
    // rw is in the SAME convention as the wheel panel: positive = LEFT,
    // matching the vehicle frame used below (+y is LEFT) -- verified
    // 2026-08-16 against a live bench-test command, no negation needed.
    var dsg = (d.steer_inverted === true) ? 1 : -1;
    // Verified 2026-08-16: dsg*road_wheel_deg is positive = LEFT (not
    // RIGHT as previously assumed below), matching arcPts' own convention
    // (positive input -> positive y -> LEFT in the +y=left frame). No sign
    // flip is needed at the call site any more -- see arcPts(rw,...) below.
    var rw = (d.road_wheel_deg == null) ? null : dsg * (+d.road_wheel_deg);
    if(rw != null && !isFinite(rw)) rw = null;
    var rwMax = (d.road_wheel_max_deg == null) ? null : Math.abs(+d.road_wheel_max_deg);
    if(rwMax != null && !isFinite(rwMax)) rwMax = null;
    var homed = !!d.steer_homed;
    var reach = Math.min(R * 0.95, 12);

    var toScreen = function(pts){
      var out = [];
      for(var j = 0; j < pts.length; j++){
        var q = _birdXY(pts[j]);
        if(q) out.push([PX(q[0], q[1]), PY(q[0], q[1])]);
      }
      return out;
    };

    // ── full-lock envelope ──────────────────────────────────────────────
    // A filled wedge between the two extreme arcs instead of two dashed
    // lines: "everything I can reach without reversing" is an area, and it
    // reads as one instantly. Neutral structural grey on purpose — this is a
    // capability of the chassis, not something commanded (teal) or measured
    // (magenta), and the semantic colours must not be diluted.
    if(wb && rwMax){
      var envL = toScreen(arcPts( rwMax, wb, reach));   // +deg = left
      var envR = toScreen(arcPts(-rwMax, wb, reach));
      if(envL.length > 1 && envR.length > 1){
        ctx.save();
        _birdPoly(ctx, envL.concat(envR.slice().reverse()));
        ctx.fillStyle = '#2C3949';
        ctx.globalAlpha = 0.34;
        ctx.fill();
        ctx.globalAlpha = 0.8;
        ctx.lineWidth = 1;
        ctx.strokeStyle = '#2C3949';
        ctx.stroke();
        ctx.restore();
      }
    }

    // ── curb points ─────────────────────────────────────────────────────
    // /ethon/curb_points has been published all along and was never drawn.
    // Rendered as a dotted boundary; consecutive points are only joined when
    // they are close enough to plausibly be the same kerb, otherwise two
    // separate kerbs get bridged by a line that does not exist.
    if(CB.length){
      ctx.save();
      ctx.strokeStyle = '#64748B';
      ctx.fillStyle = '#64748B';
      ctx.globalAlpha = 0.62;
      ctx.lineWidth = 1;
      ctx.setLineDash([2, 3]);
      ctx.beginPath();
      for(i = 1; i < CB.length; i++){
        var ax = CB[i - 1][0] - CB[i][0], ay = CB[i - 1][1] - CB[i][1];
        if(Math.sqrt(ax * ax + ay * ay) > 1.5) continue;   // metres
        ctx.moveTo(PX(CB[i - 1][0], CB[i - 1][1]), PY(CB[i - 1][0], CB[i - 1][1]));
        ctx.lineTo(PX(CB[i][0], CB[i][1]), PY(CB[i][0], CB[i][1]));
      }
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.globalAlpha = 0.85;
      for(i = 0; i < CB.length; i++){
        ctx.beginPath();
        ctx.arc(PX(CB[i][0], CB[i][1]), PY(CB[i][0], CB[i][1]), 1.2, 0, Math.PI * 2);
        ctx.fill();
      }
      ctx.restore();
    }

    // ── planner path ────────────────────────────────────────────────────
    // Teal = commanded / planned. The gradient fades with range so the near
    // end — the part the car is about to execute — carries the most weight.
    var scr = toScreen(PA);
    if(scr.length > 1){
      var last = scr[scr.length - 1];
      var grad = null;
      if(Math.abs(last[0] - cx) > 0.5 || Math.abs(last[1] - cy) > 0.5){
        grad = ctx.createLinearGradient(cx, cy, last[0], last[1]);
        grad.addColorStop(0, 'rgba(76,224,210,1)');
        grad.addColorStop(0.6, 'rgba(76,224,210,0.75)');
        grad.addColorStop(1, 'rgba(76,224,210,0.22)');
      }
      ctx.save();
      ctx.lineCap = 'round';
      ctx.lineJoin = 'round';
      ctx.strokeStyle = '#4CE0D2';
      ctx.globalAlpha = 0.16;
      ctx.lineWidth = 7;
      ctx.beginPath(); _birdSpline(ctx, scr); ctx.stroke();
      ctx.globalAlpha = 1;
      ctx.strokeStyle = grad || '#4CE0D2';
      ctx.lineWidth = 2.4;
      ctx.beginPath(); _birdSpline(ctx, scr); ctx.stroke();
      ctx.restore();
    }

    // ── predicted trajectory from the live steering angle ───────────────
    // Still gated on steer_homed: an unhomed column has no trustworthy angle,
    // and a confident-looking magenta ribbon drawn from a garbage angle is
    // exactly the kind of thing that gets a wrong call made at the wall.
    var predScr = null;
    if(wb && rw != null && homed){
      predScr = toScreen(arcPts(rw, wb, reach));   // rw already +y-left convention
    }
    if(predScr && predScr.length > 1){
      var ribbon = _birdRibbon(predScr, 7.5, 1.6);
      ctx.save();
      ctx.shadowColor = 'rgba(255,92,168,0.55)';
      ctx.shadowBlur = 14;
      var pg = ctx.createLinearGradient(predScr[0][0], predScr[0][1],
                                        predScr[predScr.length - 1][0],
                                        predScr[predScr.length - 1][1]);
      pg.addColorStop(0, 'rgba(255,92,168,0.95)');
      pg.addColorStop(1, 'rgba(255,92,168,0.30)');
      _birdPoly(ctx, ribbon);
      ctx.fillStyle = pg;
      ctx.fill();
      ctx.restore();
    }

    // ── cones ───────────────────────────────────────────────────────────
    // Safety orange, literal traffic-cone colour. Shadow then glow then disc,
    // so a cone stays findable where it overlaps the path ribbon.
    // One fence covers cones, obstacles, the ego and the scrims: unlike every
    // block above, those set fillStyle / strokeStyle / lineWidth outside any
    // save(), and drawBird was returning with them still dirty.
    ctx.save();
    for(i = 0; i < C.length; i++){
      var ccx = PX(C[i][0], C[i][1]), ccy = PY(C[i][0], C[i][1]);
      ctx.save();
      ctx.globalAlpha = 0.5;
      ctx.fillStyle = '#090B0F';
      ctx.beginPath();
      ctx.ellipse(ccx, ccy + 3, 5, 2.4, 0, 0, Math.PI * 2);
      ctx.fill();
      ctx.restore();
      var cg = ctx.createRadialGradient(ccx, ccy, 1, ccx, ccy, 11);
      cg.addColorStop(0, 'rgba(255,138,61,0.42)');
      cg.addColorStop(1, 'rgba(255,138,61,0)');
      ctx.fillStyle = cg;
      ctx.beginPath(); ctx.arc(ccx, ccy, 11, 0, Math.PI * 2); ctx.fill();
      ctx.fillStyle = '#FF8A3D';
      ctx.beginPath(); ctx.arc(ccx, ccy, 4.2, 0, Math.PI * 2); ctx.fill();
      ctx.strokeStyle = 'rgba(255,209,168,0.9)';
      ctx.lineWidth = 1;
      ctx.beginPath(); ctx.arc(ccx, ccy, 4.2, 0, Math.PI * 2); ctx.stroke();
    }

    // ── obstacles ───────────────────────────────────────────────────────
    // /ethon/obstacles was likewise invisible until now. Red outline = crit,
    // which here is genuinely a vehicle-state severity: something is in the
    // way. The halo breathes on a ~4.4 s cycle purely from the wall clock at
    // draw time (no timer is created), and holds still under reduced motion.
    if(OB.length){
      var pulse = _birdReduced() ? 0.5 : (0.5 + 0.5 * Math.sin(Date.now() / 700));
      for(i = 0; i < OB.length; i++){
        var ox = PX(OB[i][0], OB[i][1]), oy = PY(OB[i][0], OB[i][1]);
        ctx.save();
        ctx.globalAlpha = 0.22 + 0.20 * pulse;
        ctx.fillStyle = '#FF4D4F';
        ctx.beginPath(); ctx.arc(ox, oy, 10, 0, Math.PI * 2); ctx.fill();
        ctx.restore();
        ctx.strokeStyle = '#FF4D4F';
        ctx.lineWidth = 1.6;
        ctx.beginPath(); ctx.arc(ox, oy, 6, 0, Math.PI * 2); ctx.stroke();
        ctx.beginPath();
        ctx.moveTo(ox - 3, oy); ctx.lineTo(ox + 3, oy);
        ctx.moveTo(ox, oy - 3); ctx.lineTo(ox, oy + 3);
        ctx.stroke();
      }
    }

    // ── ego ─────────────────────────────────────────────────────────────
    // 1.524 m is the fallback wheelbase; drive.wheelbase_m is authoritative
    // when the drive node is publishing.
    var wbDraw = (wb > 0) ? wb : 1.524;
    // Drawn to true scale wherever it is large enough to read. Below ~16 px
    // of wheelbase the silhouette turns to mush, so the ego is clamped to a
    // minimum size — at that point it is a locator, not a clearance gauge.
    var csc = Math.max(sc, 16 / wbDraw);
    // base_link's longitudinal origin is not published, so the wheelbase is
    // drawn straddling the origin. The prediction arc still starts at the
    // origin exactly as it always did — nothing about arcPts moved.
    _birdCar(ctx, cx, cy, csc, wbDraw, (rw != null && homed) ? rw : null,
             (rw != null && homed));

    // ── scrims so the HUD stays legible over data ───────────────────────
    var topScrim = ctx.createLinearGradient(0, 0, 0, 30);
    topScrim.addColorStop(0, 'rgba(9,11,15,0.85)');
    topScrim.addColorStop(1, 'rgba(9,11,15,0)');
    ctx.fillStyle = topScrim;
    ctx.fillRect(0, 0, w, 30);
    var botScrim = ctx.createLinearGradient(0, h - 34, 0, h);
    botScrim.addColorStop(0, 'rgba(9,11,15,0)');
    botScrim.addColorStop(1, 'rgba(9,11,15,0.9)');
    ctx.fillStyle = botScrim;
    ctx.fillRect(0, h - 34, w, 34);
    ctx.restore();                       // closes the cones/obstacles/ego/scrim fence

    // ── HUD: heading cue + counts ───────────────────────────────────────
    ctx.save();
    ctx.textBaseline = 'alphabetic';
    ctx.font = _birdFont(10, 600, false);
    ctx.textAlign = 'left';
    ctx.fillStyle = '#9AA9BC';
    _birdTrack(ctx, '▲ FORWARD', 10, 16, 0.6);

    ctx.font = _birdFont(10.5, 700, true);
    ctx.textAlign = 'right';
    ctx.fillStyle = '#9AA9BC';
    var counts = 'CONES ' + C.length + ' · PATH ' + PA.length + ' PTS';
    if(OB.length) counts += ' · OBST ' + OB.length;
    if(CB.length) counts += ' · CURB ' + CB.length;
    ctx.fillText(counts, w - 10, 16);
    ctx.restore();

    // ── legend ──────────────────────────────────────────────────────────
    // The two curves are never allowed to be ambiguous, so the planner path
    // and the steering prediction always appear even when their data does
    // not; the rest of the legend only appears when it has something to
    // explain.
    var rwTxt = '';
    if(rw == null){
      rwTxt = ' (no data)';
    }else if(!homed){
      rwTxt = ' (unhomed)';
    }else{
      rwTxt = ' (' + (rw > 0 ? '+' : '') + rw.toFixed(1) + '°';
      // Turn radius straight out of the bicycle model arcPts uses:
      //   R = wheelbase / tan(delta). Magnitude only — the sign is already
      //   carried by the ribbon on screen.
      if(wb && Math.abs(rw) > 0.2){
        var turnR = Math.abs(wb / Math.tan(rw * Math.PI / 180));
        if(isFinite(turnR) && turnR < 999) rwTxt += ' · R ' + turnR.toFixed(1) + ' m';
      }
      rwTxt += ')';
    }
    var items = [];
    items.push({ k: 'line', c: '#4CE0D2', t: 'PLANNER PATH' });
    items.push({ k: 'line', c: '#FF5CA8', t: 'STEERING NOW' + rwTxt });
    if(wb && rwMax) items.push({ k: 'wedge', c: '#2C3949', t: 'FULL LOCK' });
    if(C.length)    items.push({ k: 'dot',  c: '#FF8A3D', t: 'CONES' });
    if(OB.length)   items.push({ k: 'dot',  c: '#FF4D4F', t: 'OBSTACLES' });
    if(CB.length)   items.push({ k: 'dot',  c: '#64748B', t: 'CURB' });

    ctx.save();
    ctx.font = _birdFont(10, 600, false);
    ctx.textAlign = 'left';
    ctx.textBaseline = 'middle';
    var gap = 14, sw = 14, x = 10, ly2 = h - 11;
    for(i = 0; i < items.length; i++){
      var it = items[i];
      var tw2 = _birdTrackW(ctx, it.t, 0.4);
      if(x + sw + 5 + tw2 > w - 8) break;   // drop tail entries rather than wrap
      if(it.k === 'line'){
        ctx.strokeStyle = it.c; ctx.lineWidth = 2.4; ctx.lineCap = 'round';
        ctx.beginPath(); ctx.moveTo(x, ly2); ctx.lineTo(x + sw, ly2); ctx.stroke();
      }else if(it.k === 'wedge'){
        ctx.fillStyle = it.c; ctx.globalAlpha = 0.6;
        ctx.fillRect(x, ly2 - 4, sw, 8); ctx.globalAlpha = 1;
      }else{
        ctx.fillStyle = it.c;
        ctx.beginPath(); ctx.arc(x + sw / 2, ly2, 4, 0, Math.PI * 2); ctx.fill();
      }
      ctx.fillStyle = '#9AA9BC';
      _birdTrack(ctx, it.t, x + sw + 5, ly2 + 0.5, 0.4);
      x += sw + 5 + tw2 + gap;
    }
    ctx.restore();

    // ── empty / degraded state ──────────────────────────────────────────
    // Never a bare blank panel: say which feed is missing, because "no cones"
    // and "no steering" are completely different failures and the operator
    // has to be able to tell them apart at a glance.
    var missing = [];
    if(!C.length)  missing.push('cones — none');
    if(!PA.length) missing.push('planner path — none');
    if(rw == null) missing.push('steering — no data');
    else if(!homed) missing.push('steering — unhomed');
    var barren = !C.length && !PA.length && !OB.length && !CB.length;
    if(barren){
      ctx.save();
      ctx.textAlign = 'center';
      ctx.textBaseline = 'alphabetic';
      var by = Math.max(46, h * 0.26);
      ctx.font = _birdFont(11.5, 600, false);
      ctx.fillStyle = '#9AA9BC';
      var head = 'NO PERCEPTION DATA';
      var hwd = _birdTrackW(ctx, head, 1.2);
      ctx.textAlign = 'left';
      _birdTrack(ctx, head, cx - hwd / 2, by, 1.2);
      ctx.font = _birdFont(10.5, 400, true);
      ctx.fillStyle = '#64748B';
      ctx.textAlign = 'center';
      for(i = 0; i < missing.length; i++) ctx.fillText(missing[i], cx, by + 16 + i * 14);
      ctx.restore();
    }else if(missing.length){
      ctx.save();
      ctx.font = _birdFont(10, 400, true);
      ctx.textAlign = 'right';
      ctx.textBaseline = 'alphabetic';
      ctx.fillStyle = '#64748B';
      ctx.fillText('missing: ' + missing.join(', '), w - 10, h - 26);
      ctx.restore();
    }
  }catch(err){
    // Last resort. Do not rethrow: this canvas is redrawn on every poll and a
    // throw here would take the rest of the tick with it.
    //
    // A draw call that threw between a save() and its restore() leaves
    // drawing states on the stack; nothing in the body nests deeper than three
    // levels, and restore() on an empty stack is a spec'd no-op, so unwinding
    // three levels clears any leak. drawBird is a top-level entry point (the
    // poll loop calls it directly) so there is no caller state to pop past,
    // and setupCanvas re-assigns cv.width every frame, which resets the
    // context anyway -- this is belt and braces, not the only line of defence.
    try{ ctx.restore(); ctx.restore(); ctx.restore(); }catch(e0){ /* no stack */ }
    try{
      ctx.fillStyle = '#131923';
      ctx.fillRect(0, 0, w, h);
      ctx.fillStyle = '#F5A524';
      ctx.font = _birdFont(11, 600, false);
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText('bird view render error', w / 2, h / 2);
      ctx.font = _birdFont(10, 400, true);
      ctx.fillStyle = '#64748B';
      ctx.fillText(String((err && err.message) || err).slice(0, 80), w / 2, h / 2 + 16);
    }catch(e2){ /* canvas itself is gone; nothing sane left to do */ }
  }
}

// ══ Ethon console ══════════════════════════════════════════════════════════
// Cached last-known payloads. The state tick runs at 600 ms and the history
// tick at 1200 ms, but a tab switch can demand a repaint at any moment, so
// every draw input is kept here rather than living inside a fetch closure.
let lastNodes = "";
let lastHist = null;      // last /api/history payload
let lastDrive = {};       // last /ethon/drive_status value
let lastStatus = {};      // last status block
let lastStrategy = {};    // last /ethon/strategy value
let view = "drive";

function q(id){return document.getElementById(id);}
function el(t,c,txt){const e=document.createElement(t);if(c)e.className=c;if(txt!=null)e.textContent=txt;return e;}

function toast(msg,bad){
  const t=q('toast');if(!t)return;
  t.textContent=msg;t.className=bad?'toast show err':'toast show';
  clearTimeout(toast._t);toast._t=setTimeout(()=>{t.className='toast';},2800);
}

// A rendering fault in one panel must never take the whole page down with it --
// the tick that repaints the steering visualiser is the same tick that repaints
// the arm state, and losing the arm state because a chart divided by zero would
// be genuinely dangerous. Log each distinct failure once, then carry on.
function safe(label,fn){
  try{fn();}catch(e){
    if(!safe._seen[label]){safe._seen[label]=1;console.error('draw:'+label,e);}
  }
}
safe._seen={};

// Returns null when the canvas has no laid-out size, which is exactly what
// happens while its tab is hidden (display:none). Drawing then would bake the
// 300x150 default into the backing store and the panel would still be wrong
// after you switched back to it. Callers already treat null as "skip".
function setupCanvas(id){
  const cv=q(id);if(!cv)return null;
  const w=cv.clientWidth,h=cv.clientHeight;
  if(!w||!h)return null;
  const dpr=window.devicePixelRatio||1;
  const bw=Math.round(w*dpr),bh=Math.round(h*dpr);
  // Only reallocate when the size actually changed. Assigning width/height is
  // what clears a canvas, and doing it every frame is a needless realloc --
  // every draw function paints its own opaque background anyway.
  if(cv.width!==bw||cv.height!==bh){cv.width=bw;cv.height=bh;}
  const ctx=cv.getContext('2d');
  ctx.setTransform(dpr,0,0,dpr,0,0);
  return {ctx:ctx,w:w,h:h,cv:cv};
}

// Predicted vehicle trajectory from the CURRENT steering angle (bicycle
// model). This is where the car actually goes if you hold this lock -- as
// opposed to /ethon/path, which is where the planner WANTS to go. Comparing
// the two is the whole point. Works with no cameras: turn the wheel on the
// bench and the arc sweeps.
//   turn radius R = wheelbase / tan(delta),  +delta = left
//   x = R sin(t),  y = R (1 - cos(t)),  t = arclen / R
// Carried across verbatim from the previous dashboard: the signs below are
// bench-verified and must not be re-derived.
function arcPts(degRoadWheel, wheelbase, reach){
  const d=(degRoadWheel||0)*Math.PI/180;
  const pts=[];
  if(!wheelbase||Math.abs(d)<0.0035){                 // ~0.2 deg = straight
    for(let i=0;i<=12;i++) pts.push([reach*i/12,0]);
    return pts;
  }
  const R=wheelbase/Math.tan(d);
  // t is HEADING CHANGE in radians, not distance. reach/|R| is unbounded, so
  // at full lock (R ~ 4.7 m here) a 12 m arc sweeps ~147 deg and curls back
  // across the view. Cap it at a quarter turn: past that the preview stops
  // being a useful "where am I pointed" cue.
  const tmax=Math.min(reach/Math.abs(R), Math.PI/2);
  // x uses |R| and y uses signed R. Using signed R for x too makes a
  // right-hand turn run BACKWARDS through the car (negative forward
  // distance), which draws as a diagonal across the view.
  const aR=Math.abs(R);
  for(let i=0;i<=24;i++){
    const t=tmax*i/24;
    pts.push([aR*Math.sin(t), R*(1-Math.cos(t))]);
  }
  return pts;
}

// ── view switching ─────────────────────────────────────────────────────────
const VIEWS=['drive','bench','tune','diag'];
function showView(v){
  if(VIEWS.indexOf(v)<0)return;
  view=v;
  for(const n of VIEWS){
    const pane=q('pane-'+n),tab=q('tab-'+n);
    if(pane)pane.hidden=(n!==v);
    if(tab)tab.setAttribute('aria-selected',n===v?'true':'false');
  }
  try{history.replaceState(null,'','#'+v);}catch(e){}
  // The log is only rebuilt while its tab is visible, so it needs one render
  // on arrival to catch up on everything buffered while you were elsewhere.
  if(v==='diag')renderLog();
  redrawAll();
}

function redrawAll(){
  safe('steer',()=>drawSteer(lastDrive));
  safe('motors',()=>drawMotors('motors',lastDrive));
  safe('motors_b',()=>drawMotors('motors_b',lastDrive));
  safe('energy',()=>drawEnergy('energy',lastStrategy,lastDrive,lastStatus));
  if(lastHist){
    safe('bird',()=>drawBird('bird',lastHist.cones,lastHist.path,lastDrive,
                             lastHist.obstacles,lastHist.curb));
    safe('map',()=>drawMap('map',lastHist.track,lastHist.line,lastHist.track_speed));
    safe('charts',()=>drawCharts(lastHist));
  }
}

// Series colours are deliberately none of green/amber/red: on this page those
// three mean vehicle state and nothing else, so a chart must not borrow them.
function drawCharts(hh){
  drawChart('c_speed', hh.t,hh.speed, {color:'#4CE0D2',label:'Speed',      unit:'km/h', dec:1});
  drawChart('c_energy',hh.t,hh.energy,{color:'#7C9CE0',label:'Energy',     unit:'Wh',   dec:0});
  drawChart('c_whkm',  hh.t,hh.whkm,  {color:'#B79CFF',label:'Efficiency', unit:'Wh/km',dec:0});
  drawChart('c_temp',  hh.t,hh.temp,  {color:'#FF5CA8',label:'Motor temp', unit:'°C', dec:0});
}

// ── header chips ───────────────────────────────────────────────────────────
function fmtChips(s,health){
  const box=q('chips');if(!box)return;
  const out=[];
  const add=(txt,cls,val)=>{
    const c=el('span','chip'+(cls?' '+cls:''));
    c.appendChild(document.createTextNode(txt));
    if(val!=null){const b=el('b',null,val);c.appendChild(b);}
    out.push(c);
  };
  if(s.estop||s.estop_latched)add('E-STOP','crit');
  add(s.armed===true?'ARMED':(s.armed===false?'DISARMED':'ARM ?'),s.armed===true?'ok':'');
  add('CAN',s.can_ok===true?'ok':(s.can_ok===false?'crit':''),s.can_ok===true?'OK':'DOWN');
  add('GPS',s.gps_fix===true?'ok':'warn',s.gps_fix===true?'FIX':'NO FIX');
  if(s.battery_v!=null)
    add('',s.battery_v<10.5?'crit':(s.battery_v<11.5?'warn':'ok'),s.battery_v.toFixed(1)+' V');
  const alerts=(health&&health.alerts)||[];
  if(alerts.length)add('ALERTS','warn',String(alerts.length));
  box.innerHTML='';
  for(const c of out)box.appendChild(c);

  const ab=q('armbtn');
  if(ab)ab.className='b go'+(s.armed===true?' on':'');
  const dc=q('diagcnt');
  if(dc)dc.textContent=alerts.length?('·'+alerts.length):'';
}

// ── instrument cluster ─────────────────────────────────────────────────────
function fmtCluster(s,lap,drive){
  const box=q('cluster');if(!box)return;
  const fmtlap=x=>{if(x==null)return '—';const m=Math.floor(x/60);
    return m+':'+(x-60*m).toFixed(1).padStart(4,'0');};
  const batt=s.battery_v;
  const cells=[
    {k:'Speed', v:(s.speed_kmh==null?'—':(+s.speed_kmh).toFixed(1)), u:'km/h', lead:true,
     foot:(drive.wheel_speed_ms!=null?(Math.abs(drive.wheel_speed_ms)).toFixed(2)+' m/s':'')},
    {k:'State', v:(s.estop||s.estop_latched)?'E-STOP':(s.armed===true?'ARMED':(s.armed===false?'DISARMED':'—')),
     cls:(s.estop||s.estop_latched)?'crit':(s.armed===true?'ok':'dim')},
    {k:'Mode',  v:(s.mode?String(s.mode).toUpperCase():'—'), cls:s.mode?'':'dim'},
    {k:'Lap',   v:(lap.lap==null?'—':String(lap.lap)),
     foot:'last '+fmtlap(lap.last_s)+'  best '+fmtlap(lap.best_s)},
    {k:'Battery', v:(batt==null?'—':batt.toFixed(1)), u:'V',
     cls:batt==null?'dim':(batt<10.5?'crit':(batt<11.5?'warn':'ok')),
     foot:(drive.energy_wh!=null?(+drive.energy_wh).toFixed(0)+' Wh used':'')},
    {k:'GPS',   v:(s.gps_fix===true?'FIX':(s.gps_fix===false?'NO FIX':'—')),
     cls:s.gps_fix===true?'ok':'crit'},
    {k:'Line',  v:(s.line_set===true?'SET':(s.line_set===false?'UNSET':'—')),
     cls:s.line_set===true?'ok':'dim'},
  ];
  box.innerHTML='';
  for(const c of cells){
    const d=el('div','cell'+(c.lead?' lead':''));
    d.appendChild(el('div','k',c.k));
    const line=el('div','v'+(c.cls?' '+c.cls:''));
    line.appendChild(document.createTextNode(c.v));
    if(c.u){const u=el('span','u',' '+c.u);line.appendChild(u);}
    d.appendChild(line);
    if(c.foot)d.appendChild(el('div','foot',c.foot));
    box.appendChild(d);
  }
}

// ── arm ribbon ─────────────────────────────────────────────────────────────
// Semantics copied verbatim from the original: arm_ready is the single source
// of truth, and the blocking reason comes from arm_block.
function fmtRibbon(s){
  const b=q('ribbon');if(!b)return;
  if(s.arm_ready){b.className='ribbon ok';b.textContent='DRIVE ENABLED — motors live';return;}
  const sev=(s.estop||s.estop_latched)?'crit':'warn';
  b.className='ribbon '+sev;
  b.textContent='ARM BLOCKED — '+(s.arm_block||'unknown');
}

// ── topics ─────────────────────────────────────────────────────────────────
function fmtTopics(topics){
  const box=q('topics');if(!box)return;
  const names=Object.keys(topics).sort();
  box.innerHTML='';
  for(const n of names){
    const t=topics[n];
    const card=el('div','topic'+(t.age_s>3?' stale':''));
    const head=el('div','throw');
    head.appendChild(el('span','tn',n));
    head.appendChild(el('span','age',t.age_s+'s'));
    card.appendChild(head);
    card.appendChild(el('div','ty',t.type));
    card.appendChild(el('pre',null,JSON.stringify(t.value,null,1)));
    box.appendChild(card);
  }
}

// ── Jetson health ──────────────────────────────────────────────────────────
// None of this was rendered on the old dashboard even though health_monitor
// has been publishing it all along -- a thermally throttled Jetson silently
// drops perception frame rate, which used to be invisible here.
function fmtSystem(health){
  const box=q('sysmeters');
  const sys=(health&&health.system)||{};
  if(box){
    const meters=[];
    const push=(k,val,txt,frac,cls)=>{meters.push({k:k,txt:txt,frac:frac,cls:cls});};
    if(sys.cpu_pct!=null)
      push('CPU',sys.cpu_pct,sys.cpu_pct.toFixed(0)+'%',sys.cpu_pct/100,
           sys.cpu_pct>90?'crit':(sys.cpu_pct>75?'warn':'ok'));
    if(sys.ram_used_mb!=null&&sys.ram_total_mb)
      push('RAM',null,(sys.ram_used_mb/1024).toFixed(1)+' / '+(sys.ram_total_mb/1024).toFixed(1)+' GB',
           sys.ram_used_mb/sys.ram_total_mb,
           sys.ram_used_mb/sys.ram_total_mb>0.9?'crit':(sys.ram_used_mb/sys.ram_total_mb>0.75?'warn':'ok'));
    if(sys.cpu_temp_c!=null)
      push('CPU temp',null,sys.cpu_temp_c.toFixed(1)+'°C',sys.cpu_temp_c/100,
           sys.cpu_temp_c>85?'crit':(sys.cpu_temp_c>75?'warn':'ok'));
    if(sys.gpu_temp_c!=null)
      push('GPU temp',null,sys.gpu_temp_c.toFixed(1)+'°C',sys.gpu_temp_c/100,
           sys.gpu_temp_c>85?'crit':(sys.gpu_temp_c>75?'warn':'ok'));
    if(sys.motor_temp_c!=null)
      push('Motor temp',null,sys.motor_temp_c.toFixed(0)+'°C',sys.motor_temp_c/100,
           sys.motor_temp_c>90?'crit':(sys.motor_temp_c>70?'warn':'ok'));
    if(sys.disk_free_gb!=null)
      push('Disk free',null,sys.disk_free_gb.toFixed(1)+' GB',Math.min(1,sys.disk_free_gb/64),
           sys.disk_free_gb<4?'crit':(sys.disk_free_gb<12?'warn':'ok'));
    box.innerHTML='';
    if(!meters.length){box.appendChild(el('div','empty','health_monitor is not publishing.'));}
    for(const m of meters){
      const d=el('div','meter');
      const top=el('div','top');
      top.appendChild(el('span','k',m.k));
      top.appendChild(el('span','v',m.txt));
      d.appendChild(top);
      const bar=el('div','bar');
      const i=el('i',m.cls);
      i.style.width=Math.max(0,Math.min(100,(m.frac||0)*100)).toFixed(1)+'%';
      bar.appendChild(i);d.appendChild(bar);
      box.appendChild(d);
    }
  }
  const th=q('throt');
  if(th){
    const bad=sys.throttling===true;
    th.textContent=bad?'THERMAL THROTTLING':'';
    th.style.color=bad?'var(--crit)':'';
  }
  const tbl=q('hztbl');
  if(tbl){
    tbl.innerHTML='';
    const tp=(health&&health.topics)||{};
    const names=Object.keys(tp).sort();
    const hr=el('tr');
    for(const hcell of ['Topic','Rate','Required','']) hr.appendChild(el('th',null,hcell));
    tbl.appendChild(hr);
    if(!names.length){
      const tr=el('tr');const td=el('td',null,'no rate data');
      td.colSpan=4;tr.appendChild(td);tbl.appendChild(tr);
    }
    for(const n of names){
      const r=tp[n]||{};
      const tr=el('tr');
      tr.appendChild(el('td','mono',n));
      tr.appendChild(el('td','mono',(r.hz!=null?r.hz.toFixed(1):'—')+' Hz'));
      tr.appendChild(el('td','mono',(r.min_hz!=null?r.min_hz.toFixed(1):'—')+' Hz'));
      tr.appendChild(el('td',r.status==='ok'?'pass':'fail',(r.status||'?').toUpperCase()));
      tbl.appendChild(tr);
    }
  }
}

// ── actions ────────────────────────────────────────────────────────────────
async function act(what,arg){
  try{
    const r=await fetch('/api/action',{method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify({action:what,arg:arg})});
    const j=await r.json();
    toast(j.ok?(what+(arg?' '+arg:'')+' — sent'):((j.reason||what)+' — rejected'),!j.ok);
  }catch(e){toast(what+' failed — no response from the vehicle',true);}
}
function confirmRace(){
  if(confirm('Start the 70-minute race clock? This begins the energy budget.'))act('race_start');
}

// ── parameters ─────────────────────────────────────────────────────────────
async function loadParams(){
  const node=q('pnode').value;
  const box=q('params');
  if(!node){box.className='empty';box.textContent='Pick a node to view and edit its parameters.';return;}
  box.className='';box.textContent='loading…';
  try{
    const r=await fetch('/api/params?node='+encodeURIComponent(node),{cache:'no-store'});
    const j=await r.json();
    if(!j.ok){box.className='empty';box.textContent=(j.reason||'node unreachable');return;}
    if(!j.params.length){box.className='empty';box.textContent='This node exposes no tunable parameters.';return;}
    const wrapEl=el('div','scrollx');
    const tbl=el('table','tbl');
    const hdr=el('tr');
    for(const h of ['Parameter','Type','Value',''])hdr.appendChild(el('th',null,h));
    tbl.appendChild(hdr);
    for(const p of j.params){
      const tr=el('tr');
      tr.appendChild(el('td','mono',p.name));
      tr.appendChild(el('td','mono',p.type_name));
      const tdv=el('td');
      const inp=el('input','pv');
      // Strip the JSON quotes on strings so the field shows the bare value the
      // operator expects to edit; the server re-parses by declared type.
      inp.value=JSON.stringify(p.value).replace(/^"|"$/g,'');
      inp.dataset.type=p.type;inp.dataset.name=p.name;
      tdv.appendChild(inp);tr.appendChild(tdv);
      const tdb=el('td');
      const b=el('button','b','Set');
      b.onclick=()=>setParam(node,p.name,p.type,inp.value,b);
      tdb.appendChild(b);tr.appendChild(tdb);
      tbl.appendChild(tr);
    }
    wrapEl.appendChild(tbl);
    box.innerHTML='';box.appendChild(wrapEl);
  }catch(e){box.className='empty';box.textContent='Request failed.';}
}

async function setParam(node,name,type,value,btn){
  btn.disabled=true;const old=btn.textContent;btn.textContent='…';
  try{
    const r=await fetch('/api/param',{method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify({node:node,name:name,type:type,value:value})});
    const j=await r.json();
    toast(j.ok?(name+' = '+value):(name+' rejected — '+(j.reason||'bad value')),!j.ok);
  }catch(e){toast(name+' failed — no response',true);}
  btn.disabled=false;btn.textContent=old;
}

function buildNodeShortcuts(nodes){
  const box=q('nodeshort');if(!box)return;
  box.innerHTML='';
  for(const n of nodes){
    const b=el('button','b',n.replace(/^\//,''));
    b.onclick=()=>{q('pnode').value=n;loadParams();};
    box.appendChild(b);
  }
}

// ── self test ──────────────────────────────────────────────────────────────
async function selftest(){
  const box=q('selftest');
  box.className='';box.textContent='running checks…';
  try{
    const r=await fetch('/api/selftest',{cache:'no-store'});const j=await r.json();
    box.innerHTML='';box.className='';
    const v=el('div','ribbon '+(j.ready?'ok':'crit'),j.verdict);
    v.style.borderRadius='6px';
    box.appendChild(v);
    const wrapEl=el('div','scrollx');
    const tbl=el('table','tbl');
    for(const c of (j.checks||[])){
      const tr=el('tr');
      tr.appendChild(el('td',c.ok?'pass':'fail',c.ok?'PASS':'FAIL'));
      tr.appendChild(el('td',null,c.name));
      tr.appendChild(el('td','mono',c.detail||''));
      tbl.appendChild(tr);
    }
    wrapEl.appendChild(tbl);box.appendChild(wrapEl);
  }catch(e){box.className='empty';box.textContent='Self-test request failed.';}
}

// ── camera previews ────────────────────────────────────────────────────────
// Every configured source gets a chip, not just the ones delivering frames: a
// source that is configured but dead has no preview image, so without the chip
// row you cannot tell "no camera" from "camera broken". The chip also surfaces
// calibrated=false, which is otherwise silent and is the reason detections
// never reach the planner.
function updateSrcs(fs){
  const box=q('srcs');if(!box)return;
  const src=fs.sources||{};
  const names=Object.keys(src);
  box.innerHTML='';
  if(!names.length)return;
  let uncal=0;
  for(const n of names){
    const s=src[n]||{};
    const alive=!!s.alive,cal=!!s.calibrated;
    if(alive&&!cal)uncal++;
    const chip=el('div','src');
    const d=el('span','d');
    d.style.background=alive?'var(--ok)':'var(--crit)';
    chip.appendChild(d);
    chip.appendChild(el('span','nm',n));
    chip.appendChild(el('span','fps',alive?((+s.fps||0).toFixed(1)+' fps'):'down'));
    if(alive)chip.appendChild(el('span','fps',(s.detections||0)+' det'));
    const c=el('span',null,cal?'cal':'UNCALIBRATED');
    c.style.color=cal?'var(--ok)':'var(--warn)';
    c.style.fontWeight='700';
    c.title=cal?'homography loaded — detections reach the planner'
               :'no homography — detections go to /ethon/detections_raw only and are '
                +'NEVER used by the planner. Run calibrate_homography.py.';
    chip.appendChild(c);
    box.appendChild(chip);
  }
  const note=q('camnote');
  if(note)note.textContent=uncal?(uncal+' source'+(uncal>1?'s':'')+' uncalibrated — not reaching the planner'):'';
}

let camList=[],camSeq=0;
async function camTick(){
  // Only poll while the tab that shows the tiles is actually visible. The old
  // dashboard fetched four JPEGs 5x a second forever, including while you were
  // reading the log.
  if(view!=='drive')return;
  try{
    const r=await fetch('/api/cams',{cache:'no-store'});
    const j=await r.json();
    const cams=j.cams||[];
    if(cams.join(',')!==camList.join(',')){          // rebuild only on change
      camList=cams;
      const grid=q('camgrid'),none=q('camnone');
      grid.innerHTML='';
      none.style.display=cams.length?'none':'';
      // Place each tile where its camera physically sits on the car rather than
      // in whatever order the sources were discovered: left | wide | right
      // across the top, narrow directly beneath wide. A dead source leaves its
      // cell empty on purpose -- the gap tells you which camera is missing
      // instead of silently reflowing the survivors.
      const LAYOUT={left:[1,1],wide:[2,1],right:[3,1],narrow:[2,2]};
      const ordered=[...cams].sort((a,b)=>{
        const A=LAYOUT[a],B=LAYOUT[b];
        if(A&&B)return (A[1]-B[1])||(A[0]-B[0]);
        if(A)return -1;
        if(B)return 1;
        return a.localeCompare(b);
      });
      for(const c of ordered){
        const cell=el('div','camcell');
        const pos=LAYOUT[c];
        if(pos){cell.style.gridColumn=pos[0];cell.style.gridRow=pos[1];}
        const img=el('img');img.id='cam_'+c;img.alt=c+' camera preview';
        cell.appendChild(img);cell.appendChild(el('div','camlbl',c));
        grid.appendChild(cell);
      }
    }
    camSeq++;
    for(const c of camList){
      const img=q('cam_'+c);
      // Cache-buster per request: the server is a stdlib HTTP handler with no
      // streaming, so this polls one JPEG per source rather than holding an
      // MJPEG connection open on a thread.
      if(img)img.src='/api/cam?src='+encodeURIComponent(c)+'&_='+camSeq;
    }
  }catch(e){}
}

// ── history tick ───────────────────────────────────────────────────────────
async function histTick(){
  try{
    const r=await fetch('/api/history',{cache:'no-store'});
    const hh=await r.json();
    lastHist=hh;
    if(view!=='drive')return;
    safe('map',()=>drawMap('map',hh.track,hh.line,hh.track_speed));
    safe('bird',()=>drawBird('bird',hh.cones,hh.path,lastDrive,hh.obstacles,hh.curb));
    safe('charts',()=>drawCharts(hh));
  }catch(e){}
}

// ── log ────────────────────────────────────────────────────────────────────
let logSince=0,logRows=[];
async function logTick(){
  const p=q('logpause');
  if(p&&p.checked)return;
  try{
    const r=await fetch('/api/logs?since='+logSince,{cache:'no-store'});
    const j=await r.json();
    if(j.logs&&j.logs.length){
      logRows=logRows.concat(j.logs);
      if(logRows.length>600)logRows=logRows.slice(-600);
      logSince=j.last;
      // Keep draining the ring on every tick regardless of tab -- the buffer
      // must not develop holes -- but only pay for the DOM rebuild when
      // someone is actually looking at it.
      if(view==='diag')renderLog();
    }
  }catch(e){}
}
function renderLog(){
  const sel=q('loglevel'),pre=q('log');
  if(!sel||!pre)return;
  const lvl=parseInt(sel.value,10);
  const bottom=pre.scrollTop+pre.clientHeight>=pre.scrollHeight-30;
  pre.innerHTML='';
  for(const r of logRows){
    if(r.lvl<lvl)continue;
    pre.appendChild(el('div','lg l'+r.lvl,'['+r.lvl_name+'] '+r.name+': '+r.msg));
  }
  if(bottom)pre.scrollTop=pre.scrollHeight;
}
function clearLog(){logRows=[];renderLog();}

// ══ BENCH TESTS ════════════════════════════════════════════════════════════
// These two panels command the hardware directly. The safety contract is
// unchanged from the original dashboard and must stay that way:
//   * the command is re-posted every 200 ms, comfortably inside the vehicle
//     watchdog, so simply ceasing to post releases the motors;
//   * closing or hiding the page stops the test;
//   * the drive test posts an explicit zero on stop, but the steering test
//     deliberately does NOT -- 0 deg is a real target (dead ahead), so posting
//     it would command a move instead of releasing the column. Letting the
//     watchdog expire is the correct release.
async function setFoc(on){
  try{
    const r=await fetch('/api/param',{method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify({node:'/ethon_drive',name:'use_foc',type:1,value:on?'true':'false'})});
    const j=await r.json();
    toast(j.ok?('control mode → '+(on?'FOC':'DUTY')):('mode change rejected — '+(j.reason||'')),!j.ok);
  }catch(e){toast('mode change failed — no response',true);}
}

let testTimer=null;
function testDuty(){
  const mag=parseInt(q('dutyslider').value,10)/100.0;
  return q('dir-rev').checked?-mag:mag;
}
async function postDuty(d){
  try{await fetch('/api/drivetest',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({duty:d})});}catch(e){}
}
function startTest(){
  if(testTimer)return;
  postDuty(testDuty());                                  // fire immediately
  testTimer=setInterval(()=>postDuty(testDuty()),200);   // re-post < watchdog
  q('test-start').disabled=true;
  const st=q('teststate');st.textContent='RUNNING';st.style.color='var(--ok)';
}
function stopTest(){
  if(testTimer){clearInterval(testTimer);testTimer=null;}
  postDuty(0.0);                                         // explicit stop + watchdog backstop
  q('test-start').disabled=false;
  const st=q('teststate');st.textContent='stopped';st.style.color='var(--ink3)';
}

let steerTestTimer=null;
async function postSteerDeg(d){
  try{await fetch('/api/steertest',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({deg:d})});}catch(e){}
}
function startSteerTest(){
  if(steerTestTimer)return;
  const deg=()=>parseFloat(q('steerslider').value);
  postSteerDeg(deg());                                       // fire immediately
  steerTestTimer=setInterval(()=>postSteerDeg(deg()),200);   // re-post < watchdog
  q('steertest-start').disabled=true;
  const st=q('steerteststate');st.textContent='RUNNING';st.style.color='var(--ok)';
}
function stopSteerTest(){
  if(steerTestTimer){clearInterval(steerTestTimer);steerTestTimer=null;}
  q('steertest-start').disabled=false;
  const st=q('steerteststate');st.textContent='stopped';st.style.color='var(--ink3)';
  // no explicit zero-post here -- see the note above.
}

window.addEventListener('beforeunload',()=>{if(testTimer)postDuty(0.0);});
window.addEventListener('beforeunload',()=>{if(steerTestTimer)stopSteerTest();});
document.addEventListener('visibilitychange',()=>{if(document.hidden){stopTest();stopSteerTest();}});

function kvrows(box,rows){
  box.innerHTML='';
  for(const [k,v,cls] of rows){
    const r=el('div','kvrow');
    r.appendChild(el('span','k',k));
    const ve=el('span','v',v);
    if(cls)ve.style.color='var(--'+cls+')';
    r.appendChild(ve);box.appendChild(r);
  }
}

function updateTestPanel(s,drive){
  const foc=drive.use_foc===true;
  const md=q('mode-duty'),mf=q('mode-foc');
  if(md)md.className='b'+(foc?'':' on');
  if(mf)mf.className='b'+(foc?' on':'');
  const ms=q('modestate');
  if(ms)ms.textContent=drive.use_foc==null?'':('active: '+(foc?'FOC':'DUTY'));

  const dm=drive.motors||{};
  const rows=[['test active',drive.test_active?('YES · duty '+drive.test_duty):'no',
               drive.test_active?'ok':null]];
  for(const k of Object.keys(dm)){
    if(k==='steer')continue;
    const m=dm[k]||{};
    const f=(m.faults&&m.faults.length)?(' ['+m.faults.join(',')+']'):'';
    rows.push([k,(m.vel_rps!=null?m.vel_rps+' rps':'—')+' / '
                 +(m.supply_a!=null?m.supply_a+' A':'—')+f,
               f?'crit':null]);
  }
  const tr=q('testreadout');if(tr)kvrows(tr,rows);

  const sr=q('steertestreadout');
  if(sr)kvrows(sr,[
    ['steer test active',drive.steer_test_active?('YES · target '+drive.steer_test_deg+'°'):'no',
     drive.steer_test_active?'ok':null],
    ['road-wheel angle',drive.road_wheel_deg!=null?(drive.road_wheel_deg+'°'):'—'],
    ['homed limit',drive.steer_limit_deg!=null?('±'+drive.steer_limit_deg+'°'):'—'],
  ]);
}

// ── main tick ──────────────────────────────────────────────────────────────
async function tick(){
  try{
    const r=await fetch('/api/state',{cache:'no-store'});
    const s=await r.json();
    const beat=q('beat'),bt=q('beattxt');
    if(beat)beat.className='beat up';
    if(bt)bt.textContent='live';

    const st=s.status||{};
    const tp=s.topics||{};
    const dst=(tp['/ethon/drive_status']||{}).value||{};
    const lap=(tp['/ethon/lap']||{}).value||{};
    const health=(tp['/ethon/health']||{}).value||{};
    const fusion=(tp['/ethon/fusion_status']||{}).value||{};
    const strat=(tp['/ethon/strategy']||{}).value||{};

    lastStatus=st;lastDrive=dst;lastStrategy=strat;

    fmtChips(st,health);
    fmtCluster(st,lap,dst);
    fmtRibbon(st);

    const ma=q('md-autonomy'),mc=q('md-capture');
    if(ma)ma.className='b'+(st.mode==='autonomy'?' on':'');
    if(mc)mc.className='b'+(st.mode==='capture'?' on':'');

    if(view==='drive'){
      updateSrcs(fusion);
      safe('steer',()=>drawSteer(dst));
      safe('motors',()=>drawMotors('motors',dst));
      safe('energy',()=>drawEnergy('energy',strat,dst,st));
      // Redraw the bird's-eye on the fast tick as well as the slow history
      // tick, so the predicted-trajectory arc tracks the wheel instead of
      // lagging a second behind it. Cones and path come from the cache.
      if(lastHist)safe('bird',()=>drawBird('bird',lastHist.cones,lastHist.path,dst,
                                           lastHist.obstacles,lastHist.curb));
      const sh=q('steerhint');
      if(sh)sh.textContent=dst.steer_mode?(dst.steer_mode==='foc'?'FOC':'duty'):'';
    }else if(view==='bench'){
      safe('motors_b',()=>drawMotors('motors_b',dst));
    }else if(view==='diag'){
      fmtTopics(tp);
      fmtSystem(health);
    }
    updateTestPanel(st,dst);

    const nodes=(s.param_nodes||[]).join(',');
    if(nodes!==lastNodes){
      lastNodes=nodes;
      const sel=q('pnode');const cur=sel.value;
      sel.innerHTML='<option value="">— pick a node —</option>';
      for(const n of s.param_nodes){const o=el('option',null,n);o.value=n;sel.appendChild(o);}
      sel.value=cur;
      buildNodeShortcuts(s.param_nodes||[]);
    }
  }catch(e){
    const beat=q('beat'),bt=q('beattxt');
    if(beat)beat.className='beat down';
    if(bt)bt.textContent='disconnected';
  }
}

// ── boot ───────────────────────────────────────────────────────────────────
let resizeT=null;
window.addEventListener('resize',()=>{
  clearTimeout(resizeT);resizeT=setTimeout(redrawAll,120);
});
window.addEventListener('keydown',e=>{
  if(e.key==='Escape'){stopTest();stopSteerTest();}
});

const hash=(location.hash||'').replace('#','');
showView(VIEWS.indexOf(hash)>=0?hash:'drive');

tick();    setInterval(tick,600);
histTick();setInterval(histTick,1200);
logTick(); setInterval(logTick,1200);
camTick(); setInterval(camTick,200);   // ~5 fps preview
</script>
</body>
</html>
"""


PIT_PAGE_V2 = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<meta name="color-scheme" content="dark">
<title>Ethon Pit</title>
<style>
/* ── Ethon console design system ──────────────────────────────────────────
   Ground rules, so later edits don't fight the cascade:
     * Layout is done with grid/flex + gap. Elements do NOT carry their own
       outer margins; a parent's gap owns the space between siblings. If you
       need more room, change the container, not the child.
     * Selectors stay single-class and flat. There is deliberately no rule
       that reaches across the tree (no `.panel .row span`), so a new element
       can be dropped anywhere without inheriting spacing it didn't ask for.
     * Semantic colour (ok/warn/crit) means VEHICLE STATE and nothing else.
       Teal is "commanded or planned", magenta is "actually measured", orange
       is cones. Never borrow one for the other -- the whole point is that a
       glance at the colour tells you which of those four things you're
       looking at.
   ------------------------------------------------------------------------ */
:root{
  /* ground: near-black, biased blue-green so it reads as instrument glass
     rather than as an unconsidered neutral grey */
  --ground:#090B0F;
  --bg2:#0E1218;
  --panel:#131923;
  --panel2:#182030;
  --rule:#212B3A;
  --rule2:#2C3949;

  --ink:#EAF0F7;
  --ink2:#9AA9BC;   /* blue-grey, picked to sit with the ground */
  --ink3:#64748B;

  --ok:#2ED47A;
  --warn:#F5A524;
  --crit:#FF4D4F;

  --accent:#4CE0D2;  /* commanded / planned / interactive */
  --actual:#FF5CA8;  /* actual / measured */
  --cone:#FF8A3D;    /* literal traffic-cone orange */

  --sans:system-ui,-apple-system,"Segoe UI",Roboto,"Helvetica Neue",Arial,sans-serif;
  --mono:ui-monospace,"SF Mono","Cascadia Mono",Menlo,Consolas,"DejaVu Sans Mono",monospace;

  --gap:14px;
  --pad:14px;
}
*{box-sizing:border-box}
html,body{height:100%}
body{
  margin:0;background:var(--ground);color:var(--ink);
  font:400 13px/1.45 var(--sans);
  -webkit-font-smoothing:antialiased;
  font-variant-numeric:tabular-nums;
}
:focus-visible{outline:2px solid var(--accent);outline-offset:2px;border-radius:3px}
@media (prefers-reduced-motion:reduce){
  *{animation-duration:.001ms !important;transition-duration:.001ms !important}
}

/* ── command bar ─────────────────────────────────────────────────────────
   Square-edged and full-bleed on purpose: it is the frame of the instrument,
   not another card floating inside it. */
.cmd{
  position:sticky;top:0;z-index:40;
  display:flex;align-items:center;gap:18px;flex-wrap:wrap;
  padding:0 16px;min-height:52px;
  background:rgba(9,11,15,.88);backdrop-filter:blur(10px);
  border-bottom:1px solid var(--rule);
}
.brand{display:flex;align-items:baseline;gap:9px;font-weight:700;letter-spacing:.14em;font-size:13px}
.brand .mk{color:var(--ink)}
.brand .sub{color:var(--ink3);font-weight:600;letter-spacing:.18em;font-size:10px}

.live{display:flex;align-items:center;gap:7px;font-size:11px;color:var(--ink2);letter-spacing:.06em}
.beat{width:7px;height:7px;border-radius:50%;background:var(--ink3);flex:0 0 auto}
.beat.up{background:var(--ok);box-shadow:0 0 0 3px rgba(46,212,122,.16)}
.beat.down{background:var(--crit);box-shadow:0 0 0 3px rgba(255,77,79,.16)}

/* ── view tabs ── */
.views{display:flex;gap:2px;background:var(--bg2);border:1px solid var(--rule);border-radius:7px;padding:3px}
.view{
  appearance:none;border:0;cursor:pointer;
  font:600 11px/1 var(--sans);letter-spacing:.14em;
  color:var(--ink3);background:transparent;
  padding:8px 13px;border-radius:5px;
  transition:color .12s,background .12s;
}
.view:hover{color:var(--ink2)}
.view[aria-selected="true"]{background:var(--panel2);color:var(--ink);box-shadow:inset 0 1px 0 rgba(255,255,255,.05)}
.view .cnt{margin-left:6px;color:var(--crit);font-size:10px}

/* links to the other pages: quieter than the view tabs, because switching
   document is a rarer act than switching view */
.pages{display:flex;gap:13px;align-items:center}
.pages a{font:600 10px/1 var(--sans);letter-spacing:.14em;color:var(--ink3);text-decoration:none}
.pages a:hover{color:var(--accent)}

/* ── status chips in the bar ── */
.chips{display:flex;gap:6px;flex-wrap:wrap;align-items:center;margin-left:auto}
.chip{
  display:inline-flex;align-items:center;gap:6px;
  font:600 10.5px/1 var(--sans);letter-spacing:.1em;
  color:var(--ink2);background:var(--bg2);
  border:1px solid var(--rule);border-radius:4px;padding:6px 9px;white-space:nowrap;
}
.chip b{font-weight:700;color:var(--ink);letter-spacing:0;font-family:var(--mono);font-size:11px}
.chip.ok{color:var(--ok);border-color:rgba(46,212,122,.35);background:rgba(46,212,122,.08)}
.chip.warn{color:var(--warn);border-color:rgba(245,165,36,.35);background:rgba(245,165,36,.08)}
.chip.crit{color:#fff;background:var(--crit);border-color:var(--crit)}
.chip.crit b{color:#fff}

/* ── the two controls that must never be hunted for ── */
.cmdacts{display:flex;gap:8px;align-items:center}
.estop{
  appearance:none;cursor:pointer;
  font:800 12px/1 var(--sans);letter-spacing:.16em;
  color:#fff;background:var(--crit);border:1px solid #FF6B6D;
  border-radius:5px;padding:11px 18px;
  box-shadow:0 1px 0 rgba(255,255,255,.18) inset,0 0 18px rgba(255,77,79,.28);
}
.estop:hover{background:#FF6265}
.estop:active{transform:translateY(1px)}

/* ── status ribbon ── */
.ribbon{
  display:flex;align-items:center;gap:11px;
  padding:11px 16px;font-weight:600;font-size:12.5px;letter-spacing:.02em;
  border-bottom:1px solid var(--rule);
  background:var(--bg2);color:var(--ink2);
  border-left:3px solid var(--ink3);
}
.ribbon.ok{border-left-color:var(--ok);color:var(--ok);background:rgba(46,212,122,.05)}
.ribbon.warn{border-left-color:var(--warn);color:var(--warn);background:rgba(245,165,36,.05)}
.ribbon.crit{border-left-color:var(--crit);color:var(--crit);background:rgba(255,77,79,.07)}

/* ── page frame ──
   `.pane[hidden]` outranks `.pane` on specificity, so a hidden pane stays
   hidden no matter where these two rules sit relative to each other. */
.wrap{max-width:1680px;margin:0 auto;padding:var(--gap) 16px 56px}
.pane{display:grid;gap:var(--gap)}
.pane[hidden]{display:none}
/* a grid cell that holds several stacked panels rather than one */
.stack{display:grid;gap:var(--gap);align-content:start}

/* ── instrument cluster ─────────────────────────────────────────────────
   Deliberately NOT a row of cards. One recessed strip divided by hairlines,
   the way a cluster is one piece of glass with printed dividers. */
.cluster{
  display:grid;grid-template-columns:repeat(auto-fit,minmax(132px,1fr));
  background:var(--bg2);border:1px solid var(--rule);border-radius:9px;
  overflow:hidden;
}
.cell{padding:13px 16px;border-left:1px solid var(--rule);display:flex;flex-direction:column;gap:3px}
.cell:first-child{border-left:0}
.cell .k{font:600 10px/1 var(--sans);letter-spacing:.16em;color:var(--ink3);text-transform:uppercase}
.cell .v{font:700 27px/1.05 var(--sans);letter-spacing:-.02em;color:var(--ink);font-variant-numeric:tabular-nums}
.cell .u{font:600 11px/1 var(--sans);color:var(--ink3);letter-spacing:.06em}
.cell .foot{font:500 11px/1.2 var(--mono);color:var(--ink3)}
.cell.lead{background:var(--panel)}
.cell.lead .v{font-size:44px;letter-spacing:-.035em}
.cell .v.ok{color:var(--ok)} .cell .v.warn{color:var(--warn)} .cell .v.crit{color:var(--crit)}
.cell .v.dim{color:var(--ink3)}

/* ── panels ── */
.panel{background:var(--panel);border:1px solid var(--rule);border-radius:9px;
  box-shadow:inset 0 1px 0 rgba(255,255,255,.035);display:flex;flex-direction:column;min-width:0}
.phead{display:flex;align-items:center;gap:10px;padding:10px 14px;border-bottom:1px solid var(--rule)}
.phead h2{margin:0;font:600 11px/1 var(--sans);letter-spacing:.16em;text-transform:uppercase;color:var(--ink2)}
.phead .hint{font:400 11px/1 var(--sans);color:var(--ink3);letter-spacing:0}
.phead .right{margin-left:auto;display:flex;gap:7px;align-items:center}
.pbody{padding:var(--pad);display:flex;flex-direction:column;gap:11px;min-width:0}
.pbody.flush{padding:0}

/* ── grid columns ── */
.cols{display:grid;gap:var(--gap);grid-template-columns:repeat(12,1fr)}
.c12{grid-column:span 12} .c8{grid-column:span 8} .c7{grid-column:span 7}
.c6{grid-column:span 6}   .c5{grid-column:span 5} .c4{grid-column:span 4}
@media (max-width:1180px){
  .c8,.c7,.c6,.c5,.c4{grid-column:span 12}
}

/* canvases fill their panel; heights are set per-canvas so the aspect the
   drawing code assumes is the aspect it actually gets */
canvas{display:block;width:100%}
#bird{height:520px} #steerviz{height:220px}
#motors,#motors_b{height:158px}
#map{height:262px} #energy{height:220px}
.chartgrid{display:grid;grid-template-columns:repeat(auto-fit,minmax(258px,1fr));gap:var(--gap)}
.chartgrid canvas{height:158px}
@media (max-width:1180px){ #bird{height:420px} }

/* ── buttons ── */
.btns{display:flex;flex-wrap:wrap;gap:8px;align-items:center}
button.b{
  appearance:none;cursor:pointer;
  font:600 12px/1 var(--sans);letter-spacing:.04em;
  color:var(--ink);background:var(--panel2);
  border:1px solid var(--rule2);border-radius:5px;padding:10px 15px;
  transition:border-color .12s,background .12s,color .12s;
}
button.b:hover{border-color:var(--ink3);background:#1E2839}
button.b:active{transform:translateY(1px)}
button.b:disabled{opacity:.42;cursor:not-allowed;transform:none}
button.b.go{color:var(--ok);border-color:rgba(46,212,122,.4)}
button.b.go:hover{background:rgba(46,212,122,.1);border-color:var(--ok)}
button.b.cau{color:var(--warn);border-color:rgba(245,165,36,.4)}
button.b.cau:hover{background:rgba(245,165,36,.1);border-color:var(--warn)}
button.b.dgr{color:var(--crit);border-color:rgba(255,77,79,.42)}
button.b.dgr:hover{background:rgba(255,77,79,.12);border-color:var(--crit)}
button.b.on{background:rgba(76,224,210,.12);border-color:var(--accent);color:var(--accent)}

/* ── form controls ── */
select,input[type=text],input.pv{
  font:500 12px/1 var(--mono);color:var(--ink);background:var(--bg2);
  border:1px solid var(--rule2);border-radius:5px;padding:9px 10px;
}
input.pv{width:150px}
label.opt{display:inline-flex;align-items:center;gap:6px;color:var(--ink2);font-size:12px;cursor:pointer}
input[type=range]{
  -webkit-appearance:none;appearance:none;height:4px;border-radius:2px;
  background:var(--rule2);outline:none;width:230px;
}
input[type=range]::-webkit-slider-thumb{
  -webkit-appearance:none;width:16px;height:16px;border-radius:50%;
  background:var(--accent);border:2px solid var(--ground);cursor:grab;
}
input[type=range]::-moz-range-thumb{
  width:16px;height:16px;border-radius:50%;border:2px solid var(--ground);
  background:var(--accent);cursor:grab;
}
.val{font:700 14px/1 var(--mono);color:var(--ink);min-width:52px;display:inline-block}

/* ── hazard notice: for the panels that can physically move the car ── */
.hazard{
  display:flex;gap:11px;align-items:flex-start;
  padding:11px 13px;border-radius:6px;font-size:12px;line-height:1.5;
  background:rgba(245,165,36,.07);border:1px solid rgba(245,165,36,.3);color:var(--warn);
}
.hazard .mark{font-weight:800;letter-spacing:.1em;flex:0 0 auto}
.hazard p{margin:0;color:#E8D2A8}

/* ── readouts / key-value rows ── */
.kv{display:flex;flex-direction:column;gap:0}
/* two columns of readouts, so a tall list of pairs doesn't stretch its panel
   past the ones beside it in the grid row */
.kv2{display:grid;grid-template-columns:1fr 1fr;gap:0 20px}
@media (max-width:520px){.kv2{grid-template-columns:1fr}}
.kvrow{display:flex;justify-content:space-between;gap:12px;align-items:baseline;
  padding:7px 0;border-bottom:1px solid var(--rule)}
.kvrow:last-child{border-bottom:0}
.kvrow .k{color:var(--ink3);font-size:11.5px;letter-spacing:.04em}
.kvrow .v{font:600 12.5px/1 var(--mono);color:var(--ink);text-align:right}

/* ── tables ── */
.tbl{width:100%;border-collapse:collapse}
.tbl th{text-align:left;padding:8px 10px;border-bottom:1px solid var(--rule2);
  font:600 10px/1 var(--sans);letter-spacing:.14em;text-transform:uppercase;color:var(--ink3)}
.tbl td{padding:8px 10px;border-bottom:1px solid var(--rule);vertical-align:middle;font-size:12.5px}
.tbl tr:last-child td{border-bottom:0}
.tbl td.mono{font-family:var(--mono)}
.tbl td.pass{color:var(--ok);font-weight:700} .tbl td.fail{color:var(--crit);font-weight:700}
.scrollx{overflow-x:auto}

/* ── camera tiles ── */
.camgrid{display:grid;grid-template-columns:repeat(3,1fr);gap:10px}
@media (max-width:900px){.camgrid{grid-template-columns:1fr}
  .camcell{grid-column:auto !important;grid-row:auto !important}}
.camcell{position:relative;background:#05070A;border:1px solid var(--rule);
  border-radius:7px;overflow:hidden;min-height:96px}
.camcell img{width:100%;display:block}
.camlbl{position:absolute;top:7px;left:8px;font:600 10px/1 var(--sans);letter-spacing:.14em;
  text-transform:uppercase;background:rgba(5,7,10,.72);color:var(--ink);padding:4px 7px;border-radius:3px}
.srcs{display:flex;flex-wrap:wrap;gap:7px}
.src{display:inline-flex;align-items:center;gap:7px;font:500 11px/1 var(--sans);
  background:var(--bg2);border:1px solid var(--rule);border-radius:5px;padding:6px 9px;color:var(--ink2)}
.src .nm{font-weight:700;color:var(--ink);letter-spacing:.06em;text-transform:uppercase;font-size:10px}
.src .d{width:6px;height:6px;border-radius:50%}
.src .fps{font-family:var(--mono)}

/* ── log ── */
.log{height:340px;overflow:auto;background:#05070A;border:1px solid var(--rule);
  border-radius:7px;padding:10px;font:500 11.5px/1.55 var(--mono);margin:0}
.lg{padding:1px 0;white-space:pre-wrap;word-break:break-word;color:var(--ink2)}
.l10{color:var(--ink3)} .l30{color:var(--warn)} .l40{color:var(--crit)}
.l50{color:#fff;background:rgba(255,77,79,.24);border-radius:2px}

/* ── topics ── */
.topics{display:grid;grid-template-columns:repeat(auto-fill,minmax(320px,1fr));gap:10px}
.topic{background:var(--bg2);border:1px solid var(--rule);border-radius:7px;padding:11px 12px;
  overflow:hidden;display:flex;flex-direction:column;gap:5px}
.topic .tn{color:var(--accent);font:600 12px/1.3 var(--mono);word-break:break-all}
.topic .ty{color:var(--ink3);font-size:10.5px;font-family:var(--mono)}
.topic .age{font:600 10.5px/1 var(--mono);color:var(--ink3);white-space:nowrap}
.topic.stale{border-color:rgba(245,165,36,.35)}
.topic.stale .age{color:var(--warn)}
.topic pre{margin:0;white-space:pre-wrap;word-break:break-word;
  font:500 11px/1.5 var(--mono);color:var(--ink2);max-height:210px;overflow:auto}
.throw{display:flex;justify-content:space-between;gap:8px;align-items:baseline}

/* ── system health bars ── */
.meters{display:grid;grid-template-columns:repeat(auto-fit,minmax(190px,1fr));gap:11px}
.meter{display:flex;flex-direction:column;gap:6px}
.meter .top{display:flex;justify-content:space-between;align-items:baseline;gap:8px}
.meter .k{font:600 10px/1 var(--sans);letter-spacing:.14em;text-transform:uppercase;color:var(--ink3)}
.meter .v{font:700 13px/1 var(--mono);color:var(--ink)}
.bar{height:5px;border-radius:3px;background:var(--rule);overflow:hidden}
.bar i{display:block;height:100%;border-radius:3px;background:var(--accent);transition:width .3s}
.bar i.ok{background:var(--ok)} .bar i.warn{background:var(--warn)} .bar i.crit{background:var(--crit)}

/* ── toast ── */
.toast{
  position:fixed;bottom:18px;right:18px;z-index:80;max-width:360px;
  background:var(--panel2);border:1px solid var(--rule2);border-left:3px solid var(--accent);
  border-radius:6px;padding:12px 15px;font-size:12.5px;color:var(--ink);
  opacity:0;transform:translateY(6px);pointer-events:none;
  transition:opacity .18s,transform .18s;
  box-shadow:0 12px 32px rgba(0,0,0,.55);
}
.toast.show{opacity:1;transform:none}
.toast.err{border-left-color:var(--crit)}

.muted{color:var(--ink3)}
.empty{color:var(--ink3);font-size:12px;padding:6px 0}

/* ── pit board ─────────────────────────────────────────────────────────────
   Read from the pit wall, standing, at arm's length or further. Everything is
   sized for distance: the smallest thing on this page is a unit label, and the
   largest is whatever answers "are we going to make it?".
   Layered on top of the shared console tokens. */

.pitwrap{max-width:1500px;margin:0 auto;padding:18px 18px 40px;display:grid;gap:16px}

/* the nav here is links, not tab buttons, so it needs the bits `.view`
   assumes a <button> already has */
a.view{text-decoration:none;display:inline-flex;align-items:center}

/* the verdict: one sentence, enormous, colour-coded by pace */
.verdict{
  display:grid;gap:6px;justify-items:center;text-align:center;
  padding:26px 22px;border-radius:12px;
  background:var(--panel);border:1px solid var(--rule);
  box-shadow:inset 0 1px 0 rgba(255,255,255,.04);
}
.verdict .lead{
  font:800 62px/1 var(--sans);letter-spacing:-.03em;color:var(--ink2);
  text-wrap:balance;
}
.verdict .clock{font:700 30px/1 var(--mono);color:var(--ink);letter-spacing:-.01em}
.verdict .note{font:600 12px/1 var(--sans);letter-spacing:.18em;text-transform:uppercase;color:var(--ink3)}
.verdict.up   {border-color:rgba(46,212,122,.45);background:rgba(46,212,122,.06)}
.verdict.up   .lead{color:var(--ok)}
.verdict.down {border-color:rgba(255,77,79,.45);background:rgba(255,77,79,.06)}
.verdict.down .lead{color:var(--crit)}
.verdict.flat {border-color:rgba(76,224,210,.4)}
.verdict.flat .lead{color:var(--accent)}
@media (max-width:760px){.verdict .lead{font-size:38px}.verdict .clock{font-size:22px}}

#pitenergy{height:230px}

/* big tiles */
.pittiles{display:grid;grid-template-columns:repeat(auto-fit,minmax(210px,1fr));gap:12px}
.pittile{
  background:var(--panel);border:1px solid var(--rule);border-radius:10px;
  padding:16px 18px;display:flex;flex-direction:column;gap:5px;
  box-shadow:inset 0 1px 0 rgba(255,255,255,.035);
}
.pittile .k{font:600 11px/1 var(--sans);letter-spacing:.18em;text-transform:uppercase;color:var(--ink3)}
.pittile .v{font:700 44px/1 var(--sans);letter-spacing:-.03em;color:var(--ink);font-variant-numeric:tabular-nums}
.pittile .v small{font-size:17px;font-weight:600;color:var(--ink3);letter-spacing:.02em;margin-left:5px}
.pittile .s{font:500 12.5px/1.35 var(--mono);color:var(--ink3)}
.pittile .v.ok{color:var(--ok)} .pittile .v.warn{color:var(--warn)} .pittile .v.crit{color:var(--crit)}

/* alert strip */
.pitalerts{display:flex;gap:9px;flex-wrap:wrap}
.pital{
  font:700 12px/1 var(--sans);letter-spacing:.13em;text-transform:uppercase;
  padding:11px 16px;border-radius:7px;
  background:var(--panel);border:1px solid var(--rule);color:var(--ink3);
}
.pital.ok{color:var(--ok);border-color:rgba(46,212,122,.35);background:rgba(46,212,122,.07)}
.pital.warn{color:#0B0D11;background:var(--warn);border-color:var(--warn)}
.pital.crit{color:#fff;background:var(--crit);border-color:var(--crit)}
</style>
</head>
<body>
<header class="cmd">
  <div class="brand"><span class="mk">ETHON</span><span class="sub">PIT</span></div>
  <div class="views" role="navigation" aria-label="Pages">
    <a class="view" href="/v2">CONSOLE</a>
    <a class="view" href="/pit2" aria-selected="true">PIT</a>
    <a class="view" href="/replay">REPLAY</a>
    <a class="view" href="/calib">CALIB</a>
  </div>
  <div class="chips" id="chips"></div>
  <div class="live"><span class="beat" id="beat"></span><span id="beattxt">connecting</span></div>
  <div class="cmdacts">
    <button class="b go" onclick="confirmRace()">START RACE</button>
    <button class="estop" onclick="estop()">E&#8209;STOP</button>
  </div>
</header>

<div class="pitwrap">

  <div class="verdict" id="verdict">
    <div class="note" id="v_note">energy strategist</div>
    <div class="lead" id="v_lead">&mdash;</div>
    <div class="clock" id="v_clock"></div>
  </div>

  <div class="panel">
    <div class="phead">
      <h2>Energy budget</h2>
      <span class="hint">spent against the clock</span>
      <div class="right"><span class="hint" id="v_gap"></span></div>
    </div>
    <div class="pbody flush"><canvas id="pitenergy"></canvas></div>
  </div>

  <div class="pittiles" id="tiles"></div>

  <div class="pitalerts" id="alerts"></div>

</div>
<div class="toast" id="toast" role="status" aria-live="polite"></div>
<script>
// ── gfx_energy.js ──
// ═══ race energy budget ═══════════════════════════════════════════════════
// /ethon/strategy has been reaching the browser since the strategist was
// written but has only ever been rendered on the pit board, so the person
// sitting at the console — the one who can actually change how the car is
// driven — has never seen it. This panel answers exactly one question at a
// glance: AM I GOING TO MAKE IT.
//
// The race is an energy budget, not a speed contest: a fixed usable Wh
// (battery_usable_wh, 480 Wh today) over a fixed clock (race_minutes, 70).
// Everything here is a comparison of two numbers — what you have spent
// against what the clock says you should have spent.
//
// Colour discipline in this panel:
//   teal     = PLANNED   — the budget line, the target-now marker, target rate
//   magenta  = MEASURED  — Wh actually spent, actual burn rate, the projection
//                          (a projection is an extrapolation of a MEASUREMENT,
//                           so it stays magenta and is drawn dashed/hollow to
//                           say "not banked yet")
//   grn/amb/red = the VERDICT only — on budget / marginal / will not finish.
//                 Nothing else in this panel is allowed to use them, so a red
//                 pixel anywhere in the frame always means the same thing.
//
// Nothing in here fetches, posts, mutates shared state or sets a timer. The
// only thing it touches is the canvas it is handed; there are no readout
// element ids for this panel. It is repainted by the page's own poll, which
// is also the only clock any animation in here gets.

// Design tokens. canvas 2d cannot read CSS custom properties, so the palette
// is duplicated here; keep it in step with :root.
const _EN_C = {
  ground:'#090B0F', bg2:'#0E1218', panel:'#131923', panel2:'#182030',
  rule:'#212B3A', rule2:'#2C3949',
  ink:'#EAF0F7', ink2:'#9AA9BC', ink3:'#64748B',
  ok:'#2ED47A', warn:'#F5A524', crit:'#FF4D4F',
  accent:'#4CE0D2', actual:'#FF5CA8'
};
const _EN_SANS = 'system-ui,-apple-system,"Segoe UI",Roboto,"Helvetica Neue",Arial,sans-serif';
const _EN_MONO = 'ui-monospace,"SF Mono","Cascadia Mono",Menlo,Consolas,"DejaVu Sans Mono",monospace';

// Mirror of RaceStrategist.PACE_BAND in race_services.py (line 125): the
// strategist treats +/-8% of the budget rate as "on target". We reuse the same
// number so this panel can never call something a disaster while the pit board
// is still calling it ON TARGET — outside the band we go red exactly when the
// strategist says SLOW DOWN, and the amber "marginal" grade lives strictly
// INSIDE the band, on the overspending side of it. If that constant is ever
// retuned in race_services.py, retune it here too.
const _EN_PACE_BAND = 0.08;

// How much of the bar is kept to the right of the budget line for the
// over-budget zone. Fixed rather than data-driven on purpose: the budget line
// must not slide left and right between frames, or the eye loses the one
// reference point the whole panel is built around.
const _EN_OVER_ZONE = 0.16;

// ── small helpers, all prefixed so they cannot collide with the other canvas
// ── modules sharing this script block ─────────────────────────────────────

// Every field of /ethon/strategy can be null (the strategist publishes null
// for rate/projection/per-lap until it has enough race to mean anything) and
// after a bad JSON round trip a number can arrive as a string. Anything that
// is not a finite number is "no reading". A NaN that reaches a canvas
// coordinate silently draws nothing at all, which is far worse on a race
// dashboard than an honest em dash.
function _enNum(v){ if(v==null) return null; const n=+v; return isFinite(n)?n:null; }

function _enFix(v,dec){ return v==null ? '—' : (+v).toFixed(dec==null?0:dec); }

function _enMmss(s){
  const n=_enNum(s);
  if(n==null) return '—:—';
  const t=Math.max(0,Math.round(n));
  return Math.floor(t/60)+':'+String(t%60).padStart(2,'0');
}

function _enRGBA(hex,a){
  const h=String(hex).replace('#','');
  const r=parseInt(h.substring(0,2),16), g=parseInt(h.substring(2,4),16), b=parseInt(h.substring(4,6),16);
  return 'rgba('+r+','+g+','+b+','+a+')';
}

function _enRR(ctx,x,y,w,h,r){
  const rr=Math.max(0,Math.min(r,w/2,h/2));
  ctx.beginPath();
  ctx.moveTo(x+rr,y);ctx.arcTo(x+w,y,x+w,y+h,rr);
  ctx.arcTo(x+w,y+h,x,y+h,rr);ctx.arcTo(x,y+h,x,y,rr);
  ctx.arcTo(x,y,x+w,y,rr);ctx.closePath();
}

// Manual letter-spacing: ctx.letterSpacing only landed in Chrome 99 and the
// pit laptop is not guaranteed to be newer than 90.
function _enTracked(ctx,txt,x,y,sp,align){
  txt=String(txt==null?'':txt);
  const ws=[]; let total=0;
  for(let i=0;i<txt.length;i++){
    const cw=ctx.measureText(txt.charAt(i)).width;
    ws.push(cw); total+=cw+(i<txt.length-1?sp:0);
  }
  let px=x;
  if(align==='center') px=x-total/2; else if(align==='right') px=x-total;
  const old=ctx.textAlign; ctx.textAlign='left';
  for(let i=0;i<txt.length;i++){ ctx.fillText(txt.charAt(i),px,y); px+=ws[i]+sp; }
  ctx.textAlign=old;
  return total;
}

function _enTrackedWidth(ctx,txt,sp){
  txt=String(txt==null?'':txt);
  let total=0;
  for(let i=0;i<txt.length;i++) total+=ctx.measureText(txt.charAt(i)).width+(i<txt.length-1?sp:0);
  return total;
}

// Ellipsise to a width. This panel is span-4 on a 12-column grid and goes
// full-width under 1000 px, so the same string has to survive both a 300 px
// and a 900 px canvas. A sentence that runs off the edge of a race dashboard
// looks like a rendering bug and gets the whole panel distrusted.
function _enClip(ctx,txt,maxW){
  txt=String(txt==null?'':txt);
  if(maxW<=0) return '';
  if(ctx.measureText(txt).width<=maxW) return txt;
  let lo=0, hi=txt.length;
  while(lo<hi){
    const mid=(lo+hi+1)>>1;
    if(ctx.measureText(txt.substring(0,mid)+'…').width<=maxW) lo=mid; else hi=mid-1;
  }
  return lo>0?(txt.substring(0,lo)+'…'):'';
}

// The one uppercase micro-label style used throughout the panel.
function _enLabel(ctx,txt,x,y,color,align){
  ctx.font='600 10px '+_EN_SANS;
  ctx.fillStyle=color||_EN_C.ink3;
  ctx.textBaseline='alphabetic';
  return _enTracked(ctx,String(txt).toUpperCase(),x,y,0.7,align||'left');
}

// prefers-reduced-motion, asked fresh on every draw so a mid-session OS change
// is picked up. Wrapped because matchMedia is absent in some embedded webviews.
function _enReduced(){
  try{ return !!(window.matchMedia && window.matchMedia('(prefers-reduced-motion: reduce)').matches); }
  catch(e){ return false; }
}

// A slow breathe for the "will not finish" pill, and nothing else. This module
// owns no timer — it only ever samples the wall clock at whatever rate the
// page happens to repaint (600 ms on the console, 1 s on the pit board), so
// the motion has to survive coarse, irregular sampling. A low-amplitude alpha
// ramp does; a blink would just look like a rendering fault. Under
// prefers-reduced-motion it is pinned fully opaque, never dimmed, so the
// warning is if anything MORE readable.
function _enPulse(){
  if(_enReduced()) return 1;
  const ph=(Date.now()%1800)/1800;
  return 0.80+0.20*(0.5+0.5*Math.cos(ph*2*Math.PI));
}

// Diagonal hatch, used only for the over-budget zone. Clipped rather than
// computed per line so it cannot bleed past the budget line.
function _enHatch(ctx,x,y,w,h,color,alpha,step){
  if(w<=0||h<=0) return;
  ctx.save();
  ctx.beginPath();ctx.rect(x,y,w,h);ctx.clip();
  ctx.strokeStyle=_enRGBA(color,alpha);ctx.lineWidth=1;
  const s=step||7;
  ctx.beginPath();
  for(let i=-h;i<w+h;i+=s){ ctx.moveTo(x+i,y+h); ctx.lineTo(x+i+h,y); }
  ctx.stroke();
  ctx.restore();
}

// ── the model ─────────────────────────────────────────────────────────────
// Everything the painter needs, normalised once, so no drawing code ever has
// to think about nulls or units again.
function _enModel(strategy,drive,status){
  const s=(strategy&&typeof strategy==='object')?strategy:{};
  const d=(drive&&typeof drive==='object')?drive:{};
  const st=(status&&typeof status==='object')?status:{};

  const g={};
  g.err     = (typeof s.error==='string'&&s.error)?s.error:null;
  g.raceOn  = (s.race_on===true);
  g.budget  = _enNum(s.wh_budget);
  g.used    = _enNum(s.wh_used);
  g.remain  = _enNum(s.wh_remaining);
  g.elapsed = _enNum(s.elapsed_s);
  g.left_s  = _enNum(s.remaining_s);
  g.rate    = _enNum(s.rate_wh_min);      // null for the first 30 s of race
  g.target  = _enNum(s.budget_wh_min);
  g.proj    = _enNum(s.projected_wh);     // null while rate is null
  g.pace    = (typeof s.pace==='string'&&s.pace!=='-')?s.pace:null;
  g.paceN   = _enNum(s.pace_n);
  g.whLap   = _enNum(s.wh_per_lap);
  g.lastLap = _enNum(s.last_lap_wh);
  g.laps    = _enNum(s.laps_done);
  g.pct     = _enNum(s.battery_pct);
  g.boot    = _enNum(s.wh_total_since_boot);

  // Pre-race fallbacks. drive_status is published at 2 Hz whether or not the
  // strategist is alive, so the calm state can still show something true even
  // if /ethon/strategy is missing entirely.
  if(g.boot==null) g.boot=_enNum(d.energy_wh);
  // supply_v is measured at the Krakens; status.battery_v is the same number
  // relayed through the dashboard's status block, so it is the fallback rather
  // than a second opinion.
  g.volts = _enNum(d.supply_v);
  if(g.volts==null) g.volts=_enNum(st.battery_v);

  // Total race length, reconstructed. The strategist publishes elapsed_s and
  // remaining_s but not race_minutes, and remaining_s is clamped at 0 — so
  // once the clock expires this sum collapses to elapsed and the target marker
  // correctly parks on the full budget rather than running off the end.
  const total=(g.elapsed!=null&&g.left_s!=null)?(g.elapsed+g.left_s):null;
  g.raceS=(total!=null&&total>0)?total:null;

  // Where the schedule says you should be RIGHT NOW. Straight-line pacing is
  // what the strategist itself budgets against (budget_wh_min is a flat rate),
  // so anything cleverer here would disagree with the number next to it.
  g.targetNow=null;
  if(g.raceOn&&g.budget!=null&&g.elapsed!=null&&g.raceS!=null){
    g.targetNow=g.budget*Math.max(0,Math.min(1,g.elapsed/g.raceS));
  }
  // Positive = spent less than the schedule = energy in hand.
  g.slack=(g.targetNow!=null&&g.used!=null)?(g.targetNow-g.used):null;
  // Positive = projected to finish over budget.
  g.over=(g.proj!=null&&g.budget!=null)?(g.proj-g.budget):null;

  return g;
}

// The verdict. Three grades, and they are deliberately pinned to the
// strategist's own arithmetic so the console and the pit board can never tell
// two different stories:
//   projected <= budget                  -> ok    "ON BUDGET"
//   0 < overrun <= PACE_BAND of budget   -> warn  "MARGINAL"
//   overrun  >  PACE_BAND of budget      -> crit  "WILL NOT FINISH"
// Because projected = rate * race_minutes and budget = budget_rate *
// race_minutes, projected/budget is identically rate/budget_rate — so the crit
// grade fires on exactly the same condition as the strategist's "SLOW DOWN",
// and amber only ever appears inside the band it still calls ON TARGET. The
// strategist's own word is shown verbatim further down the panel; this pill
// grades severity, it does not form a second opinion.
function _enVerdict(g){
  if(g.err) return {lvl:'crit', txt:'STRATEGIST ERROR', col:_EN_C.crit};
  if(!g.raceOn) return {lvl:'idle', txt:'RACE NOT STARTED', col:_EN_C.ink3};
  // Already spent it. A rate extrapolation is irrelevant once the tank is
  // empty, so this outranks the projection.
  if(g.budget!=null&&g.used!=null&&g.used>=g.budget) return {lvl:'crit', txt:'OVER BUDGET', col:_EN_C.crit};
  if(g.left_s!=null&&g.left_s<=0) return {lvl:'idle', txt:'TIME UP', col:_EN_C.ink2};
  // The strategist withholds rate/projection until 30 s of race have run,
  // because a 5 s sample of a standing start projects nonsense. Say so rather
  // than inventing a verdict.
  if(g.proj==null||g.budget==null||g.budget<=0) return {lvl:'idle', txt:'MEASURING PACE', col:_EN_C.ink2};
  const r=g.proj/g.budget;
  if(r<=1.0) return {lvl:'ok', txt:'ON BUDGET', col:_EN_C.ok};
  if(r<=1.0+_EN_PACE_BAND) return {lvl:'warn', txt:'MARGINAL', col:_EN_C.warn};
  return {lvl:'crit', txt:'WILL NOT FINISH', col:_EN_C.crit};
}

// ── header: section label + verdict pill ──────────────────────────────────
function _enHeader(ctx,x,y,w,v){
  ctx.font='700 10px '+_EN_SANS;
  const tw=_enTrackedWidth(ctx,v.txt,0.8);
  const pw=tw+18, ph=18, px=x+w-pw, py=y-2;

  // The verdict outranks the section label: on a narrow canvas the label is
  // dropped rather than allowed to run underneath the pill. The panel heading
  // already says "Energy" in the DOM above the canvas, so nothing is lost.
  ctx.font='600 10px '+_EN_SANS;
  if(_enTrackedWidth(ctx,'ENERGY BUDGET',0.7)+14<=w-pw){
    _enLabel(ctx,'Energy budget',x,y+10,_EN_C.ink2,'left');
  }

  const alpha=(v.lvl==='crit')?_enPulse():1;

  ctx.save();
  ctx.globalAlpha=alpha;
  _enRR(ctx,px,py,pw,ph,9);
  ctx.fillStyle=(v.lvl==='idle')?_EN_C.panel2:_enRGBA(v.col,0.16);
  ctx.fill();
  ctx.strokeStyle=(v.lvl==='idle')?_EN_C.rule2:_enRGBA(v.col,0.55);
  ctx.lineWidth=1;ctx.stroke();
  ctx.restore();

  ctx.fillStyle=v.col;ctx.textBaseline='alphabetic';
  _enTracked(ctx,v.txt,px+pw/2,py+12.5,0.8,'center');
}

// ── the three-up metric strip ─────────────────────────────────────────────
// Cell value sizes track the panel width so the numbers stay large on the
// console's 1/3-width column and do not collide on a narrow pit-board window.
function _enCell(ctx,x,w,yTop,hh,label,value,unit,col,sub,subCol){
  // Bounded by the cell WIDTH so three cells never collide, and by the cell
  // HEIGHT so the sub-line underneath is not sitting in the digits' descenders
  // when the panel is short.
  const big=Math.max(16,Math.min(27,Math.round(w*0.30),hh-24));
  ctx.font='600 10px '+_EN_SANS;
  // clipped a little short of the cell: _enLabel adds manual letter-spacing on
  // top of whatever measureText reported here.
  _enLabel(ctx,_enClip(ctx,String(label).toUpperCase(),Math.max(12,w-9)),x,yTop+9,_EN_C.ink3,'left');

  ctx.textBaseline='alphabetic';
  ctx.textAlign='left';
  ctx.font='700 '+big+'px '+_EN_MONO;
  const vy=yTop+9+big+2;
  ctx.fillStyle=col||_EN_C.ink;
  ctx.fillText(value,x,vy);
  if(unit){
    const vw=ctx.measureText(value).width;
    ctx.font='600 10px '+_EN_SANS;
    ctx.fillStyle=_EN_C.ink3;
    ctx.fillText(unit,x+vw+4,vy);
  }
  if(sub){
    ctx.font='500 10px '+_EN_SANS;
    ctx.fillStyle=subCol||_EN_C.ink3;
    ctx.fillText(_enClip(ctx,sub,w),x,Math.min(yTop+hh-1,vy+12));
  }
}

function _enStrip(ctx,x,y,w,h,g,v){
  const cw=w/3;
  ctx.save();
  ctx.strokeStyle=_EN_C.rule;ctx.lineWidth=1;
  ctx.beginPath();
  // Hairlines are drawn on the half pixel so they stay 1 device pixel wide
  // after the devicePixelRatio transform instead of blurring across two.
  ctx.moveTo(Math.round(x+cw)+0.5,y+2);ctx.lineTo(Math.round(x+cw)+0.5,y+h-2);
  ctx.moveTo(Math.round(x+2*cw)+0.5,y+2);ctx.lineTo(Math.round(x+2*cw)+0.5,y+h-2);
  ctx.stroke();
  ctx.restore();

  const pad=10, cellW=cw-pad;

  if(!g.raceOn){
    // Calm state. Everything shown here is real and available before the
    // clock starts: the configured budget, the pack voltage at the Krakens,
    // and the full race length the strategist is holding ready.
    _enCell(ctx,x,cellW,y,h,'Budget',_enFix(g.budget,0),'Wh',_EN_C.accent,
            g.pct!=null?(_enFix(g.pct,0)+'% pack estimate'):null,_EN_C.ink3);
    _enCell(ctx,x+cw+pad,cellW,y,h,'Battery',_enFix(g.volts,1),'V',_EN_C.actual,
            g.boot!=null?(_enFix(g.boot,1)+' Wh since boot'):null,_EN_C.ink3);
    _enCell(ctx,x+2*cw+pad,cellW,y,h,'Race clock',_enMmss(g.left_s),null,_EN_C.ink2,
            'ready',_EN_C.ink3);
    return;
  }

  // Projected finish. Magenta would be the honest colour for an extrapolated
  // measurement, but this is THE number the verdict is made of, so it carries
  // the verdict colour and the bar below carries the magenta.
  _enCell(ctx,x,cellW,y,h,'Projected',_enFix(g.proj,0),'Wh',
          g.proj==null?_EN_C.ink3:v.col,
          g.proj==null?'needs 30 s of race':('budget '+_enFix(g.budget,0)+' Wh'),_EN_C.ink3);

  // The gap, signed, because "480 vs 512" makes you do arithmetic while
  // "+32 OVER" does not.
  let gapTxt='—', gapCol=_EN_C.ink3, gapLbl='vs budget', gapSub=null;
  if(g.over!=null){
    const o=g.over;
    if(Math.abs(o)<0.5){ gapTxt='0'; }
    else gapTxt=(o>0?'+':'')+_enFix(o,0);
    gapCol=(o>0)?v.col:_EN_C.ok;
    gapLbl=(o>0)?'over budget':'under budget';
    gapSub=(o>0)?'at this burn rate':'spare at the flag';
  }
  _enCell(ctx,x+cw+pad,cellW,y,h,gapLbl,gapTxt,'Wh',gapCol,gapSub,_EN_C.ink3);

  _enCell(ctx,x+2*cw+pad,cellW,y,h,'Time left',_enMmss(g.left_s),null,_EN_C.ink,
          g.elapsed!=null?(_enMmss(g.elapsed)+' elapsed'):null,_EN_C.ink3);
}

// ── the budget bar ────────────────────────────────────────────────────────
// One rail, 0 Wh on the left, the budget line fixed near the right with the
// over-budget zone hatched beyond it. Magenta fill = spent. Teal marker =
// where the clock says you should be. Hollow magenta caret = where this burn
// rate lands you. If the caret is left of the budget line you finish.
function _enBudgetBar(ctx,x,y,w,h,g){
  const barH=Math.max(16,Math.min(22,Math.round(h*0.40)));
  const barY=y+14;
  const budget=(g.budget!=null&&g.budget>0)?g.budget:null;

  // Header line of the block: spent on the left in magenta, budget on the
  // right in teal, so the bar underneath needs no legend.
  ctx.textBaseline='alphabetic';ctx.textAlign='left';
  const lw=_enLabel(ctx,g.raceOn?'Spent':'Since boot',x,y+9,_EN_C.ink3,'left');
  ctx.font='700 11px '+_EN_MONO;ctx.fillStyle=_EN_C.actual;
  ctx.fillText(_enFix(g.raceOn?g.used:g.boot,1)+' Wh',x+lw+7,y+9);

  ctx.textAlign='right';
  ctx.font='700 11px '+_EN_MONO;ctx.fillStyle=_EN_C.accent;
  const bTxt=_enFix(budget,0)+' Wh';
  ctx.fillText(bTxt,x+w,y+9);
  const bw=ctx.measureText(bTxt).width;
  ctx.textAlign='left';
  _enLabel(ctx,'Budget',x+w-bw-7,y+9,_EN_C.ink3,'right');

  // Track.
  ctx.save();
  _enRR(ctx,x,barY,w,barH,5);
  ctx.fillStyle=_EN_C.panel2;ctx.fill();
  ctx.strokeStyle=_EN_C.rule;ctx.lineWidth=1;ctx.stroke();
  ctx.restore();

  if(budget==null){
    ctx.font='500 11px '+_EN_SANS;ctx.fillStyle=_EN_C.ink3;ctx.textAlign='center';
    ctx.fillText('no budget published',x+w/2,barY+barH/2+4);
    ctx.textAlign='left';
    return;
  }

  // Domain. The budget line sits at a FIXED fraction of the rail so it never
  // moves between frames; everything past it is the over-budget zone.
  const dmax=budget*(1+_EN_OVER_ZONE);
  const X=function(wh){ return x+Math.max(0,Math.min(1,wh/dmax))*w; };
  const xb=X(budget);

  // Over-budget zone.
  ctx.save();
  _enRR(ctx,x,barY,w,barH,5);ctx.clip();
  ctx.fillStyle=_enRGBA(_EN_C.crit,0.07);
  ctx.fillRect(xb,barY,x+w-xb,barH);
  _enHatch(ctx,xb,barY,x+w-xb,barH,_EN_C.crit,0.16,7);
  ctx.restore();

  // Spent fill. Before the race the meter shown is wh_total_since_boot, which
  // the strategist EXCLUDES from the race total (it re-zeros at START RACE —
  // see _on_start in race_services.py), so it is drawn dimmed to say "this
  // does not count against the budget yet".
  const spent=g.raceOn?g.used:g.boot;
  if(spent!=null&&spent>0){
    ctx.save();
    _enRR(ctx,x,barY,w,barH,5);ctx.clip();
    ctx.globalAlpha=g.raceOn?1:0.40;
    const xs=X(spent);
    const grd=ctx.createLinearGradient(x,0,xs,0);
    grd.addColorStop(0,_enRGBA(_EN_C.actual,0.55));
    grd.addColorStop(1,_EN_C.actual);
    ctx.fillStyle=grd;
    ctx.fillRect(x,barY,Math.max(2,xs-x),barH);
    ctx.restore();
  }

  // Budget line: the wall. Drawn over the fill, under the markers.
  ctx.save();
  ctx.strokeStyle=_EN_C.accent;ctx.lineWidth=2;
  ctx.beginPath();ctx.moveTo(Math.round(xb),barY-3);ctx.lineTo(Math.round(xb),barY+barH+3);ctx.stroke();
  ctx.restore();

  // Target-now marker: teal, planned. A filled down-triangle above the rail
  // plus a dashed line through it, so it reads even where it overlaps the
  // magenta fill.
  if(g.targetNow!=null){
    const xt=X(g.targetNow);
    ctx.save();
    ctx.setLineDash([3,3]);ctx.strokeStyle=_enRGBA(_EN_C.accent,0.85);ctx.lineWidth=1.5;
    ctx.beginPath();ctx.moveTo(xt,barY);ctx.lineTo(xt,barY+barH);ctx.stroke();
    ctx.restore();
    ctx.fillStyle=_EN_C.accent;
    ctx.beginPath();
    ctx.moveTo(xt,barY-1);ctx.lineTo(xt-4.5,barY-7);ctx.lineTo(xt+4.5,barY-7);
    ctx.closePath();ctx.fill();
  }

  // Projection caret: hollow magenta, below the rail, with a dashed run from
  // where you are now to where you end up. Hollow and dashed on purpose —
  // this Wh has not been spent yet.
  if(g.proj!=null&&g.used!=null&&g.raceOn){
    const xu=X(g.used), xp=X(g.proj);
    ctx.save();
    ctx.setLineDash([4,3]);ctx.strokeStyle=_enRGBA(_EN_C.actual,0.65);ctx.lineWidth=1.5;
    ctx.beginPath();ctx.moveTo(xu,barY+barH+6);ctx.lineTo(xp,barY+barH+6);ctx.stroke();
    ctx.restore();
    ctx.save();
    ctx.strokeStyle=_EN_C.actual;ctx.lineWidth=1.5;
    ctx.beginPath();
    ctx.moveTo(xp,barY+barH+2);ctx.lineTo(xp-4.5,barY+barH+9);ctx.lineTo(xp+4.5,barY+barH+9);
    ctx.closePath();ctx.stroke();
    ctx.restore();
    // Clamped projections must say so, or a caret parked on the right-hand
    // edge reads as "just barely over" when it may be double the budget.
    if(g.proj>dmax){
      ctx.fillStyle=_EN_C.crit;ctx.font='700 10px '+_EN_MONO;ctx.textAlign='right';
      ctx.fillText('»',x+w-1,barY+barH+10);ctx.textAlign='left';
    }
  }

  // Footer line of the block: the slack, in words, because "in hand" and
  // "overspent" need no interpretation at 60 km/h.
  const fy=y+h-1;
  ctx.font='500 10px '+_EN_SANS;
  if(!g.raceOn){
    ctx.fillStyle=_EN_C.ink3;
    ctx.fillText(_enClip(ctx,'not counted — START RACE re-zeros this meter',w),x,fy);
  }else if(g.slack!=null){
    const ahead=g.slack>=0;
    ctx.fillStyle=_EN_C.ink3;
    const head='▲ target now  ';
    ctx.fillText(head,x,fy);
    const hw=ctx.measureText(head).width;
    ctx.font='700 10px '+_EN_MONO;
    ctx.fillStyle=ahead?_EN_C.ok:_EN_C.crit;
    const num=_enFix(Math.abs(g.slack),1)+' Wh';
    // Measured while the mono face is still selected: mono is the wider of the
    // two, so measuring after the switch back to sans would tuck the tail of
    // the sentence under the digits.
    const nw=ctx.measureText(num).width;
    ctx.fillText(num,x+hw,fy);
    ctx.font='500 10px '+_EN_SANS;
    ctx.fillStyle=_EN_C.ink3;
    ctx.fillText(ahead?' in hand':' overspent',x+hw+nw+1,fy);
  }
}

// ── burn rate rail ────────────────────────────────────────────────────────
// Actual Wh/min against the flat budget rate. One rail rather than two so the
// comparison is a distance, not a subtraction.
function _enRateRail(ctx,x,y,w,h,g){
  ctx.textBaseline='alphabetic';ctx.textAlign='left';
  const lw=_enLabel(ctx,'Burn rate',x,y+9,_EN_C.ink3,'left');
  ctx.font='700 12px '+_EN_MONO;ctx.fillStyle=(g.rate==null)?_EN_C.ink3:_EN_C.actual;
  ctx.fillText(_enFix(g.rate,1),x+lw+7,y+9);
  const nw=ctx.measureText(_enFix(g.rate,1)).width;
  ctx.font='600 10px '+_EN_SANS;ctx.fillStyle=_EN_C.ink3;
  ctx.fillText('Wh/min',x+lw+11+nw,y+9);

  ctx.textAlign='right';
  ctx.font='700 12px '+_EN_MONO;ctx.fillStyle=_EN_C.accent;
  const tTxt=_enFix(g.target,1);
  ctx.fillText(tTxt,x+w,y+9);
  const tw=ctx.measureText(tTxt).width;
  ctx.textAlign='left';
  _enLabel(ctx,'Target',x+w-tw-7,y+9,_EN_C.ink3,'right');

  const railY=y+15, railH=8;
  ctx.save();
  _enRR(ctx,x,railY,w,railH,4);
  ctx.fillStyle=_EN_C.panel2;ctx.fill();
  ctx.strokeStyle=_EN_C.rule;ctx.lineWidth=1;ctx.stroke();
  ctx.restore();

  // Scale on the larger of the two so neither can leave the rail, with
  // headroom so a rate exactly on target does not paint the whole thing.
  const hi=Math.max(g.rate||0,g.target||0);
  if(hi>0){
    const dmax=hi*1.35;
    const X=function(v){ return x+Math.max(0,Math.min(1,v/dmax))*w; };
    if(g.rate!=null&&g.rate>0){
      ctx.save();
      _enRR(ctx,x,railY,w,railH,4);ctx.clip();
      ctx.fillStyle=_EN_C.actual;
      ctx.fillRect(x,railY,Math.max(2,X(g.rate)-x),railH);
      ctx.restore();
    }
    if(g.target!=null&&g.target>0){
      const xt=X(g.target);
      ctx.save();
      ctx.strokeStyle=_EN_C.accent;ctx.lineWidth=2;
      ctx.beginPath();ctx.moveTo(Math.round(xt),railY-3);ctx.lineTo(Math.round(xt),railY+railH+3);ctx.stroke();
      ctx.restore();
    }
  }

  // Sub-line. The strategist's own verdict word is echoed verbatim so the
  // console and the pit board are quoting one source; the pill above grades
  // it, this line attributes it.
  const fy=y+h-1;
  ctx.font='500 10px '+_EN_SANS;ctx.fillStyle=_EN_C.ink3;ctx.textAlign='left';

  // Per-lap energy on the right when there is any, because it is the number
  // you actually act on: it turns "12 Wh over" into "one slower lap". Measured
  // first so the attribution line on the left can be clipped around it.
  let right='';
  if(g.whLap!=null){
    right=_enFix(g.whLap,1)+' Wh/lap';
    if(g.laps!=null&&g.laps>0) right+='  ·  '+_enFix(g.laps,0)+(g.laps===1?' lap':' laps');
  }
  const rw=right?ctx.measureText(right).width+12:0;

  let left;
  if(g.err) left='strategist: '+g.err;
  else if(g.pace) left='strategist: '+g.pace;
  else if(g.raceOn) left='strategist: measuring';
  else left='flat pacing — '+_enFix(g.target,1)+' Wh/min all race';
  ctx.fillText(_enClip(ctx,left,w-rw),x,fy);

  if(right){
    ctx.textAlign='right';
    ctx.fillText(right,x+w,fy);
    ctx.textAlign='left';
  }
}

// ── layout + paint ────────────────────────────────────────────────────────
// The panel is 220 CSS px tall on the console and 230 on the pit board, and
// both pages drop it to full width under 1000 px. Rather than hardcode those,
// the blocks state what they need and the leftover is spread between them;
// blocks fall away from the bottom up if the canvas is ever made shorter, so a
// squeezed panel loses detail instead of overprinting itself.
function _enPaint(ctx,w,h,g,v){
  ctx.clearRect(0,0,w,h);
  ctx.fillStyle=_EN_C.panel;ctx.fillRect(0,0,w,h);
  ctx.textBaseline='alphabetic';ctx.textAlign='left';
  ctx.lineJoin='round';

  const pad=Math.max(10,Math.min(16,Math.round(w*0.034)));
  const x=pad, iw=Math.max(40,w-2*pad);
  const avail=h-2*pad;

  const headH=16, stripH=46, barH=56, railH=40;
  let showBar=true, showRail=true;
  if(avail<headH+stripH+barH+railH+16) showRail=false;
  if(avail<headH+stripH+barH+8){ showBar=false; showRail=false; }

  const need=headH+stripH+(showBar?barH:0)+(showRail?railH:0);
  const slots=1+(showBar?1:0)+(showRail?1:0);
  // Gaps are capped low on purpose: slack is worth more as a bigger number in
  // the strip than as more air between blocks. Those three figures are what
  // gets read from across the pit box.
  const gap=Math.max(6,Math.min(12,Math.floor((avail-need)/Math.max(1,slots))));
  const extra=Math.max(0,avail-need-gap*slots);
  const sH=stripH+Math.min(16,extra);

  let y=pad;
  _enHeader(ctx,x,y,iw,v);
  y+=headH+gap;
  _enStrip(ctx,x,y,iw,sH,g,v);
  y+=sH+gap;
  if(showBar){
    ctx.save();
    ctx.strokeStyle=_EN_C.rule;ctx.lineWidth=1;
    ctx.beginPath();ctx.moveTo(x,Math.round(y-gap/2)+0.5);ctx.lineTo(x+iw,Math.round(y-gap/2)+0.5);ctx.stroke();
    ctx.restore();
    _enBudgetBar(ctx,x,y,iw,barH,g);
    y+=barH+gap;
  }
  if(showRail) _enRateRail(ctx,x,y,iw,railH,g);
}

// A canvas that has never been laid out (hidden tab, panel not yet in the
// flow) reports 0x0; painting into it would throw on the gradient and leave a
// blank rectangle behind when the tab is shown. The page redraws on tab
// change, so returning quietly is correct.
function drawEnergy(id,strategy,drive,status){
  const s=setupCanvas(id);
  if(!s||!s.ctx||!s.w||!s.h) return;
  const ctx=s.ctx, w=s.w, h=s.h;
  try{
    const g=_enModel(strategy,drive,status);
    _enPaint(ctx,w,h,g,_enVerdict(g));
  }catch(e){
    // Last-ditch: the caller already wraps draws, but a half-painted energy
    // panel showing a stale bar is worse than one that admits it is broken,
    // so repaint the surface and say so. Everything in here is primitive
    // enough that it cannot throw in turn.
    try{
      ctx.clearRect(0,0,w,h);
      ctx.fillStyle=_EN_C.panel;ctx.fillRect(0,0,w,h);
      ctx.font='600 11px '+_EN_SANS;ctx.fillStyle=_EN_C.crit;ctx.textAlign='left';
      ctx.fillText('energy panel failed to draw — see console',12,Math.round(h/2));
    }catch(e2){}
    if(!drawEnergy._logged){ drawEnergy._logged=1; try{ console.error('drawEnergy',e); }catch(e3){} }
  }
}
drawEnergy._logged=0;

// ══ Ethon pit board ════════════════════════════════════════════════════════
// A standalone document, so it carries its own copies of the small helpers
// rather than sharing a bundle with the console page. The only piece of real
// drawing code it borrows is drawEnergy, which is included above this block.
function q(id){return document.getElementById(id);}
function el(t,c,txt){const e=document.createElement(t);if(c)e.className=c;if(txt!=null)e.textContent=txt;return e;}

function toast(msg,bad){
  const t=q('toast');if(!t)return;
  t.textContent=msg;t.className=bad?'toast show err':'toast show';
  clearTimeout(toast._t);toast._t=setTimeout(()=>{t.className='toast';},2800);
}

function setupCanvas(id){
  const cv=q(id);if(!cv)return null;
  const w=cv.clientWidth,h=cv.clientHeight;
  if(!w||!h)return null;
  const dpr=window.devicePixelRatio||1;
  const bw=Math.round(w*dpr),bh=Math.round(h*dpr);
  if(cv.width!==bw||cv.height!==bh){cv.width=bw;cv.height=bh;}
  const ctx=cv.getContext('2d');
  ctx.setTransform(dpr,0,0,dpr,0,0);
  return {ctx:ctx,w:w,h:h,cv:cv};
}

function safe(label,fn){
  try{fn();}catch(e){
    if(!safe._seen[label]){safe._seen[label]=1;console.error('draw:'+label,e);}
  }
}
safe._seen={};

function mmss(s){
  if(s==null)return '—';
  s=Math.max(0,Math.round(s));
  const m=Math.floor(s/60);
  return m+':'+String(s%60).padStart(2,'0');
}
function fmtlap(s){
  if(s==null)return '—';
  const m=Math.floor(s/60);
  return m+':'+(s-60*m).toFixed(1).padStart(4,'0');
}
function num(v,dec){return (v==null||!isFinite(v))?'—':(+v).toFixed(dec==null?0:dec);}

async function act(action){
  try{
    const r=await fetch('/api/action',{method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify({action:action})});
    const j=await r.json();
    toast(j.ok?(action+' — sent'):((j.reason||action)+' — rejected'),!j.ok);
  }catch(e){toast(action+' failed — no response from the vehicle',true);}
}
function confirmRace(){
  if(confirm('Start the 70-minute race clock? This begins the energy budget.'))act('race_start');
}
function estop(){act('estop');}

function tile(k,v,unit,sub,cls){
  const t=el('div','pittile');
  t.appendChild(el('div','k',k));
  const val=el('div','v'+(cls?' '+cls:''));
  val.appendChild(document.createTextNode(v));
  if(unit){const u=el('small',null,unit);val.appendChild(u);}
  t.appendChild(val);
  if(sub)t.appendChild(el('div','s',sub));
  return t;
}

// The verdict answers one question from across the garage: is the car going to
// finish on the energy it has? pace_n is the strategist's own call (-1 behind,
// 0 on plan, +1 ahead) so the board follows it rather than second-guessing.
function fmtVerdict(strat){
  const box=q('verdict'),lead=q('v_lead'),clock=q('v_clock'),note=q('v_note');
  if(!strat.race_on){
    box.className='verdict';
    note.textContent='standby';
    lead.textContent='RACE NOT STARTED';
    clock.textContent=num(strat.wh_budget,0)+' Wh budget · '+mmss(strat.remaining_s)+' clock';
    return;
  }
  const n=strat.pace_n;
  box.className='verdict '+(n<0?'down':(n>0?'up':'flat'));
  note.textContent='pace';
  lead.textContent=(strat.pace||'—').toUpperCase();
  clock.textContent=mmss(strat.remaining_s)+' remaining · lap '+num(strat.laps_done,0);

  const gap=q('v_gap');
  if(gap){
    if(strat.projected_wh!=null&&strat.wh_budget!=null){
      const d=strat.projected_wh-strat.wh_budget;
      gap.textContent=(d>0?'+':'')+d.toFixed(0)+' Wh vs budget at this rate';
      gap.style.color=d>0?'var(--crit)':'var(--ok)';
    }else{gap.textContent='';}
  }
}

function fmtChips(s){
  const box=q('chips');if(!box)return;
  const out=[];
  const add=(txt,cls)=>{out.push(el('span','chip'+(cls?' '+cls:''),txt));};
  if(s.estop||s.estop_latched)add('E-STOP','crit');
  add(s.armed===true?'ARMED':'DISARMED',s.armed===true?'ok':'');
  add(s.gps_fix===true?'GPS FIX':'GPS NO FIX',s.gps_fix===true?'ok':'warn');
  if(s.battery_v!=null)add(s.battery_v.toFixed(1)+' V',
    s.battery_v<10.5?'crit':(s.battery_v<11.5?'warn':'ok'));
  box.innerHTML='';
  for(const c of out)box.appendChild(c);
}

async function tick(){
  try{
    const r=await fetch('/api/state',{cache:'no-store'});
    const st=await r.json();
    const beat=q('beat'),bt=q('beattxt');
    if(beat)beat.className='beat up';
    if(bt)bt.textContent='live';

    const tp=st.topics||{};
    const s=st.status||{};
    const strat=(tp['/ethon/strategy']||{}).value||{};
    const lap=(tp['/ethon/lap']||{}).value||{};
    const corr=(tp['/ethon/corridor']||{}).value||{};
    const drive=(tp['/ethon/drive_status']||{}).value||{};
    const health=(tp['/ethon/health']||{}).value||{};

    fmtChips(s);
    fmtVerdict(strat);
    safe('energy',()=>drawEnergy('pitenergy',strat,drive,s));

    const temps=Object.keys(drive.motors||{})
      .map(k=>(drive.motors[k]||{}).temp_c)
      .filter(x=>x!=null&&isFinite(x));
    const hottest=temps.length?Math.max.apply(null,temps):null;

    const overBurn=(strat.rate_wh_min!=null&&strat.budget_wh_min)
      ? strat.rate_wh_min/strat.budget_wh_min : null;

    const tiles=q('tiles');tiles.innerHTML='';
    tiles.appendChild(tile('Battery',num(strat.battery_pct,0),'%',
      num(strat.wh_remaining,0)+' Wh left of '+num(strat.wh_budget,0)+' (estimate)',
      strat.battery_pct==null?'':(strat.battery_pct<15?'crit':(strat.battery_pct<30?'warn':'ok'))));
    tiles.appendChild(tile('Pack voltage',num(drive.supply_v,1),'V','measured at the Krakens',
      drive.supply_v==null?'':(drive.supply_v<10.5?'crit':(drive.supply_v<11.5?'warn':''))));
    tiles.appendChild(tile('Burn rate',num(strat.rate_wh_min,1),'Wh/min',
      'budget '+num(strat.budget_wh_min,1)+' Wh/min',
      overBurn==null?'':(overBurn>1.15?'crit':(overBurn>1.0?'warn':'ok'))));
    tiles.appendChild(tile('Speed',num(s.speed_kmh,1),'km/h',''));
    tiles.appendChild(tile('Lap',num(lap.lap,0),'',
      'last '+fmtlap(lap.last_s)+' · best '+fmtlap(lap.best_s)));
    tiles.appendChild(tile('Energy / lap',num(strat.wh_per_lap,1),'Wh',
      'last lap '+(strat.last_lap_wh!=null?num(strat.last_lap_wh,1)+' Wh':'—')));
    tiles.appendChild(tile('Motor temp',num(hottest,0),'°C','hottest of '+temps.length,
      hottest==null?'':(hottest>90?'crit':(hottest>70?'warn':'ok'))));
    tiles.appendChild(tile('Projected',num(strat.projected_wh,0),'Wh',
      'at finish vs '+num(strat.wh_budget,0)+' Wh budget',
      (strat.projected_wh!=null&&strat.wh_budget)
        ? (strat.projected_wh>strat.wh_budget?'crit':'ok') : ''));

    const al=q('alerts');al.innerHTML='';
    const pil=(txt,cls)=>al.appendChild(el('div','pital'+(cls?' '+cls:''),txt));
    pil(s.estop||s.estop_latched?'E-STOP':'estop clear',(s.estop||s.estop_latched)?'crit':'ok');
    pil(s.armed?'armed':'disarmed',s.armed?'ok':'');
    pil(s.gps_fix===true?'GPS ok':'GPS no fix',s.gps_fix===true?'ok':'crit');
    pil('corridor: '+(corr.state||'—'),
        (corr.state==='warn'||corr.state==='off')?'warn':'');
    pil(s.can_ok===false?'CAN down':'CAN ok',s.can_ok===false?'crit':'ok');
    for(const a of ((health&&health.alerts)||[]))pil(String(a),'warn');
  }catch(e){
    const beat=q('beat'),bt=q('beattxt');
    if(beat)beat.className='beat down';
    if(bt)bt.textContent='disconnected';
  }
}

let rT=null;
window.addEventListener('resize',()=>{clearTimeout(rT);rT=setTimeout(tick,150);});
tick();setInterval(tick,1000);
</script>
</body>
</html>
"""


class Handler(BaseHTTPRequestHandler):
    server_version = "EthonDash/1.0"

    def log_message(self, *_a):
        pass

    def _send(self, code, body, ctype):
        if isinstance(body, str):
            body = body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, obj, code=200):
        self._send(code, json.dumps(obj), "application/json")

    def _send_cam(self, src):
        """Serve one annotated camera frame written by birdseye_fusion.

        `src` is matched against the discovered source list rather than being
        pasted into a path, so a crafted query cannot read arbitrary files.
        """
        if src not in _list_cams():
            self._json({"ok": False, "reason": "unknown source"}, 404)
            return
        try:
            with open(os.path.join(CAM_DIR, CAM_PREFIX + src + ".jpg"),
                      "rb") as f:
                blob = f.read()
        except OSError:
            self._json({"ok": False, "reason": "no frame yet"}, 404)
            return
        self._send(200, blob, "image/jpeg")

    def _send_calib_snapshot(self, src):
        """Serve a calibration snapshot at FULL resolution, no downscaling --
        the /calib page's pixel picker must read coordinates off exactly the
        same pixels calibrate_homography.py itself wrote, or the u,v fed to
        --solve won't match what it captured.

        `src` is matched against _list_calib_snapshots() rather than pasted
        into a path, same defensive pattern as _send_cam.
        """
        if src not in _list_calib_snapshots():
            self._json({"ok": False, "reason": "unknown snapshot"}, 404)
            return
        try:
            with open(os.path.join(CALIB_DIR, src + "_snapshot.jpg"),
                      "rb") as f:
                blob = f.read()
        except OSError:
            self._json({"ok": False, "reason": "read failed"}, 404)
            return
        self._send(200, blob, "image/jpeg")

    def _body(self):
        try:
            n = int(self.headers.get("Content-Length", 0))
        except (TypeError, ValueError):
            n = 0
        if not n:
            return {}
        try:
            return json.loads(self.rfile.read(n).decode("utf-8"))
        except (ValueError, UnicodeDecodeError):
            return {}

    def do_GET(self):
        node = self.server.node
        path = urlparse(self.path)
        p = path.path
        if p in ("/", "/dashboard", "/index.html"):
            self._send(200, PAGE, "text/html; charset=utf-8")
        elif p == "/pit":
            self._send(200, PIT_PAGE, "text/html; charset=utf-8")
        elif p == "/v2":
            self._send(200, PAGE_V2, "text/html; charset=utf-8")
        elif p == "/pit2":
            self._send(200, PIT_PAGE_V2, "text/html; charset=utf-8")
        elif p == "/replay":
            self._send(200, REPLAY_PAGE, "text/html; charset=utf-8")
        elif p == "/calib":
            self._send(200, CALIB_PAGE, "text/html; charset=utf-8")
        elif p == "/api/cams":
            self._json({"cams": _list_cams()})
        elif p == "/api/cam":
            q = parse_qs(path.query)
            self._send_cam((q.get("src") or [""])[0])
        elif p == "/api/calib_snapshots":
            self._json({"snapshots": _list_calib_snapshots()})
        elif p == "/api/calib_snapshot":
            q = parse_qs(path.query)
            self._send_calib_snapshot((q.get("src") or [""])[0])
        elif p == "/api/state":
            self._json(node.state())
        elif p == "/api/selftest":
            self._json(node.selftest())
        elif p == "/api/sessions":
            self._json({"sessions": node.list_sessions()})
        elif p == "/api/session":
            q = parse_qs(path.query)
            data = node.read_session((q.get("f") or [""])[0])
            if data is None:
                self._json({"ok": False, "reason": "bad session"}, 404)
            else:
                self._json(data)
        elif p == "/api/history":
            self._json(node.history())
        elif p == "/api/logs":
            q = parse_qs(path.query)
            try:
                since = int((q.get("since") or ["0"])[0])
            except ValueError:
                since = 0
            self._json(node.logs(since))
        elif p == "/api/params":
            q = parse_qs(path.query)
            tgt = (q.get("node") or [""])[0]
            if tgt not in PARAM_NODES:
                self._json({"ok": False, "reason": "unknown node"}, 400)
                return
            params = node.list_params(tgt)
            if params is None:
                self._json({"ok": False, "reason": "node unreachable"}, 200)
            else:
                self._json({"ok": True, "node": tgt, "params": params})
        elif p == "/favicon.ico":
            self._send(204, b"", "image/x-icon")
        else:
            self._json({"ok": False, "reason": "not found"}, 404)

    def do_POST(self):
        node = self.server.node
        p = urlparse(self.path).path
        body = self._body()
        if p == "/api/action":
            ok, reason = node.action(body.get("action", ""), body.get("arg"))
            self._json({"ok": ok, "reason": reason})
        elif p == "/api/drivetest":
            ok, reason = node.drive_test(body.get("duty", 0.0))
            self._json({"ok": ok, "reason": reason})
        elif p == "/api/steertest":
            ok, reason = node.steer_test(body.get("deg", 0.0))
            self._json({"ok": ok, "reason": reason})
        elif p == "/api/param":
            tgt = body.get("node", "")
            if tgt not in PARAM_NODES:
                self._json({"ok": False, "reason": "unknown node"}, 400)
                return
            ok, reason = node.set_param(
                tgt, body.get("name", ""), body.get("type"), body.get("value"))
            self._json({"ok": ok, "reason": reason})
        else:
            self._json({"ok": False, "reason": "not found"}, 404)


def main():
    rclpy.init()
    node = EthonDashboard()
    exe = MultiThreadedExecutor(num_threads=4)
    exe.add_node(node)
    spin = threading.Thread(target=exe.spin, daemon=True)
    spin.start()
    time.sleep(2.0)
    node.rescan()
    httpd = ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), Handler)
    httpd.node = node
    node.get_logger().info("HTTP server listening on 0.0.0.0:%d" % HTTP_PORT)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        httpd.shutdown()
        exe.shutdown()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
