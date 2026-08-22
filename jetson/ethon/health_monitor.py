#!/usr/bin/env python3
"""
Ethon health monitor — watchdog for the Jetson autonomy stack.

Monitors at 1 Hz and publishes a single JSON object on /ethon/health:

    {"ok": bool, "topics": {...rates}, "system": {...}, "perc1": bool,
     "alerts": [strings]}

What is watched:
  - Topic liveness: actual receive rate of /ethon/cones, /cmd_vel and
    /ethon/drive_status measured over a sliding window and compared to
    expected minimums. A topic with no traffic in the window is "stale".
  - System: CPU% (/proc/stat deltas), RAM used/total (/proc/meminfo),
    disk free on /, all thermal zones (CPU/GPU temps + throttle hint
    above 85 C).
  - Jetson clocks from /sys/kernel/debug/bpmp/debug/clk when readable
    (needs root) — skipped silently otherwise.
  - perc-1 Pi reachability via a TCP connect probe every 10 s, done in
    a background thread so the 1 Hz timer never blocks on the network.

ESTOP reflex (latched in ethon_drive, published here exactly once):
  - any motor temp reported on /ethon/drive_status above 90 C, OR
  - /ethon/cones silent for > 5 s while /cmd_vel was recently nonzero
    (i.e. perception died while the vehicle is being driven).

Degradations are logged at WARN once on appearance (and recovery at
INFO) so the journal stays readable; the full state is always in the
JSON message for the HUD / telemetry logger.

Runs as a node inside ethon-stack.service (ros2 launch ethon_stack.launch.py)
alongside birdseye_fusion / cone_corridor_planner / ethon_drive -- there is no
separate ethon-health.service, despite what an earlier version of this
docstring claimed.
"""

import glob
import json
import os
import shutil
import socket
import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseArray, Twist
from std_msgs.msg import Bool, String

# ── cadence ──────────────────────────────────────────────────────────
TICK_PERIOD_S    = 1.0     # publish / evaluate rate
RATE_WINDOW_S    = 3.0     # sliding window for topic-rate measurement

# ── topic liveness: (topic, msg type, minimum acceptable Hz) ─────────
TOPIC_SPECS = (
    ("/ethon/cones",        PoseArray, 5.0),
    ("/cmd_vel",            Twist,     10.0),
    ("/ethon/drive_status", String,    1.0),
)

# ── estop reflex thresholds ──────────────────────────────────────────
MOTOR_TEMP_ESTOP_C = 90.0  # any motor above this → estop
CONES_SILENT_S     = 5.0   # perception silence while driving → estop
CMD_ACTIVE_S       = 2.0   # "recently nonzero" window for /cmd_vel
CMD_EPS            = 0.01  # |linear.x| or |angular.z| above this = nonzero

# ── system thresholds ────────────────────────────────────────────────
THROTTLE_TEMP_C  = 85.0    # any thermal zone above this → throttle hint
DISK_MIN_FREE_GB = 2.0     # below this free space on / → alert
THERMAL_GLOB     = "/sys/devices/virtual/thermal/thermal_zone*/temp"

# ── Jetson clocks (debugfs, root only — degrade gracefully) ──────────
BPMP_CLK_DIR = "/sys/kernel/debug/bpmp/debug/clk"
BPMP_CLOCKS = {            # logical name → candidate clock dir names
    "gpu": ("nafll_gpc0", "gpc0clk"),
    "emc": ("emc",),
}

# ── perc-1 reachability probe ────────────────────────────────────────
# Kept in sync with PERC1_HOSTS in birdseye_fusion.py / calibrate_homography.py
# -- this drifted out of sync once already (found 2026-08-13 reporting
# "perc-1 unreachable" against a stale 192.168.0.101 lease from before the
# perc-1 rebuild, while perc-1 was actually up and streaming fine over the
# direct Ethernet link). Probing PORT 5002 specifically only proves ONE of
# perc-1's two streams is up; that's still a reasonable single-probe proxy
# for "is the box reachable at all", not a per-camera health check -- per-
# camera liveness is what /ethon/fusion_status.sources.{left,right}.alive is
# for, and that already has its own accurate check in birdseye_fusion.py.
PERC1_HOSTS = [
    "10.10.10.2",           # direct Ethernet cable, primary path
    "perception-1.local",   # mDNS fallback, if perc-1 is also on wifi
    "100.107.192.42",       # Tailscale (node is named perception-1-1 because
                            # the dead node still holds the perception-1 name)
]
PERC1_PORT      = 5002
PERC1_TIMEOUT_S = 0.5       # per host -- worst case ~0.5s * len(PERC1_HOSTS)
PERC1_PERIOD_S  = 10.0


def _extract_temps(obj):
    """Recursively collect numeric values under keys containing 'temp'
    from an arbitrary JSON structure (tolerant of drive_status schema
    changes)."""
    temps = []
    if isinstance(obj, dict):
        for key, val in obj.items():
            if "temp" in key.lower():
                if isinstance(val, (int, float)):
                    temps.append(float(val))
                elif isinstance(val, list):
                    temps.extend(float(v) for v in val if isinstance(v, (int, float)))
                else:
                    temps.extend(_extract_temps(val))
            else:
                temps.extend(_extract_temps(val))
    elif isinstance(obj, list):
        for item in obj:
            temps.extend(_extract_temps(item))
    return temps


class HealthMonitor(Node):
    """1 Hz system/topic health publisher with an estop reflex."""

    def __init__(self):
        super().__init__("health_monitor")
        now = time.monotonic()

        # per-topic receive timestamps (pruned to RATE_WINDOW_S each tick)
        self._rx = {name: deque() for name, _, _ in TOPIC_SPECS}
        self._last_cones_rx = now            # grace period at startup
        self._last_nonzero_cmd = float("-inf")
        self._max_motor_temp = None

        # estop latch (publish exactly once; drive node latches too)
        self._estop_fired = False
        self._estop_reason = ""

        # /proc/stat bookkeeping for CPU% deltas
        self._prev_cpu = None                # (total_jiffies, idle_jiffies)

        # alert-set diffing so degradations log once, not every second
        self._prev_alerts = set()

        for name, msg_type, _ in TOPIC_SPECS:
            self.create_subscription(msg_type, name, self._make_rx_cb(name), 10)

        self._health_pub = self.create_publisher(String, "/ethon/health", 10)
        # transient-local so a (re)starting drive node still sees the latch
        estop_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._estop_pub = self.create_publisher(Bool, "/ethon/estop", estop_qos)

        # perc-1 probe runs in its own thread — never block the timer
        self._perc1_ok = False
        self._stop = threading.Event()
        self._perc1_thread = threading.Thread(
            target=self._perc1_loop, name="perc1_probe", daemon=True
        )
        self._perc1_thread.start()

        self.create_timer(TICK_PERIOD_S, self._tick)
        self.get_logger().info("health_monitor up — publishing /ethon/health at 1 Hz")

    # ── subscriptions ────────────────────────────────────────────────
    def _make_rx_cb(self, name):
        """Build a callback that timestamps arrivals for rate measurement
        and dispatches topic-specific handling."""
        def _cb(msg):
            now = time.monotonic()
            self._rx[name].append(now)
            if name == "/ethon/cones":
                self._last_cones_rx = now
            elif name == "/cmd_vel":
                if abs(msg.linear.x) > CMD_EPS or abs(msg.angular.z) > CMD_EPS:
                    self._last_nonzero_cmd = now
            elif name == "/ethon/drive_status":
                self._on_drive_status(msg)
        return _cb

    def _on_drive_status(self, msg):
        """Pull the hottest motor temperature out of the drive node's
        JSON status, tolerating schema drift or non-JSON payloads."""
        try:
            temps = _extract_temps(json.loads(msg.data))
        except (json.JSONDecodeError, TypeError, ValueError):
            return
        if temps:
            self._max_motor_temp = max(temps)

    # ── perc-1 reachability (background thread) ──────────────────────
    def _perc1_loop(self):
        """TCP-connect probe to the perc-1 MJPEG port every 10 s.

        Tries PERC1_HOSTS in order and succeeds on the first reachable one --
        same fallback order as birdseye_fusion's TcpSource, so this probe
        reports "unreachable" only when every path actually is.
        """
        while not self._stop.is_set():
            ok = False
            for host in PERC1_HOSTS:
                try:
                    with socket.create_connection(
                            (host, PERC1_PORT), timeout=PERC1_TIMEOUT_S):
                        ok = True
                        break
                except OSError:
                    continue
            self._perc1_ok = ok
            self._stop.wait(PERC1_PERIOD_S)

    # ── system readers (all degrade to None / {} on failure) ─────────
    def _cpu_percent(self):
        """CPU utilisation since the previous tick from /proc/stat."""
        try:
            with open("/proc/stat") as f:
                fields = [int(x) for x in f.readline().split()[1:]]
        except (OSError, ValueError, IndexError):
            return None
        idle = fields[3] + (fields[4] if len(fields) > 4 else 0)  # idle+iowait
        total = sum(fields)
        prev, self._prev_cpu = self._prev_cpu, (total, idle)
        if prev is None or total <= prev[0]:
            return 0.0
        d_total = total - prev[0]
        d_idle = idle - prev[1]
        return round(100.0 * (1.0 - d_idle / d_total), 1)

    @staticmethod
    def _meminfo_mb():
        """(used MB, total MB) from /proc/meminfo, or (None, None)."""
        try:
            info = {}
            with open("/proc/meminfo") as f:
                for line in f:
                    key, _, rest = line.partition(":")
                    info[key] = int(rest.split()[0])           # kB
            total = info["MemTotal"]
            avail = info.get("MemAvailable", total)
            return round((total - avail) / 1024.0), round(total / 1024.0)
        except (OSError, KeyError, ValueError, IndexError):
            return None, None

    @staticmethod
    def _thermal_zones():
        """{zone type: temp C} for every readable thermal zone."""
        zones = {}
        for temp_path in glob.glob(THERMAL_GLOB):
            try:
                with open(temp_path) as f:
                    milli = int(f.read().strip())
                with open(os.path.join(os.path.dirname(temp_path), "type")) as f:
                    label = f.read().strip()
                zones[label] = round(milli / 1000.0, 1)
            except (OSError, ValueError):
                continue
        return zones

    @staticmethod
    def _bpmp_clocks_mhz():
        """{name: MHz} from bpmp debugfs; {} when not accessible (needs
        root) — caller omits the key entirely in that case."""
        clocks = {}
        if not os.access(BPMP_CLK_DIR, os.R_OK):
            return clocks
        for name, candidates in BPMP_CLOCKS.items():
            for cand in candidates:
                try:
                    with open(os.path.join(BPMP_CLK_DIR, cand, "rate")) as f:
                        clocks[name] = round(int(f.read().strip()) / 1e6)
                    break
                except (OSError, ValueError):
                    continue
        return clocks

    @staticmethod
    def _zone_temp(zones, keyword):
        """First zone whose type contains keyword (case-insensitive)."""
        for label, temp in zones.items():
            if keyword in label.lower():
                return temp
        return None

    # ── 1 Hz evaluation ──────────────────────────────────────────────
    def _tick(self):
        now = time.monotonic()
        alerts = []

        # topic liveness
        topics = {}
        for name, _, min_hz in TOPIC_SPECS:
            times = self._rx[name]
            while times and now - times[0] > RATE_WINDOW_S:
                times.popleft()
            hz = round(len(times) / RATE_WINDOW_S, 1)
            if not times:
                status = "stale"
            elif hz < min_hz:
                status = "slow"
            else:
                status = "ok"
            topics[name] = {"hz": hz, "min_hz": min_hz, "status": status}
            if status != "ok":
                alerts.append(f"{name} {status} ({hz:.1f} Hz, want >= {min_hz:.0f})")

        # system stats
        zones = self._thermal_zones()
        used_mb, total_mb = self._meminfo_mb()
        try:
            disk_free_gb = round(shutil.disk_usage("/").free / 2**30, 1)
        except OSError:
            disk_free_gb = None
        hottest = max(zones.values()) if zones else None
        throttling = hottest is not None and hottest > THROTTLE_TEMP_C
        system = {
            "cpu_pct": self._cpu_percent(),
            "ram_used_mb": used_mb,
            "ram_total_mb": total_mb,
            "disk_free_gb": disk_free_gb,
            "cpu_temp_c": self._zone_temp(zones, "cpu"),
            "gpu_temp_c": self._zone_temp(zones, "gpu"),
            "temps_c": zones,
            "throttling": throttling,
        }
        if self._max_motor_temp is not None:
            system["motor_temp_c"] = round(self._max_motor_temp, 1)
        clocks = self._bpmp_clocks_mhz()
        if clocks:
            system["clocks_mhz"] = clocks

        if throttling:
            alerts.append(f"thermal throttle risk: {hottest:.1f} C > {THROTTLE_TEMP_C:.0f} C")
        if disk_free_gb is not None and disk_free_gb < DISK_MIN_FREE_GB:
            alerts.append(f"disk low: {disk_free_gb:.1f} GB free on /")
        if not self._perc1_ok:
            alerts.append(
                f"perc-1 unreachable (tried {', '.join(PERC1_HOSTS)} :{PERC1_PORT})")

        self._check_estop(now, alerts)

        payload = {
            "ok": not alerts,
            "topics": topics,
            "system": system,
            "perc1": self._perc1_ok,
            "alerts": alerts,
        }
        self._health_pub.publish(String(data=json.dumps(payload)))

        # log only transitions, never steady-state spam
        current = set(alerts)
        for alert in sorted(current - self._prev_alerts):
            self.get_logger().warning(f"degraded: {alert}")
        for alert in sorted(self._prev_alerts - current):
            self.get_logger().info(f"recovered: {alert}")
        self._prev_alerts = current

    def _check_estop(self, now, alerts):
        """Estop reflex: motor over-temp, or perception silent while the
        vehicle is being commanded. Publishes /ethon/estop exactly once
        (the drive node latches it)."""
        if self._estop_fired:
            alerts.append(f"ESTOP latched: {self._estop_reason}")
            return

        reason = None
        if self._max_motor_temp is not None and self._max_motor_temp > MOTOR_TEMP_ESTOP_C:
            reason = (f"motor temp {self._max_motor_temp:.1f} C "
                      f"> {MOTOR_TEMP_ESTOP_C:.0f} C")
        else:
            cones_silent_s = now - self._last_cones_rx
            cmd_active = now - self._last_nonzero_cmd < CMD_ACTIVE_S
            if cones_silent_s > CONES_SILENT_S and cmd_active:
                reason = (f"/ethon/cones silent {cones_silent_s:.1f} s "
                          f"while /cmd_vel active")

        if reason:
            self._estop_fired = True
            self._estop_reason = reason
            self._estop_pub.publish(Bool(data=True))
            self.get_logger().error(f"ESTOP TRIGGERED → /ethon/estop true: {reason}")
            alerts.append(f"ESTOP latched: {reason}")

    def destroy_node(self):
        self._stop.set()
        super().destroy_node()


def main():
    rclpy.init()
    node = HealthMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
