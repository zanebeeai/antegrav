#!/usr/bin/env python3
"""Race-day services for the team 843 EV — three small nodes, one process:

CorridorWarning  driver assist while a HUMAN drives: watches the planner's
                 midline (/ethon/path) and warns when the car is meaningfully
                 off the cone corridor. Consumed by wheel_bridge (Nextion WARN
                 zone + amber LED flash) and the web dashboard.
                 Publishes /ethon/corridor  JSON {state, dev_m, cones}
                 state: no_path | ok | warn | off

RaceStrategist   energy pacing for the 70-minute race on the Interstate
                 MTX-35 (12 V, 55 Ah AGM ~ 660 Wh nominal; usable budget is a
                 parameter, default 480 Wh — AGM under sustained race draw).
                 Compares actual Wh/min against the race budget and issues a
                 pace verdict. Race clock starts via /ethon/race/start (Empty,
                 e.g. the dashboard button).
                 Publishes /ethon/strategy JSON at 1 Hz.

SessionLogger    flat CSV telemetry log per driving session under
                 /home/jetson/ethon/logs/ — speed, energy, laps, GPS, flags.
                 A session opens on first activity (armed or rolling) and
                 rotates after 5 min of quiet. Replayed by the dashboard.

All three are read-only observers except the strategy/corridor topics they
publish; none of them can command motion. Runs as ethon-race.service.
"""

import csv
import json
import math
import os
import signal
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, String
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix
from rclpy.qos import qos_profile_sensor_data

LOG_DIR = "/home/jetson/ethon/logs"

T_PATH, T_CONES = "/ethon/path", "/ethon/cones"
T_CORRIDOR = "/ethon/corridor"
T_DRIVE, T_LAP = "/ethon/drive_status", "/ethon/lap"
T_STRATEGY = "/ethon/strategy"
T_RACE_START = "/ethon/race/start"
T_ESTOP, T_ARM = "/ethon/estop", "/ethon/hmi/arm"
T_GPS = "/gps/fix"


# ────────────────────────────────────────────────────────────── corridor ────
class CorridorWarning(Node):
    """Lateral-deviation watchdog: how far is the car off the cone midline?

    dev_m = the midline's y offset at the car (+ = midline left of car,
    i.e. the car sits RIGHT of the corridor). Only meaningful while the
    planner has fresh cones to build a path from; otherwise state=no_path
    (silent — never nag the driver about vision the car doesn't have).
    """

    def __init__(self):
        super().__init__("corridor_warning")
        self.declare_parameter("warn_dev_m", 0.5)
        self.declare_parameter("off_dev_m", 0.9)
        self.declare_parameter("path_stale_s", 1.5)

        self._path_t = 0.0
        self._dev = None
        self._cones = 0
        self._last_state = None

        self.create_subscription(Path, T_PATH, self._on_path, 10)
        self.create_subscription(PoseArray, T_CONES, self._on_cones, 10)
        self._pub = self.create_publisher(String, T_CORRIDOR, 10)
        self.create_timer(0.2, self._publish)   # 5 Hz
        self.get_logger().info("corridor_warning up -- watching %s" % T_PATH)

    def _on_cones(self, m):
        self._cones = len(m.poses)

    def _on_path(self, m):
        if not m.poses:
            return
        # midline point nearest the car (min |x| in base_link)
        best = min(m.poses, key=lambda p: abs(p.pose.position.x))
        self._dev = best.pose.position.y
        self._path_t = time.monotonic()

    def _publish(self):
        warn = float(self.get_parameter("warn_dev_m").value)
        off = float(self.get_parameter("off_dev_m").value)
        stale = float(self.get_parameter("path_stale_s").value)
        if time.monotonic() - self._path_t > stale or self._dev is None:
            state, dev = "no_path", None
        else:
            dev = self._dev
            a = abs(dev)
            state = "off" if a >= off else ("warn" if a >= warn else "ok")
        if state != self._last_state:
            self._last_state = state
            log = self.get_logger()
            (log.warning if state in ("warn", "off") else log.info)(
                "corridor: %s%s" % (state,
                                    "" if dev is None else " dev=%.2fm" % dev))
        self._pub.publish(String(data=json.dumps(
            {"state": state,
             "dev_m": None if dev is None else round(dev, 2),
             "cones": self._cones},
            separators=(",", ":"))))


# ───────────────────────────────────────────────────────────── strategist ───
class RaceStrategist(Node):
    """Energy pacing: Wh spent vs the race budget, verdict for the driver.

    The drive node's energy_wh integrator resets when ethon-stack restarts;
    an offset accumulator makes the total monotonic across restarts so a
    mid-race estop-clear doesn't wipe the race's consumption history.
    """

    PACE_BAND = 0.08          # +/-8% of budget rate counts as "on target"

    def __init__(self):
        super().__init__("race_strategist")
        self.declare_parameter("battery_usable_wh", 480.0)
        self.declare_parameter("race_minutes", 70.0)

        self._energy_raw = 0.0     # last energy_wh seen from drive_status
        self._wh_offset = 0.0      # accumulated across drive restarts
        self._lap = 0
        self._last_s = None
        self._race_t0 = None       # monotonic at race start
        self._race_wh0 = 0.0       # total wh at race start
        self._race_lap0 = 0
        self._lap_marks = []       # (lap, total_wh) at each lap increment

        self.create_subscription(String, T_DRIVE, self._on_drive, 10)
        self.create_subscription(String, T_LAP, self._on_lap, 10)
        self.create_subscription(Empty, T_RACE_START, self._on_start, 10)
        self._pub = self.create_publisher(String, T_STRATEGY, 10)
        self.create_timer(1.0, self._publish)
        self.get_logger().info(
            "race_strategist up -- budget %.0f Wh over %.0f min"
            % (float(self.get_parameter("battery_usable_wh").value),
               float(self.get_parameter("race_minutes").value)))

    # total consumed Wh, monotonic across drive-node restarts
    def _total_wh(self):
        return self._wh_offset + self._energy_raw

    def _on_drive(self, m):
        try:
            d = json.loads(m.data)
        except (ValueError, TypeError):
            return
        wh = d.get("energy_wh")
        if not isinstance(wh, (int, float)):
            return
        if wh < self._energy_raw - 5.0:     # integrator reset (stack restart)
            self._wh_offset += self._energy_raw
            self.get_logger().warning(
                "drive energy integrator reset -- carrying %.0f Wh forward"
                % self._wh_offset)
        self._energy_raw = wh

    def _on_lap(self, m):
        try:
            d = json.loads(m.data)
        except (ValueError, TypeError):
            return
        lap = d.get("lap", 0)
        self._last_s = d.get("last_s")
        if lap > self._lap:
            self._lap_marks.append((lap, self._total_wh()))
            if len(self._lap_marks) > 200:
                del self._lap_marks[:100]
        self._lap = lap

    def _on_start(self, _m):
        self._race_t0 = time.monotonic()
        self._race_wh0 = self._total_wh()
        self._race_lap0 = self._lap
        self._lap_marks = []
        self.get_logger().warning(
            "RACE START -- clock running, %0.f Wh consumed so far excluded"
            % self._race_wh0)

    def _publish(self):
        budget = float(self.get_parameter("battery_usable_wh").value)
        race_s = float(self.get_parameter("race_minutes").value) * 60.0
        # Guard the divisions below: a 0 (or negative) budget/duration would
        # otherwise raise at 1 Hz and take the strategist down mid-race.
        if budget <= 0.0 or race_s <= 0.0:
            self._pub.publish(String(data=json.dumps(
                {"race_on": False,
                 "error": "battery_usable_wh and race_minutes must be > 0"},
                separators=(",", ":"))))
            return
        total = self._total_wh()
        out = {
            "race_on": self._race_t0 is not None,
            "wh_budget": round(budget, 0),
            "wh_total_since_boot": round(total, 1),
            "battery_pct": round(100.0 * max(0.0, 1.0 - total / budget), 1),
            "lap": self._lap,
            "last_lap_s": self._last_s,
        }
        if self._race_t0 is None:
            out.update({"elapsed_s": 0, "remaining_s": round(race_s),
                        "wh_used": 0.0, "wh_remaining": round(budget, 1),
                        "pace": "-", "pace_n": 0, "rate_wh_min": None,
                        "budget_wh_min": round(budget / (race_s / 60.0), 1),
                        "projected_wh": None, "laps_done": 0,
                        "wh_per_lap": None, "last_lap_wh": None})
        else:
            elapsed = time.monotonic() - self._race_t0
            used = total - self._race_wh0
            remaining_s = max(0.0, race_s - elapsed)
            budget_rate = budget / (race_s / 60.0)          # Wh per minute
            rate = used / (elapsed / 60.0) if elapsed > 30.0 else None
            projected = rate * (race_s / 60.0) if rate is not None else None
            if rate is None:
                pace, pace_n = "-", 0
            elif rate > budget_rate * (1.0 + self.PACE_BAND):
                pace, pace_n = "SLOW DOWN", -1
            elif rate < budget_rate * (1.0 - self.PACE_BAND):
                pace, pace_n = "PACE IN HAND", 1
            else:
                pace, pace_n = "ON TARGET", 0
            laps_done = self._lap - self._race_lap0
            wh_per_lap = (used / laps_done) if laps_done > 0 else None
            last_lap_wh = None
            if len(self._lap_marks) >= 2:
                last_lap_wh = self._lap_marks[-1][1] - self._lap_marks[-2][1]
            elif len(self._lap_marks) == 1:
                last_lap_wh = self._lap_marks[-1][1] - self._race_wh0
            out.update({
                "elapsed_s": round(elapsed),
                "remaining_s": round(remaining_s),
                "wh_used": round(used, 1),
                "wh_remaining": round(max(0.0, budget - used), 1),
                "rate_wh_min": None if rate is None else round(rate, 1),
                "budget_wh_min": round(budget_rate, 1),
                "pace": pace, "pace_n": pace_n,
                "projected_wh": None if projected is None
                                else round(projected, 0),
                "laps_done": laps_done,
                "wh_per_lap": None if wh_per_lap is None
                              else round(wh_per_lap, 1),
                "last_lap_wh": None if last_lap_wh is None
                               else round(last_lap_wh, 1),
            })
        self._pub.publish(String(data=json.dumps(out, separators=(",", ":"))))


# ──────────────────────────────────────────────────────────────── logger ────
class SessionLogger(Node):
    """CSV telemetry recorder. A session = a stretch of activity (armed or
    rolling); the file opens lazily on the first active sample and rotates
    after 5 min of quiet, so bench-idle days don't pile up empty logs."""

    ACTIVE_SPEED_MS = 0.3
    HOLD_S = 60.0          # keep logging this long after last activity
    ROTATE_IDLE_S = 300.0  # close the file after this much quiet
    RATE_HZ = 2.0

    FIELDS = ["t", "iso", "speed_ms", "energy_wh", "wh_per_km", "temp_c",
              "lap", "cur_s", "last_s", "lat", "lon", "armed", "estop"]

    def __init__(self):
        super().__init__("session_logger")
        self._drive = {}
        self._lap = {}
        self._lat = self._lon = None
        self._armed = False
        self._estop = False
        self._last_active = 0.0
        self._fh = None
        self._csv = None
        self._path = None

        os.makedirs(LOG_DIR, exist_ok=True)
        self.create_subscription(String, T_DRIVE, self._on_drive, 10)
        self.create_subscription(String, T_LAP, self._on_lap, 10)
        self.create_subscription(NavSatFix, T_GPS, self._on_gps,
                                 qos_profile_sensor_data)
        self.create_subscription(Bool, T_ESTOP, self._on_estop, 10)
        self.create_subscription(Bool, T_ARM, self._on_arm, 10)
        self.create_timer(1.0 / self.RATE_HZ, self._tick)
        self.get_logger().info("session_logger up -- logs in %s" % LOG_DIR)

    def _on_drive(self, m):
        try:
            self._drive = json.loads(m.data)
        except (ValueError, TypeError):
            pass

    def _on_lap(self, m):
        try:
            self._lap = json.loads(m.data)
        except (ValueError, TypeError):
            pass

    def _on_gps(self, m):
        if (m.status.status >= 0 and math.isfinite(m.latitude)
                and not (m.latitude == 0.0 and m.longitude == 0.0)):
            self._lat, self._lon = m.latitude, m.longitude

    def _on_estop(self, m):
        self._estop = bool(m.data)

    def _on_arm(self, m):
        self._armed = bool(m.data)

    def _speed(self):
        ws = self._drive.get("wheel_speed_ms")
        return abs(ws) if isinstance(ws, (int, float)) else 0.0

    def _open(self):
        name = "session_%s.csv" % time.strftime("%Y%m%d_%H%M%S")
        self._path = os.path.join(LOG_DIR, name)
        self._fh = open(self._path, "w", newline="")
        self._csv = csv.writer(self._fh)
        self._csv.writerow(self.FIELDS)
        self.get_logger().warning("session log opened: %s" % self._path)

    def _close(self):
        if self._fh is not None:
            try:
                self._fh.close()
            except OSError:
                pass
            self.get_logger().warning("session log closed: %s" % self._path)
        self._fh = self._csv = self._path = None

    def _tick(self):
        now = time.monotonic()
        active = self._armed or self._speed() > self.ACTIVE_SPEED_MS
        if active:
            self._last_active = now
        since = now - self._last_active

        if self._fh is None:
            if not active:
                return
            self._open()
        elif since > self.ROTATE_IDLE_S:
            self._close()
            return
        elif since > self.HOLD_S:
            return                      # holding the file open, not writing

        motors = self._drive.get("motors") or {}
        temps = [v.get("temp_c") for v in motors.values()
                 if isinstance(v, dict)
                 and isinstance(v.get("temp_c"), (int, float))]
        row = [
            round(time.time(), 2),
            time.strftime("%H:%M:%S"),
            round(self._speed(), 2),
            self._drive.get("energy_wh"),
            self._drive.get("wh_per_km"),
            max(temps) if temps else None,
            self._lap.get("lap"),
            self._lap.get("cur_s"),
            self._lap.get("last_s"),
            None if self._lat is None else round(self._lat, 7),
            None if self._lon is None else round(self._lon, 7),
            int(self._armed),
            int(self._estop),
        ]
        try:
            self._csv.writerow(row)
            self._fh.flush()
        except OSError as exc:
            self.get_logger().error("log write failed: %s" % exc)
            self._close()


def _on_term(signum, _frame):
    raise SystemExit(signum)


def main():
    rclpy.init()
    nodes = [CorridorWarning(), RaceStrategist(), SessionLogger()]
    exe = MultiThreadedExecutor(num_threads=3)
    for n in nodes:
        exe.add_node(n)
    signal.signal(signal.SIGTERM, _on_term)
    try:
        exe.spin()
    except (KeyboardInterrupt, SystemExit, ExternalShutdownException):
        pass
    finally:
        for n in nodes:
            n.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
