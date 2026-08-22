#!/usr/bin/env python3
"""GPS lap timer for the team 843 Electrathon EV.

Turns a GPS fix into lap times for the Nextion dashboard. The start/finish
line is the car's position when this node first gets a fix (i.e. wherever the
car is powered on), and can be re-marked at the current position at any time
via the MARK button on the HMI. A lap is logged each time the car re-enters
the geofence around that line after having left it.

Subscribes:
  /gps/fix         sensor_msgs/NavSatFix   position (from the NMEA driver;
                                           same topic fleet_localization uses)
  /ethon/lap/mark  std_msgs/Empty          (re)set the start/finish line here

Publishes:
  /ethon/lap       std_msgs/String  JSON {lap, cur_s, last_s, best_s, delta_s,
                                          line_set, fix, dist_m}

Lap detection is proximity-based: the car must travel beyond arm_factor x the
geofence radius away (so we "arm"), then come back within the radius, and the
lap must be at least min_lap_s long. This is the standard hobby-GPS method and
is robust to sitting on the line. A true line-crossing + live per-distance
delta would need a recorded best-lap trace (future work); delta_s here is the
last completed lap minus the best (negative = a new best).
"""

import json
import math
import signal
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Empty, String

GPS_TOPIC = "/gps/fix"
MARK_TOPIC = "/ethon/lap/mark"
LAP_TOPIC = "/ethon/lap"
PUBLISH_HZ = 10.0          # /ethon/lap rate -> how fast the running lap time updates
FIX_HOLD_S = 3.0           # keep showing "fix" through brief GPS dropouts (debounce)
EARTH_R_M = 6371000.0


def _haversine_m(lat1, lon1, lat2, lon2):
    """Great-circle distance in metres (fine for the <100 m scales here)."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = (math.sin(dp / 2.0) ** 2
         + math.cos(p1) * math.cos(p2) * math.sin(dl / 2.0) ** 2)
    return 2.0 * EARTH_R_M * math.asin(min(1.0, math.sqrt(a)))


class LapTimer(Node):
    def __init__(self):
        super().__init__("lap_timer")
        self.declare_parameter("geofence_radius_m", 20.0)
        self.declare_parameter("min_lap_s", 15.0)
        self.declare_parameter("arm_factor", 2.0)  # leave radius*arm_factor to arm
        self.declare_parameter("auto_set_on_first_fix", True)

        # ---- state ----
        self._line = None         # (lat, lon) start/finish, None until set
        self._lap_start = None    # time.monotonic() at lap start
        self._last_s = None
        self._best_s = None
        self._delta_s = None
        self._lap = 0
        self._armed = False       # have we left the zone since the last cross?
        self._last_fix_t = None   # monotonic of last VALID fix (for fix debounce)
        self._dist_m = None
        self._last_pos = None     # (lat, lon) most recent valid fix

        self.create_subscription(NavSatFix, GPS_TOPIC, self._on_fix,
                                 qos_profile_sensor_data)
        self.create_subscription(Empty, MARK_TOPIC, self._on_mark, 10)
        self._pub = self.create_publisher(String, LAP_TOPIC, 10)
        self.create_timer(1.0 / PUBLISH_HZ, self._publish)
        self.get_logger().info("lap_timer up -- waiting for %s" % GPS_TOPIC)

    # ------------------------------------------------------------------ helpers

    def _set_line(self, lat, lon, reason):
        self._line = (lat, lon)
        self._lap_start = time.monotonic()
        self._armed = False
        self.get_logger().warning(
            "start/finish line set @ %.6f, %.6f (%s)" % (lat, lon, reason))

    @staticmethod
    def _valid(m):
        return (m.status.status >= 0                      # >= STATUS_FIX (0)
                and math.isfinite(m.latitude)
                and math.isfinite(m.longitude)
                and not (m.latitude == 0.0 and m.longitude == 0.0))

    # ------------------------------------------------------------------ I/O

    def _on_mark(self, _msg):
        if self._last_pos is None:
            self.get_logger().warning("MARK ignored -- no GPS fix yet")
            return
        # re-marking starts a fresh session
        self._last_s = self._best_s = self._delta_s = None
        self._lap = 0
        self._set_line(self._last_pos[0], self._last_pos[1], "MARK button")

    def _on_fix(self, m):
        if not self._valid(m):
            return                 # ignore a momentary bad sentence (debounced below)
        self._last_fix_t = time.monotonic()
        lat, lon = m.latitude, m.longitude
        self._last_pos = (lat, lon)

        if self._line is None:
            if self.get_parameter("auto_set_on_first_fix").value:
                self._set_line(lat, lon, "first fix / power-on")
            return

        radius = float(self.get_parameter("geofence_radius_m").value)
        arm_d = radius * float(self.get_parameter("arm_factor").value)
        min_lap = float(self.get_parameter("min_lap_s").value)
        d = _haversine_m(lat, lon, self._line[0], self._line[1])
        self._dist_m = d

        if d > arm_d:
            self._armed = True
        elif self._armed and d < radius:
            elapsed = time.monotonic() - self._lap_start
            if elapsed >= min_lap:
                self._last_s = elapsed
                self._best_s = (elapsed if self._best_s is None
                                else min(self._best_s, elapsed))
                self._delta_s = self._last_s - self._best_s
                self._lap += 1
                self._lap_start = time.monotonic()
                self._armed = False
                self.get_logger().warning(
                    "LAP %d: %.3f s (best %.3f s)"
                    % (self._lap, self._last_s, self._best_s))

    def _publish(self):
        cur = (time.monotonic() - self._lap_start
               if self._lap_start is not None else 0.0)
        msg = {
            "lap": self._lap,
            "cur_s": round(cur, 3),
            "last_s": None if self._last_s is None else round(self._last_s, 3),
            "best_s": None if self._best_s is None else round(self._best_s, 3),
            "delta_s": None if self._delta_s is None else round(self._delta_s, 3),
            "line_set": self._line is not None,
            "fix": (self._last_fix_t is not None
                    and time.monotonic() - self._last_fix_t < FIX_HOLD_S),
            "dist_m": None if self._dist_m is None else round(self._dist_m, 1),
            # start/finish coords + geofence radius so a map view can draw them
            "line_lat": None if self._line is None else round(self._line[0], 7),
            "line_lon": None if self._line is None else round(self._line[1], 7),
            "geofence_m": float(self.get_parameter("geofence_radius_m").value),
        }
        self._pub.publish(String(data=json.dumps(msg, separators=(",", ":"))))


def _on_term(signum, _frame):
    raise SystemExit(signum)


def main():
    rclpy.init()
    node = LapTimer()
    signal.signal(signal.SIGTERM, _on_term)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
