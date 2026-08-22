#!/usr/bin/env python3
"""Nextion steering-wheel HMI for the team 843 Electrathon EV.

Sim-racing style dashboard, drawn DIRECTLY from the Jetson Orin NX over UART
(/dev/ttyTHS1, 40-pin pins 8/10) -- no Arduino, no SimHub, and crucially NO
custom .tft design. The whole layout is RENDERED at runtime with Nextion draw
commands (xstr/fill) on stock firmware, and touch comes from raw coordinate
streaming (sendxy=1) hit-tested against drawn buttons. Because stock firmware
only has font 0, the big SPEED number is drawn as fill-based 7-segment digits.

Layout (480x320):
  - left edge:   shift-light style speed bar
  - top-left:    CURRENT / LAST / BEST lap times (from the GPS lap_timer)
  - top-right:   DELTA (last vs best) + lap counter + GPS status
  - centre:      big 7-segment SPEED (km/h) + gear letter
  - thin row:    Wh used / Wh per km / hottest motor temp
  - banner:      ARMED / DISARMED / E-STOP state + mode (tap right half to flip)
  - bottom row:  ARM | DISARM | E-STOP | MARK (set GPS start/finish line)

Subscribes: /ethon/health, /ethon/drive_status, /ethon/estop, /cmd_vel, /ethon/lap
Publishes:  /ethon/estop (Bool), /ethon/hmi/arm (Bool), /ethon/hmi/mode (String),
            /ethon/lap/mark (Empty)

If the layout runs off-screen, SCREEN_W/SCREEN_H below are wrong for your panel
(2.4"/2.8"=320x240, 3.2"=400x240, 3.5"=480x320, 4.3"=480x272).
"""

import json
import signal
import subprocess

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty, String
from geometry_msgs.msg import Twist

import serial   # python3-serial

# ── link / panel ───────────────────────────────────────────────────────────
SERIAL_PORT = "/dev/ttyTHS1"
BAUD = 9600                    # match the panel (stock = 9600)
SCREEN_W, SCREEN_H = 480, 320  # set to YOUR panel's pixels (see header)
REFRESH_HZ = 5.0
FONT = 0                       # stock firmware usually only has font 0
EOL = b"\xff\xff\xff"

# Helper that flips the capture/autonomy systemd units, run via sudo -n (needs
# the /etc/sudoers.d/ethon-hmi drop-in installed at deploy time).
MODE_SCRIPT = "/home/jetson/ethon/ethon_set_mode.sh"

# ── Nextion 16-bit (565) colours as decimals ───────────────────────────────
BLACK, WHITE, GREY = 0, 65535, 21130
RED, GREEN, AMBER, BLUE = 63488, 2016, 64512, 1023
CYAN, DKRED, DKGREEN, DKGREY = 2047, 38912, 1024, 12678

# ── topics ─────────────────────────────────────────────────────────────────
T_HEALTH, T_DRIVE, T_ESTOP, T_CMDVEL = (
    "/ethon/health", "/ethon/drive_status", "/ethon/estop", "/cmd_vel")
T_HMI_ARM, T_HMI_MODE = "/ethon/hmi/arm", "/ethon/hmi/mode"
T_LAP, T_LAP_MARK = "/ethon/lap", "/ethon/lap/mark"

# ── big 7-segment digits (drawn with fill rects; works on stock firmware) ───
SEG = {
    "0": "abcdef", "1": "bc", "2": "abdeg", "3": "abcdg", "4": "fgbc",
    "5": "afgcd", "6": "afgcde", "7": "abc", "8": "abcdefg", "9": "abcdfg",
    "-": "g", " ": "",
}
SPEED_W, SPEED_H, SPEED_T, SPEED_GAP = 64, 90, 10, 8
SPEED_DIGITS = 3
SPEED_X = (SCREEN_W - (SPEED_DIGITS * SPEED_W + (SPEED_DIGITS - 1) * SPEED_GAP)) // 2
SPEED_Y = 138             # below the Wh/Wh-km/temp stat row (avoids clipping)
BAR_W = 16
BAR_SEGS = 20
BAR_MAX_KMH = 50.0


def draw_digit(nx, x, y, ch, w, h, t, on, off):
    """Render one 7-segment glyph with fill rects (lit=on, unlit=off)."""
    if ch == " ":
        nx.fill(x, y, w, h, BLACK)
        return
    segs = SEG.get(ch, "")
    sh = (h - 3 * t) // 2
    rects = {
        "a": (x + t, y, w - 2 * t, t),
        "f": (x, y + t, t, sh),
        "b": (x + w - t, y + t, t, sh),
        "g": (x + t, y + t + sh, w - 2 * t, t),
        "e": (x, y + 2 * t + sh, t, sh),
        "c": (x + w - t, y + 2 * t + sh, t, sh),
        "d": (x + t, y + h - t, w - 2 * t, t),
    }
    for name, (rx, ry, rw, rh) in rects.items():
        nx.fill(rx, ry, rw, rh, on if name in segs else off)


def fmt_lap(s):
    """Seconds -> 'M:SS.mmm' (or placeholder when unset)."""
    if s is None:
        return "-:--.---"
    m = int(s // 60)
    return "%d:%06.3f" % (m, s - 60 * m)


def fmt_delta(s):
    if s is None:
        return "--"
    return "%+.3f" % s


class NextionLink:
    """Serial link: draw primitives out, raw touch coords in."""

    def __init__(self, port, baud, logger, on_touch):
        self._log = logger
        self._on_touch = on_touch
        self._buf = bytearray()
        self._ser = serial.Serial(port, baud, timeout=0)
        self.send("")             # flush partial cmd
        self.send("bkcmd=0")      # no command ACKs
        self.send("sendxy=1")     # stream raw touch coords (0x67 frames)

    def send(self, cmd: str) -> None:
        try:
            self._ser.write(cmd.encode("ascii", "ignore") + EOL)
        except (serial.SerialException, OSError) as exc:
            self._log.warning(f"nextion write failed: {exc}")

    # draw helpers --------------------------------------------------------
    def cls(self, color=BLACK):
        self.send("cls %d" % color)

    def fill(self, x, y, w, h, color):
        self.send("fill %d,%d,%d,%d,%d" % (x, y, w, h, color))

    def box_text(self, x, y, w, h, text, fg=WHITE, bg=BLACK, big=0):
        # xstr draws text AND fills its box (sta=1 solid bg) -> overwrites in
        # place, no clear/flicker. xcenter=1 ycenter=1 centres it.
        self.send('xstr %d,%d,%d,%d,%d,%d,%d,1,1,1,"%s"'
                  % (x, y, w, h, big or FONT, fg, bg,
                     str(text).replace('"', "'")))

    # touch in ------------------------------------------------------------
    def poll(self):
        try:
            data = self._ser.read(256)
        except (serial.SerialException, OSError) as exc:
            self._log.warning(f"nextion read failed: {exc}")
            return
        if not data:
            return
        self._buf.extend(data)
        while True:
            idx = self._buf.find(EOL)
            if idx < 0:
                if len(self._buf) > 64:
                    del self._buf[:-3]
                break
            frame = bytes(self._buf[:idx])
            del self._buf[: idx + 3]
            # 0x67 = touch coordinate: 67 xH xL yH yL state
            if frame and frame[0] == 0x67 and len(frame) >= 6:
                x = (frame[1] << 8) | frame[2]
                y = (frame[3] << 8) | frame[4]
                self._on_touch(x, y, frame[5])   # state 1=press 0=release

    def close(self):
        try:
            self.send("sendxy=0")
            self._ser.close()
        except Exception:
            pass


class NextionHMI(Node):
    def __init__(self):
        super().__init__("nextion_hmi")
        log = self.get_logger()

        # cached state
        self._speed = 0.0            # m/s
        self._armed = False
        self._estop = False
        self._energy_wh = None
        self._wh_per_km = None
        self._temp_c = None
        self._mode = "capture"
        self._warn = ""
        # lap state (from /ethon/lap)
        self._lap = 0
        self._cur_s = None
        self._last_s = None
        self._best_s = None
        self._delta_s = None
        self._line_set = False
        self._fix = False

        # change-detection caches (the 9600 link is slow -- only redraw deltas)
        self._shown = {}
        self._seg_cells = {}     # speed digit index -> last char
        self._bar_shown = {}     # bar segment index -> lit?

        # bottom button row: x, y, w, h, label, action
        bw, by, bh = SCREEN_W // 4, SCREEN_H - 24, 24
        self._buttons = [
            (0,        by, bw, bh, "ARM",    "arm"),
            (bw,       by, bw, bh, "DISARM", "disarm"),
            (2 * bw,   by, bw, bh, "E-STOP", "estop"),
            (3 * bw,   by, SCREEN_W - 3 * bw, bh, "MARK", "mark"),
        ]
        # tap the MODE zone of the status banner to toggle mode
        self._mode_hit = (160, SCREEN_H - 52, 158, 24)

        try:
            self._nx = NextionLink(SERIAL_PORT, BAUD, log, self._on_touch)
            self._draw_static()
        except (serial.SerialException, OSError) as exc:
            log.error(f"cannot open Nextion on {SERIAL_PORT}: {exc} -- HEADLESS")
            self._nx = None

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._estop_pub = self.create_publisher(Bool, T_ESTOP, latched)
        self._arm_pub = self.create_publisher(Bool, T_HMI_ARM, 10)
        self._mode_pub = self.create_publisher(String, T_HMI_MODE, 10)
        self._mark_pub = self.create_publisher(Empty, T_LAP_MARK, 10)

        self.create_subscription(String, T_HEALTH, self._on_health, 10)
        self.create_subscription(String, T_DRIVE, self._on_drive, 10)
        self.create_subscription(Bool, T_ESTOP, self._on_estop, 10)
        self.create_subscription(Twist, T_CMDVEL, self._on_cmd, 10)
        self.create_subscription(String, T_LAP, self._on_lap, 10)
        self.create_timer(1.0 / REFRESH_HZ, self._refresh)
        log.info("nextion_hmi up (%dx%d)" % (SCREEN_W, SCREEN_H) if self._nx
                 else "nextion_hmi up (display offline)")

    # ── static chrome: labels + buttons drawn once ─────────────────────────
    def _draw_static(self):
        nx = self._nx
        nx.cls(BLACK)
        # lap labels (left column)
        nx.box_text(BAR_W + 4, 4, 44, 22, "CUR", GREY, BLACK)
        nx.box_text(BAR_W + 4, 30, 44, 22, "LAST", GREY, BLACK)
        nx.box_text(BAR_W + 4, 56, 44, 22, "BEST", GREY, BLACK)
        # DELTA label beside its value (side-by-side avoids vertical clipping)
        nx.box_text(300, 8, 78, 26, "DELTA", GREY, BLACK)
        # (stat-row labels removed -- the value strings carry their own units now)
        # KMH label under the big number
        nx.box_text(SCREEN_W // 2 - 40, SPEED_Y + SPEED_H + 2, 80, 22,
                    "KMH", GREY, BLACK)
        # button chrome
        for (x, y, w, h, label, _a) in self._buttons:
            nx.fill(x + 1, y + 1, w - 2, h - 2, DKGREY)
            nx.box_text(x + 1, y + 1, w - 2, h - 2, label, WHITE, DKGREY)

    # ── topic callbacks ────────────────────────────────────────────────────
    def _on_cmd(self, m: Twist):
        self._speed = abs(m.linear.x)

    def _on_estop(self, m: Bool):
        if m.data:
            self._estop = True

    def _on_health(self, m: String):
        try:
            d = json.loads(m.data)
        except (ValueError, TypeError):
            return
        a = d.get("alerts") or []
        self._warn = "" if not a else str(a[0])[:24]

    def _on_drive(self, m: String):
        try:
            d = json.loads(m.data)
        except (ValueError, TypeError):
            return
        ws = d.get("wheel_speed_ms")
        if isinstance(ws, (int, float)):
            self._speed = abs(ws)
        self._armed = bool(d.get("enabled"))
        self._energy_wh = d.get("energy_wh")
        self._wh_per_km = d.get("wh_per_km")
        temps = [v.get("temp_c") for v in (d.get("motors") or {}).values()
                 if isinstance(v, dict) and isinstance(v.get("temp_c"), (int, float))]
        self._temp_c = max(temps) if temps else None

    def _on_lap(self, m: String):
        try:
            d = json.loads(m.data)
        except (ValueError, TypeError):
            return
        self._lap = d.get("lap", 0)
        self._cur_s = d.get("cur_s")
        self._last_s = d.get("last_s")
        self._best_s = d.get("best_s")
        self._delta_s = d.get("delta_s")
        self._line_set = bool(d.get("line_set"))
        self._fix = bool(d.get("fix"))

    # ── touch -> action (hit-test drawn buttons) ───────────────────────────
    def _on_touch(self, x, y, state):
        if state != 0:            # act on RELEASE
            return
        for (bx, by, bw, bh, _label, action) in self._buttons:
            if bx <= x < bx + bw and by <= y < by + bh:
                self._fire(action, None)
                return
        mx, my, mw, mh = self._mode_hit
        if mx <= x < mx + mw and my <= y < my + mh:
            self._fire("mode", "autonomy" if self._mode == "capture" else "capture")

    def _fire(self, action, arg):
        if action == "estop":
            self._estop_pub.publish(Bool(data=True))
            self.get_logger().error("HMI E-STOP -> /ethon/estop true")
        elif action == "arm":
            self._arm_pub.publish(Bool(data=True))
            self.get_logger().warning("HMI arm requested")
        elif action == "disarm":
            self._arm_pub.publish(Bool(data=False))
            self.get_logger().warning("HMI disarm requested")
        elif action == "mark":
            self._mark_pub.publish(Empty())
            self.get_logger().warning("HMI MARK -> set GPS start/finish line")
        elif action == "mode":
            self._mode = arg
            self._mode_pub.publish(String(data=arg))
            self.get_logger().warning("HMI mode -> %s" % arg)
            self._switch_mode(arg)

    def _switch_mode(self, target):
        """Fire-and-forget capture<->autonomy switch via the sudo helper.

        Non-blocking (Popen, not run): toggling the units takes a second or two
        and must not stall the refresh timer. Switching to autonomy only LAUNCHES
        the stack -- the planner boots DISARMED, so the car does not move until
        the ARM button is pressed.
        """
        if target not in ("capture", "autonomy"):
            return
        try:
            subprocess.Popen(
                ["sudo", "-n", MODE_SCRIPT, target],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except OSError as exc:
            self.get_logger().warning("mode switch exec failed: %s" % exc)

    # ── render (change-detected; the 9600 link can't redraw everything) ────
    def _put(self, key, x, y, w, h, text, fg=WHITE, bg=BLACK):
        if self._shown.get(key) != (text, fg, bg):
            self._shown[key] = (text, fg, bg)
            self._nx.box_text(x, y, w, h, text, fg, bg)

    def _draw_speed(self, kmh):
        s = "%*d" % (SPEED_DIGITS, min(999, int(round(kmh))))
        for i, ch in enumerate(s):
            if self._seg_cells.get(i) == ch:
                continue
            self._seg_cells[i] = ch
            cx = SPEED_X + i * (SPEED_W + SPEED_GAP)
            draw_digit(self._nx, cx, SPEED_Y, ch,
                       SPEED_W, SPEED_H, SPEED_T, GREEN, DKGREY)

    def _draw_bar(self, frac):
        lit = int(round(max(0.0, min(1.0, frac)) * BAR_SEGS))
        seg_h = SCREEN_H // BAR_SEGS
        for i in range(BAR_SEGS):
            on = i < lit
            if self._bar_shown.get(i) == on:
                continue
            self._bar_shown[i] = on
            y = SCREEN_H - (i + 1) * seg_h
            if not on:
                col = DKGREY
            elif i < BAR_SEGS * 0.6:
                col = GREEN
            elif i < BAR_SEGS * 0.85:
                col = AMBER
            else:
                col = RED
            self._nx.fill(1, y + 1, BAR_W - 2, seg_h - 2, col)

    def _refresh(self):
        if self._nx is None:
            return
        self._nx.poll()
        kmh = self._speed * 3.6

        # lap times
        self._put("cur", BAR_W + 50, 4, 224, 24, fmt_lap(self._cur_s), GREEN)
        self._put("last", BAR_W + 50, 30, 224, 24, fmt_lap(self._last_s), WHITE)
        self._put("best", BAR_W + 50, 56, 224, 24, fmt_lap(self._best_s), CYAN)
        dcol = GREEN if (self._delta_s is not None and self._delta_s <= 0) else RED
        self._put("delta", 382, 8, 94, 26, fmt_delta(self._delta_s), dcol)
        self._put("lap", 300, 40, 176, 24, "LAP %d" % self._lap, WHITE)
        if not self._fix:
            gps, gcol = "GPS: NO FIX", RED
        elif not self._line_set:
            gps, gcol = "GPS: no line", AMBER
        else:
            gps, gcol = "GPS: ok", GREEN
        self._put("gps", 300, 66, 176, 22, gps, gcol)

        # stat row: one line each, unit baked in -> no stacked-label clipping
        self._put("wh", BAR_W + 4, 92, 156, 24,
                  "-- Wh" if self._energy_wh is None else "%.0f Wh" % self._energy_wh)
        self._put("whkm", 184, 92, 150, 24,
                  "-- Wh/km" if self._wh_per_km is None
                  else "%.0f Wh/km" % self._wh_per_km)
        self._put("temp", 346, 92, 130, 24,
                  "-- C" if self._temp_c is None else "%.0f C" % self._temp_c)

        # big speed + gear letter
        self._draw_speed(kmh)
        self._draw_bar(kmh / BAR_MAX_KMH)
        gear = "!" if self._estop else ("D" if self._armed else "N")
        self._put("gear", BAR_W + 6, SPEED_Y + SPEED_H // 2 - 12, 40, 24,
                  gear, AMBER)

        # status banner (left half = state, right half = mode; tap right to flip)
        if self._estop:
            state, scol = "E-STOP", RED
        elif self._armed:
            state, scol = "ARMED", GREEN
        else:
            state, scol = "DISARMED", DKGREY
        by = SCREEN_H - 52
        self._put("state", 0, by, 158, 24, state, WHITE, scol)
        self._put("mode", 160, by, 158, 24, "MODE:" + self._mode.upper(),
                  BLACK, AMBER)
        # warn gets its own zone so it never hides the mode
        self._put("warn", 320, by, SCREEN_W - 320, 24,
                  self._warn[:14] if self._warn else "", WHITE,
                  DKRED if self._warn else BLACK)

    def shutdown(self):
        if self._nx is not None:
            self._nx.cls(BLACK)
            self._nx.box_text(1, 1, SCREEN_W - 2, 30, "HMI offline", GREY, BLACK)
            self._nx.close()


def _on_term(signum, _frame):
    raise SystemExit(signum)


def main():
    rclpy.init()
    node = NextionHMI()
    signal.signal(signal.SIGTERM, _on_term)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
