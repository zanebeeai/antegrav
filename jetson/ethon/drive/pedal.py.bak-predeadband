"""Manual-drive pedal input, and its two selectable release behaviours.

XIAO RP2040 (see pico/pedal_code.py) streams normalised pedal position
(0.0-1.0 ASCII float, one line per sample) over its own dedicated USB CDC
port — NOT /cmd_vel, so it can never fight cone_corridor_planner's publishes
there. Optional hardware: if the port is not present, the pedal is simply
inert and nothing else about the node's behaviour changes.

Two pedal modes, selected live by the `pedal_mode` dashboard parameter
(string, default "one_pedal"):

  one_pedal (default) — releasing the pedal drives the commanded speed to 0
    and lets the shared torque map (torque_map.py) handle it: since that map
    reads "commanded speed below actual" as brake, lifting off regen-brakes,
    same as autonomy easing off /cmd_vel. This is today's only behaviour.

  coast — releasing the pedal commands zero torque, not brake. Forward
    output is proportional to pedal position with no speed-tracking and no
    braking term, so letting go simply stops driving and the car freewheels
    (conventional, non-regen accelerator-pedal feel).

Reverse-hold (see TOPIC_REVERSE in node.py) and the wheel brake button
behave identically in both modes — only forward-release behaviour differs,
and only for pedal-sourced commands. An unrecognised pedal_mode value falls
back to one_pedal (the more conservative choice: braking-on-release rather
than silently coasting).
"""
import time

import serial

from .torque_map import drive_output
from .util import _clamp

PEDAL_PORT = "/dev/ethon-pedal"   # udev symlink -- TODO once hardware is
                                   # plugged in and its USB serial is known
                                   # (see 99-ethon-usb.rules)
PEDAL_BAUD = 115200
PEDAL_TIMEOUT_S = 0.3              # stale pedal link -> treated as unplugged

# Link freshness alone CANNOT mean "the human is driving". Measured 2026-08-18:
# the XIAO streams "0.000" about 90x/second whenever it is merely plugged in, so
# the old link-freshness-only test made pedal_active permanently True. The
# consequences were severe and silent:
#   * steering.py releases the steer motor to neutral whenever pedal_active is
#     set (the hand-back rule), so a connected-but-untouched pedal made
#     AUTONOMOUS STEERING IMPOSSIBLE; and
#   * node.py gives the pedal speed authority over /cmd_vel when armed, so the
#     resting 0.000 also pinned the planner's commanded speed to zero.
# So "active" now additionally requires an actual press.
PEDAL_ENGAGE_FRAC = 0.03           # above this = a foot is genuinely on it

# ...and manual authority is held briefly after lift-off. Without this, normal
# one-pedal modulation (which dips through zero constantly) would hand steering
# back and forth between human and autonomy several times a second -- violent,
# and far worse than either mode alone. This also preserves regen-on-release:
# the release still resolves inside the manual path, as one_pedal intends.
PEDAL_RELEASE_HOLD_S = 1.5

MODE_ONE_PEDAL = "one_pedal"
MODE_COAST = "coast"
DEFAULT_PEDAL_MODE = MODE_ONE_PEDAL


class PedalLink:
    """Non-blocking reader for the manual-drive pedal's ASCII serial stream."""

    def __init__(self, logger):
        self.frac = 0.0
        self._time = 0.0          # last VALID SAMPLE (link liveness)
        self._engaged = 0.0       # last sample above PEDAL_ENGAGE_FRAC
        self._rx = bytearray()
        self._ser = None
        self._log = logger
        try:
            self._ser = serial.Serial(PEDAL_PORT, PEDAL_BAUD, timeout=0)
            logger.info("pedal link open on %s" % PEDAL_PORT)
        except (serial.SerialException, OSError) as exc:
            logger.warning(
                "pedal link not available (%s: %s) -- manual-drive pedal "
                "inert, everything else unaffected" % (PEDAL_PORT, exc))

    def pump(self):
        """Non-blocking read of the pedal's newline-delimited ASCII stream.

        Called every tick. Parses only the LAST complete line in whatever
        arrived since the last pump -- if several samples queued up (a
        scheduling hiccup), older ones are simply superseded, never queued
        and replayed late. A partial trailing line is kept for next time.
        """
        if self._ser is None:
            return
        try:
            data = self._ser.read(256)
        except (serial.SerialException, OSError) as exc:
            self._log.warning("pedal link read failed: %s" % exc)
            self._ser = None
            return
        if not data:
            return
        self._rx.extend(data)
        lines = self._rx.split(bytes([10]))
        self._rx = bytearray(lines[-1])   # keep any partial tail
        for line in reversed(lines[:-1]):  # newest complete line wins
            try:
                frac = float(line.strip())
            except ValueError:
                continue
            self.frac = _clamp(frac, 0.0, 1.0)
            self._time = time.monotonic()
            if self.frac > PEDAL_ENGAGE_FRAC:
                self._engaged = self._time
            break

    def active(self, now, timeout_s=PEDAL_TIMEOUT_S):
        """True only when the pedal is BOTH connected and actually being used.

        Two separate conditions, deliberately: a stale link means unplugged, and
        a live link that has not been pressed within PEDAL_RELEASE_HOLD_S means
        the driver's foot is off. See the notes on PEDAL_ENGAGE_FRAC above --
        conflating these two is what blocked autonomous steering entirely.
        """
        if (now - self._time) > timeout_s:
            return False                       # link stale / unplugged
        return (now - self._engaged) <= PEDAL_RELEASE_HOLD_S


class PedalMode:
    name = "base"

    def compute(self, pedal_frac, wheel_ms, limit, cfg, use_foc, live):
        """Return (is_duty: bool, value: float) ready for set_control()."""
        raise NotImplementedError


class OnePedalMode(PedalMode):
    name = MODE_ONE_PEDAL

    def compute(self, pedal_frac, wheel_ms, limit, cfg, use_foc, live):
        v_cmd = pedal_frac * cfg.max_speed_ms
        return drive_output(v_cmd, wheel_ms, limit, use_foc, cfg, live)


class CoastPedalMode(PedalMode):
    name = MODE_COAST

    def compute(self, pedal_frac, wheel_ms, limit, cfg, use_foc, live):
        frac = _clamp(pedal_frac, 0.0, 1.0)
        if use_foc:
            return False, _clamp(frac * limit, 0.0, limit)
        hi = limit / max(cfg.max_drive_a, 1.0)   # same thermal-derated duty
                                                  # cap the torque map uses
        return True, _clamp(frac * hi, 0.0, hi)


_IMPLS = {m.name: m for m in (OnePedalMode(), CoastPedalMode())}


def get_pedal_mode(name: str) -> PedalMode:
    return _IMPLS.get(name, _IMPLS[DEFAULT_PEDAL_MODE])
