#!/usr/bin/env python3
"""NeoPixel (WS2812) lighting for the team 1360 Electrathon EV.

A single addressable strip runs along the roll cage. The first stretch faces
REARWARD and behaves as combined tail + brake lights; the remainder faces
FORWARD as a white daytime-running light. The strip is driven DIRECTLY from the
Jetson Orin NX over SPI (MOSI), encoding the WS2812 1-wire timing into the SPI
bitstream -- no Arduino, no rpi_ws281x (that lib is Pi-only). Only spidev is
needed, and it is already installed.

Because the car is SINGLE-PEDAL (lifting off the throttle regen-brakes), there
is no brake pedal to tap. Instead the brake lights follow the drive node the
same way a Tesla does: when the motors are regen-braking (net supply current
goes NEGATIVE) or the car is decelerating, the rear LEDs ramp from a dim tail
glow up to full-bright red, proportional to braking effort. E-stop flashes the
whole strip as a hazard.

Subscribes:
  /ethon/drive_status  std_msgs/String  JSON: wheel_speed_ms, enabled,
                                         motors{label:{supply_a,...}}  (2 Hz)
  /ethon/estop         std_msgs/Bool    latched stop indicator

Publishes: nothing (output-only node).

WIRING / HARDWARE NOTES
  * Data: SPI0 MOSI on the 40-pin header (pin 19) -> strip DIN, via a ~330 ohm
    series resistor close to the first LED. /dev/spidev0.0 must exist (it does).
  * The Jetson MOSI idles at 3.3 V logic. WS2812B usually latches a 3.3 V data
    line if the lead is short; for a long run add a 74AHCT125 level shifter to
    5 V. Keep DIN lead short either way.
  * Power: do NOT run a long strip off the header 5 V pin. Use a separate 5 V
    supply sized for the strip (~60 mA/LED at full white) and tie its ground to
    the Jetson ground (common GND is mandatory).
  * Group jetson is already in 'gpio', which owns /dev/spidev* -- no sudo.

CONFIGURE FOR YOUR CAR
  * Set N_REAR / N_FRONT below to your physical LED counts. Rear LEDs are strip
    indices [0 .. N_REAR-1] (default 0..50 -> 51 brake/tail LEDs), front LEDs
    are the next N_FRONT. If your forward run is wired first, swap the slices or
    set REAR_FIRST = False.
  * Bench-check wiring with:  python3 led_lights.py --selftest
"""

import argparse
import json
import signal
import sys
import time

# ── strip layout (EDIT to match the physical wiring) ────────────────────────
N_REAR = 51            # rear-facing brake/tail LEDs  -> indices 0 .. 50
N_FRONT = 0            # forward-facing DRL LEDs (set to your count; 0 = none)
REAR_FIRST = True      # True: rear LEDs come first on the strip (indices 0..)

# ── SPI / driver config ─────────────────────────────────────────────────────
SPI_BUS, SPI_DEV = 0, 0        # /dev/spidev0.0  (MOSI = header pin 19)
SPI_HZ = 2_400_000             # 2.4 MHz -> exactly 3 SPI bits per WS2812 bit
RESET_US = 300                 # latch gap; WS2812B-V5 wants >280 us

# ── look + feel ─────────────────────────────────────────────────────────────
RENDER_HZ = 40.0               # frame rate (smooths the 2 Hz drive_status data)
MAX_BRIGHT = 0.60              # global ceiling (current + eye safety), 0..1
TAIL_LEVEL = 0.14              # rear running-light brightness (fraction of full)
FRONT_LEVEL = 0.45             # forward DRL brightness (white), 0..1
BRAKE_FULL_A = 18.0            # net regen current (A) that means "full brake"
BRAKE_ON_A = 1.5              # below this magnitude, not braking (noise floor)
BRAKE_FULL_DECEL = 2.5         # m/s^2 deceleration that means "full brake"
ATTACK = 0.85                  # brake-light rise smoothing (high = snappy)
RELEASE = 0.20                # brake-light fall smoothing (low = lingers a bit)
STALE_S = 1.5                  # no drive_status for this long -> standby
FLASH_HZ = 2.5                 # e-stop hazard flash rate

# ── topics ──────────────────────────────────────────────────────────────────
T_DRIVE = "/ethon/drive_status"
T_ESTOP = "/ethon/estop"


class Ws2812Spi:
    """Drive a WS2812 strip over SPI by expanding each colour bit to 3 SPI bits.

    At 2.4 MHz one SPI bit is ~0.417 us. A WS2812 '0' is encoded 0b100
    (0.42 us high / 0.83 us low) and a '1' as 0b110 (0.83 us high / 0.42 us
    low) -- both within the WS2812B timing window. 8 colour bits -> 24 SPI bits
    -> 3 bytes, so each pixel (GRB) is 9 SPI bytes, sent as one continuous
    transfer per frame followed by a low reset gap to latch.
    """

    def __init__(self, n, bus=SPI_BUS, dev=SPI_DEV, hz=SPI_HZ, reset_us=RESET_US):
        import spidev                     # imported lazily: laptop has no spidev
        self.n = n
        self._spi = spidev.SpiDev()
        self._spi.open(bus, dev)
        self._spi.max_speed_hz = hz
        self._spi.mode = 0
        self._lut = self._build_lut()
        reset_bytes = max(1, int(hz * reset_us / 1e6 / 8))
        self._reset = bytes(reset_bytes)
        self._grb = bytearray(n * 3)      # framebuffer, GRB per pixel

    @staticmethod
    def _build_lut():
        """value 0..255 -> 3 SPI bytes carrying its 8 bits, 3 SPI bits each."""
        lut = []
        for v in range(256):
            bits = 0
            for i in range(8):
                bits = (bits << 3) | (0b110 if (v >> (7 - i)) & 1 else 0b100)
            lut.append(bytes(((bits >> 16) & 0xFF,
                              (bits >> 8) & 0xFF,
                              bits & 0xFF)))
        return lut

    def set_rgb(self, i, r, g, b):
        if 0 <= i < self.n:
            o = i * 3
            self._grb[o] = g & 0xFF       # WS2812 wire order is G, R, B
            self._grb[o + 1] = r & 0xFF
            self._grb[o + 2] = b & 0xFF

    def fill(self, r, g, b):
        for i in range(self.n):
            self.set_rgb(i, r, g, b)

    def show(self):
        lut = self._lut
        out = bytearray()
        for byte in self._grb:
            out += lut[byte]
        out += self._reset
        self._spi.writebytes2(bytes(out))

    def clear(self):
        self._grb = bytearray(self.n * 3)
        try:
            self.show()
        except Exception:
            pass

    def close(self):
        try:
            self.clear()
            self._spi.close()
        except Exception:
            pass


def _clamp(x, lo=0.0, hi=1.0):
    return lo if x < lo else hi if x > hi else x


class LedController:
    """Maps car state -> strip colours. Driver-agnostic (testable headless)."""

    def __init__(self, strip, logger=None):
        self._strip = strip
        self._log = logger
        self.n_total = N_REAR + N_FRONT
        # rear/front index slices on the physical strip
        if REAR_FIRST:
            self._rear = range(0, N_REAR)
            self._front = range(N_REAR, N_REAR + N_FRONT)
        else:
            self._front = range(0, N_FRONT)
            self._rear = range(N_FRONT, N_FRONT + N_REAR)

        # latest car state
        self._speed = 0.0
        self._supply_a = 0.0          # net supply current (NEGATIVE = regen)
        self._enabled = False
        self._estop = False
        self._last_drive_t = None
        self._prev_speed = None
        self._prev_speed_t = None
        self._brake = 0.0             # smoothed brake intensity 0..1

    # ── state input (called from ROS callbacks or the selftest) ─────────────

    def update_drive(self, d, now):
        ws = d.get("wheel_speed_ms")
        if isinstance(ws, (int, float)):
            spd = abs(ws)
            if self._prev_speed_t is not None:
                dt = now - self._prev_speed_t
                if 0.0 < dt < 2.0:        # m/s^2 deceleration (positive = slowing)
                    self._decel = max(0.0, (self._prev_speed - spd) / dt)
            self._prev_speed, self._prev_speed_t = spd, now
            self._speed = spd
        self._enabled = bool(d.get("enabled"))
        amps = [m.get("supply_a") for m in (d.get("motors") or {}).values()
                if isinstance(m, dict) and isinstance(m.get("supply_a"), (int, float))]
        if amps:
            self._supply_a = sum(amps)
        self._last_drive_t = now

    def set_estop(self, on):
        if on:
            self._estop = True            # latched, like the rest of the stack

    _decel = 0.0

    # ── per-frame render ────────────────────────────────────────────────────

    def _brake_target(self, now):
        """0..1 braking intensity from regen current and deceleration."""
        regen = max(0.0, -self._supply_a) - BRAKE_ON_A
        b_regen = _clamp(regen / max(1e-3, BRAKE_FULL_A - BRAKE_ON_A))
        b_decel = _clamp(self._decel / BRAKE_FULL_DECEL)
        return max(b_regen, b_decel)

    def render(self, now):
        strip = self._strip
        stale = (self._last_drive_t is None
                 or (now - self._last_drive_t) > STALE_S)

        # e-stop -> hazard flash overrides everything
        if self._estop:
            on = (int(now * FLASH_HZ * 2) % 2) == 0
            r = int(255 * MAX_BRIGHT) if on else 0
            for i in self._rear:
                strip.set_rgb(i, r, 0, 0)
            for i in self._front:                 # front flashes amber
                strip.set_rgb(i, r, int(r * 0.5), 0)
            strip.show()
            return

        # brake intensity, smoothed (snappy rise, gentle fall)
        target = 0.0 if stale else self._brake_target(now)
        a = ATTACK if target > self._brake else RELEASE
        self._brake += (target - self._brake) * a

        # rear: tail glow ramping to full brake red
        level = TAIL_LEVEL + (1.0 - TAIL_LEVEL) * self._brake
        r = int(255 * MAX_BRIGHT * level)
        for i in self._rear:
            strip.set_rgb(i, r, 0, 0)

        # front: white DRL (dim if we have no fresh telemetry)
        fl = FRONT_LEVEL * (0.4 if stale else 1.0)
        w = int(255 * MAX_BRIGHT * fl)
        for i in self._front:
            strip.set_rgb(i, w, w, w)

        strip.show()


# ── ROS node ────────────────────────────────────────────────────────────────

def _build_node():
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool, String

    class EthonLeds(Node):
        def __init__(self):
            super().__init__("ethon_leds")
            log = self.get_logger()
            try:
                strip = Ws2812Spi(N_REAR + N_FRONT)
                log.info("LED strip up on /dev/spidev%d.%d (%d LEDs)"
                         % (SPI_BUS, SPI_DEV, N_REAR + N_FRONT))
            except Exception as exc:       # noqa: BLE001 -- match HMI headless pattern
                log.error("cannot open SPI LED strip: %s -- running HEADLESS "
                          "(no output). Is spidev0.0 enabled and wired?" % exc)
                strip = _NullStrip(N_REAR + N_FRONT)
            self._ctl = LedController(strip, log)
            self._strip = strip

            latched = QoSProfile(depth=1,
                                 reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(String, T_DRIVE, self._on_drive, 10)
            self.create_subscription(Bool, T_ESTOP, self._on_estop, latched)
            self.create_timer(1.0 / RENDER_HZ, self._tick)

        def _on_drive(self, msg):
            try:
                d = json.loads(msg.data)
            except (ValueError, TypeError):
                return
            self._ctl.update_drive(d, time.monotonic())

        def _on_estop(self, msg):
            self._ctl.set_estop(bool(msg.data))

        def _tick(self):
            try:
                self._ctl.render(time.monotonic())
            except Exception as exc:       # never let a frame error kill the node
                self.get_logger().warning("LED render error: %s" % exc)

        def shutdown(self):
            self._strip.close()

    return rclpy, Node, ExternalShutdownException, EthonLeds


class _NullStrip:
    """Stand-in when SPI can't be opened: swallow output, stay alive."""

    def __init__(self, n):
        self.n = n

    def set_rgb(self, *_):
        pass

    def fill(self, *_):
        pass

    def show(self):
        pass

    def close(self):
        pass


def _on_term(signum, _frame):
    raise SystemExit(signum)


def main():
    rclpy, _Node, ExternalShutdownException, EthonLeds = _build_node()
    rclpy.init()
    node = EthonLeds()
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


# ── bench self-test (no ROS): validate wiring + the 0-50 brake split ────────

def selftest():
    print("LED self-test: %d rear (brake) + %d front (DRL). Ctrl-C to stop."
          % (N_REAR, N_FRONT))
    strip = Ws2812Spi(N_REAR + N_FRONT)
    ctl = LedController(strip)
    try:
        # 1) wipe each colour so you can confirm count + order (GRB vs RGB)
        for name, rgb in (("RED", (255, 0, 0)), ("GREEN", (0, 255, 0)),
                          ("BLUE", (0, 0, 255))):
            print("  fill", name)
            strip.fill(int(rgb[0] * MAX_BRIGHT), int(rgb[1] * MAX_BRIGHT),
                       int(rgb[2] * MAX_BRIGHT))
            strip.show()
            time.sleep(0.8)
        strip.clear()
        # 2) simulate a brake ramp: lift-off regen growing from 0 to full
        print("  brake ramp (rear should glow dim->bright red)")
        t0 = time.monotonic()
        while time.monotonic() - t0 < 4.0:
            now = time.monotonic()
            frac = (now - t0) / 4.0
            ctl.update_drive({"wheel_speed_ms": 5.0, "enabled": True,
                              "motors": {"d": {"supply_a": -BRAKE_FULL_A * frac}}},
                             now)
            ctl.render(now)
            time.sleep(1.0 / RENDER_HZ)
        # 3) e-stop hazard flash
        print("  e-stop hazard flash (3 s)")
        ctl.set_estop(True)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 3.0:
            ctl.render(time.monotonic())
            time.sleep(1.0 / RENDER_HZ)
    except KeyboardInterrupt:
        pass
    finally:
        strip.close()
        print("self-test done, strip cleared.")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Ethon NeoPixel brake/DRL lights")
    ap.add_argument("--selftest", action="store_true",
                    help="run a no-ROS wiring/brake demo and exit")
    args, _ = ap.parse_known_args()
    if args.selftest:
        selftest()
        sys.exit(0)
    main()
