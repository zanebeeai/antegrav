"""Accelerator pedal firmware -- Seeed XIAO RP2040, CircuitPython.

Reads the pedal potentiometer on A0 and streams normalised position (0.0 =
released, 1.0 = floored) over USB serial as plain ASCII, one line per sample:

    0.000\n
    0.734\n
    ...

Deliberately NOT the wheel hub's framed multi-channel protocol (SOF/CRC8/
channel routing) -- that exists to multiplex Nextion + LED + button traffic
over one link. This is a single scalar value on its own dedicated USB CDC
port, so a plain newline-delimited stream is simpler and easier to debug
(readable directly from a serial terminal) with nothing to gain from framing.

ethon_drive.py on the Jetson reads this directly (see drive/pedal.py) -- there
is no intermediate bridge process. Low latency matters here: this is a
throttle input.

Wiring: pedal potentiometer wiper -> A0, outer legs -> 3V3 and GND. A0 is
XIAO RP2040's GPIO26 (ADC0) -- NOT GPIO18, which is a plain digital pin and
cannot read an analog voltage on this board. Only A0/A1/A2 (GPIO26/27/28)
have ADC on the XIAO RP2040.

If the pedal reads backwards (1.0 at rest, 0.0 floored), swap PEDAL_MIN and
PEDAL_MAX below rather than rewiring.

This board also drives a static white NeoPixel strip -- see the block below.
That is a deliberate afterthought bolted onto the throttle board, so it is
written to be incapable of affecting the throttle: see the notes there.

NOTE: this file is the repo mirror. The LIVE copy is /mnt/pedal/code.py on the
CIRCUITPY mount, which auto-reloads on write. Sync live -> repo before editing
and deploy repo -> live afterwards, same convention as pico/code.py.
"""

import time

import analogio
import board

# ── calibration -- MEASURED 2026-08-17 on the actual pedal ─────────────────
# Raw analogio.AnalogIn.value is 16-bit (0-65535) regardless of the RP2040's
# native 12-bit ADC resolution; CircuitPython scales it up internally.
# Pedal's real electrical range is 0.26-1.0 of full scale, not 0.0-1.0 --
# the potentiometer's mechanical travel doesn't sweep its full resistance.
PEDAL_MIN = 17089        # raw reading at fully RELEASED (0.26 * 65535)
PEDAL_MAX = 65535        # raw reading at fully FLOORED  (1.00 * 65535)
DEADBAND_LOW = 0.02      # normalised position below this reports as exactly 0.0
                          # -- pots don't rest at a perfectly repeatable raw
                          # value, and a throttle that reads 0.01 at rest would
                          # creep the car. Zero must mean zero.

SAMPLE_HZ = 100.0
SMOOTH_ALPHA = 0.08      # exponential smoothing -- raw ADC is noisy enough
                          # that an unsmoothed throttle feels twitchy.
                          # Started at 0.3; raised to 0.08 (heavier filtering)
                          # 2026-08-18 after real-world testing showed
                          # noise-driven jitter reaching the drive motors as
                          # rapid accel/regen alternation. Trades a little
                          # response lag for a controllable throttle.

# ── NeoPixel strip -- static white ────────────────────────────────────────
# Data line is on the pad silkscreened D6, which is GPIO0 on the XIAO RP2040.
# board.D6 is the right name for it; do NOT "fix" this to board.D4, which is
# the pad carrying GPIO6 -- the two are on opposite edges of the board and
# only one of them has the strip soldered to it.
#
# D6 doubles as the default UART TX pad. Nothing here opens a UART (the REPL
# and the pedal stream both run over USB CDC), so the pin is free.
#
# The strip runs from its own 5V supply. That supply's ground MUST be bonded
# to the XIAO's ground or the data line has no reference and the strip will
# either stay dark or show garbage. The RP2040 drives 3.3V data into a 5V
# WS2812, which is out of spec but works reliably when the first pixel is
# close to the board -- if the first pixel misbehaves and the rest are fine,
# that is the symptom, and the fix is a level shifter, not this code.
#
# WHY THIS IS SAFE ON A THROTTLE BOARD, and the rule for anyone editing:
# the colour is constant, so the strip is written exactly ONCE at startup and
# never touched again. neopixel_write() disables interrupts for roughly 30 us
# per pixel while it bit-bangs the protocol; doing that inside the 100 Hz
# sample loop would inject jitter straight into the throttle signal. Keep the
# write out of the loop. If a future change needs animation, drive it from a
# second board rather than this one.
PIXEL_PIN = board.D6           # silkscreen D6 == GPIO0
NUM_PIXELS = 120               # SET TO YOUR STRIP LENGTH. Over-stating this is
                               # harmless (surplus bits fall off the end of a
                               # shorter strip); under-stating it leaves the
                               # tail dark. Doubled 60 -> 120 on 2026-08-18.
PIXEL_BRIGHTNESS = 1.0         # full white is ~60 mA per pixel: 120 px = ~7.2 A
                               # at 5V. Lower this if the supply sags -- the
                               # symptom is the far end of the strip drifting
                               # yellow/orange as the injected 5V droops.
PIXEL_COLOUR = (255, 255, 255)


def _init_pixels():
    """Light the strip white. Never raises.

    A lighting fault must not be able to take the throttle offline, so every
    failure mode here -- missing library, wrong pin, no memory -- is swallowed
    and reported as a comment line. drive/pedal.py parses each line with
    float() inside `except ValueError`, so a non-numeric line is skipped
    rather than misread as a pedal position.
    """
    try:
        import neopixel

        strip = neopixel.NeoPixel(
            PIXEL_PIN, NUM_PIXELS,
            brightness=PIXEL_BRIGHTNESS, auto_write=False)
        strip.fill(PIXEL_COLOUR)
        strip.show()
        return strip
    except Exception as exc:            # noqa: BLE001 -- deliberate catch-all
        print("# neopixel init failed, throttle unaffected: %s" % exc)
        return None


def _clamp01(x):
    return 0.0 if x < 0.0 else 1.0 if x > 1.0 else x


# Held only so CircuitPython's garbage collector cannot reclaim the object and
# release the pin out from under the strip.
_pixels = _init_pixels()

pedal = analogio.AnalogIn(board.A0)
smoothed = 0.0
period = 1.0 / SAMPLE_HZ

while True:
    t0 = time.monotonic()

    raw = pedal.value
    frac = _clamp01((raw - PEDAL_MIN) / (PEDAL_MAX - PEDAL_MIN))
    if frac < DEADBAND_LOW:
        frac = 0.0

    smoothed += SMOOTH_ALPHA * (frac - smoothed)

    print("%.3f" % smoothed)

    dt = time.monotonic() - t0
    if dt < period:
        time.sleep(period - dt)
