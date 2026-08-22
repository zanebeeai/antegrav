"""Small primitives shared across the drive package."""

BANNER = "*" * 68

# Below this commanded/actual speed: torque_map skips regen gating (FOC) and
# steering falls back to the measured wheel speed for the ackermann reference
# (see torque_map.py and steering.py).
ACKERMANN_MIN_SPEED_MS = 0.3

# Phoenix6 unmanaged-control watchdog: devices disable unless fed this many
# seconds of "enable" per call. Fed once per fresh 50 Hz tick.
FEED_ENABLE_S = 0.1


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def _round(x, ndigits: int = 2):
    """Round for JSON, passing None through."""
    return None if x is None else round(float(x), ndigits)
