"""The shared v_cmd -> motor-output law.

Used for every drive source that speaks in commanded speed: autonomy
(/cmd_vel), the wheel brake button (v_cmd forced to 0), reverse-hold, and
one-pedal manual-drive mode. Coast pedal mode is the one exception — it
bypasses this map entirely and commands output directly from pedal position
(see pedal.py) because this law's whole point is to brake whenever the
commanded speed is below the actual speed, which is exactly the "lift off to
brake" behaviour coast mode must NOT have.

Continuous everywhere, including across v_cmd = 0 while rolling:
  speed_err <= deadband            -> drive torque, fading to zero as the
                                       error approaches the deadband edge
  speed_err >  deadband (rolling)  -> regen, ramping from zero at the edge
                                       to full over regen_ramp_ms (regardless
                                       of throttle sign — braking never
                                       collapses when the command crosses
                                       into reverse)
  stopped + reverse command        -> capped reverse torque
"""
from .util import _clamp, ACKERMANN_MIN_SPEED_MS


def drive_output(v_cmd, wheel_ms, limit, use_foc, cfg, live):
    """Return (is_duty: bool, value: float) ready for set_control().

    `cfg` is the boot-time vehicle.yaml namespace; `live` carries the
    dashboard-tunable values (regen_k, duty_kv_ms, duty_kp, duty_brake_kp,
    duty_brake_max) already resolved by the caller for this tick.
    """
    if use_foc:
        # FOC path: torque is the actuator, so the map below IS the
        # controller — the vehicle's own inertia integrates torque into
        # speed and damps the loop.
        throttle = _clamp(v_cmd / cfg.max_speed_ms, -1.0, 1.0)
        speed_err = wheel_ms - v_cmd
        if wheel_ms > ACKERMANN_MIN_SPEED_MS and \
                speed_err > cfg.regen_deadband_ms:
            amps = -min(cfg.regen_a, limit) * live.regen_k * min(
                1.0, (speed_err - cfg.regen_deadband_ms) / cfg.regen_ramp_ms)
        elif throttle >= 0.0:
            fade = 1.0 - _clamp(speed_err / cfg.regen_deadband_ms, 0.0, 1.0)
            amps = throttle * limit * fade
        else:
            amps = max(-cfg.reverse_cap, throttle) * limit
        return False, amps

    # Duty path: duty is a SPEED-like actuator (steady state ~ duty x free
    # speed), so the torque map must NOT be pushed through it — its 0.5 m/s
    # fade band becomes a P speed loop with ~36x gain, which is the
    # surging/jitter this replaced. Instead: feedforward sized by the
    # measured speed-per-duty, plus a gentle P trim. Continuous through zero
    # error; lifting off drops v_cmd, the error goes negative and the same
    # law brakes — same single-pedal feel, but braking gets a steeper gain
    # and its own LOW cap (negative duty while rolling is plugging, harsher
    # per duty than regen current, and it is what made lift-off buck).
    err = v_cmd - wheel_ms
    kv = max(live.duty_kv_ms, 0.1)
    kp = live.duty_kp if err >= 0.0 else live.duty_brake_kp * live.regen_k
    duty = v_cmd / kv + kp * err
    hi = limit / max(cfg.max_drive_a, 1.0)   # thermal derate, as a fraction
                                             # of full duty
    lo = -abs(live.duty_brake_max) * live.regen_k
    if v_cmd < 0.0 and wheel_ms <= ACKERMANN_MIN_SPEED_MS:
        # Commanded reverse may exceed the brake cap, up to the same reverse
        # fraction the torque map allows — but only once the car is (nearly)
        # stopped or already rolling backwards. Holding reverse at speed
        # brakes at the normal cap first; it must not unlock 2x-deep
        # plugging while rolling forward.
        lo = min(lo, -cfg.reverse_cap * hi)
    return True, _clamp(duty, lo, hi)
