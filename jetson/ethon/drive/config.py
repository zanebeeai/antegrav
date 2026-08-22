"""vehicle.yaml loading: every operator-tunable vehicle constant, with
built-in safe defaults if the file is missing or a key is malformed.

Keys marked MEASURE are placeholders — measure on the car before driving.
"""
from types import SimpleNamespace

import yaml

CONFIG_PATH = "/home/jetson/ethon/vehicle.yaml"

CONFIG_DEFAULTS = {
    # CAN
    "can_bus": "can0",
    "drive_master_id": 0,
    "drive_follower_ids": [1, 2],
    "steer_can_id": 3,
    "cancoder_can_id": 4,
    # drivetrain
    "wheel_dia_m": 26 * 0.0254,     # 26" rear wheel
    "gear_ratio": 11.46,            # motor rotations per wheel rotation
    "max_speed_ms": 8.0,            # /cmd_vel saturation speed
    "drive_inverted": False,        # BENCH-VERIFY: +cmd must roll forward
    "steer_inverted": False,        # BENCH-VERIFY: +angular.z must steer LEFT
    "max_drive_a": 80.0,            # full torque current (cool motors)
    "min_drive_a": 50.0,            # derated current at temp ceiling
    "derate_lo_c": 55.0,
    "derate_hi_c": 70.0,
    "reverse_cap": 0.30,            # reverse torque fraction
    "reverse_speed_frac": 0.30,     # hold-to-reverse SPEED cap: full pedal in
                                    # reverse commands this fraction of
                                    # max_speed_ms (0.30 -> 2.4 m/s)
    "regen_a": 40.0,                # max regen braking current
    "regen_deadband_ms": 0.5,       # m/s below wheel speed before regen
    "regen_ramp_ms": 2.0,           # m/s of speed error for full regen
    # control mode: True = TorqueCurrentFOC (needs a Phoenix Pro license per
    # device); False = DutyCycleOut, open-loop, no license required. Live-
    # toggleable from the dashboard. Default False so the drive works on
    # unlicensed devices; flip to True once Pro licensing is sorted.
    "use_foc": False,
    "test_max_duty": 0.30,          # bench-test duty-cycle safety cap (fraction)
    # Duty-mode (use_foc false) velocity law. Duty cycle is a SPEED-like
    # actuator (steady-state wheel speed ~ duty x free speed), not a torque:
    # running the torque map's amps through it turns the map's 0.5 m/s fade
    # band into a proportional speed loop with a gain of ~36 — far past
    # oscillation. Duty mode instead uses feedforward + a gentle P trim:
    #   duty = v_cmd/duty_kv_ms + duty_kp * (v_cmd - wheel_ms)
    # with braking on its own steeper gain and its own LOW cap, because
    # negative duty on a rolling motor is plugging (drives against back-EMF),
    # much harsher than the same current in regen.
    "duty_kv_ms": 18.0,             # TUNE — wheel m/s at duty 1.0 (~free speed:
                                    # 100 rps rotor / 11.46 x pi x 0.66 m ≈ 18)
    "duty_kp": 0.02,                # TUNE — duty per m/s of speed error (drive)
    "duty_brake_kp": 0.08,          # TUNE — duty per m/s of overspeed (braking)
    "duty_brake_max": 0.15,         # braking/plugging duty cap (before
                                    # regen_strength scaling)
    "duty_ramp_s": 0.15,            # device-side 0->full duty ramp, seconds —
                                    # smooths every 50 Hz step at 1 kHz on the
                                    # Talon itself (also covers the bench test)
    # steering
    "wheelbase_m": 1.60,            # MEASURE
    "steer_col_ratio": 12.0,        # MEASURE — steering-wheel deg per road-wheel deg
    "steer_belt_ratio": 4.0,        # MEASURE — motor rots per column rot
    "steer_limit_rot": 1.0,         # MEASURE — soft limit, +/- column rots
    "steer_max_a": 15.0,            # driver must be able to overpower
    # Steering closed-loop gains. Two independent slots so the use_foc toggle
    # switches control mode WITHOUT retuning: slot 0 drives
    # PositionTorqueCurrentFOC (gain units: amps per rotor-rotation of error,
    # needs a Pro licence); slot 1 drives PositionDutyCycle (units: duty
    # fraction per rotor-rotation of error, no licence). Error is in ROTOR
    # rotations, so 1 column rotation = steer_belt_ratio rotor rotations.
    # ALL FOUR ARE UNTUNED STARTING POINTS — bench-tune with wheels off the
    # ground, raising kp until it tracks without hunting, then add kd.
    "steer_kp_foc": 8.0,            # TUNE — A per rotor rot of error
    "steer_kd_foc": 0.1,            # TUNE
    "steer_kp_duty": 0.30,          # TUNE — duty per rotor rot of error
    "steer_kd_duty": 0.02,          # TUNE — duty per rotor rps; 0.004 was ~zero
                                    # damping and the position loop hunted
                                    # through the gear backlash
    "steer_slew_col_rps": 1.5,      # column target slew limit, rot/s — the raw
                                    # 50 Hz Ackermann target dithers with omega
                                    # noise; the wheel should not chase that.
                                    # 1.5 still covers the full ±0.40 rot range
                                    # in ~0.27 s. Bench steer_test bypasses it.
    "steer_peak_duty": 0.25,        # duty cap for the non-FOC steering path —
                                    # the hand-overpower guarantee in duty mode
                                    # (stator limit still applies on top)
    # homing method: "cancoder" (seed rotor from the absolute CANcoder) or
    # "lock_to_lock" (sweep to both mechanical stops, centre on the midpoint,
    # derive soft limits from the measured range).
    "steer_home_method": "cancoder",
    # ── lock-to-lock homing tuning (only used when method == "lock_to_lock") ──
    # The sweep runs open-loop DutyCycleOut (no Pro licence) but the TORQUE it
    # can apply against a stop is set by the stator current limit, which is
    # temporarily dropped to steer_home_current_a for the sweep and restored to
    # steer_max_a afterwards. Keep this LOW: it must be enough to reach the
    # stop but gentle enough that stalling against it cannot damage the belt,
    # pulleys or column, and so a hand on the wheel always wins.
    "steer_home_current_a": 8.0,    # TUNE — stator current cap during homing
    # Sweep speed. duty, stall_rps and timeout MUST be tuned together: the
    # stall threshold has to sit well BELOW the free-running rotor velocity at
    # this duty, or the sweep reads "stalled" the instant it starts moving and
    # records a false stop. Halving the duty roughly halves that velocity.
    "steer_home_duty": 0.05,        # open-loop duty swept toward each stop
    "steer_home_stall_rps": 0.15,   # |rotor velocity| below this = stalled
    "steer_home_stall_s": 0.50,     # ...sustained this long = stop reached
    "steer_home_timeout_s": 15.0,   # abort a sweep if no stop found in this time
    "steer_home_margin_rot": 0.05,  # column rots kept inside each stop
    "steer_home_right_sign": 1.0,   # duty sign that drives RIGHT (flip to -1.0
                                    # if the first sweep goes the wrong way)
    # Plausibility floor, in COLUMN rotations of half-range. A sweep that
    # stalls early (tight spot, someone's hand, too high a stall threshold)
    # yields a tiny range; accepting it silently ENABLES steering that is
    # effectively locked straight. Reject anything below this instead.
    "steer_home_min_half_rot": 0.15,
    "cancoder_offset_rot": 0.0,     # MEASURE — CANcoder reading at centre
    "cancoder_invert": False,
    "allow_unhomed_steering": False,
    # safety
    "geometry_measured": False,     # set true ONLY after filling every MEASURE
                                    # key from the real car. While false the
                                    # node refuses to command motors at all.
    # timing
    "cmd_timeout_s": 0.25,          # stale /cmd_vel -> disable
}


def _coerce(default, value):
    """Coerce a YAML value to the type of its built-in default."""
    if isinstance(default, bool):                 # bool first: bool < int
        if not isinstance(value, bool):
            raise TypeError("expected bool")
        return value
    if isinstance(default, int):
        return int(value)
    if isinstance(default, float):
        return float(value)
    if isinstance(default, str):
        return str(value)
    if isinstance(default, list):
        return [int(x) for x in value]
    raise TypeError(f"unsupported config type {type(default).__name__}")


def load_vehicle_config(path: str, logger) -> SimpleNamespace:
    """Load vehicle.yaml over CONFIG_DEFAULTS. Any failure -> defaults."""
    cfg = {k: (list(v) if isinstance(v, list) else v)
           for k, v in CONFIG_DEFAULTS.items()}
    try:
        with open(path, "r") as f:
            data = yaml.safe_load(f) or {}
    except FileNotFoundError:
        logger.warning(
            f"{path} not found — using built-in defaults "
            "(geometry placeholders are UNMEASURED)")
        return SimpleNamespace(**cfg)
    except Exception as exc:
        logger.error(f"failed to read {path}: {exc} — using built-in defaults")
        return SimpleNamespace(**cfg)

    if not isinstance(data, dict):
        logger.error(f"{path}: top level must be a mapping — using defaults")
        return SimpleNamespace(**cfg)

    for key, value in data.items():
        if key not in cfg:
            logger.warning(f"{path}: unknown key '{key}' ignored")
            continue
        try:
            cfg[key] = _coerce(cfg[key], value)
        except Exception:
            logger.error(
                f"{path}: bad value for '{key}' ({value!r}) — "
                f"keeping default {cfg[key]!r}")
    logger.info(f"vehicle config loaded from {path}")
    return SimpleNamespace(**cfg)
