"""Steer-Kraken ownership: config, homing, and the per-tick ackermann command.

Two control paths, selected by the same `use_foc` toggle as the drive:
    use_foc true  -> PositionTorqueCurrentFOC, gain slot 0 (needs Pro licence)
    use_foc false -> PositionDutyCycle,        gain slot 1 (no licence),
                     output bounded by `steer_peak_duty`.
Separate slots mean flipping the toggle needs no retuning.

Homed at construction by `steer_home_method`: "cancoder" (absolute, needs the
CANcoder within +/-0.5 column rot of centre) or "lock_to_lock" (sweep both
mechanical stops current-limited, centre on the midpoint, derive the soft
limits from the measured range). If homing fails, `enabled` is False unless
`allow_unhomed_steering: true` (bench only, wheels centred).

HAND-BACK RULE (safety-relevant): the column is released to NeutralOut
whenever the car is not actually being autonomy-driven — disarmed, or a
human is on the pedal/brake — so a driver can always turn the wheel by hand.
Gate this on `armed`/pedal/brake, never on command freshness alone (the
planner keeps publishing zero /cmd_vel while disarmed, so freshness alone
never tells this apart from autonomy actually driving).
"""
import math
import time

from phoenix6 import unmanaged
from phoenix6.configs import TalonFXConfiguration
from phoenix6.controls import DutyCycleOut, PositionDutyCycle, PositionTorqueCurrentFOC
from phoenix6.hardware import CANcoder, TalonFX
from phoenix6.signals import InvertedValue

from .can_bus import CONFIG_APPLY_RETRIES, apply_device_config, signal_value
from .util import ACKERMANN_MIN_SPEED_MS, BANNER, FEED_ENABLE_S, _clamp

SIGNAL_PROBE_TIMEOUT_S = 0.5   # CANcoder / version-signal probe timeout

# Lock-to-lock homing sweep tuning. UNTESTED ON HARDWARE — tune against the
# real steering before trusting it; a stall threshold that is too high reads
# a false stop mid-travel, too low never triggers.
STEER_HOME_RATE_HZ = 100.0     # sweep control-loop rate
STEER_HOME_SETTLE_S = 0.6      # pause between the two sweeps and before centring
STEER_HOME_CENTRE_TIMEOUT_S = 20.0  # abort the drive-to-centre if it never
                                    # arrives (scales with steer_home_duty —
                                    # centring creeps at the same duty)
STEER_HOME_CENTRE_TOL_ROT = 0.05    # rotor rotations from centre = "centred"


class Steering:
    """Owns the steer Kraken: config, homing, and the per-tick command."""

    def __init__(self, can_bus: str, cfg, neutral, logger):
        self.cfg = cfg
        self.can_bus = can_bus
        self._neutral = neutral
        self.log = logger

        self.steer = TalonFX(cfg.steer_can_id, can_bus)
        # Keep one CANcoder handle for homing and synchronized capture. Creating
        # a second Phoenix device owner in the recorder would split status-rate
        # configuration and make timestamps harder to reason about.
        self.cancoder = CANcoder(cfg.cancoder_can_id, can_bus)
        self._req_foc = PositionTorqueCurrentFOC(0, slot=0)
        self._req_duty = PositionDutyCycle(0, slot=1)
        self._lock_half_rotor = None   # measured half-range (lock-to-lock)
        self._col_out = None           # slewed column target (column rot);
                                        # None = steering currently released

        # Config the steer motor BEFORE homing so a lock-to-lock sweep drives
        # against the stops current-limited (stator limit) and the FOC torque
        # cap keeps position control hand-overpowerable. Soft limits start
        # from the config; lock-to-lock overrides them from the measured
        # range below.
        scfg = TalonFXConfiguration()
        scfg.motor_output.inverted = (
            InvertedValue.CLOCKWISE_POSITIVE if cfg.steer_inverted
            else InvertedValue.COUNTER_CLOCKWISE_POSITIVE)
        lim_rotor = cfg.steer_limit_rot * cfg.steer_belt_ratio
        scfg.software_limit_switch.forward_soft_limit_enable = True
        scfg.software_limit_switch.forward_soft_limit_threshold = lim_rotor
        scfg.software_limit_switch.reverse_soft_limit_enable = True
        scfg.software_limit_switch.reverse_soft_limit_threshold = -lim_rotor
        scfg.torque_current.peak_forward_torque_current = cfg.steer_max_a
        scfg.torque_current.peak_reverse_torque_current = -cfg.steer_max_a
        scfg.current_limits.stator_current_limit = cfg.steer_max_a
        scfg.current_limits.stator_current_limit_enable = True
        # Closed-loop gains. Without these the position requests produce ZERO
        # output no matter the control mode.
        scfg.slot0.k_p = float(cfg.steer_kp_foc)     # FOC: amps per rotor rot
        scfg.slot0.k_d = float(cfg.steer_kd_foc)
        scfg.slot1.k_p = float(cfg.steer_kp_duty)    # duty: fraction per rot
        scfg.slot1.k_d = float(cfg.steer_kd_duty)
        # Duty cap for the non-FOC path: this is what keeps the steering
        # hand-overpowerable when torque_current limits do not apply.
        peak_duty = _clamp(abs(float(cfg.steer_peak_duty)), 0.0, 1.0)
        scfg.motor_output.peak_forward_duty_cycle = peak_duty
        scfg.motor_output.peak_reverse_duty_cycle = -peak_duty
        steer_cfg_ok = apply_device_config(self.steer, scfg, "steer", logger)

        # Home (config applied first, above). lock_to_lock also re-derives
        # the soft limits from the measured stop-to-stop range.
        if steer_cfg_ok and cfg.steer_home_method == "lock_to_lock":
            # Homing must never take the node down: a raised exception here
            # would exit __init__, systemd would restart, and the steering
            # would sweep again on every restart — a crash loop that keeps
            # driving the column into its stops. Fail closed instead.
            try:
                self.homed = self._home_lock_to_lock()
            except Exception as exc:
                logger.error("lock-to-lock homing raised %s: %s — steering "
                             "DISABLED (node continues drive-only)"
                             % (type(exc).__name__, exc))
                self.homed = False
                try:
                    self.steer.set_control(self._neutral)
                except Exception:
                    pass
            # The sweep runs with soft limits DISABLED and a reduced current
            # cap, so the full operating config must be re-applied afterwards
            # whether homing succeeded or not — otherwise a failed home would
            # leave the device with no soft limits at all.
            if self.homed and self._lock_half_rotor is not None:
                scfg.software_limit_switch.forward_soft_limit_threshold = \
                    self._lock_half_rotor
                scfg.software_limit_switch.reverse_soft_limit_threshold = \
                    -self._lock_half_rotor
            if not apply_device_config(self.steer, scfg,
                                       "steer soft-limits", logger):
                logger.error("could not restore steer soft limits/current cap "
                             "after homing — steering DISABLED")
                self.homed = False
                steer_cfg_ok = False
        else:
            self.homed = self._home_from_cancoder()

        # Steering is only commanded when homed (or explicitly allowed
        # unhomed) AND the torque-cap/soft-limit config actually applied —
        # without the 15 A cap the driver may not be able to overpower it.
        self.enabled = (self.homed or cfg.allow_unhomed_steering) and steer_cfg_ok
        self._log_mode(steer_cfg_ok)

    @property
    def lock_half_rotor(self):
        return self._lock_half_rotor

    @property
    def column_target(self):
        """Latest commanded column position in rotations, or None if released."""
        return self._col_out

    # ── homing ───────────────────────────────────────────────────────────

    def _home_from_cancoder(self) -> bool:
        """Seed the steering rotor position from the column CANcoder.

        The CANcoder absolute range is +/-0.5 rotation, so homing is only
        unambiguous when the column is within half a turn of centre at boot
        — park the car with the wheels roughly straight.
        """
        cfg, log = self.cfg, self.log
        try:
            sig = self.cancoder.get_absolute_position()
            sig.wait_for_update(SIGNAL_PROBE_TIMEOUT_S)
            if not sig.status.is_ok():
                log.warning(
                    f"CANcoder {cfg.cancoder_can_id} not responding "
                    f"({sig.status.name})")
                return False
            raw = float(sig.value)
        except Exception as exc:
            log.warning(f"CANcoder probe failed: {exc}")
            return False

        col_rot = (-raw if cfg.cancoder_invert else raw) - cfg.cancoder_offset_rot
        for attempt in range(1, CONFIG_APPLY_RETRIES + 1):
            try:
                status = self.steer.set_position(col_rot * cfg.steer_belt_ratio)
                if status.is_ok():
                    log.info(
                        f"steering homed from CANcoder: column at "
                        f"{col_rot:+.3f} rot from centre")
                    return True
                log.warning(
                    f"steer set_position attempt {attempt}: {status.name}")
            except Exception as exc:
                log.warning(f"steer set_position attempt {attempt}: {exc}")
            time.sleep(0.1)
        return False

    def _home_lock_to_lock(self) -> bool:
        """Home the steering on the mechanical stops, then park at centre.

        Sequence: sweep RIGHT until the rotor stalls against the stop, record
        it; sweep LEFT the same way; centre = midpoint of the two; re-zero
        the encoder there and drive back to it; soft limits come from the
        measured half-range minus a margin.

        Runs open-loop DutyCycleOut with FOC disabled (no Pro licence
        needed). The torque it can push into a stop is bounded by the stator
        current limit, which is dropped to ``steer_home_current_a`` for the
        sweep and restored to ``steer_max_a`` afterwards — low enough that
        stalling cannot hurt the belt/pulleys/column and a hand on the wheel
        always wins. All thresholds are vehicle.yaml params (steer_home_*).

        The device soft limits are DISABLED for the duration: at power-on the
        rotor zero is arbitrary (no CANcoder), so a pre-homing soft limit can
        halt the sweep short of the real stop and record a false lock. They
        are re-applied from the measured range by the caller.

        !! UNTESTED ON HARDWARE !! Verify with the wheels off the ground and
        a hand ready. A stall threshold too high reads a false stop
        mid-travel; too low never triggers.
        """
        log = self.log
        cfg = self.cfg
        vel = self.steer.get_velocity()
        pos = self.steer.get_position()
        period = 1.0 / STEER_HOME_RATE_HZ
        right = 1.0 if float(cfg.steer_home_right_sign) >= 0 else -1.0

        _cc_sig = None
        try:
            _cc_sig = self.cancoder.get_absolute_position()
        except Exception as _exc:
            log.warn("    [cancoder trace] probe setup failed: %s" % _exc)

        def _set_stator_limit(amps, tag):
            """Temporarily re-limit stator current (bounds stall torque)."""
            c = TalonFXConfiguration()
            c.motor_output.inverted = (
                InvertedValue.CLOCKWISE_POSITIVE if cfg.steer_inverted
                else InvertedValue.COUNTER_CLOCKWISE_POSITIVE)
            c.software_limit_switch.forward_soft_limit_enable = False
            c.software_limit_switch.reverse_soft_limit_enable = False
            c.torque_current.peak_forward_torque_current = amps
            c.torque_current.peak_reverse_torque_current = -amps
            c.current_limits.stator_current_limit = amps
            c.current_limits.stator_current_limit_enable = True
            return apply_device_config(self.steer, c, tag, log)

        def find_stop(direction, name):
            req = DutyCycleOut(direction * float(cfg.steer_home_duty),
                               enable_foc=False)
            below_since = None
            t0 = time.monotonic()
            n = 0
            while time.monotonic() - t0 < float(cfg.steer_home_timeout_s):
                unmanaged.feed_enable(FEED_ENABLE_S)
                self.steer.set_control(req)
                time.sleep(period)
                vel.refresh()
                n += 1
                if n % 10 == 0:
                    pos.refresh()
                    cc_str = "n/a"
                    if _cc_sig is not None:
                        _cc_sig.refresh()
                        cc_str = "%+.5f" % float(_cc_sig.value)
                    log.warn("    [trace %s] t=%.2f pos=%+.3f vel=%+.4f cc=%s below_since=%s"
                              % (name, time.monotonic() - t0, float(pos.value),
                                 float(vel.value), cc_str,
                                 "%.2f" % (time.monotonic() - below_since) if below_since else "None"))
                if abs(float(vel.value)) < float(cfg.steer_home_stall_rps):
                    if below_since is None:
                        below_since = time.monotonic()
                    elif (time.monotonic() - below_since
                          >= float(cfg.steer_home_stall_s)):
                        self.steer.set_control(self._neutral)
                        pos.refresh()
                        p = float(pos.value)
                        log.warn("  %s stop found at %+.3f rotor rot" % (name, p))
                        return p
                else:
                    below_since = None
            self.steer.set_control(self._neutral)
            log.error("  %s stop NOT found within %.1fs"
                      % (name, float(cfg.steer_home_timeout_s)))
            return None

        def drive_to(target):
            """Creep back to centre under the same current-limited duty.

            Stops on arrival, on overshoot (error changes sign — no point
            hunting back and forth at open-loop duty), or on timeout.
            """
            t0 = time.monotonic()
            sign0 = None
            while time.monotonic() - t0 < STEER_HOME_CENTRE_TIMEOUT_S:
                pos.refresh()
                err = target - float(pos.value)
                if abs(err) <= STEER_HOME_CENTRE_TOL_ROT:
                    self.steer.set_control(self._neutral)
                    return True
                s = 1.0 if err > 0 else -1.0
                if sign0 is None:
                    sign0 = s
                elif s != sign0:            # crossed the target — close enough
                    self.steer.set_control(self._neutral)
                    return True
                unmanaged.feed_enable(FEED_ENABLE_S)
                self.steer.set_control(
                    DutyCycleOut(float(cfg.steer_home_duty) * s,
                                 enable_foc=False))
                time.sleep(period)
            self.steer.set_control(self._neutral)
            return False

        home_a = float(cfg.steer_home_current_a)
        log.warn("steering lock-to-lock homing at %.1f A / %.2f duty — "
                 "RIGHT then LEFT then centre. KEEP CLEAR, hand ready."
                 % (home_a, float(cfg.steer_home_duty)))
        if not _set_stator_limit(home_a, "steer homing current"):
            log.error("could not apply homing current limit — homing ABORTED, "
                      "steering DISABLED (refusing to sweep uncapped).")
            return False
        try:
            right_stop = find_stop(right, "RIGHT")
            time.sleep(STEER_HOME_SETTLE_S)
            left_stop = (find_stop(-right, "LEFT")
                         if right_stop is not None else None)
            self.steer.set_control(self._neutral)
            if right_stop is None or left_stop is None:
                log.error("lock-to-lock homing FAILED — did not find both "
                          "stops (right=%s left=%s). Steering DISABLED."
                          % (right_stop, left_stop))
                return False

            centre = (right_stop + left_stop) / 2.0
            half_rotor = abs(right_stop - left_stop) / 2.0
            margin_rotor = (float(cfg.steer_home_margin_rot)
                            * cfg.steer_belt_ratio)
            self._lock_half_rotor = max(0.0, half_rotor - margin_rotor)
            half_col = (self._lock_half_rotor / cfg.steer_belt_ratio
                        if cfg.steer_belt_ratio else 0.0)
            min_col = float(cfg.steer_home_min_half_rot)
            if self._lock_half_rotor <= 0.0 or half_col < min_col:
                log.error(
                    "homing REJECTED: measured half-range %.3f column rot "
                    "(%.3f rotor) is below the %.3f plausibility floor. A stop "
                    "was almost certainly found early — check the steering is "
                    "free to move, lower steer_home_stall_rps, or raise "
                    "steer_home_stall_s. Steering DISABLED."
                    % (half_col, self._lock_half_rotor, min_col))
                self._lock_half_rotor = None
                return False

            time.sleep(STEER_HOME_SETTLE_S)
            centred = drive_to(centre)
            pos.refresh()
            if _cc_sig is not None:
                _cc_sig.refresh()
                log.warn("    [cancoder trace] raw absolute_position AT CENTRE = %+.5f rot"
                          % float(_cc_sig.value))
            # re-zero so centre reads 0 regardless of where we stopped
            self.steer.set_position(float(pos.value) - centre)
            log.warn("steering homed: RIGHT %+.3f / LEFT %+.3f rotor rot, "
                     "half-range %.3f rot (%.3f column rot), centred=%s"
                     % (right_stop, left_stop, self._lock_half_rotor,
                        self._lock_half_rotor / max(cfg.steer_belt_ratio,
                                                    1e-6), centred))
            if not centred:
                log.warn("  (did not reach centre within %.1fs — zero was set "
                         "anyway; check the wheel is near straight)"
                         % STEER_HOME_CENTRE_TIMEOUT_S)
            return True
        finally:
            self.steer.set_control(self._neutral)
            # always restore the operating current cap, even on failure
            _set_stator_limit(cfg.steer_max_a, "steer operating current")

    def _log_mode(self, steer_cfg_ok: bool):
        log = self.log
        if self.homed and self.enabled:
            return  # happy path already logged in _home_from_cancoder
        log.error(BANNER)
        if not steer_cfg_ok:
            log.error("*  STEERING CONFIG (15 A cap / soft limits) FAILED TO APPLY")
            log.error("*  -> DRIVE-ONLY MODE. Fix CAN / device before steering.")
        elif self.enabled:
            log.error("*  STEERING UNHOMED — no CANcoder, allow_unhomed_steering")
            log.error("*  is true. Assuming the wheels are CENTRED right now.")
            log.error("*  Bench testing only — centre the wheels before boot.")
            try:
                self.steer.set_position(0.0)
            except Exception as exc:
                log.error(f"*  zero-seed failed: {exc}")
        else:
            log.error("*  STEERING NOT HOMED (CANcoder absent/dead) and")
            log.error("*  allow_unhomed_steering is false -> DRIVE-ONLY MODE.")
            log.error("*  The steering motor will never be commanded.")
        log.error(BANNER)

    # ── runtime ──────────────────────────────────────────────────────────

    def release(self):
        """Hand the column back to the driver (NeutralOut)."""
        if not self.enabled:
            return
        self._col_out = None
        try:
            self.steer.set_control(self._neutral)
        except Exception:
            pass

    def tick(self, *, v_cmd, omega, wheel_ms, armed, pedal_active,
             brake_active, steer_test_active, steer_test_deg, use_foc,
             slew_col_rps, tick_hz):
        """Ackermann angle -> column rotation -> motor position, per tick."""
        if not self.enabled:
            return
        cfg = self.cfg
        if steer_test_active:
            # Bench test: bypasses the hand-back rule (deliberate dashboard
            # command, wheels OFF the ground).
            col_rot = steer_test_deg * cfg.steer_col_ratio / 360.0
        else:
            # Also releases whenever the pedal is active (manual drive) even
            # if armed -- the human hand-steers directly during manual
            # driving, the motor must not fight them by holding a computed
            # angle.
            if not armed or pedal_active or brake_active:
                self.release()
                return
            # Reference speed for the bicycle model: prefer the command, but
            # fall back to MEASURED wheel speed so a stop command mid-corner
            # does not snap the steering to centre while the car is moving.
            v_ref = v_cmd if abs(v_cmd) > ACKERMANN_MIN_SPEED_MS else wheel_ms
            if abs(v_ref) > ACKERMANN_MIN_SPEED_MS:
                road_rad = math.atan(cfg.wheelbase_m * omega / v_ref)
            else:
                road_rad = 0.0
            col_rot = math.degrees(road_rad) * cfg.steer_col_ratio / 360.0

        # Clamp to the MEASURED lock-to-lock half-range when homing found
        # one, otherwise to the configured soft limit. Commanding past the
        # real stop just stalls the motor against it.
        col_limit = cfg.steer_limit_rot
        if self._lock_half_rotor is not None and cfg.steer_belt_ratio:
            col_limit = self._lock_half_rotor / cfg.steer_belt_ratio
        col_rot = _clamp(col_rot, -col_limit, col_limit)

        if steer_test_active:
            # Bench tuning wants clean steps — no slew, and drop the slew
            # state so post-test re-engagement re-seeds from measured position.
            self._col_out = None
        else:
            # Rate-limit the target: the raw Ackermann angle recomputed at
            # 50 Hz dithers with every omega/v sample, and a position loop
            # faithfully chasing that dither is wheel jitter. Slew from the
            # MEASURED column position on engage so there is never a snap.
            if self._col_out is None:
                meas = signal_value(self.steer, "get_position")
                seed = (meas / cfg.steer_belt_ratio
                        if meas is not None and cfg.steer_belt_ratio
                        else col_rot)
                self._col_out = _clamp(seed, -col_limit, col_limit)
            step = abs(slew_col_rps) / tick_hz
            self._col_out += _clamp(col_rot - self._col_out, -step, step)
            col_rot = self._col_out

        rotor_target = col_rot * cfg.steer_belt_ratio
        # Same toggle as the drive motors: FOC needs a Pro licence, the duty
        # path does not. Gains live in separate slots so no retuning is
        # needed when flipping the switch.
        if use_foc:
            self.steer.set_control(self._req_foc.with_position(rotor_target))
        else:
            self.steer.set_control(self._req_duty.with_position(rotor_target))
