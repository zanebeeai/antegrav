#!/usr/bin/env python3
"""Ethon drive controller — Kraken X60s on Jetson via CAN0 + Phoenix 6 (no RoboRIO).

Module layout (see the other files in this package for the full detail on
each piece):
  config.py      vehicle.yaml loading (CONFIG_DEFAULTS, load_vehicle_config)
  can_bus.py     CAN bus resolution + Phoenix6 device-config/fault helpers
  torque_map.py  the shared v_cmd -> motor-output speed-tracking law
  pedal.py       manual-drive pedal link + the two pedal modes (this is
                 where "one_pedal" vs "coast" release behaviour lives)
  steering.py    steer Kraken: homing, hand-back rule, per-tick ackermann
  node.py        (this file) wires it all together into the ROS2 node

Drivetrain (port of the proven Robot.java logic):
  - 3x Kraken X60 drive (CAN 0 master, 1 & 2 followers, aligned) -> 11.46:1
    reduction -> single 26" rear wheel
  - manual-drive pedal has two selectable release behaviours (`pedal_mode`
    dashboard param, default "one_pedal"): "one_pedal" is release-brakes
    (positive cmd accelerates, lifting off regen-brakes -- FOC mode only; in
    duty mode the map is replaced by a feedforward+P velocity law with a
    device-side duty ramp, see torque_map.py); "coast" is release-freewheels
    (proportional torque/duty, zero at release, no braking term). Autonomy
    (/cmd_vel), the wheel brake button, and reverse-hold always use the
    one_pedal-style speed-tracking law regardless of the pedal_mode setting
    -- see pedal.py's module docstring for the full reasoning.
  - reverse torque capped at REVERSE_CAP of the (derated) limit
  - thermal derate: 80 A -> 50 A linearly over 55 -> 70 C (hottest motor)

Steering: see steering.py. Kraken X60 (CAN 4), spur gears (5:1 = 18T:90T,
module 1.25, 67.5 mm C-C) to the steering column, with soft limits and a
15 A torque cap so the driver can ALWAYS overpower the motor.

Config:
  /home/jetson/ethon/vehicle.yaml (yaml.safe_load) overrides every entry in
  config.CONFIG_DEFAULTS. Missing file -> built-in defaults with a warning.
  A commented template lives next to this file in the repo (vehicle.yaml).

Topics:
  sub  /cmd_vel             geometry_msgs/Twist  linear.x m/s, angular.z rad/s
  sub  /ethon/estop         std_msgs/Bool        true latches motors off until restart
  pub  /ethon/drive_status  std_msgs/String      JSON health snapshot at 2 Hz

Safety model (vehicle carries a human — software fails SILENT, never ACTIVE):
  - Phoenix 6 non-FRC devices neutral themselves unless
    unmanaged.feed_enable() is called continuously. We feed ONLY while a
    fresh /cmd_vel exists and the e-stop latch is clear. Process/ROS death
    -> Krakens disable within FEED_ENABLE_S (0.1 s) on their own; a stalled
    planner -> within cmd_timeout_s + FEED_ENABLE_S (~0.35 s).
  - geometry_measured false in vehicle.yaml (or missing/unreadable yaml)
    -> CONFIG HOLD: motors are never commanded with placeholder geometry.
  - /ethon/estop true sends NeutralOut and latches disabled until restart.
  - SIGINT / SIGTERM -> NeutralOut to every motor before exit.
  - The physical e-stop relay on motor power remains REQUIRED. This code is
    never the last line of defense.

Run via systemd: ethon-drive.service (after ethon-can.service). Kept as a
top-level script (not an installed ROS2 package) because the unit execs
/home/jetson/ethon/ethon_drive.py directly -- that file is now a thin shim
that imports main() from here; see PROJECT_HANDOFF.md.
"""

import json
import math
import signal
import threading
import time
from types import SimpleNamespace

import rclpy
from geometry_msgs.msg import Twist
from phoenix6 import unmanaged
from phoenix6.configs import TalonFXConfiguration
from phoenix6.controls import DutyCycleOut, Follower, NeutralOut, TorqueCurrentFOC
from phoenix6.hardware import TalonFX
from phoenix6.signals import InvertedValue, MotorAlignmentValue
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64, String

from .can_bus import apply_device_config, read_faults, resolve_can_bus, signal_value
from .config import CONFIG_PATH, load_vehicle_config
from .pedal import DEFAULT_PEDAL_MODE, PedalLink, get_pedal_mode
from .steering import Steering
from .torque_map import drive_output
from .util import FEED_ENABLE_S, _clamp, _round

# ── topics ────────────────────────────────────────────────────────────────
TOPIC_CMD_VEL = "/cmd_vel"
TOPIC_ESTOP = "/ethon/estop"
TOPIC_ARMED = "/ethon/hmi/armed"   # latched Bool from the planner; single
                                   # source of truth for "autonomy is driving"
TOPIC_STATUS = "/ethon/drive_status"
TOPIC_DRIVE_TEST = "/ethon/drive_test"       # bench-test direct duty override (Float64)
TOPIC_STEER_TEST = "/ethon/steer_test_deg"   # bench-test direct road-angle override (Float64, deg)

# ── wheel brake button ──────────────────────────────────────────────────
# Any of the 3 encoder push-buttons on the wheel (see pico/code.py's
# BUTTON_MAP) sends a momentary "brake" token, relayed by wheel_bridge.py as
# a Bool on this topic. Top priority in _tick() -- above even the manual
# pedal -- forces full regen-decelerate for BRAKE_TIMEOUT_S, re-triggerable
# by pressing again. A TAP-for-a-pulse design, not proportional hold-to-
# brake: the wheel firmware only reports button PRESS events today, not
# release, so there is no clean way to know when the driver lets go.
TOPIC_BRAKE = "/ethon/brake_request"
BRAKE_TIMEOUT_S = 1.5

# ── wheel reverse button (hold to reverse throttle) ─────────────────────
# wheel_bridge.py re-publishes True at 10 Hz while the button is held (see
# its _push_reverse). REVERSE_TIMEOUT_S is deliberately a bit looser than
# that 0.1s period -- comfortable margin against one missed tick -- but
# still tight enough that a dropped link releases it quickly, not stuck.
TOPIC_REVERSE = "/ethon/reverse_request"
REVERSE_TIMEOUT_S = 0.5
TEST_TIMEOUT_S = 0.5           # bench-test command watchdog (dashboard re-posts
                               # continuously; motors stop within this if it stops)

# ── loop rates / timing ───────────────────────────────────────────────────
TICK_HZ = 50.0                 # control loop
STATUS_HZ = 2.0                # /ethon/drive_status publish rate
EFFIC_MIN_DIST_M = 5.0         # min distance before reporting Wh/km (avoid /~0)

BANNER = "*" * 68


class EthonDrive(Node):
    """50 Hz torque-based drive + steering controller for the Ethon EV."""

    def __init__(self):
        super().__init__("ethon_drive")
        log = self.get_logger()

        self.cfg = load_vehicle_config(CONFIG_PATH, log)
        cfg = self.cfg

        # Must exist before ANY set_control call. Steering homing runs during
        # __init__ and neutrals the motor on every exit path, so this cannot
        # be deferred to the state block further down.
        self._neutral = NeutralOut()

        self.can_bus = resolve_can_bus(cfg.can_bus, cfg.drive_master_id, log)

        # ── drive motors ──
        self.drive = TalonFX(cfg.drive_master_id, self.can_bus)
        self.followers = [TalonFX(i, self.can_bus) for i in cfg.drive_follower_ids]
        dcfg = TalonFXConfiguration()
        # Invert is set EXPLICITLY from vehicle.yaml: configurator.apply()
        # with a full config object resets any Tuner-flashed invert to
        # factory default, so the yaml knob is the single source of truth.
        # Followers use Follower (not StrictFollower) and so track the
        # master's invert.
        dcfg.motor_output.inverted = (
            InvertedValue.CLOCKWISE_POSITIVE if cfg.drive_inverted
            else InvertedValue.COUNTER_CLOCKWISE_POSITIVE)
        dcfg.torque_current.peak_forward_torque_current = cfg.max_drive_a
        dcfg.torque_current.peak_reverse_torque_current = -cfg.max_drive_a
        # Stator current limit protects the motor in DutyCycleOut / VoltageOut
        # (non-FOC) modes, where the torque-current peaks above do not apply.
        # This is a base feature (no Pro license needed) and is what makes
        # the open-loop duty path and the bench test safe against a stall.
        dcfg.current_limits.stator_current_limit = cfg.max_drive_a
        dcfg.current_limits.stator_current_limit_enable = True
        # Runs on the Talon at 1 kHz, so a ragged 50 Hz command stream (or a
        # step from the bench test) can never slam the output. Watchdog
        # disable and NeutralOut are NOT ramped — safety cuts stay instant.
        dcfg.open_loop_ramps.duty_cycle_open_loop_ramp_period = float(
            cfg.duty_ramp_s)
        apply_device_config(self.drive, dcfg, "drive master", log)
        for f in self.followers:
            apply_device_config(f, dcfg, f"drive follower {f.device_id}", log)
            f.set_control(Follower(cfg.drive_master_id,
                                   motor_alignment=MotorAlignmentValue.ALIGNED))
        self._drive_req = TorqueCurrentFOC(0)     # FOC path (needs Pro license)
        self._drive_req_duty = DutyCycleOut(0)    # non-FOC path + bench test

        # ── steering ──
        self._steering = Steering(self.can_bus, cfg, self._neutral, log)

        # ── state ──
        self._lock = threading.Lock()
        self._cmd = (0.0, 0.0)          # (linear.x m/s, angular.z rad/s)
        self._cmd_time = 0.0            # monotonic of last cmd; 0 = never
        self._estop_latched = False
        self._estop_clear_logged = False
        self._shutdown_done = False

        # Refuse to drive on placeholder geometry: a missing/garbled
        # vehicle.yaml must not silently arm motors with guessed ratios.
        self.config_hold = not cfg.geometry_measured
        if self.config_hold:
            log.error(BANNER)
            log.error("*  CONFIG HOLD — geometry_measured is false (or")
            log.error("*  vehicle.yaml missing/unreadable). Motors will NOT")
            log.error("*  be commanded. Measure the car, fill vehicle.yaml,")
            log.error("*  set geometry_measured: true, restart this node.")
            log.error(BANNER)

        # ── ROS interfaces ──
        self.create_subscription(Twist, TOPIC_CMD_VEL, self._on_cmd, 10)
        # E-stop arrives both ways: health_monitor latches it TRANSIENT_LOCAL
        # (so a restarting drive node still sees it — a volatile-only sub
        # would re-arm overheated motors after a systemd restart), while a
        # manual `ros2 topic pub` is plain volatile. Subscribe to both.
        self.create_subscription(Bool, TOPIC_ESTOP, self._on_estop, 10)
        self.create_subscription(
            Bool, TOPIC_ESTOP, self._on_estop,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # Armed state. The planner publishes this LATCHED (TRANSIENT_LOCAL) on
        # every arm change and at boot; it is the single source of truth for
        # "autonomy is driving". It is NOT the same as /cmd_vel freshness — the
        # planner keeps publishing zero cmd_vel while DISARMED, so without this
        # the steering would hold centre against the driver's hands whenever
        # the node is up. Default False: assume the human is driving.
        self._armed = False
        self.create_subscription(
            Bool, TOPIC_ARMED, self._on_armed,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._status_pub = self.create_publisher(String, TOPIC_STATUS, 10)

        # Driver-adjustable regen (e-brake) strength, nudged live from wheel
        # encoder 3 via set_parameters. Scales the regen current only (drive
        # torque untouched). Floored at 0.2 in use: regen is the ONLY service
        # brake right now, so the dial must never reach zero braking.
        self.declare_parameter("regen_strength", 1.0)
        # Control mode (live-toggleable from the dashboard) + bench-test cap.
        self.declare_parameter("use_foc", bool(cfg.use_foc))
        self.declare_parameter("test_max_duty", float(cfg.test_max_duty))
        # Duty-mode velocity-law gains + steering slew, live-tuneable from the
        # dashboard param editor so smoothness can be dialled in on the bench
        # without a restart per attempt. vehicle.yaml sets the boot defaults;
        # persist a value you like by writing it back there.
        self.declare_parameter("duty_kv_ms", float(cfg.duty_kv_ms))
        self.declare_parameter("duty_kp", float(cfg.duty_kp))
        self.declare_parameter("duty_brake_kp", float(cfg.duty_brake_kp))
        self.declare_parameter("duty_brake_max", float(cfg.duty_brake_max))
        self.declare_parameter(
            "steer_slew_col_rps", float(cfg.steer_slew_col_rps))
        # Manual-drive pedal release behaviour (see pedal.py): "one_pedal"
        # (default, today's only behaviour) or "coast". Dashboard-editable,
        # same generic string-parameter path as every other live-tunable
        # knob here -- no dashboard code changes needed for this.
        self.declare_parameter("pedal_mode", DEFAULT_PEDAL_MODE)
        # Bench test: a direct open-loop duty-cycle override on /ethon/drive_test
        # (Float64, -1..1). Fresh + nonzero takes priority over /cmd_vel and
        # bypasses the torque map + steering. Always non-FOC (no license).
        self._test_duty = 0.0
        self._test_time = 0.0
        self.create_subscription(
            Float64, TOPIC_DRIVE_TEST, self._on_test, 10)
        # Bench test: direct target ROAD-WHEEL angle in degrees on
        # /ethon/steer_test_deg (Float64). Fresh takes priority over the
        # Ackermann-computed angle and bypasses the armed hand-back rule --
        # same bench-test intent as /ethon/drive_test, wheels OFF the ground.
        self._steer_test_deg = 0.0
        self._steer_test_time = 0.0
        self.create_subscription(
            Float64, TOPIC_STEER_TEST, self._on_steer_test, 10)

        # Manual-drive pedal. Non-fatal if absent -- the pedal is optional
        # hardware, not something the node depends on to come up.
        self._pedal = PedalLink(log)

        # Wheel brake button (see TOPIC_BRAKE above).
        self._brake_time = 0.0
        self.create_subscription(Bool, TOPIC_BRAKE, self._on_brake, 10)

        # Wheel reverse button (see TOPIC_REVERSE above).
        self._reverse_time = 0.0
        self.create_subscription(Bool, TOPIC_REVERSE, self._on_reverse, 10)
        # net battery energy + distance integrators for the efficiency readout
        self._energy_wh = 0.0
        self._dist_m = 0.0
        self._energy_last_t = None
        self.create_timer(1.0 / TICK_HZ, self._tick)
        self.create_timer(1.0 / STATUS_HZ, self._publish_status)

        log.info(
            f"ethon_drive up — bus '{self.can_bus}', steering "
            f"{'ENABLED' if self._steering.enabled else 'DISABLED (drive-only)'}, "
            f"{'CONFIG HOLD (no motion)' if self.config_hold else 'ready'}, "
            f"waiting for {TOPIC_CMD_VEL}")

    # ── callbacks ────────────────────────────────────────────────────────

    def _on_cmd(self, msg: Twist):
        with self._lock:
            self._cmd = (msg.linear.x, msg.angular.z)
            self._cmd_time = time.monotonic()

    def _on_armed(self, msg: Bool):
        """Track the planner's latched armed flag (steering hand-back gate)."""
        new = bool(msg.data)
        if new != self._armed:
            self._armed = new
            self.get_logger().warning(
                "ARMED — steering under autonomy control" if new else
                "DISARMED — steering released to the driver (motor neutral)")
            if not new:
                # Let go immediately rather than waiting for the next tick.
                self._steering.release()

    def _on_estop(self, msg: Bool):
        if msg.data:
            if not self._estop_latched:
                self._estop_latched = True
                self._neutral_all()
                self.get_logger().error(
                    "E-STOP RECEIVED — motors neutral, LATCHED OFF until "
                    "node restart")
        elif self._estop_latched and not self._estop_clear_logged:
            self._estop_clear_logged = True
            self.get_logger().warning(
                "e-stop cleared on topic, but the latch requires a node "
                "restart — staying disabled")

    def _on_test(self, msg: Float64):
        with self._lock:
            self._test_duty = float(msg.data)
            self._test_time = time.monotonic()

    def _on_steer_test(self, msg: Float64):
        with self._lock:
            self._steer_test_deg = float(msg.data)
            self._steer_test_time = time.monotonic()

    def _on_brake(self, msg: Bool):
        if bool(msg.data):
            with self._lock:
                self._brake_time = time.monotonic()

    def _on_reverse(self, msg: Bool):
        # Only True refreshes the watchdog; a False (or the watchdog simply
        # expiring) both mean "not held" -- either way it just goes stale.
        if bool(msg.data):
            with self._lock:
                self._reverse_time = time.monotonic()

    # ── 50 Hz control loop ───────────────────────────────────────────────

    def _tick(self):
        if self._estop_latched or self.config_hold:
            return  # no feed_enable -> devices stay disabled

        now = time.monotonic()
        cfg = self.cfg
        with self._lock:
            (v_cmd, omega), t_cmd = self._cmd, self._cmd_time
            test_duty, test_t = self._test_duty, self._test_time
            steer_test_deg, steer_test_t = \
                self._steer_test_deg, self._steer_test_time
            brake_t = self._brake_time
            reverse_t = self._reverse_time

        # ── bench-test override ──
        # A fresh, nonzero /ethon/drive_test command drives the wheels directly
        # with open-loop duty cycle, bypassing the torque map and leaving
        # steering untouched. Always non-FOC, so it works on unlicensed devices.
        # Same watchdog shape as /cmd_vel: if the dashboard stops re-posting,
        # this goes stale within TEST_TIMEOUT_S and the motors neutral out.
        if (now - test_t) <= TEST_TIMEOUT_S and abs(test_duty) > 1e-3:
            unmanaged.feed_enable(FEED_ENABLE_S)
            cap = abs(float(self.get_parameter("test_max_duty").value))
            self.drive.set_control(
                self._drive_req_duty.with_output(_clamp(test_duty, -cap, cap)))
            return

        # ── manual-drive pedal ──
        # Pumped every tick so a freshly-plugged-in pedal is picked up
        # immediately. Fresh + armed makes the pedal the SOLE authority over
        # speed this tick, in place of /cmd_vel -- it never publishes to that
        # topic, so it cannot fight cone_corridor_planner's publishes there.
        # Pedal steering hand-back lives in Steering.tick.
        self._pedal.pump()
        pedal_active = self._pedal.active(now)
        brake_active = (now - brake_t) <= BRAKE_TIMEOUT_S
        reverse_active = (now - reverse_t) <= REVERSE_TIMEOUT_S
        # This tick's drive output comes from the selected pedal-mode
        # strategy (pedal.py) rather than the shared v_cmd speed-tracking
        # law below -- only for a plain forward pedal command, never for
        # brake/reverse (those always brake/plug the same in both modes).
        via_pedal_mode = False

        # ── wheel brake button — HIGHEST priority, above even the pedal ──
        # Forces v_cmd to 0, i.e. exactly what one-pedal mode already does
        # on a released pedal: full regen decelerate. Same steering
        # hand-back as the pedal (see Steering.tick).
        if brake_active and self._armed:
            v_cmd, omega = 0.0, 0.0
        elif pedal_active and self._armed:
            if reverse_active:
                # Hold-to-reverse: the whole pedal travel maps onto the
                # reduced reverse span, so full pedal is reverse_speed_frac
                # of max speed (30% -> 2.4 m/s), not 8 m/s backwards. Same
                # in both pedal modes -- reverse braking-to-reverse is a
                # deliberate held-button action, not a lift-off surprise.
                v_cmd = (-self._pedal.frac * cfg.reverse_speed_frac
                         * cfg.max_speed_ms)
            else:
                via_pedal_mode = True
            omega = 0.0
        elif now - t_cmd > cfg.cmd_timeout_s:
            # Stale command: stop feeding enable — every Kraken neutrals on
            # its own watchdog within FEED_ENABLE_S. Nothing else to do.
            return

        unmanaged.feed_enable(FEED_ENABLE_S)

        limit = self._thermal_limit()
        wheel_ms = self._wheel_speed_ms()
        use_foc = bool(self.get_parameter("use_foc").value)
        live = SimpleNamespace(
            regen_k=_clamp(
                float(self.get_parameter("regen_strength").value), 0.2, 1.0),
            duty_kv_ms=float(self.get_parameter("duty_kv_ms").value),
            duty_kp=float(self.get_parameter("duty_kp").value),
            duty_brake_kp=float(self.get_parameter("duty_brake_kp").value),
            duty_brake_max=float(self.get_parameter("duty_brake_max").value),
        )

        if via_pedal_mode:
            mode = get_pedal_mode(str(self.get_parameter("pedal_mode").value))
            is_duty, value = mode.compute(
                self._pedal.frac, wheel_ms, limit, cfg, use_foc, live)
        else:
            is_duty, value = drive_output(
                v_cmd, wheel_ms, limit, use_foc, cfg, live)
        req = self._drive_req_duty if is_duty else self._drive_req
        self.drive.set_control(req.with_output(value))

        # ── steering: ackermann -> column angle -> motor rotations ──
        self._steering.tick(
            v_cmd=v_cmd, omega=omega, wheel_ms=wheel_ms, armed=self._armed,
            pedal_active=pedal_active, brake_active=brake_active,
            steer_test_active=(now - steer_test_t) <= TEST_TIMEOUT_S,
            steer_test_deg=steer_test_deg, use_foc=use_foc,
            slew_col_rps=float(self.get_parameter("steer_slew_col_rps").value),
            tick_hz=TICK_HZ)

    def _thermal_limit(self) -> float:
        """Linear current derate from the hottest drive motor temperature."""
        cfg = self.cfg
        t = max(m.get_device_temp().value
                for m in [self.drive] + self.followers)
        if t <= cfg.derate_lo_c:
            return cfg.max_drive_a
        if t >= cfg.derate_hi_c:
            return cfg.min_drive_a
        frac = (t - cfg.derate_lo_c) / (cfg.derate_hi_c - cfg.derate_lo_c)
        return cfg.max_drive_a - frac * (cfg.max_drive_a - cfg.min_drive_a)

    def _wheel_speed_ms(self) -> float:
        rps = self.drive.get_velocity().value / self.cfg.gear_ratio
        return rps * math.pi * self.cfg.wheel_dia_m

    # ── 2 Hz status publisher ────────────────────────────────────────────

    def _motor_table(self):
        rows = [(f"drive_{self.drive.device_id}", self.drive)]
        rows += [(f"drive_{f.device_id}", f) for f in self.followers]
        rows.append(("steer", self._steering.steer))
        return rows

    def _publish_status(self):
        cfg = self.cfg
        with self._lock:
            t_cmd = self._cmd_time
        cmd_age = (time.monotonic() - t_cmd) if t_cmd > 0.0 else None
        fresh = cmd_age is not None and cmd_age <= cfg.cmd_timeout_s

        if not self._steering.enabled:
            steering = "disabled"
        elif self._steering.homed:
            steering = "homed"
        else:
            steering = "unhomed-allowed"

        col_rot = signal_value(self._steering.steer, "get_position")
        if col_rot is not None:
            col_rot /= cfg.steer_belt_ratio

        motors = {}
        for label, m in self._motor_table():
            motors[label] = {
                "temp_c": _round(signal_value(m, "get_device_temp"), 1),
                "torque_a": _round(signal_value(m, "get_torque_current"), 1),
                "supply_a": _round(signal_value(m, "get_supply_current"), 1),
                "vel_rps": _round(signal_value(m, "get_velocity"), 1),
                "faults": read_faults(m),
            }

        # ── net energy + efficiency integrators ──
        # Battery power = pack voltage x total supply current, summed across
        # every motor. Supply current goes NEGATIVE under regen, so this nets
        # out recovered charge — i.e. true energy drawn from the pack. Integrated
        # at STATUS_HZ; absurd dt gaps (scheduling stalls) are skipped so a
        # hiccup can't inject a huge spurious Wh.
        # Pack voltage is the same on every motor's supply rail, so read it
        # from whichever motor actually responds rather than hardcoding the
        # master — during bench testing only one Kraken may be wired up, and
        # it's often not the master.
        supply_v = None
        for _, m in self._motor_table():
            supply_v = signal_value(m, "get_supply_voltage")
            if supply_v is not None:
                break
        supply_a = [m["supply_a"] for m in motors.values()
                    if m["supply_a"] is not None]
        wheel_ms = self._wheel_speed_ms()
        now = time.monotonic()
        if self._energy_last_t is not None:
            dt = now - self._energy_last_t
            if 0.0 < dt <= 4.0 / STATUS_HZ:
                if supply_v is not None and supply_a:
                    self._energy_wh += supply_v * sum(supply_a) * dt / 3600.0
                self._dist_m += abs(wheel_ms) * dt
        self._energy_last_t = now
        wh_per_km = (self._energy_wh / (self._dist_m / 1000.0)
                     if self._dist_m >= EFFIC_MIN_DIST_M else None)

        lock_half_rotor = self._steering.lock_half_rotor
        status = {
            "t": round(time.time(), 2),
            "can_bus": self.can_bus,
            "enabled": fresh and not self._estop_latched
                       and not self.config_hold,
            "estop_latched": self._estop_latched,
            "config_hold": self.config_hold,
            "cmd_age_s": _round(cmd_age),
            "steering": steering,
            "wheel_speed_ms": _round(wheel_ms),
            "thermal_limit_a": _round(self._thermal_limit(), 1),
            "regen_strength": _round(_clamp(float(
                self.get_parameter("regen_strength").value), 0.2, 1.0), 2),
            "use_foc": bool(self.get_parameter("use_foc").value),
            "pedal_mode": str(self.get_parameter("pedal_mode").value),
            # Published because this pair is a HARD GATE on autonomy and was
            # previously invisible: while pedal_active is true, steering.py
            # neutrals the steer motor AND the pedal overrides the planner's
            # speed, so a silently-streaming pedal disables autonomy completely
            # with nothing on any dashboard to explain why.
            "pedal_active": bool(self._pedal.active(time.monotonic())),
            "pedal_frac": _round(self._pedal.frac, 3),
            "test_active": ((time.monotonic() - self._test_time) <= TEST_TIMEOUT_S
                            and abs(self._test_duty) > 1e-3),
            "test_duty": _round(self._test_duty, 2),
            "steer_test_active": (
                (time.monotonic() - self._steer_test_time) <= TEST_TIMEOUT_S),
            "steer_test_deg": _round(self._steer_test_deg, 1),
            "steer_col_rot": _round(col_rot, 3),
            "road_wheel_deg": _round(
                col_rot * 360.0 / cfg.steer_col_ratio, 1)
                if col_rot is not None else None,
            # Usable travel each side of centre, in COLUMN rotations. After
            # lock_to_lock homing this is the measured half-range (stops minus
            # margin); otherwise it is the configured soft limit. Lets the
            # dashboard show travel against the real lock instead of guessing.
            "steer_limit_col_rot": _round(
                (lock_half_rotor / cfg.steer_belt_ratio)
                if (lock_half_rotor is not None and cfg.steer_belt_ratio)
                else cfg.steer_limit_rot, 3),
            "steer_limit_deg": _round(
                ((lock_half_rotor / cfg.steer_belt_ratio)
                 if (lock_half_rotor is not None and cfg.steer_belt_ratio)
                 else cfg.steer_limit_rot) * 360.0 / cfg.steer_col_ratio, 1)
                if cfg.steer_col_ratio else None,
            "steer_homed": bool(self._steering.homed),
            "steer_mode": ("foc" if bool(self.get_parameter("use_foc").value)
                           else "duty"),
            # Direction convention, published so the dashboard can render the
            # wheel the way it physically turns instead of assuming a sign.
            "steer_inverted": bool(cfg.steer_inverted),
            "armed": bool(self._armed),
            # Geometry for the dashboard's predicted-trajectory overlay: the
            # bicycle model needs the wheelbase, and the steering envelope
            # needs the road-wheel angle at full lock.
            "wheelbase_m": _round(cfg.wheelbase_m, 3),
            "road_wheel_max_deg": _round(
                (lock_half_rotor / cfg.steer_belt_ratio
                 if (lock_half_rotor is not None and cfg.steer_belt_ratio)
                 else cfg.steer_limit_rot)
                * 360.0 / cfg.steer_col_ratio, 1)
                if cfg.steer_col_ratio else None,
            "supply_v": _round(supply_v, 1),
            "energy_wh": _round(self._energy_wh, 1),
            "wh_per_km": _round(wh_per_km, 0),
            "dist_m": _round(self._dist_m, 1),
            "motors": motors,
        }
        self._status_pub.publish(
            String(data=json.dumps(status, separators=(",", ":"))))

    # ── shutdown ─────────────────────────────────────────────────────────

    def _neutral_all(self):
        for label, m in self._motor_table():
            try:
                m.set_control(self._neutral)
            except Exception as exc:
                self.get_logger().error(f"failed to neutral {label}: {exc}")

    def shutdown(self):
        """Neutral every motor exactly once before process exit."""
        if self._shutdown_done:
            return
        self._shutdown_done = True
        self._estop_latched = True      # stop _tick from re-commanding
        self._neutral_all()
        self.get_logger().info("ethon_drive shutdown — all motors neutral")


def main():
    rclpy.init()
    node = EthonDrive()

    def _on_term(signum, _frame):
        raise SystemExit(signum)

    # SIGINT is handled by rclpy (KeyboardInterrupt / ExternalShutdown);
    # SIGTERM (systemd stop) must also reach the neutral-out path below.
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
            pass  # context may already be down (external shutdown)


if __name__ == "__main__":
    main()
