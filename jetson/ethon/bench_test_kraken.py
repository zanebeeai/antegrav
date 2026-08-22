#!/usr/bin/env python3
"""
Kraken X60 first-spin bench test — Jetson + CANivore, Phoenix 6 Python.

Interactive, step-by-step guided test for the FIRST power-on of a Kraken
on the bench.  The motor must be FREE SPINNING — NOT attached to the
vehicle, NOT coupled to the drivetrain or steering belt.  All currents
are capped low (5 A) for safety.

Steps (Enter to run each, 's' to skip, 'q' to abort at any point):
  1. Enumerate bus      — probe CAN IDs 0-10 on "canivore" then "can0"
  2. License check      — Phoenix Pro state per device (FOC requires it)
  3. Select + configure — pick device, coast neutral, 5 A limits
  4. Spin test          — DutyCycleOut(0.05) for 2 s, confirm direction
  5. FOC test           — TorqueCurrentFOC(3 A) for 2 s (licensed only)
  6. Watchdog kill test — stop feeding enable, time until auto-disable
  7. Steering pos test  — PositionTorqueCurrentFOC +-0.25 rot @ 5 A cap

Safety model (same as ethon_drive.py):
  - Phoenix 6 non-FRC devices disable themselves unless
    unmanaged.feed_enable() is called continuously.  This script feeds
    the enable ONLY inside an active motion step; everything else runs
    with the watchdog starved, so a crash anywhere fails SILENT.
  - Step 6 deliberately starves the watchdog while the motor is moving
    to prove the auto-disable actually works before the motor ever
    touches the vehicle.

NOTE: step 3 applies a full TalonFXConfiguration (factory defaults +
test limits).  Re-apply the vehicle configuration afterwards — do not
drive on bench-test config.

Usage:  python3 bench_test_kraken.py
"""

import sys
import time
import traceback

try:
    from phoenix6 import CANBus, unmanaged
    from phoenix6.hardware import TalonFX
    from phoenix6.controls import (
        DutyCycleOut,
        NeutralOut,
        PositionTorqueCurrentFOC,
        TorqueCurrentFOC,
    )
    from phoenix6.configs import TalonFXConfiguration
    from phoenix6.signals import NeutralModeValue
except ImportError as exc:
    print(f"FATAL: phoenix6 import failed: {exc}", flush=True)
    print("Install with:  pip3 install phoenix6", flush=True)
    sys.exit(1)

# ── bus / probe constants ────────────────────────────────────────────
CANIVORE_BUS    = "canivore"           # CANivore USB bus name
FALLBACK_BUS    = "can0"               # socketcan fallback
PROBE_IDS       = range(0, 11)         # CAN IDs scanned in step 1
PROBE_TIMEOUT_S = 0.25                 # per-signal probe timeout

# ── test parameters (bench-safe: motor free spinning, low current) ──
TEST_CURRENT_A    = 5.0                # stator/supply/torque cap, steps 3-7
SPIN_DUTY         = 0.05               # step 4 duty cycle
SPIN_DURATION_S   = 2.0                # steps 4 & 5 run time
FOC_TEST_A        = 3.0                # step 5 torque current
FEED_ENABLE_S     = 0.1                # enable lease per feed (100 ms)
FEED_PERIOD_S     = 0.05               # feed/control loop period (50 ms)
TELEMETRY_S       = 0.25               # telemetry print period in motion loops

# ── watchdog kill test (step 6) ──────────────────────────────────────
WATCHDOG_SPINUP_S  = 1.0               # spin time before cutting the feed
WATCHDOG_OBSERVE_S = 3.0               # max time to watch for disable/stop
WATCHDOG_MAX_S     = 1.0               # must disable well under this
VEL_STOPPED_RPS    = 0.10              # |velocity| below this counts as stopped

# ── steering test (step 7) ──────────────────────────────────────────
STEER_CAN_ID       = 3                 # steering Kraken (belt to column)
STEER_TEST_ROT     = 0.25              # rotor rotations to move and back
STEER_MOVE_S       = 1.5               # settle time per move
STEER_POS_TOL_ROT  = 0.05              # final return-to-zero tolerance

PASS, FAIL, WARN, SKIP = "PASS", "FAIL", "WARN", "SKIP"


def say(msg: str = "") -> None:
    """Print immediately (script may run over ssh — never buffer)."""
    print(msg, flush=True)


def ask(prompt: str) -> str:
    """Prompt the user; EOF (non-interactive pipe closed) aborts."""
    try:
        return input(prompt).strip().lower()
    except EOFError:
        say("\nstdin closed — aborting.")
        return "q"


def read_signal(sig, timeout: float = PROBE_TIMEOUT_S):
    """Refresh a StatusSignal with a timeout.

    Returns (ok, value).  Never raises — prints the raw exception so
    API mismatches between phoenix6 versions are visible, not fatal.
    """
    try:
        sig.wait_for_update(timeout)
        return sig.status.is_ok(), sig.value
    except Exception as exc:                       # noqa: BLE001 — bench tool
        say(f"      signal exception: {type(exc).__name__}: {exc}")
        return False, None


class BenchTest:
    """State shared across steps: bus, discovered devices, results."""

    def __init__(self):
        self.bus_name = None               # bus that answered the probe
        self.found = {}                    # id -> {"fw": str, "temp": float|None, "pro": bool|None}
        self.talon = None                  # device under test (step 3+)
        self.talon_id = None
        self.talon_pro = None              # tri-state: True/False/None(unknown)
        self.steer_talon = None            # step 7 device (may alias self.talon)
        self.results = []                  # (step name, status, note)

    # ── shared helpers ───────────────────────────────────────────────

    def neutral_all(self) -> None:
        """Best-effort NeutralOut on every device we ever commanded."""
        for talon in (self.talon, self.steer_talon):
            if talon is None:
                continue
            try:
                talon.set_control(NeutralOut())
            except Exception as exc:               # noqa: BLE001
                say(f"  neutral_all exception: {type(exc).__name__}: {exc}")

    def hold_control(self, talon: TalonFX, request, duration_s: float):
        """Apply `request` for duration_s, feeding the enable watchdog.

        Prints velocity/current telemetry every TELEMETRY_S.  Always
        leaves the motor in NeutralOut (and then stops feeding, so the
        device disables itself within FEED_ENABLE_S regardless).

        Returns (peak_abs_vel_rps, last_vel_rps, peak_abs_stator_a).
        """
        vel = talon.get_velocity()
        cur = talon.get_stator_current()
        peak_v = last_v = peak_a = 0.0
        ctrl_err_shown = False
        t0 = time.monotonic()
        next_print = t0
        try:
            while time.monotonic() - t0 < duration_s:
                unmanaged.feed_enable(FEED_ENABLE_S)
                status = talon.set_control(request)
                if not status.is_ok() and not ctrl_err_shown:
                    say(f"    set_control status: {status.name}")
                    ctrl_err_shown = True
                vel.refresh()
                cur.refresh()
                last_v = vel.value
                peak_v = max(peak_v, abs(last_v))
                peak_a = max(peak_a, abs(cur.value))
                now = time.monotonic()
                if now >= next_print:
                    say(f"    t={now - t0:4.2f}s  vel={last_v:+8.2f} rps"
                        f"  stator={cur.value:+6.2f} A")
                    next_print = now + TELEMETRY_S
                time.sleep(FEED_PERIOD_S)
        finally:
            talon.set_control(NeutralOut())
        return peak_v, last_v, peak_a

    # ── step 1: enumerate ────────────────────────────────────────────

    def step_enumerate(self):
        for bus_name in (CANIVORE_BUS, FALLBACK_BUS):
            say(f"\n  Probing bus '{bus_name}' (IDs"
                f" {PROBE_IDS.start}-{PROBE_IDS.stop - 1},"
                f" {PROBE_TIMEOUT_S}s/id)...")
            self._print_bus_status(bus_name)
            found = self._probe_bus(bus_name)
            if found:
                self.bus_name = bus_name
                self.found = found
                break
            say(f"  no TalonFX devices answered on '{bus_name}'")

        if not self.found:
            return FAIL, "no devices on canivore or can0 — check power/USB/termination"

        say(f"\n  Devices on '{self.bus_name}':")
        say(f"  {'ID':>3}  {'firmware':<12} {'temp':>7}  pro-licensed")
        for dev_id, info in sorted(self.found.items()):
            temp = f"{info['temp']:.1f}C" if info["temp"] is not None else "?"
            pro = {True: "YES", False: "NO", None: "unknown"}[info["pro"]]
            say(f"  {dev_id:>3}  {info['fw']:<12} {temp:>7}  {pro}")
        say("  (TalonFX-only scan — the CANcoder on ID 4, if fitted, will not appear)")
        return PASS, f"{len(self.found)} device(s) on '{self.bus_name}'"

    def _print_bus_status(self, bus_name: str) -> None:
        """Bus-level health, best effort (API differs across phoenix6 versions)."""
        try:
            status = CANBus(bus_name).status
            say(f"    bus status={status.status.name}"
                f"  util={status.bus_utilization * 100.0:.1f}%"
                f"  bus_off_count={status.bus_off_count}")
        except Exception as exc:                   # noqa: BLE001
            say(f"    bus status unavailable: {type(exc).__name__}: {exc}")

    def _probe_bus(self, bus_name: str) -> dict:
        found = {}
        for dev_id in PROBE_IDS:
            try:
                talon = TalonFX(dev_id, bus_name)
                ok, major = read_signal(talon.get_version_major())
                if not ok:
                    continue
                _, minor = read_signal(talon.get_version_minor())
                _, bugfix = read_signal(talon.get_version_bugfix())
                _, build = read_signal(talon.get_version_build())
                fw = f"{major}.{minor}.{bugfix}.{build}"
                temp_ok, temp = read_signal(talon.get_device_temp())
                pro = self._read_pro_licensed(talon)
                found[dev_id] = {
                    "fw": fw,
                    "temp": temp if temp_ok else None,
                    "pro": pro,
                }
                say(f"    id {dev_id}: TalonFX fw {fw}")
            except Exception as exc:               # noqa: BLE001
                say(f"    id {dev_id}: probe exception:"
                    f" {type(exc).__name__}: {exc}")
        return found

    @staticmethod
    def _read_pro_licensed(talon: TalonFX):
        """Pro-license state, or None if the signal is unavailable."""
        try:
            ok, value = read_signal(talon.get_is_pro_licensed())
            return bool(value) if ok else None
        except Exception as exc:                   # noqa: BLE001
            say(f"      is_pro_licensed unavailable: {type(exc).__name__}: {exc}")
            return None

    # ── step 2: license check ────────────────────────────────────────

    def step_license(self):
        if not self.found:
            return SKIP, "no devices enumerated (run step 1)"

        unlicensed = [i for i, d in self.found.items() if d["pro"] is False]
        unknown = [i for i, d in self.found.items() if d["pro"] is None]

        for dev_id, info in sorted(self.found.items()):
            pro = {True: "Pro LICENSED", False: "NOT licensed", None: "UNKNOWN"}[info["pro"]]
            say(f"  id {dev_id}: {pro}")

        if unlicensed:
            say("")
            say("  " + "!" * 62)
            say(f"  !! IDs {unlicensed} are NOT Pro licensed.")
            say("  !! TorqueCurrentFOC / PositionTorqueCurrentFOC WILL FAIL on")
            say("  !! them (UnlicensedDevice fault, motor will not move).")
            say("  !! ethon_drive.py is FOC-only — fix licensing before vehicle use.")
            say("  " + "!" * 62)
            return FAIL, f"unlicensed: {unlicensed}"
        if unknown:
            return WARN, f"license state unknown for {unknown} — FOC may fail"
        return PASS, "all devices Pro licensed"

    # ── step 3: select + configure ───────────────────────────────────

    def step_select_configure(self):
        if not self.found:
            return SKIP, "no devices enumerated (run step 1)"

        default_id = min(self.found)
        raw = ask(f"  Device ID to test [{default_id}]: ")
        if raw == "q":
            return SKIP, "aborted at device selection"
        try:
            dev_id = int(raw) if raw else default_id
        except ValueError:
            return FAIL, f"'{raw}' is not a CAN ID"
        if dev_id not in self.found:
            if ask(f"  id {dev_id} did not answer the probe — use anyway? [y/N]: ") != "y":
                return SKIP, f"id {dev_id} not on bus, declined"

        try:
            talon = TalonFX(dev_id, self.bus_name)
            status = talon.configurator.apply(self._test_config())
        except Exception as exc:                   # noqa: BLE001
            say(f"  config exception: {type(exc).__name__}: {exc}")
            return FAIL, f"id {dev_id}: config apply raised"
        if not status.is_ok():
            return FAIL, f"id {dev_id}: config apply -> {status.name}"

        self.talon = talon
        self.talon_id = dev_id
        info = self.found.get(dev_id)
        self.talon_pro = info["pro"] if info else None
        say(f"  id {dev_id} configured: coast neutral,"
            f" {TEST_CURRENT_A:.0f} A stator/supply/torque limits")
        say("  (full config overwrite — re-apply vehicle config before driving)")
        return PASS, f"id {dev_id} ready, {TEST_CURRENT_A:.0f} A caps"

    @staticmethod
    def _test_config() -> TalonFXConfiguration:
        cfg = TalonFXConfiguration()
        cfg.motor_output.neutral_mode = NeutralModeValue.COAST
        cfg.current_limits.stator_current_limit = TEST_CURRENT_A
        cfg.current_limits.stator_current_limit_enable = True
        cfg.current_limits.supply_current_limit = TEST_CURRENT_A
        cfg.current_limits.supply_current_limit_enable = True
        cfg.torque_current.peak_forward_torque_current = TEST_CURRENT_A
        cfg.torque_current.peak_reverse_torque_current = -TEST_CURRENT_A
        return cfg

    # ── step 4: duty-cycle spin test ─────────────────────────────────

    def step_spin(self):
        if self.talon is None:
            return SKIP, "no device selected (run step 3)"

        say(f"  Motor id {self.talon_id} will spin at {SPIN_DUTY * 100:.0f}%"
            f" duty for {SPIN_DURATION_S:.0f} s.")
        say("  CONFIRM the rotor is completely free — nothing attached.")
        if ask("  Spin now? [y/N]: ") != "y":
            return SKIP, "declined by user"

        peak_v, _, peak_a = self.hold_control(
            self.talon, DutyCycleOut(SPIN_DUTY), SPIN_DURATION_S)
        say(f"  peak |velocity| {peak_v:.2f} rps, peak |stator| {peak_a:.2f} A")

        if peak_v < VEL_STOPPED_RPS:
            return FAIL, "motor did not move — wiring/enable/limits?"
        say("  CTRE convention: positive output = counter-clockwise viewed"
            " from the rotor (shaft) face.")
        if ask("  Did the motor spin smoothly in that direction? [y/N]: ") == "y":
            return PASS, f"spun, peak {peak_v:.1f} rps @ {peak_a:.1f} A"
        return FAIL, "user reports wrong direction or rough spin"

    # ── step 5: FOC torque test ──────────────────────────────────────

    def step_foc(self):
        if self.talon is None:
            return SKIP, "no device selected (run step 3)"
        if self.talon_pro is False:
            return SKIP, "device not Pro licensed — FOC unavailable"
        if self.talon_pro is None:
            if ask("  License state unknown — attempt FOC anyway? [y/N]: ") != "y":
                return SKIP, "license unknown, declined"

        say(f"  TorqueCurrentFOC({FOC_TEST_A:.0f} A) for {SPIN_DURATION_S:.0f} s —"
            " a free rotor will spin up FAST. Keep clear.")
        if ask("  Run FOC test? [y/N]: ") != "y":
            return SKIP, "declined by user"

        peak_v, _, peak_a = self.hold_control(
            self.talon, TorqueCurrentFOC(FOC_TEST_A), SPIN_DURATION_S)
        say(f"  peak |velocity| {peak_v:.2f} rps, peak |stator| {peak_a:.2f} A")

        if self._unlicensed_fault_active():
            return FAIL, "UnlicensedDevice fault — FOC refused, check licensing"
        if peak_v < VEL_STOPPED_RPS:
            return FAIL, "no motion under FOC command"
        return PASS, f"FOC ok, peak {peak_v:.1f} rps @ {peak_a:.1f} A"

    def _unlicensed_fault_active(self) -> bool:
        try:
            ok, faulted = read_signal(
                self.talon.get_fault_unlicensed_feature_in_use())
            return bool(faulted) if ok else False
        except Exception as exc:                   # noqa: BLE001
            say(f"  unlicensed-fault signal unavailable:"
                f" {type(exc).__name__}: {exc}")
            return False

    # ── step 6: watchdog kill test ───────────────────────────────────

    def step_watchdog(self):
        if self.talon is None:
            return SKIP, "no device selected (run step 3)"

        say("  THE critical test: motor is started, then the enable feed is")
        say("  deliberately cut while the duty-cycle command stays active.")
        say("  The Kraken must disable ITSELF — this is what protects the")
        say("  driver if this software ever hangs on the vehicle.")
        if ask("  Run watchdog kill test? [y/N]: ") != "y":
            return SKIP, "declined by user"

        vel = self.talon.get_velocity()
        enable_sig = self._device_enable_signal()
        request = DutyCycleOut(SPIN_DUTY)

        try:
            # spin-up phase: feed normally
            t0 = time.monotonic()
            while time.monotonic() - t0 < WATCHDOG_SPINUP_S:
                unmanaged.feed_enable(FEED_ENABLE_S)
                self.talon.set_control(request)
                time.sleep(FEED_PERIOD_S)
            vel.refresh()
            say(f"  spinning at {vel.value:+.2f} rps —"
                " CUTTING enable feed NOW (command stays active)")

            # starvation phase: no feed_enable, no set_control
            t_cut = time.monotonic()
            t_disabled = None
            t_stopped = None
            while time.monotonic() - t_cut < WATCHDOG_OBSERVE_S:
                if enable_sig is not None and t_disabled is None:
                    if self._reports_disabled(enable_sig):
                        t_disabled = time.monotonic() - t_cut
                        say(f"  device reports DISABLED at t={t_disabled:.3f} s")
                vel.refresh()
                if abs(vel.value) < VEL_STOPPED_RPS:
                    t_stopped = time.monotonic() - t_cut
                    say(f"  velocity reached zero at t={t_stopped:.3f} s")
                    break
                time.sleep(0.005)
        finally:
            self.talon.set_control(NeutralOut())

        say(f"  last feed lease was {FEED_ENABLE_S * 1000:.0f} ms,"
            f" so ~{FEED_ENABLE_S:.1f} s residual enable is expected.")
        if t_disabled is not None:
            note = f"auto-disable in {t_disabled:.3f} s"
            if t_stopped is not None:
                note += f", stopped in {t_stopped:.3f} s"
            else:
                note += ", still coasting (coast neutral — expected)"
            return (PASS, note) if t_disabled <= WATCHDOG_MAX_S else (FAIL, note)
        if t_stopped is not None:
            note = f"stopped in {t_stopped:.3f} s (enable signal unavailable)"
            return (PASS, note) if t_stopped <= WATCHDOG_MAX_S else (FAIL, note)
        return FAIL, (f"motor never disabled/stopped within"
                      f" {WATCHDOG_OBSERVE_S:.0f} s — DO NOT put on vehicle")

    def _device_enable_signal(self):
        """DeviceEnable StatusSignal, or None if this phoenix6 lacks it."""
        try:
            sig = self.talon.get_device_enable()
            sig.wait_for_update(PROBE_TIMEOUT_S)
            return sig
        except Exception as exc:                   # noqa: BLE001
            say(f"  device_enable signal unavailable"
                f" ({type(exc).__name__}: {exc}) — falling back to velocity")
            return None

    @staticmethod
    def _reports_disabled(enable_sig) -> bool:
        try:
            enable_sig.refresh()
            return "DISABLE" in str(enable_sig.value).upper()
        except Exception:                          # noqa: BLE001
            return False

    # ── step 7: steering position test ───────────────────────────────

    def step_steering(self):
        if self.bus_name is None:
            return SKIP, "no bus enumerated (run step 1)"

        say(f"  Optional: closed-loop position test for the steering Kraken"
            f" (default id {STEER_CAN_ID}).")
        raw = ask(f"  Steering device ID [{STEER_CAN_ID}], or 's' to skip: ")
        if raw in ("s", "q"):
            return SKIP, "declined by user"
        try:
            dev_id = int(raw) if raw else STEER_CAN_ID
        except ValueError:
            return FAIL, f"'{raw}' is not a CAN ID"
        if dev_id not in self.found:
            if ask(f"  id {dev_id} did not answer the probe — use anyway? [y/N]: ") != "y":
                return SKIP, f"id {dev_id} not on bus, declined"
        info = self.found.get(dev_id)
        if info and info["pro"] is False:
            return SKIP, f"id {dev_id} not Pro licensed — PositionTorqueCurrentFOC unavailable"

        say(f"  Moves +{STEER_TEST_ROT} rotor rotations and back at"
            f" {TEST_CURRENT_A:.0f} A cap. Current rotor position becomes zero.")
        if ask("  Run steering position test? [y/N]: ") != "y":
            return SKIP, "declined by user"

        try:
            if dev_id == self.talon_id and self.talon is not None:
                talon = self.talon
            else:
                talon = TalonFX(dev_id, self.bus_name)
                status = talon.configurator.apply(self._test_config())
                if not status.is_ok():
                    return FAIL, f"id {dev_id}: config apply -> {status.name}"
            self.steer_talon = talon
            talon.set_position(0)
        except Exception as exc:                   # noqa: BLE001
            say(f"  setup exception: {type(exc).__name__}: {exc}")
            return FAIL, f"id {dev_id}: steering setup raised"

        pos = talon.get_position()
        for target, label in ((STEER_TEST_ROT, "out"), (0.0, "return")):
            say(f"  -> {label}: target {target:+.2f} rot")
            self.hold_control(
                talon, PositionTorqueCurrentFOC(target), STEER_MOVE_S)
            pos.refresh()
            say(f"     position {pos.value:+.3f} rot"
                f" (err {pos.value - target:+.3f})")

        final_err = abs(pos.value)
        if final_err <= STEER_POS_TOL_ROT:
            return PASS, f"returned to zero, err {final_err:.3f} rot"
        return FAIL, f"final error {final_err:.3f} rot > {STEER_POS_TOL_ROT} tol"

    # ── runner ───────────────────────────────────────────────────────

    def run(self) -> int:
        say("=" * 70)
        say("  KRAKEN X60 BENCH TEST — first spin-up")
        say("  Motor must be FREE SPINNING, off the vehicle. Currents capped"
            f" at {TEST_CURRENT_A:.0f} A.")
        say("=" * 70)

        steps = (
            ("1. Enumerate bus & devices", self.step_enumerate),
            ("2. Pro license check",       self.step_license),
            ("3. Select + configure",      self.step_select_configure),
            ("4. Duty-cycle spin test",    self.step_spin),
            ("5. FOC torque test",         self.step_foc),
            ("6. Watchdog kill test",      self.step_watchdog),
            ("7. Steering position test",  self.step_steering),
        )

        aborted = False
        try:
            for name, fn in steps:
                if aborted:
                    self.results.append((name, SKIP, "aborted"))
                    continue
                choice = ask(f"\n=== {name} ===  [Enter]=run  s=skip  q=abort > ")
                if choice == "q":
                    self.results.append((name, SKIP, "aborted"))
                    aborted = True
                    continue
                if choice == "s":
                    self.results.append((name, SKIP, "skipped by user"))
                    continue
                try:
                    status, note = fn()
                except Exception:                  # noqa: BLE001 — show raw
                    say("  UNHANDLED EXCEPTION in step:")
                    traceback.print_exc()
                    sys.stderr.flush()
                    status, note = FAIL, "unhandled exception (see traceback)"
                finally:
                    self.neutral_all()
                self.results.append((name, status, note))
                say(f"  -> {status}: {note}")
        except KeyboardInterrupt:
            say("\nCtrl+C — stopping motors and exiting.")
            aborted = True
        finally:
            self.neutral_all()

        return self._summary(aborted)

    def _summary(self, aborted: bool) -> int:
        say("\n" + "=" * 70)
        say("  SUMMARY" + ("  (aborted)" if aborted else ""))
        say("=" * 70)
        say(f"  {'step':<30} {'result':<7} note")
        say(f"  {'-' * 30} {'-' * 7} {'-' * 28}")
        for name, status, note in self.results:
            say(f"  {name:<30} {status:<7} {note}")
        failed = [n for n, s, _ in self.results if s == FAIL]
        say("=" * 70)
        if failed:
            say(f"  RESULT: FAIL — {len(failed)} step(s) failed."
                "  Do NOT install on vehicle.")
            return 1
        if aborted:
            say("  RESULT: INCOMPLETE — aborted before all steps ran.")
            return 1
        say("  RESULT: PASS — re-apply vehicle config before installing.")
        return 0


def main() -> None:
    sys.exit(BenchTest().run())


if __name__ == "__main__":
    main()
