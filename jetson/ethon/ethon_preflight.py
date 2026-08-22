#!/usr/bin/env python3
"""Pre-flight checks + live monitor for the cone-corridor autonomy test.

Everything is read through the dashboard HTTP API, so this needs no ROS
environment and no sourcing -- it runs from any shell on the Jetson.

  python3 ethon_preflight.py check     # one-shot readiness report, PASS/FAIL
  python3 ethon_preflight.py steer     # Phase 0: steering sweep, cmd vs actual
  python3 ethon_preflight.py monitor   # live view (Phases 1-3)

WHY THIS EXISTS
The autonomy pipeline is already complete; what was missing was a way to see
whether it is behaving BEFORE trusting it with a moving car. `check` catches
the config/health traps, `steer` proves the steering chain's sign and scale
(the one error that turns corridor-following into corridor-leaving), and
`monitor` shows cone geometry, the four motion gates, and midline deviation
in one place during the run.

Read-only except `steer`, which posts to /ethon/steer_test_deg (a bench-test
topic that bypasses arming, is duty-capped, and self-releases in 0.5 s when
this stops posting).
"""
import json
import math
import subprocess
import sys
import time
import urllib.request

BASE = "http://127.0.0.1"
STEER_REPOST_S = 0.2          # < the drive node's 0.5 s test watchdog
OK, WARN, BAD = "PASS", "WARN", "FAIL"


# ── plumbing ──────────────────────────────────────────────────────────────

def api_get(path, timeout=6):
    with urllib.request.urlopen(BASE + path, timeout=timeout) as r:
        return json.loads(r.read().decode())


def api_post(path, payload, timeout=6):
    req = urllib.request.Request(
        BASE + path, data=json.dumps(payload).encode(),
        headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode())


def topics():
    d = api_get("/api/state")
    return d.get("topics", d)


def topic(t, name):
    """(value, age_s) for a topic, or (None, None) if never seen."""
    e = t.get(name)
    if not e:
        return None, None
    return e.get("value"), e.get("age_s")


def svc(name):
    r = subprocess.run(["systemctl", "is-active", name],
                       capture_output=True, text=True)
    return r.stdout.strip()


def line(status, label, detail=""):
    mark = {OK: "  ok  ", WARN: " warn ", BAD: " FAIL "}[status]
    print("[%s] %-30s %s" % (mark, label, detail))
    return status


# ── check ─────────────────────────────────────────────────────────────────

def cmd_check():
    print("=" * 72)
    print("ETHON PRE-FLIGHT  --  cone-corridor autonomy readiness")
    print("=" * 72)
    res = []

    # services / mode
    for s in ("ethon-stack", "ethon-drive", "ethon-dashboard"):
        st = svc(s)
        res.append(line(OK if st == "active" else BAD, s, st))
    cap = svc("ethon-capture")
    res.append(line(OK if cap != "active" else BAD, "mode is AUTONOMY",
                    "ethon-capture=%s%s" % (cap, "" if cap != "active"
                                            else "  <- run ethon_set_mode.sh autonomy")))

    try:
        t = topics()
    except Exception as exc:
        line(BAD, "dashboard /api/state", str(exc))
        return 1

    # topic liveness -- these are the pipeline's arteries
    for name, limit in (("/ethon/cones", 1.0), ("/ethon/obstacles", 1.0),
                        ("/cmd_vel", 1.0), ("/ethon/drive_status", 2.0)):
        v, age = topic(t, name)
        if age is None:
            res.append(line(BAD, name, "never published"))
        else:
            res.append(line(OK if age <= limit else BAD, name,
                            "age %.2fs (limit %.1f)" % (age, limit)))

    # cameras: only CALIBRATED sources reach the planner
    fus, _ = topic(t, "/ethon/fusion_status")
    if isinstance(fus, dict) and isinstance(fus.get("sources"), dict):
        srcs = fus["sources"]
        cal = [n for n, s in srcs.items() if s.get("calibrated")]
        alive = [n for n, s in srcs.items() if s.get("alive")]
        res.append(line(OK if cal else BAD, "calibrated cameras",
                        "%s  (alive: %s)" % (", ".join(cal) or "NONE",
                                             ", ".join(alive) or "none")))
    else:
        res.append(line(WARN, "fusion_status", "unavailable"))

    # drive node health
    ds, _ = topic(t, "/ethon/drive_status")
    ds = ds or {}
    res.append(line(OK if ds.get("steer_homed") else BAD, "steering homed",
                    str(ds.get("steering"))))
    res.append(line(OK if not ds.get("estop_latched") else BAD,
                    "e-stop clear", "latched=%s" % ds.get("estop_latched")))
    res.append(line(OK if not ds.get("config_hold") else BAD,
                    "config hold clear", "hold=%s" % ds.get("config_hold")))
    faults = {n: m.get("faults") for n, m in (ds.get("motors") or {}).items()
              if m.get("faults")}
    res.append(line(OK if not faults else WARN, "motor faults",
                    json.dumps(faults) if faults else "none"))
    res.append(line(OK if ds.get("use_foc") is False else WARN,
                    "control mode is DUTY",
                    "use_foc=%s (car is unlicensed)" % ds.get("use_foc")))

    # steering envelope -> the hard course constraint
    mx = ds.get("road_wheel_max_deg")
    wb = ds.get("wheelbase_m")
    if mx and wb:
        r = wb / math.tan(math.radians(mx))
        res.append(line(OK, "steering envelope",
                        "+/-%.1f deg  ->  min turn radius %.1f m "
                        "(design course >= %.0f m)" % (mx, r, r * 1.5)))
    else:
        res.append(line(WARN, "steering envelope", "unavailable"))

    # the 5 m/s first-arm trap
    spd = get_param("/cone_corridor_planner", "target_speed_ms")
    if spd is None:
        res.append(line(WARN, "target_speed_ms", "could not read"))
    else:
        res.append(line(OK if spd <= 1.5 else BAD, "target_speed_ms",
                        "%.2f m/s%s" % (spd, "" if spd <= 1.5
                                        else "  <- TOO FAST for a first run")))

    armed, _ = topic(t, "/ethon/hmi/armed")
    res.append(line(OK if not (armed or {}).get("data") else WARN,
                    "planner disarmed at rest",
                    "armed=%s" % (armed or {}).get("data")))

    bad = res.count(BAD)
    warn = res.count(WARN)
    print("-" * 72)
    print("%d pass, %d warn, %d FAIL" % (res.count(OK), warn, bad))
    print("VERDICT:", "NOT READY -- fix the FAILs above" if bad
          else ("ready (review warnings)" if warn else "READY"))
    print("Next: `steer` with the wheels clear, then `monitor`.")
    return 1 if bad else 0


def get_param(node, name):
    """Best-effort single param read; None if the API shape differs."""
    try:
        d = api_get("/api/params?node=%s" % node)
    except Exception:
        return None
    for key in ("params", "parameters", "values"):
        got = d.get(key)
        if isinstance(got, dict) and name in got:
            try:
                return float(got[name])
            except (TypeError, ValueError):
                return None
        if isinstance(got, list):
            for it in got:
                if isinstance(it, dict) and it.get("name") == name:
                    try:
                        return float(it.get("value"))
                    except (TypeError, ValueError):
                        return None
    if name in d:
        try:
            return float(d[name])
        except (TypeError, ValueError):
            return None
    return None


# ── steer sweep (Phase 0) ─────────────────────────────────────────────────

def hold_steer(deg, secs):
    """Post a steer-test angle for `secs`, sampling the measured angle."""
    samples = []
    t0 = time.monotonic()
    last_post = 0.0
    while time.monotonic() - t0 < secs:
        now = time.monotonic()
        if now - last_post >= STEER_REPOST_S:
            last_post = now
            try:
                api_post("/api/steertest", {"deg": float(deg)})
            except Exception:
                pass
        try:
            ds, _ = topic(topics(), "/ethon/drive_status")
            m = (ds or {}).get("road_wheel_deg")
            if m is not None:
                samples.append((now - t0, float(m)))
        except Exception:
            pass
        time.sleep(0.1)
    return samples


def cmd_steer():
    print("=" * 72)
    print("PHASE 0 -- STEERING SWEEP")
    print("=" * 72)
    print("!! WHEELS MUST BE CLEAR OF THE GROUND OR FREE TO TURN.")
    print("!! Convention: POSITIVE degrees must steer the wheels LEFT.")
    print("   A wrong sign here is the one error that makes the corridor")
    print("   follower steer OFF the course. Watch the wheels, not the screen.")
    print()
    try:
        input("Press ENTER when clear (Ctrl-C to abort)... ")
    except KeyboardInterrupt:
        print("\naborted")
        return 1

    ds, _ = topic(topics(), "/ethon/drive_status")
    mx = float((ds or {}).get("road_wheel_max_deg") or 12.0)
    steps = [0.0, round(mx * 0.35, 1), round(mx * 0.7, 1), 0.0,
             -round(mx * 0.35, 1), -round(mx * 0.7, 1), 0.0]

    print("\n%-10s %-10s %-9s %-8s %s" %
          ("cmd deg", "meas deg", "err deg", "settled", "direction"))
    print("-" * 60)
    rows = []
    try:
        for d in steps:
            s = hold_steer(d, 2.6)
            if not s:
                print("%-10.1f %-10s %-9s %-8s %s"
                      % (d, "no data", "-", "-", "-"))
                continue
            tail = [v for tt, v in s if tt >= 1.6] or [s[-1][1]]
            meas = sum(tail) / len(tail)
            spread = max(tail) - min(tail)
            settled = spread < 0.8
            direction = "-" if abs(d) < 0.05 else (
                "LEFT" if meas > 0 else "RIGHT")
            print("%-10.1f %-10.2f %-9.2f %-8s %s"
                  % (d, meas, meas - d, "yes" if settled else "HUNTING",
                     direction))
            rows.append((d, meas, settled))
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        try:
            api_post("/api/steertest", {"deg": 0.0})
        except Exception:
            pass
        print("\n(stopped posting -- the 0.5 s watchdog releases the motor)")

    nz = [(d, m) for d, m, _ in rows if abs(d) > 0.05]
    print("\n--- verdict ---")
    if not nz:
        print("no usable samples.")
        return 1
    if any((d > 0) != (m > 0) for d, m in nz):
        print("SIGN IS WRONG: positive command did not steer LEFT.")
        print("  -> flip `steer_inverted` in vehicle.yaml and restart")
        print("     ethon-drive. Do NOT patch signs anywhere else.")
        return 1
    errs = [abs(m - d) for d, m in nz]
    scale = sum(m / d for d, m in nz) / len(nz)
    print("sign OK (positive -> LEFT).")
    print("mean |error| %.2f deg, worst %.2f deg" % (sum(errs) / len(errs),
                                                     max(errs)))
    print("measured/commanded ratio %.3f" % scale)
    if abs(scale - 1.0) > 0.15:
        print("  -> off by >15%%: correct steer_col_ratio to %.1f"
              % (12.0 * scale))
    if not all(s for _, _, s in rows if True):
        print("  -> some steps HUNTING: raise steer_kd_duty before kp.")
    return 0


# ── monitor (Phases 1-3) ──────────────────────────────────────────────────

def cmd_monitor():
    print("live monitor -- Ctrl-C to stop")
    print("cones: count/nearest | gates: what blocks motion | dev_m: "
          "offset from midline (success = within 0.5 m)")
    print()
    hdr = ("%-7s %-6s %-16s %-8s %-9s %-9s %-8s %s" %
           ("armed", "cones", "nearest cone x,y", "corridor", "dev_m",
            "cmd_deg", "act_deg", "spd cmd/act"))
    try:
        n = 0
        while True:
            if n % 20 == 0:
                print(hdr)
                print("-" * 96)
            n += 1
            try:
                t = topics()
            except Exception:
                print("(api unreachable)")
                time.sleep(1.0)
                continue
            ds, _ = topic(t, "/ethon/drive_status")
            ds = ds or {}
            cones, _ = topic(t, "/ethon/cones")
            corr, _ = topic(t, "/ethon/corridor")
            corr = corr or {}
            cv, _ = topic(t, "/cmd_vel")
            armed, _ = topic(t, "/ethon/hmi/armed")

            poses = (cones or {}).get("poses") or []
            cnt = (cones or {}).get("pose_count")
            cnt = len(poses) if poses else (cnt or 0)
            if poses:
                near = min(poses, key=lambda p: p["position"]["x"])
                nearest = "%.2f, %+.2f" % (near["position"]["x"],
                                           near["position"]["y"])
            else:
                nearest = "-"

            vx = ((cv or {}).get("linear") or {}).get("x") or 0.0
            wz = ((cv or {}).get("angular") or {}).get("z") or 0.0
            wb = float(ds.get("wheelbase_m") or 1.524)
            if abs(vx) > 0.3:
                cmd_deg = math.degrees(math.atan(wb * (wz / vx)))
            else:
                cmd_deg = 0.0
            act = ds.get("road_wheel_deg")
            dev = corr.get("dev_m")

            print("%-7s %-6s %-16s %-8s %-9s %-9s %-8s %s" % (
                (armed or {}).get("data"), cnt, nearest,
                corr.get("state"),
                "-" if dev is None else "%+.2f" % dev,
                "%+.1f" % cmd_deg,
                "-" if act is None else "%+.1f" % act,
                "%.2f/%.2f" % (vx, ds.get("wheel_speed_ms") or 0.0)))
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nstopped")
    return 0


def main():
    # Line-buffer stdout: without this, piping/tee-ing any mode (especially
    # `monitor`, which is the one you WILL want in a log during a run) buffers
    # 8 KB at a time and shows nothing until it flushes -- and loses the tail
    # entirely if the process is killed.
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except Exception:
        pass
    mode = (sys.argv[1] if len(sys.argv) > 1 else "check").lower()
    if mode == "check":
        return cmd_check()
    if mode == "steer":
        return cmd_steer()
    if mode == "monitor":
        return cmd_monitor()
    print(__doc__)
    return 2


if __name__ == "__main__":
    sys.exit(main())
