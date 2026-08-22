#!/usr/bin/env python3
"""Live GPS signal monitor -- run this OUTSIDE to see what the receiver hears.

The dashboard's GPS pill only shows FIX / UNFIXED, which cannot tell apart the
two very different failures:

    satellites in view = 0   -> no RF is reaching the receiver at all
                               (antenna unplugged/broken/blocked, or the
                                vehicle's own electronics are jamming L1)
    satellites in view > 0   -> the antenna works and it is acquiring; a fix
      but no fix                needs ~4 sats at >=30 dBHz, give it 30-60 s
                                (longer on a cold start with no almanac)

This prints both, live, once a second, so you can watch it while you move the
car/antenna around.

  sudo python3 /home/jetson/ethon/gps_diag.py          # 5 min then exits
  sudo python3 /home/jetson/ethon/gps_diag.py 900      # or a custom duration

It takes over /dev/ttyTHS1, so it STOPS ethon-gps on entry and RESTARTS it on
exit (including Ctrl-C / crash). Nothing is written to the receiver and no
configuration is changed -- read-only.

USEFUL TEST -- is the car jamming its own GPS?
  The 4 Kraken controllers, the DC-DC converters and the USB3 cameras all emit
  broadband noise around L1 (1575 MHz). Run this with the traction system
  powered DOWN, note the satellite count, then power it up and watch whether
  the count collapses. If it does, the antenna needs moving away from the
  offenders (and/or a ground plane under it).
"""
import collections
import subprocess
import sys
import time

sys.path.insert(0, "/home/jetson/.local/lib/python3.10/site-packages")
import serial   # noqa: E402  (python3-serial, same dep as the drivers)

PORT, BAUD = "/dev/ttyTHS1", 38400
SERVICE = "ethon-gps"
SUDO_PW = "yahboom"
QUALITY = {0: "NO FIX", 1: "GPS fix", 2: "DGPS", 4: "RTK fix", 5: "RTK float"}
CONSTELLATION = {"GP": "GPS", "GL": "GLONASS", "GA": "Galileo",
                 "GB": "BeiDou", "GQ": "QZSS", "GN": "combined"}


def _svc(action):
    try:
        subprocess.run(["sudo", "-S", "systemctl", action, SERVICE],
                       input=SUDO_PW + "\n", text=True, timeout=20,
                       capture_output=True)
    except Exception as exc:
        print("  (could not %s %s: %s)" % (action, SERVICE, exc))


def main(dur):
    print("stopping %s to take the port ..." % SERVICE)
    _svc("stop")
    time.sleep(1.0)
    try:
        run(dur)
    finally:
        print("\nrestarting %s ..." % SERVICE)
        _svc("start")
        time.sleep(1.5)
        r = subprocess.run(["systemctl", "is-active", SERVICE],
                           capture_output=True, text=True)
        print("%s is %s" % (SERVICE, r.stdout.strip() or "unknown"))


def run(dur):
    ser = serial.Serial(PORT, BAUD, timeout=0.2)
    buf = bytearray()
    sats = {}                     # (constellation, prn) -> snr string
    gga = None
    last_print = 0.0
    best_ever = 0
    sats_ever = 0
    t0 = time.monotonic()
    print("watching %s @ %d for %.0fs -- Ctrl-C to stop early\n"
          % (PORT, BAUD, dur))
    print("%-9s %-8s %-5s %-7s %-9s %s"
          % ("elapsed", "fix", "used", "HDOP", "in_view", "SNR dBHz (top 6)"))

    while time.monotonic() - t0 < dur:
        d = ser.read(2048)
        if d:
            buf.extend(d)
        while b"\n" in buf:
            i = buf.index(b"\n")
            line = bytes(buf[:i]).decode("ascii", "ignore").strip()
            del buf[:i + 1]
            if len(line) < 7 or line[0] != "$":
                continue
            talker, kind = line[1:3], line[3:6]
            f = line.split(",")
            if kind == "GGA":
                gga = f
            elif kind == "GSV":
                # $xxGSV,total,msg,numInView,(prn,elev,az,snr)x4
                body = f[4:]
                for k in range(0, max(0, len(body) - 3), 4):
                    prn = body[k]
                    snr = body[k + 3].split("*")[0]
                    if prn:
                        sats[(talker, prn)] = snr

        now = time.monotonic()
        if now - last_print >= 1.0:
            last_print = now
            snrs = sorted((int(s) for s in sats.values() if s.isdigit()),
                          reverse=True)
            sats_ever = max(sats_ever, len(sats))
            if snrs:
                best_ever = max(best_ever, snrs[0])
            q = used = hdop = "?"
            if gga:
                try:
                    q = QUALITY.get(int(gga[6]) if gga[6] else 0, gga[6])
                except (ValueError, IndexError):
                    q = "?"
                used = (gga[7] or "0") if len(gga) > 7 else "?"
                hdop = (gga[8] or "-") if len(gga) > 8 else "?"
            print("%-9.0f %-8s %-5s %-7s %-9d %s"
                  % (now - t0, q, used, hdop, len(sats),
                     " ".join(str(x) for x in snrs[:6]) or "(none)"))
            # per-constellation breakdown whenever anything is visible
            if sats:
                per = collections.Counter(t for (t, _) in sats)
                print("          seen by: %s" % ", ".join(
                    "%s=%d" % (CONSTELLATION.get(t, t), n)
                    for t, n in sorted(per.items())))
            sats.clear()          # GSV is re-sent every cycle; don't accumulate

    ser.close()
    print("\n== verdict ==")
    if sats_ever == 0:
        print("ZERO satellites seen the whole run.")
        print("The receiver is alive and talking, but no RF is reaching it.")
        print("Check, in this order:")
        print("  1. antenna connector seated (U.FL/SMA), cable not pinched or broken")
        print("  2. if it is the module's built-in patch: it must face the SKY,")
        print("     not sideways/down, and not under carbon fibre, metal or bodywork")
        print("  3. active antenna bias -- a passive antenna on a port expecting")
        print("     an active one (or vice versa) reads as deaf")
        print("  4. self-jamming: retest with the traction system powered down")
        print("     (see the note at the top of this file)")
        print("  5. swap in a known-good antenna/module to isolate")
    elif best_ever < 20:
        print("Satellites seen (max %d) but SNR peaked at only %d dBHz -- too weak"
              % (sats_ever, best_ever))
        print("to fix. That is a degraded antenna path or RF noise, not a")
        print("patience problem. Move the antenna away from the electronics,")
        print("give it a ground plane, and check the cable/connector.")
    else:
        print("Antenna path is GOOD: up to %d satellites, best SNR %d dBHz."
              % (sats_ever, best_ever))
        print("If it still would not fix, it needs TIME (cold start with no")
        print("almanac can take several minutes) or the backup battery is dead")
        print("so every power-cycle is a full cold start.")


if __name__ == "__main__":
    try:
        main(float(sys.argv[1]) if len(sys.argv) > 1 else 300.0)
    except KeyboardInterrupt:
        pass
