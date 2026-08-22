#!/usr/bin/env python3
"""Flash a .tft to the Nextion over the Jetson UART (whmi-wri upload protocol).

Usage:
    sudo systemctl stop ethon-hmi          # free /dev/ttyTHS1 first
    python3 nextion_upload.py <file.tft>   # [--port ...] [--baud 9600] [--upload-baud 9600]
    sudo systemctl start ethon-hmi

Talks to the panel at its CURRENT baud (--baud, 9600 here), tells it to receive
<filesize> bytes, then streams the file in 4096-byte blocks, waiting for the
panel's 0x05 'ready' byte after the handshake and each block. On success the
panel reboots into the new project. Keeping --upload-baud == --baud avoids a
mid-transfer baud switch (a blank .tft is tiny, so 9600 is plenty fast).
"""
import argparse
import os
import sys
import time

import serial

EOL = b"\xff\xff\xff"
CHUNK = 4096


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tft")
    ap.add_argument("--port", default="/dev/ttyTHS1")
    ap.add_argument("--baud", type=int, default=9600, help="panel's CURRENT baud")
    ap.add_argument("--upload-baud", type=int, default=9600)
    a = ap.parse_args()

    if not os.path.isfile(a.tft):
        print("no such file: %s" % a.tft)
        return 2
    size = os.path.getsize(a.tft)
    print("flashing %s (%d bytes) via %s @%d (upload @%d)"
          % (a.tft, size, a.port, a.baud, a.upload_baud))

    s = serial.Serial(a.port, a.baud, timeout=2)

    def wait_ack(where):
        deadline = time.time() + 3.0
        got = b""
        while time.time() < deadline:
            b = s.read(1)
            if b:
                got += b
                if b == b"\x05":      # Nextion 'ready for next block'
                    return True
        print("  ! no 0x05 ack at %s (got %r)" % (where, got))
        return False

    s.reset_input_buffer()
    s.write(EOL)                                    # clear any partial command
    time.sleep(0.1)
    s.write(("whmi-wri %d,%d,0" % (size, a.upload_baud)).encode())
    s.write(EOL)
    s.flush()
    if a.upload_baud != a.baud:                     # switch only if asked to
        time.sleep(0.2)
        s.baudrate = a.upload_baud
    if not wait_ack("handshake"):
        print("handshake failed -- panel powered? on %s at %d? ethon-hmi stopped?"
              % (a.port, a.baud))
        return 3

    sent = 0
    with open(a.tft, "rb") as f:
        while True:
            chunk = f.read(CHUNK)
            if not chunk:
                break
            s.write(chunk)
            s.flush()
            sent += len(chunk)
            if not wait_ack("block @%d" % sent):
                print("upload STALLED at %d/%d bytes" % (sent, size))
                return 4
            print("  %d/%d bytes" % (sent, size))
    print("UPLOAD COMPLETE -- panel reboots into the new project")
    s.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
