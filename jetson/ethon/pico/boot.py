# boot.py — runs once at power-up, BEFORE code.py, to configure USB.
#
# Enables a SECOND USB CDC serial endpoint ("data") that is separate from the
# REPL/console. The Jetson talks to the wheel hub over the DATA endpoint; the
# console endpoint stays free for debugging in a terminal. Without this, code.py
# would have to share one port with the REPL.
#
# Deploy: copy this file to the CIRCUITPY drive root, then hard-reset the Pico
# (replug) so the new USB descriptor takes effect. After this, the Pico exposes
# TWO /dev/ttyACM* ports on the Jetson — the udev rule (99-ethon-usb.rules) maps
# the DATA one to /dev/ethon-wheel by interface number.
import usb_cdc

usb_cdc.enable(console=True, data=True)
