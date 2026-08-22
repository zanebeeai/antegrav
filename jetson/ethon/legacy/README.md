# Retired modules

Superseded by `wheel_bridge.py` (service `ethon-wheel`), which owns the
Nextion display, the NeoPixel strip and the wheel buttons over the single
framed USB link to the Pico. The old `ethon-hmi` / `ethon-leds` services no
longer exist and nothing in the tree imports these files.

Kept for reference only — the Nextion drawing primitives in `nextion_hmi.py`
are the origin of the ones reused in `wheel_bridge.py`.

Retired 2026-08-03.
