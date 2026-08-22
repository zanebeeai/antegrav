#!/usr/bin/env python3
"""
Ethon active-learning data capture — runs on Jetson Orin NX in the car.

Captures both CSI cameras, runs TRT inference, and saves frames that are
worth labeling:
  - UNCERTAIN: any detection with conf in [0.30, 0.60)  → model is unsure,
    these frames teach it the most
  - PERIODIC : one frame every PERIODIC_SEC per camera   → scene diversity
  - NEGATIVE : frame with zero detections every NEG_SEC  → hard negatives

Each saved frame gets a JSON sidecar with model predictions (pre-labels for
faster human review) and capture metadata.

Disk guard: stops saving below MIN_FREE_GB, prunes oldest day-directories
below PRUNE_FREE_GB.

Run via systemd: ethon-capture.service
"""

import sys, json, time, signal, shutil, threading
from datetime import datetime
from pathlib import Path

import cv2
from ultralytics import YOLO

# ── config ────────────────────────────────────────────────────────────
MODEL_PATH    = "/home/jetson/ethon/models/ethon_v1.engine"  # swap to ethon_v1 when ready
DATA_ROOT     = Path("/home/jetson/ethon/capture_data")
CAMERAS       = [0, 1]            # nvarguscamerasrc sensor-ids
CAP_W, CAP_H  = 1280, 720
CAP_FPS       = 30                # camera rate; inference runs slower, frames drop
INFER_CONF    = 0.20              # run model at low conf so we see uncertain dets
UNCERT_LO     = 0.30
UNCERT_HI     = 0.60
PERIODIC_SEC  = 10.0              # diversity sample interval per camera
NEG_SEC       = 30.0              # negative (no-detection) sample interval
UNCERT_COOLDOWN = 2.0             # min seconds between uncertain saves per cam
JPEG_QUALITY  = 92
MIN_FREE_GB   = 20                # stop saving below this
PRUNE_FREE_GB = 30                # prune oldest days below this
CLASS_NAMES   = {}                # filled from model

_running = True


def disk_free_gb(path: Path) -> float:
    return shutil.disk_usage(path).free / 1e9


def prune_oldest(root: Path):
    """Delete oldest day-directories until free space recovers."""
    while disk_free_gb(root) < PRUNE_FREE_GB:
        days = sorted(d for d in root.iterdir() if d.is_dir())
        if len(days) <= 1:          # never delete today's data
            return
        victim = days[0]
        print(f"[disk] pruning {victim}", flush=True)
        shutil.rmtree(victim, ignore_errors=True)


def gst_pipeline(sensor_id: int) -> str:
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} wbmode=1 ! "
        f"video/x-raw(memory:NVMM),width={CAP_W},height={CAP_H},framerate={CAP_FPS}/1 ! "
        "nvvidconv ! video/x-raw,format=BGRx ! "
        "videoconvert ! video/x-raw,format=BGR ! "
        "appsink drop=true max-buffers=1 sync=false"
    )


class CamCapture:
    """Continuously reads one camera into a 1-slot buffer."""

    def __init__(self, sensor_id: int):
        self.sensor_id = sensor_id
        self.cap = cv2.VideoCapture(gst_pipeline(sensor_id), cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            raise RuntimeError(f"camera sensor-id={sensor_id} failed to open")
        self._frame = None
        self._lock = threading.Lock()
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while _running:
            ok, f = self.cap.read()
            if ok:
                with self._lock:
                    self._frame = f
            else:
                time.sleep(0.01)

    def latest(self):
        with self._lock:
            return None if self._frame is None else self._frame.copy()

    def release(self):
        self.cap.release()


def save_frame(cam_id: int, frame, results, reason: str):
    """Write JPEG + JSON sidecar under DATA_ROOT/YYYY-MM-DD/."""
    now = datetime.now()
    day_dir = DATA_ROOT / now.strftime("%Y-%m-%d")
    img_dir = day_dir / "images"
    meta_dir = day_dir / "meta"
    img_dir.mkdir(parents=True, exist_ok=True)
    meta_dir.mkdir(parents=True, exist_ok=True)

    stem = f"cam{cam_id}_{now.strftime('%H%M%S_%f')[:-3]}_{reason}"
    cv2.imwrite(str(img_dir / f"{stem}.jpg"), frame,
                [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])

    dets = []
    boxes = results[0].boxes
    if boxes is not None:
        for b in boxes:
            dets.append({
                "cls": int(b.cls[0]),
                "name": CLASS_NAMES.get(int(b.cls[0]), str(int(b.cls[0]))),
                "conf": round(float(b.conf[0]), 4),
                "xywhn": [round(float(v), 6) for v in b.xywhn[0]],
            })
    meta = {
        "ts": now.isoformat(),
        "camera": cam_id,
        "reason": reason,
        "model": Path(MODEL_PATH).name,
        "detections": dets,
    }
    (meta_dir / f"{stem}.json").write_text(json.dumps(meta))
    return stem


def main():
    global CLASS_NAMES
    print(f"Loading model {MODEL_PATH}", flush=True)
    model = YOLO(MODEL_PATH, task="detect")

    DATA_ROOT.mkdir(parents=True, exist_ok=True)

    cams = []
    for sid in CAMERAS:
        try:
            cams.append(CamCapture(sid))
            print(f"camera {sid} OK", flush=True)
        except RuntimeError as e:
            print(f"[warn] {e} — continuing without it", flush=True)
    if not cams:
        print("[fatal] no cameras", flush=True)
        sys.exit(1)

    last_periodic = {c.sensor_id: 0.0 for c in cams}
    last_negative = {c.sensor_id: 0.0 for c in cams}
    last_uncert   = {c.sensor_id: 0.0 for c in cams}
    saved = {"uncertain": 0, "periodic": 0, "negative": 0}
    last_disk_check = 0.0
    disk_ok = True

    def stop(*_):
        global _running
        _running = False
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)

    print("Capture loop running", flush=True)
    while _running:
        now = time.monotonic()

        # disk guard every 60s
        if now - last_disk_check > 60:
            last_disk_check = now
            free = disk_free_gb(DATA_ROOT)
            if free < PRUNE_FREE_GB:
                prune_oldest(DATA_ROOT)
            disk_ok = disk_free_gb(DATA_ROOT) > MIN_FREE_GB
            if not disk_ok:
                print(f"[disk] only {free:.1f} GB free — capture paused", flush=True)

        if not disk_ok:
            time.sleep(5)
            continue

        for cam in cams:
            frame = cam.latest()
            if frame is None:
                continue
            sid = cam.sensor_id

            res = model(frame, verbose=False, imgsz=640, half=True, conf=INFER_CONF)
            if not CLASS_NAMES:
                names = res[0].names or {}
                CLASS_NAMES = names if isinstance(names, dict) else dict(enumerate(names))

            boxes = res[0].boxes
            confs = [] if boxes is None else [float(b.conf[0]) for b in boxes]

            uncertain = any(UNCERT_LO <= c < UNCERT_HI for c in confs)
            empty = len(confs) == 0

            if uncertain and now - last_uncert[sid] >= UNCERT_COOLDOWN:
                save_frame(sid, frame, res, "uncertain")
                last_uncert[sid] = now
                saved["uncertain"] += 1
            elif empty and now - last_negative[sid] >= NEG_SEC:
                save_frame(sid, frame, res, "negative")
                last_negative[sid] = now
                saved["negative"] += 1
            elif now - last_periodic[sid] >= PERIODIC_SEC:
                save_frame(sid, frame, res, "periodic")
                last_periodic[sid] = now
                saved["periodic"] += 1

        # ~5 inference rounds/sec total across cameras
        time.sleep(0.1)

    for c in cams:
        c.release()
    print(f"Stopped. Saved: {saved}", flush=True)


if __name__ == "__main__":
    main()
