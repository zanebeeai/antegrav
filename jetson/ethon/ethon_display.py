#!/usr/bin/env python3
"""
Ethon Vision — 4-camera 2x2 HDMI display
Top-left:     sensor-id=0  IMX477  (road_v1)   [Jetson CSI]
Top-right:    sensor-id=1  IMX219  (cones_v2)  [Jetson CSI]
Bottom-left:  perc-1:5001  OV5647  (road_v1)   [TCP + Jetson inference]
Bottom-right: perc-1:5002  IMX708  (cones_v2)  [TCP + Jetson inference]

Mouse: click panel to rotate 90° CW  |  Keys: Q=quit  1/2/3/4=rotate panel
"""
import os, sys, traceback, cv2, numpy as np, threading, time, socket, struct

os.environ["DISPLAY"] = ":0"
os.environ["XAUTHORITY"] = "/run/user/1000/gdm/Xauthority"

from ultralytics import YOLO

MODEL_DIR   = "/home/jetson/ethon/models"
DISP_W, DISP_H = 1920, 1080
PW, PH      = DISP_W // 2, DISP_H // 2   # 960 × 540 per panel
CONF        = 0.45

PERC1_IPS   = ["192.168.0.101", "100.86.169.68"]
PERC1_PORT1 = 5001   # OV5647
PERC1_PORT2 = 5002   # IMX708

# cv2.rotate codes indexed by rotation step (0=0°, 1=90°CW, 2=180°, 3=90°CCW)
_ROT = [None,
        cv2.ROTATE_90_CLOCKWISE,
        cv2.ROTATE_180,
        cv2.ROTATE_90_COUNTERCLOCKWISE]


def rotate_frame(f, step):
    return f if step == 0 else cv2.rotate(f, _ROT[step])


def gst_pipeline(sensor_id):
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} wbmode=1 tnr-mode=0 ee-mode=0 ! "
        "video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1 ! "
        "nvvidconv ! video/x-raw,format=BGRx ! "
        "videobalance saturation=1.5 contrast=1.15 ! "
        "videoconvert ! video/x-raw,format=BGR ! "
        "appsink drop=true max-buffers=1 sync=false"
    )


def blank(msg="NO SIGNAL"):
    f = np.zeros((PH, PW, 3), np.uint8)
    cv2.putText(f, msg, (20, PH // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (80, 80, 80), 2)
    return f


def _fix_names(res):
    orig = res[0].names or {}
    if not isinstance(orig, dict):
        orig = dict(enumerate(orig))
    full = {i: str(i) for i in range(256)}
    full.update(orig)
    res[0].names = full


BTN_W, BTN_H = 90, 40   # rotate button size (top-right of each panel)


def draw_rotate_btn(f):
    """Draw a ↻ button in the top-right corner of a panel."""
    x1, y1 = PW - BTN_W - 8, 8
    x2, y2 = PW - 8, 8 + BTN_H
    cv2.rectangle(f, (x1, y1), (x2, y2), (40, 40, 40), -1)
    cv2.rectangle(f, (x1, y1), (x2, y2), (140, 140, 140), 1)
    cv2.putText(f, "rotate", (x1 + 6, y1 + 27),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (220, 220, 220), 1)


def overlay(f, label, fps, rot, color=(0, 255, 80)):
    rot_str = ["", " 90CW", " 180", " 90CCW"][rot]
    for col, thick, y, txt in [
        ((0, 0, 0), 4, 44, label + rot_str), (color, 2, 44, label + rot_str),
        ((0, 0, 0), 3, 86, f"{fps:.1f} FPS"),  (color, 2, 86, f"{fps:.1f} FPS"),
    ]:
        cv2.putText(f, txt, (12, y), cv2.FONT_HERSHEY_SIMPLEX,
                    1.1 if y == 44 else 1.0, col, thick)
    draw_rotate_btn(f)


class LocalCamWorker:
    def __init__(self, sensor_id, model_path, label, task="detect"):
        self.sensor_id = sensor_id
        self.label     = label
        self.model     = YOLO(model_path, task=task)
        self.cap       = cv2.VideoCapture(gst_pipeline(sensor_id), cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            print(f"[FATAL] sensor-id={sensor_id} failed", flush=True)
            sys.exit(1)
        self._frame   = blank(f"sensor-id={sensor_id} starting…")
        self._lock    = threading.Lock()
        self._fps     = 0.0
        self._rot     = 0          # rotation step, written by main thread
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def rotate(self):
        self._rot = (self._rot + 1) % 4

    def _loop(self):
        t0, n = time.perf_counter(), 0
        while self._running:
            try:
                ok, raw = self.cap.read()
                if not ok:
                    time.sleep(0.005)
                    continue
                rot = self._rot
                inp = rotate_frame(raw, rot)
                res = self.model(inp, verbose=False, imgsz=640, half=True, conf=CONF)
                _fix_names(res)
                ann = cv2.resize(res[0].plot(), (PW, PH))
                n += 1
                elapsed = time.perf_counter() - t0
                if elapsed >= 1.0:
                    self._fps = n / elapsed
                    n, t0 = 0, time.perf_counter()
                overlay(ann, self.label, self._fps, rot)
                with self._lock:
                    self._frame = ann
            except Exception as e:
                print(f"[local cam{self.sensor_id}] {e}", flush=True)
                traceback.print_exc()
                time.sleep(0.1)

    def get_frame(self):
        with self._lock:
            return self._frame.copy()

    def stop(self):
        self._running = False
        self.cap.release()


class NetworkCamWorker:
    """Receives length-prefixed JPEG from perc-1, runs Jetson TRT inference.

    Two threads: _recv_thread drains TCP as fast as possible into a 1-slot
    buffer; _infer_thread pulls from that buffer and runs inference so slow
    inference never blocks the socket and causes TCP stalls / reconnects.
    """

    def __init__(self, ips, port, label, model_path=None, task="detect"):
        self.ips   = ips
        self.port  = port
        self.label = label
        self.model = YOLO(model_path, task=task) if model_path else None
        self._frame    = blank(f"{label} connecting…")
        self._lock     = threading.Lock()
        self._fps      = 0.0
        self._rot      = 0
        self._running  = True
        # 1-slot raw-frame queue: recv thread overwrites, infer thread reads
        self._raw      = None
        self._raw_lock = threading.Lock()
        self._raw_evt  = threading.Event()
        threading.Thread(target=self._recv_thread, daemon=True).start()
        threading.Thread(target=self._infer_thread, daemon=True).start()

    def rotate(self):
        self._rot = (self._rot + 1) % 4

    def _connect(self):
        for ip in self.ips:
            try:
                s = socket.create_connection((ip, self.port), timeout=3)
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
                s.settimeout(None)   # blocking recv — timeout=3 from create_connection lingers otherwise
                print(f"[net {self.label}] {ip}:{self.port} OK", flush=True)
                return s
            except OSError:
                continue
        return None

    @staticmethod
    def _recv_exact(sock, n):
        buf = bytearray()
        while len(buf) < n:
            chunk = sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("disconnected")
            buf += chunk
        return bytes(buf)

    def _recv_thread(self):
        """Drain TCP as fast as possible; drop frames if infer is behind."""
        while self._running:
            sock = self._connect()
            if sock is None:
                with self._lock:
                    self._frame = blank(f"{self.label} offline")
                time.sleep(3)
                continue
            try:
                while self._running:
                    size = struct.unpack(">I", self._recv_exact(sock, 4))[0]
                    if size == 0 or size > 10_000_000:
                        raise ValueError(f"bad frame size {size}")
                    data = self._recv_exact(sock, size)
                    # overwrite previous unprocessed frame (drop it)
                    with self._raw_lock:
                        self._raw = data
                    self._raw_evt.set()
            except Exception as e:
                print(f"[net {self.label}] {e}", flush=True)
                try:
                    sock.close()
                except Exception:
                    pass
                with self._lock:
                    self._frame = blank(f"{self.label} reconnecting…")
                time.sleep(2)

    def _infer_thread(self):
        t0, n = time.perf_counter(), 0
        while self._running:
            if not self._raw_evt.wait(timeout=1.0):
                continue
            self._raw_evt.clear()
            with self._raw_lock:
                data = self._raw
            if data is None:
                continue
            try:
                bgr = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
                if bgr is None:
                    continue
                rot = self._rot
                inp = rotate_frame(bgr, rot)
                if self.model is not None:
                    res = self.model(inp, verbose=False, imgsz=640, half=True, conf=CONF)
                    _fix_names(res)
                    f = cv2.resize(res[0].plot(), (PW, PH))
                else:
                    f = cv2.resize(inp, (PW, PH))
                n += 1
                elapsed = time.perf_counter() - t0
                if elapsed >= 1.0:
                    self._fps = n / elapsed
                    n, t0 = 0, time.perf_counter()
                overlay(f, self.label, self._fps, rot, color=(255, 160, 0))
                with self._lock:
                    self._frame = f
            except Exception as e:
                print(f"[infer {self.label}] {e}", flush=True)
                traceback.print_exc()

    def get_frame(self):
        with self._lock:
            return self._frame.copy()

    def stop(self):
        self._running = False


def main():
    print("Loading models…", flush=True)

    cam0 = LocalCamWorker(0, f"{MODEL_DIR}/road_v1_best.engine",
                          "IMX477 | road_v1", task="detect")
    cam1 = LocalCamWorker(1, f"{MODEL_DIR}/cones_v2_best.engine",
                          "IMX219 | cones_v2", task="detect")
    cam2 = NetworkCamWorker(PERC1_IPS, PERC1_PORT1, "perc-1 | OV5647",
                            model_path=f"{MODEL_DIR}/road_v1_best.engine", task="detect")
    cam3 = NetworkCamWorker(PERC1_IPS, PERC1_PORT2, "perc-1 | IMX708",
                            model_path=f"{MODEL_DIR}/cones_v2_best.engine", task="detect")

    cameras = [cam0, cam1, cam2, cam3]
    time.sleep(3)

    cv2.namedWindow("Ethon Vision", cv2.WINDOW_NORMAL)
    cv2.setWindowProperty("Ethon Vision", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    print("Running — click panel to rotate  |  Q=quit  1/2/3/4=rotate panel", flush=True)

    def on_mouse(event, x, y, flags, _):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        col = x // PW   # 0=left  1=right
        row = y // PH   # 0=top   1=bottom
        cameras[row * 2 + col].rotate()

    cv2.setMouseCallback("Ethon Vision", on_mouse)

    rotate_keys = {ord('1'): 0, ord('2'): 1, ord('3'): 2, ord('4'): 3}

    try:
        while True:
            top    = np.hstack([cam0.get_frame(), cam1.get_frame()])
            bottom = np.hstack([cam2.get_frame(), cam3.get_frame()])
            cv2.imshow("Ethon Vision", np.vstack([top, bottom]))
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            if key in rotate_keys:
                cameras[rotate_keys[key]].rotate()
    finally:
        for c in cameras:
            c.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
