"""
components.py — Cac class xu ly tin hieu cho he thong Object Tracking.
PIDController, KalmanTracker, SerialManager,
ReIDManager, estimate_distance.
"""

import cv2
import numpy as np
import math
import time
import serial
import threading
import queue

# ── Trang thai he thong ──────────────────────────────────────────
STATE_IDLE      = "IDLE"
STATE_TRACKING  = "TRACKING"
STATE_RETURNING = "RETURNING"
STATE_WAITING   = "WAITING"

RETURN_TIMEOUT = 60  # frame mat tay truoc khi quay ve Home

# ── Kalman Filter ────────────────────────────────────────────────
KALMAN_PN    = 0.8
KALMAN_MN    = 0.1
KALMAN_ADAPT = 8.0
VEL_CLAMP    = 150.0
BBOX_EMA     = 0.55

# ── Adaptive Error Scaling ───────────────────────────────────────
BBOX_REF_W = 200.0   # px — kich thuoc bbox "chuan"
SCALE_MIN  = 0.6
SCALE_MAX  = 4.0

# ── Uoc luong khoang cach ────────────────────────────────────────
OBJ_WIDTH_CM  = 18.0   # Kich thuoc tham chieu vat the (cm) — thay doi tuy doi tuong
FOCAL_LENGTH  = 333.0




# ══════════════════════════════════════════════════════════════════
class PIDController:
    """PID voi Deadzone + Hysteresis, Anti-windup, EMA output."""
    def __init__(self, kp, ki, kd,
                 out_min=-10.0, out_max=10.0, deadzone=18, output_ema=0.4):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self.deadzone   = deadzone
        self.output_ema = output_ema
        self.integral   = 0.0
        self.prev_err   = 0.0
        self.prev_out   = 0.0
        self.prev_time  = time.perf_counter()
        self._in_deadzone = False

    def compute(self, error):
        abs_err = abs(error)
        if self._in_deadzone:
            if abs_err < self.deadzone * 1.5:
                self.integral = 0.0
                self.prev_out *= 0.15
                self.prev_err = error
                return self.prev_out
            else:
                self._in_deadzone = False
        elif abs_err < self.deadzone:
            self._in_deadzone = True
            self.integral = 0.0
            self.prev_out *= 0.15
            self.prev_err = error
            return self.prev_out

        now = time.perf_counter()
        dt  = max(now - self.prev_time, 1e-3)
        self.prev_time = now

        if error * self.prev_err < 0:
            self.integral = 0.0
        self.integral = max(-20.0, min(20.0, self.integral + error * dt))

        d_raw = self.kd * (error - self.prev_err) / dt
        d_out = max(-self.out_max * 0.5, min(self.out_max * 0.5, d_raw))
        self.prev_err = error

        raw = max(self.out_min, min(self.out_max,
              self.kp * error + self.ki * self.integral + d_out))
        out = self.output_ema * raw + (1 - self.output_ema) * self.prev_out
        self.prev_out = out
        return out

    def reset(self):
        self.integral  = 0.0
        self.prev_err  = 0.0
        self.prev_out  = 0.0
        self.prev_time = time.perf_counter()
        self._in_deadzone = False


# ══════════════════════════════════════════════════════════════════
class SerialManager:
    """Quan ly giao tiep UART voi ESP32 (worker thread + queue)."""
    def __init__(self, port, baud):
        self.port, self.baud = port, baud
        self.ser  = None
        self.q    = queue.Queue(maxsize=3)
        self.connected = False
        self.sensor = {"pan_deg": 0.0, "tilt_deg": 0.0}
        self._connect()
        threading.Thread(target=self._worker, daemon=True).start()

    def _connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
            time.sleep(1.5)
            while self.ser.in_waiting:
                print("[ESP32] " + self.ser.readline().decode("utf-8", "ignore").strip())
            self.connected = True
            print("[Serial] Ket noi " + self.port + " thanh cong")
        except Exception as e:
            print("[Serial] Loi: " + str(e))
            self.connected = False

    def send(self, cmd):
        if not self.connected:
            return
        if self.q.full():
            try: self.q.get_nowait()
            except: pass
        try: self.q.put_nowait(cmd)
        except: pass

    def _worker(self):
        while True:
            try:
                cmd = self.q.get(timeout=0.05)
                if self.ser is not None and self.connected:
                    self.ser.write((cmd + "\n").encode())
                    while self.ser.in_waiting:
                        line = self.ser.readline().decode("utf-8", "ignore").strip()
                        if line.startswith("POS"):
                            p = line.split()
                            if len(p) >= 3:
                                try:
                                    self.sensor["pan_deg"]  = float(p[1])
                                    self.sensor["tilt_deg"] = float(p[2])
                                except: pass
            except queue.Empty:
                pass
            except Exception as e:
                print("[Serial] Worker loi: " + str(e))
                self.connected = False
                self.ser = None
                time.sleep(2)
                self._connect()

    def close(self):
        if self.ser:
            self.ser.close()


# ══════════════════════════════════════════════════════════════════
class KalmanTracker:
    """Bo loc Kalman 2D — lam min quy dao + du doan khi mat dau."""
    def __init__(self):
        self._init_kf()
        self.initialized = False
        self.miss  = 0
        self.ema_cx = 0.0
        self.ema_cy = 0.0
        self.ema_w  = 0.0

    def _init_kf(self):
        kf = cv2.KalmanFilter(4, 2)
        dt = 1.0
        kf.transitionMatrix    = np.array(
            [[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]], np.float32)
        kf.measurementMatrix   = np.array([[1,0,0,0],[0,1,0,0]], np.float32)
        pn = np.diag([KALMAN_PN, KALMAN_PN, KALMAN_PN*2, KALMAN_PN*2]).astype(np.float32)
        kf.processNoiseCov     = pn
        kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * KALMAN_MN
        kf.errorCovPost        = np.eye(4, dtype=np.float32)
        self.kf = kf

    def update(self, cx, cy, bbox_w=0):
        if not self.initialized:
            self.kf.statePost = np.array([[cx],[cy],[0],[0]], np.float32)
            self.ema_cx = cx; self.ema_cy = cy
            self.ema_w  = bbox_w if bbox_w > 0 else 1.0
            self.initialized = True
        self.miss = 0
        m    = np.array([[np.float32(cx)],[np.float32(cy)]])
        pred = self.kf.predict()
        innov = math.hypot(cx - pred[0,0], cy - pred[1,0])
        if innov > 15:
            scale = 1.0 + math.sqrt(innov/15.0) * KALMAN_ADAPT
            pn = np.diag([KALMAN_PN*scale, KALMAN_PN*scale,
                          KALMAN_PN*scale*2, KALMAN_PN*scale*2]).astype(np.float32)
        else:
            pn = np.diag([KALMAN_PN, KALMAN_PN,
                          KALMAN_PN*2, KALMAN_PN*2]).astype(np.float32)
        self.kf.processNoiseCov = pn
        self.kf.correct(m)
        s = self.kf.statePost.flatten()
        kf_cx, kf_cy = float(s[0]), float(s[1])
        vx, vy = float(s[2]), float(s[3])
        spd = math.hypot(vx, vy)
        if spd > VEL_CLAMP:
            r = VEL_CLAMP / spd; vx *= r; vy *= r
            self.kf.statePost[2] = np.float32(vx)
            self.kf.statePost[3] = np.float32(vy)
        self.ema_cx = BBOX_EMA * kf_cx + (1-BBOX_EMA) * self.ema_cx
        self.ema_cy = BBOX_EMA * kf_cy + (1-BBOX_EMA) * self.ema_cy
        if bbox_w > 0:
            self.ema_w = BBOX_EMA * bbox_w + (1-BBOX_EMA) * self.ema_w
        return self.ema_cx, self.ema_cy, vx, vy

    def predict_missing(self):
        if not self.initialized:
            return None
        self.miss += 1
        if self.miss > 8:
            self.initialized = False
            return None
        s = self.kf.predict().flatten()
        return float(s[0]), float(s[1])


# ══════════════════════════════════════════════════════════════════
def estimate_distance(bbox_w_px):
    if bbox_w_px <= 0:
        return 0.0
    return (OBJ_WIDTH_CM * FOCAL_LENGTH) / bbox_w_px


# ══════════════════════════════════════════════════════════════════
class ReIDManager:
    """Nhan dang lai muc tieu sau khi ByteTrack cap ID moi."""
    REID_RADIUS_PX = 180
    REID_WINDOW_S  = 2.5
    VELOCITY_COMP  = True

    def __init__(self):
        self._ghost: dict | None = None

    def record(self, tid: int, x: float, y: float,
               vx: float = 0.0, vy: float = 0.0) -> None:
        self._ghost = {
            "x": x, "y": y, "vx": vx, "vy": vy,
            "time": time.perf_counter(), "old_tid": tid,
        }

    def on_lost(self) -> None:
        if self._ghost is not None:
            self._ghost["time"] = time.perf_counter()

    def try_reid(self, all_detections: dict) -> int | None:
        if self._ghost is None or not all_detections:
            return None
        elapsed = time.perf_counter() - self._ghost["time"]
        if elapsed > self.REID_WINDOW_S:
            self._ghost = None
            return None
        # Uoc luong vi tri bang ngoai suy van toc
        if self.VELOCITY_COMP:
            dt_frames = elapsed * 30.0
            est_x = self._ghost["x"] + self._ghost["vx"] * dt_frames
            est_y = self._ghost["y"] + self._ghost["vy"] * dt_frames
        else:
            est_x, est_y = self._ghost["x"], self._ghost["y"]
        # Tim detection gan nhat
        best_tid, best_dist = None, float("inf")
        for tid, d in all_detections.items():
            dist = math.hypot(d["ema_x"] - est_x, d["ema_y"] - est_y)
            if dist < best_dist:
                best_dist, best_tid = dist, tid
        if best_dist <= self.REID_RADIUS_PX:
            old_tid = self._ghost["old_tid"]
            self._ghost = None
            if best_tid != old_tid:
                print(f"[ReID] Nhan lai muc tieu: ID {old_tid} -> {best_tid}"
                      f" (dist={best_dist:.0f}px, dt={elapsed:.2f}s)")
            return best_tid
        return None

    def cancel(self) -> None:
        self._ghost = None

    @property
    def is_searching(self) -> bool:
        if self._ghost is None:
            return False
        return time.perf_counter() - self._ghost["time"] <= self.REID_WINDOW_S

    @property
    def search_progress(self) -> float:
        if self._ghost is None:
            return 0.0
        elapsed = time.perf_counter() - self._ghost["time"]
        return min(1.0, elapsed / self.REID_WINDOW_S)
