"""
app.py — Core Application Module
Chứa class GimbalApp quản lý toàn bộ hệ thống nhận diện và điều khiển.
Hệ thống giám sát theo dõi vật thể thời gian thực (Realtime Object Tracking).
"""

import cv2
import numpy as np
import math
import time
import threading
import queue
from collections import defaultdict
from ultralytics import YOLO

from components import (
    PIDController, KalmanTracker, SerialManager,
    ReIDManager, estimate_distance,
    STATE_IDLE, STATE_TRACKING, STATE_RETURNING, STATE_WAITING,
    RETURN_TIMEOUT
)
from hud import (
    draw_crosshair, draw_tracking_line,
    draw_bbox_idle, draw_bbox_tracking, draw_bbox_selected,
    draw_hud_overlay, draw_countdown,
    HUD_WARN, HUD_BRIGHT, HUD_MID
)

class GimbalApp:
    def __init__(self, config):
        """Khởi tạo ứng dụng với cấu hình được truyền từ main."""
        self.cfg = config
        
        # ── Các hằng số điều khiển thủ công ──
        self.MANUAL_STEP_DEG    = 5.0
        self.PAN_STEPS_PER_DEG  = 16.931
        self.TILT_STEPS_PER_DEG = 12.698
        self.FF_GAIN            = 0.018  # Feed-Forward gain cho dự đoán chuyển động
        
        # ── Các biến trạng thái (State) ──
        self.state         = STATE_IDLE
        self.paused        = False
        self.is_fullscreen = False
        self.home_pan_deg  = 0.0
        self.home_tilt_deg = 0.0
        self.miss_count    = 0
        self.return_sent   = False
        self.return_timer  = 0.0

        # ── Quản lý mục tiêu và tracking ──
        self.trackers       = defaultdict(KalmanTracker)
        self.reid           = ReIDManager()
        self.selected_tid   = None   # ID mục tiêu đang bị khóa cứng (bởi người dùng)
        self.tracking_tid   = None   # ID mục tiêu hệ thống đang bám theo
        self.detect_confirm = 0
        self.CONFIRM_FRAMES = 3

        # ── Hiệu năng và đo lường ──
        self.fps_buf = []
        self.avg_fps = 0.0
        self.n_targets = 0
        
        # ── Hàng đợi sự kiện chuột ──
        self.click_q    = queue.Queue()
        self.click_dets = {}

        # ── Khởi tạo các module con ──
        self._init_pid()
        self._init_serial()
        self._init_yolo()
        self._init_camera()
        self._start_cmd_sender()

    def _init_pid(self):
        """Khởi tạo bộ điều khiển PID cho cả 2 trục Pan và Tilt."""
        self.pid_pan  = PIDController(kp=0.009, ki=0.00002, kd=0.002,
                                      out_min=-6.5, out_max=6.5,
                                      deadzone=20, output_ema=0.22)
        self.pid_tilt = PIDController(kp=0.009, ki=0.00002, kd=0.002,
                                      out_min=-6.5, out_max=6.5,
                                      deadzone=20, output_ema=0.22)

    def _init_serial(self):
        """Kết nối tới ESP32 thông qua cổng Serial."""
        if self.cfg.get("serial_enable", True):
            self.serial_mgr = SerialManager(self.cfg["serial_port"], self.cfg["serial_baud"])
        else:
            self.serial_mgr = None

    def _init_yolo(self):
        """Load mô hình YOLO và làm nóng GPU."""
        print("[App] Dang load AI Model...")
        self.model = YOLO(self.cfg["model_path"])
        # Chạy thử 5 frame trống để warm-up TensorRT / CUDA
        dummy = np.zeros((640, 640, 3), dtype=np.uint8)
        for _ in range(5):
            self.model.track(dummy, tracker=self.cfg["tracker"], persist=True,
                             conf=self.cfg["conf"], half=True, device=0, verbose=False)
        print("[App] AI Model da san sang.")

    def _init_camera(self):
        """Khởi tạo Webcam với độ phân giải và FPS chỉ định."""
        self.cap = cv2.VideoCapture(self.cfg["webcam_id"])
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.cfg["width"])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.cfg["height"])
        self.cap.set(cv2.CAP_PROP_FPS,          30)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)
        
        self.W = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.H = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.img_cx = self.W // 2
        self.img_cy = self.H // 2

    def _start_cmd_sender(self):
        """Tạo một luồng ngầm (thread) để gửi lệnh UART liên tục (50Hz) mượt mà."""
        self.cmd_state = {'vel_pan': 0.0, 'vel_tilt': 0.0, 'active': False, 'updated_t': 0.0}
        self.cmd_lock  = threading.Lock()
        self.cmd_stop  = threading.Event()
        self.last_yolo_t = time.perf_counter()

        def sender_loop():
            DT_SEND = 0.020  # 50Hz
            EXPIRE  = 0.090  # Quá 90ms không có update -> dừng động cơ
            while not self.cmd_stop.is_set():
                time.sleep(DT_SEND)
                with self.cmd_lock:
                    active = self.cmd_state['active']
                    vp = self.cmd_state['vel_pan']
                    vt = self.cmd_state['vel_tilt']
                    age = time.perf_counter() - self.cmd_state['updated_t']
                
                # Gửi lệnh nếu hệ thống đang tracking và lệnh chưa hết hạn
                if active and age < EXPIRE and self.serial_mgr and self.serial_mgr.connected:
                    dp = vp * DT_SEND
                    dt_ = vt * DT_SEND
                    if abs(dp) > 0.08 or abs(dt_) > 0.08:
                        self.serial_mgr.send(f"A {dp:.3f} {dt_:.3f}")

        threading.Thread(target=sender_loop, daemon=True).start()

    # ════════════════════════════════════════════════════════════════
    # CÁC HÀM XỬ LÝ SỰ KIỆN VÀ ĐIỀU KHIỂN
    # ════════════════════════════════════════════════════════════════

    def _on_mouse(self, ev, x, y, flags, param):
        """Callback khi người dùng click chuột vào màn hình."""
        if ev == cv2.EVENT_LBUTTONDOWN:
            # Kiểm tra xem có click trúng khung bbox của mục tiêu nào không
            for tid, d in self.click_dets.items():
                bx1, by1, bx2, by2 = d["box"]
                if bx1 <= x <= bx2 and by1 <= y <= by2:
                    if self.selected_tid == tid:
                        self.selected_tid = None
                        print("[Select] Bo khoa - Chuyen sang Auto Mode")
                    else:
                        self.selected_tid = tid
                        print(f"[Select] Da khoa cung muc tieu ID={tid}")
                    return
            # Nếu click ra ngoài, lưu tọa độ (có thể dùng sau này)
            self.click_q.put((x, y))

    def _send_manual(self, pan_d, tilt_d):
        """Gửi lệnh quay động cơ thủ công (bằng phím mũi tên)."""
        if self.serial_mgr and self.serial_mgr.connected:
            ps = int(round(pan_d  * self.PAN_STEPS_PER_DEG))
            ts = int(round(tilt_d * self.TILT_STEPS_PER_DEG))
            self.serial_mgr.send(f"M {ps} {ts}")

    def _set_home(self):
        """Lưu vị trí hiện tại làm vị trí HOME."""
        if self.serial_mgr:
            self.home_pan_deg  = self.serial_mgr.sensor["pan_deg"]
            self.home_tilt_deg = self.serial_mgr.sensor["tilt_deg"]
            print(f"[Home] Da set Home: Pan={self.home_pan_deg}, Tilt={self.home_tilt_deg}")

    def _goto_home(self):
        """Lệnh cho Gimbal quay về vị trí HOME."""
        if self.serial_mgr and self.serial_mgr.connected:
            self.serial_mgr.send(f"G {self.home_pan_deg:.2f} {self.home_tilt_deg:.2f}")

    def start_tracking(self):
        """Bật chế độ tự động bám đuổi."""
        self.state = STATE_TRACKING
        self.miss_count = 0
        self.return_sent = False
        self.pid_pan.reset()
        self.pid_tilt.reset()
        with self.cmd_lock:
            self.cmd_state.update(active=True, vel_pan=0.0, vel_tilt=0.0,
                                  updated_t=time.perf_counter())
        if self.serial_mgr and self.serial_mgr.connected:
            self.serial_mgr.send("D 999.0")
        print("[Track] Bat dau Tracking")

    def stop_tracking(self):
        """Dừng chế độ bám đuổi, trả về trạng thái IDLE."""
        self.state = STATE_IDLE
        self.miss_count = 0
        self.return_sent = False
        self.selected_tid = None
        self.reid.cancel()
        self.pid_pan.reset()
        self.pid_tilt.reset()
        with self.cmd_lock:
            self.cmd_state.update(active=False, vel_pan=0.0, vel_tilt=0.0)
        if self.serial_mgr and self.serial_mgr.connected:
            self.serial_mgr.send("X")
            self.serial_mgr.send("D 0.0")
        print("[Track] Da dung Tracking")

    # ════════════════════════════════════════════════════════════════
    # VÒNG LẶP CHÍNH CỦA ỨNG DỤNG
    # ════════════════════════════════════════════════════════════════

    def run(self):
        """Hàm chính: Đọc camera, chạy AI, tính toán PID, và vẽ HUD."""
        cv2.namedWindow(self.cfg["window_name"], cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.cfg["window_name"], self.W, self.H)
        cv2.setMouseCallback(self.cfg["window_name"], self._on_mouse)

        fail_count = 0
        reconnects = 0
        t_prev = time.perf_counter()
        last_dist_t = 0.0

        # Giá trị mặc định — tránh NameError nếu paused=True ngay từ đầu
        frame          = np.zeros((self.H, self.W, 3), dtype=np.uint8)
        all_detections = {}
        detected_ids   = set()
        current_dist   = 0.0
        off_x, off_y   = 0.0, 0.0

        while True:
            if not self.paused:
                ret, frame = self.cap.read()
                if not ret:
                    fail_count += 1
                    if fail_count >= 30:
                        reconnects += 1
                        if reconnects > 3:
                            print("Mat ket noi Webcam! Thoat...")
                            break
                        self.cap.release(); time.sleep(1)
                        self._init_camera()
                        fail_count = 0
                    continue
                fail_count = 0

                # 1. Chạy AI YOLO + ByteTrack
                results = self.model.track(
                    frame, tracker=self.cfg["tracker"], persist=True,
                    conf=self.cfg["conf"], iou=0.3, imgsz=480,
                    half=True, device=0, verbose=False, max_det=self.cfg["max_targets"]
                )

                detected_ids   = set()
                all_detections = {}

                # 2. Xử lý kết quả AI qua Bộ lọc Kalman
                if results[0].boxes.id is not None:
                    boxes = results[0].boxes.xyxy.cpu().numpy()
                    ids   = results[0].boxes.id.cpu().numpy().astype(int)
                    for box, tid in zip(boxes, ids):
                        x1, y1, x2, y2 = map(int, box)
                        raw_cx = (x1+x2)/2.0
                        raw_cy = (y1+y2)/2.0
                        bbox_w = x2 - x1
                        ema_x, ema_y, vx, vy = self.trackers[tid].update(raw_cx, raw_cy, bbox_w)
                        detected_ids.add(tid)
                        all_detections[tid] = {
                            "box": (x1, y1, x2, y2),
                            "ema_x": ema_x, "ema_y": ema_y,
                            "bbox_w": self.trackers[tid].ema_w,
                            "vx": vx, "vy": vy,
                        }

                self.click_dets = dict(all_detections)

                # Dọn dẹp tracker cũ bị mất dấu quá lâu
                for tid in list(self.trackers.keys()):
                    if tid not in detected_ids:
                        if self.trackers[tid].predict_missing() is None:
                            del self.trackers[tid]

                # 3. Tính năng Nhận diện lại (Re-ID) khi mục tiêu bị mất
                if self.selected_tid is not None:
                    if self.selected_tid in all_detections:
                        d_sel = all_detections[self.selected_tid]
                        self.reid.record(self.selected_tid, d_sel["ema_x"], d_sel["ema_y"], d_sel["vx"], d_sel["vy"])
                    elif self.selected_tid not in self.trackers:
                        self.reid.on_lost()
                        new_tid = self.reid.try_reid(all_detections)
                        if new_tid is not None:
                            self.selected_tid = new_tid
                        elif not self.reid.is_searching:
                            print("[ReID] Het thoi gian tim - Ve Auto mode")
                            self.selected_tid = None
                else:
                    self.reid.cancel()

                # Tự động tracking lại nếu đang chờ
                if self.state in (STATE_RETURNING, STATE_WAITING):
                    if all_detections:
                        self.detect_confirm += 1
                        if self.detect_confirm >= self.CONFIRM_FRAMES:
                            self.detect_confirm = 0
                            self.start_tracking()
                    else:
                        self.detect_confirm = max(0, self.detect_confirm - 1)
                else:
                    self.detect_confirm = 0

                # =======================================================
                # 4. CỖ MÁY TRẠNG THÁI (STATE MACHINE)
                # =======================================================
                current_dist = 0.0
                off_x, off_y = 0.0, 0.0

                if self.state == STATE_IDLE:
                    # Chờ lệnh, chỉ vẽ viền nháp
                    for d in all_detections.values():
                        draw_bbox_idle(frame, d["box"])

                elif self.state == STATE_TRACKING:
                    # Chon muc tieu uu tien
                    if self.selected_tid is not None and self.selected_tid in all_detections:
                        best_tid = self.selected_tid
                    elif all_detections:
                        best_tid = min(all_detections.keys(), key=lambda t: math.hypot(
                                       all_detections[t]["ema_x"] - self.img_cx,
                                       all_detections[t]["ema_y"] - self.img_cy))
                    else:
                        best_tid = None

                    if best_tid is not None:
                        self.tracking_tid = best_tid
                        self.miss_count = 0
                        d = all_detections[best_tid]

                        # Vẽ khung nhắm HUD
                        if self.selected_tid is not None and best_tid == self.selected_tid:
                            draw_bbox_selected(frame, d["box"], best_tid)
                        else:
                            draw_bbox_tracking(frame, d["box"])

                        draw_crosshair(frame, self.img_cx, self.img_cy)
                        draw_tracking_line(frame, self.img_cx, self.img_cy, d["ema_x"], d["ema_y"])

                        off_x = d["ema_x"] - self.img_cx
                        off_y = d["ema_y"] - self.img_cy
                        current_dist = estimate_distance(d["bbox_w"])
                        
                        # --- Tính toán PID & Gửi Serial ---
                        pid_out_pan  =  self.pid_pan.compute(off_x)
                        pid_out_tilt = -self.pid_tilt.compute(off_y)
                        ff_pan  =  self.FF_GAIN * d["vx"]
                        ff_tilt = -self.FF_GAIN * d["vy"]
                        delta_pan  = max(-6.5, min(6.5, pid_out_pan  + ff_pan))
                        delta_tilt = max(-6.5, min(6.5, pid_out_tilt + ff_tilt))

                        now_yolo = time.perf_counter()
                        dt_yolo  = min(max(now_yolo - self.last_yolo_t, 0.020), 0.080)
                        self.last_yolo_t = now_yolo
                        
                        with self.cmd_lock:
                            self.cmd_state['vel_pan']   = delta_pan  / dt_yolo
                            self.cmd_state['vel_tilt']  = delta_tilt / dt_yolo
                            self.cmd_state['updated_t'] = now_yolo

                        now_t = time.perf_counter()
                        if (self.serial_mgr and self.serial_mgr.connected and 
                            now_t - last_dist_t >= 0.2):
                            last_dist_t = now_t
                            self.serial_mgr.send(f"D {current_dist:.1f}")

                        # Ve cac tay khong duoc chon
                        for tid2, d2 in all_detections.items():
                            if tid2 != best_tid:
                                draw_bbox_idle(frame, d2["box"])

                        if len(all_detections) > 1 and self.selected_tid is None:
                            cv2.putText(frame, "Click tay de LOCK", (self.img_cx - 70, self.img_cy - 30),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,215,255), 1)
                    else:
                        # Mất mục tiêu -> Vẽ thanh loading Re-ID
                        if self.reid.is_searching:
                            new_tid = self.reid.try_reid(all_detections)
                            if new_tid is not None:
                                self.selected_tid = new_tid
                            else:
                                prog  = self.reid.search_progress
                                bar_w = 220
                                bx = self.img_cx - bar_w // 2
                                by = self.img_cy - 60
                                cv2.rectangle(frame, (bx-2, by-2), (bx+bar_w+2, by+18), (30,30,30), -1)
                                filled = int(bar_w * (1.0 - prog))
                                if filled > 0:
                                    r = int(255 * prog); g = int(200 * (1.0 - prog))
                                    cv2.rectangle(frame, (bx, by), (bx+filled, by+14), (0, g, r), -1)
                                cv2.rectangle(frame, (bx-2, by-2), (bx+bar_w+2, by+18), (80,80,80), 1)
                                remain = (1.0 - prog) * ReIDManager.REID_WINDOW_S
                                cv2.putText(frame, f"Re-ID dang tim... {remain:.1f}s", (bx, by-6),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0,215,255), 1)

                        self.miss_count += 1
                        pred_pos = None
                        if self.tracking_tid is not None and self.tracking_tid in self.trackers:
                            pred_pos = self.trackers[self.tracking_tid].predict_missing()

                        # Du doan muc tieu trong 6 khung hinh dau tien bi che
                        if pred_pos is not None and self.miss_count <= 6:
                            pred_x, pred_y = pred_pos
                            off_x = pred_x - self.img_cx; off_y = pred_y - self.img_cy
                            pid_out_pan  =  self.pid_pan.compute(off_x)  * 0.6
                            pid_out_tilt = -self.pid_tilt.compute(off_y) * 0.6
                            now_yolo = time.perf_counter()
                            dt_yolo  = min(max(now_yolo - self.last_yolo_t, 0.020), 0.080)
                            self.last_yolo_t = now_yolo
                            with self.cmd_lock:
                                self.cmd_state['vel_pan']   = pid_out_pan  / dt_yolo
                                self.cmd_state['vel_tilt']  = pid_out_tilt / dt_yolo
                                self.cmd_state['updated_t'] = now_yolo
                            cv2.putText(frame, "Predicting...", (self.img_cx-60, self.img_cy-20),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,200,200), 2)
                        elif self.miss_count <= RETURN_TIMEOUT:
                            draw_countdown(frame, self.W, self.H, self.miss_count)
                        else:
                            self.state = STATE_RETURNING
                            self.return_sent = False
                            self.tracking_tid = None
                            self.pid_pan.reset(); self.pid_tilt.reset()
                            print("[Track] Mat tay -> Quay ve Home")

                elif self.state == STATE_RETURNING:
                    if not self.return_sent:
                        self._goto_home()
                        self.return_sent = True
                        self.return_timer = time.perf_counter()
                        if self.serial_mgr and self.serial_mgr.connected:
                            self.serial_mgr.send("D 0.5")
                    
                    txt = "Dang quay ve Home..."
                    (tw, _), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)
                    cv2.putText(frame, txt, ((self.W-tw)//2, self.H//2),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, HUD_WARN, 2)
                    
                    if time.perf_counter() - self.return_timer >= 1.5:
                        if self.serial_mgr:
                            cp = self.serial_mgr.sensor["pan_deg"]
                            ct = self.serial_mgr.sensor["tilt_deg"]
                            if abs(cp - self.home_pan_deg) < 3.0 and abs(ct - self.home_tilt_deg) < 3.0:
                                self.state = STATE_WAITING
                                if self.serial_mgr.connected:
                                    self.serial_mgr.send("D 0.0")
                        else:
                            self.state = STATE_WAITING

                elif self.state == STATE_WAITING:
                    for d in all_detections.values():
                        draw_bbox_idle(frame, d["box"])
                    txt = "Dang cho tay..."
                    (tw, _), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)
                    cv2.putText(frame, txt, ((self.W-tw)//2, self.H//2),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, HUD_BRIGHT, 2)

                if not all_detections and self.state not in (STATE_RETURNING, STATE_WAITING):
                    self.pid_pan.reset(); self.pid_tilt.reset()

                # Tinh toan FPS
                t_now = time.perf_counter()
                fps = 1.0 / (t_now - t_prev + 1e-9)
                t_prev = t_now
                self.fps_buf.append(fps)
                if len(self.fps_buf) > 30: self.fps_buf.pop(0)
                self.avg_fps = sum(self.fps_buf) / len(self.fps_buf)
                self.n_targets = len(detected_ids)

            # 5. Vẽ giao diện HUD lên màn hình
            _target_xy = None
            if self.tracking_tid is not None and self.tracking_tid in all_detections:
                _target_xy = (all_detections[self.tracking_tid]["ema_x"], all_detections[self.tracking_tid]["ema_y"])
            elif all_detections:
                d_first = next(iter(all_detections.values()))
                _target_xy = (d_first["ema_x"], d_first["ema_y"])

            draw_hud_overlay(
                frame, self.W, self.H, self.state,
                self.serial_mgr.connected if self.serial_mgr else False,
                self.serial_mgr.sensor["pan_deg"]  if self.serial_mgr else 0.0,
                self.serial_mgr.sensor["tilt_deg"] if self.serial_mgr else 0.0,
                current_dist, self.avg_fps, self.n_targets,
                off_x, off_y,
                selected_tid=self.selected_tid,
                target_xy=_target_xy,
            )

            # Cảnh báo Tạm dừng
            if self.paused:
                overlay_p = frame.copy()
                cv2.rectangle(overlay_p, (self.W//2-160, self.H//2-22),
                             (self.W//2+160, self.H//2+14), (0,15,0), -1)
                cv2.addWeighted(overlay_p, 0.7, frame, 0.3, 0, frame)
                cv2.rectangle(frame, (self.W//2-160, self.H//2-22),
                             (self.W//2+160, self.H//2+14), (100,210,100), 1)
                cv2.putText(frame, "[ PAUSED — press P to resume ]",
                           (self.W//2-148, self.H//2+6), cv2.FONT_HERSHEY_SIMPLEX,
                           0.50, (180,255,180), 1)

            cv2.imshow(self.cfg["window_name"], frame)

            # 6. Bắt sự kiện bàn phím
            key = cv2.waitKeyEx(1)
            char_key = key & 0xFF
            
            if char_key == ord("q"): break
            elif char_key == ord("f"):
                self.is_fullscreen = not self.is_fullscreen
                prop = cv2.WINDOW_FULLSCREEN if self.is_fullscreen else cv2.WINDOW_NORMAL
                cv2.setWindowProperty(self.cfg["window_name"], cv2.WND_PROP_FULLSCREEN, prop)
            elif char_key == ord("p"): self.paused = not self.paused
            elif char_key == ord("t"): self.start_tracking() if self.state == STATE_IDLE else self.stop_tracking()
            elif char_key == ord("h"): self._set_home()
            elif char_key == ord("r"):
                if self.state == STATE_TRACKING: self.stop_tracking()
                self._goto_home()
            elif key == 2490368 or char_key == 82: self._send_manual(0, -self.MANUAL_STEP_DEG)  # Mui ten len
            elif key == 2621440 or char_key == 84: self._send_manual(0,  self.MANUAL_STEP_DEG)  # Mui ten xuong
            elif key == 2424832 or char_key == 81: self._send_manual(-self.MANUAL_STEP_DEG, 0)  # Mui ten trai
            elif key == 2555904 or char_key == 83: self._send_manual( self.MANUAL_STEP_DEG, 0)  # Mui ten phai
            elif char_key == ord("u"):
                if self.selected_tid is not None:
                    print("[Select] Bo khoa (phim U)")
                    self.selected_tid = None

        self._cleanup()

    def _cleanup(self):
        """Dọn dẹp và tắt an toàn hệ thống."""
        self.cmd_stop.set()
        if self.serial_mgr:
            if self.serial_mgr.connected:
                self.serial_mgr.send("X"); self.serial_mgr.send("D 0.0")
                time.sleep(0.2)
            self.serial_mgr.close()
        self.cap.release()
        cv2.destroyAllWindows()
        print("[App] He thong tat an toan.")
