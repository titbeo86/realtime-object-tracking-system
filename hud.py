"""
hud.py — Military Thermal HUD drawing functions.
Phong cach: man hinh nhin dem/nhiet (Night Vision / Thermal Imaging).
"""

import cv2
import numpy as np
import math

from components import (STATE_IDLE, STATE_TRACKING, STATE_RETURNING,
                        STATE_WAITING, RETURN_TIMEOUT)

# ── Bang mau HUD ─────────────────────────────────────────────────
HUD_BRIGHT = (180, 255, 180)   # xanh la sang — text chinh, crosshair
HUD_MID    = (100, 210, 100)   # xanh la vua  — text phu, duong ke
HUD_DIM    = (50,  130,  50)   # xanh la toi  — vien, luoi mo
HUD_WHITE  = (220, 255, 220)   # trang xanh   — nhan manh
HUD_WARN   = (80,  80,  255)   # do            — canh bao
HUD_LOCK   = (0,   215, 255)   # vang-teal     — LOCKED target
FONT       = cv2.FONT_HERSHEY_SIMPLEX


def apply_green_tint(frame: np.ndarray) -> np.ndarray:
    """Night Vision green tint: grayscale + CLAHE + green channel."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    gray = clahe.apply(gray)
    zeros = np.zeros_like(gray)
    green = cv2.merge([zeros, gray, zeros])
    return cv2.addWeighted(frame, 0.15, green, 0.85, 0)


def _put_hud_text(frame, text, x, y, color=None, scale=0.42, thickness=1):
    if color is None:
        color = HUD_MID
    cv2.putText(frame, text, (x, y), FONT, scale, color, thickness, cv2.LINE_AA)


def draw_crosshair(frame, cx, cy):
    """Tam ngam full-frame military style: duong chia + bracket + reticle + mil-dot."""
    H_f, W_f = frame.shape[:2]
    AA = cv2.LINE_AA
    # Duong chia 4 o
    cv2.line(frame, (0, cy), (W_f, cy), HUD_DIM, 1, AA)
    cv2.line(frame, (cx, 0), (cx, H_f), HUD_DIM, 1, AA)
    # Bracket ngoai lon
    BW, BH, SZ = 300, 220, 28
    bx1, by1 = cx - BW//2, cy - BH//2
    bx2, by2 = cx + BW//2, cy + BH//2
    for (px, py), (dx, dy) in [((bx1,by1),(1,1)), ((bx2,by1),(-1,1)),
                                ((bx1,by2),(1,-1)), ((bx2,by2),(-1,-1))]:
        cv2.line(frame, (px, py), (px+dx*SZ, py), HUD_MID, 2, AA)
        cv2.line(frame, (px, py), (px, py+dy*SZ), HUD_MID, 2, AA)
    # Reticle trung tam
    R_IN, GAP, LL = 12, 20, 26
    cv2.circle(frame, (cx, cy), R_IN, HUD_BRIGHT, 1, AA)
    cv2.line(frame, (cx, cy-GAP), (cx, cy-GAP-LL), HUD_BRIGHT, 2, AA)
    cv2.line(frame, (cx, cy+GAP), (cx, cy+GAP+LL), HUD_BRIGHT, 2, AA)
    cv2.line(frame, (cx-GAP, cy), (cx-GAP-LL, cy), HUD_BRIGHT, 2, AA)
    cv2.line(frame, (cx+GAP, cy), (cx+GAP+LL, cy), HUD_BRIGHT, 2, AA)
    cv2.circle(frame, (cx, cy), 2, HUD_WHITE, -1, AA)
    # Mil-dot
    for off in (-240, -160, -80, 80, 160, 240):
        h = 5 if off % 160 == 0 else 3
        cv2.line(frame, (cx+off, cy-h), (cx+off, cy+h), HUD_DIM, 1, AA)


def draw_tracking_line(frame, cx, cy, tx, ty):
    cv2.line(frame, (cx, cy), (int(tx), int(ty)), HUD_DIM, 1, cv2.LINE_AA)


def _draw_corner_bracket(frame, x1, y1, x2, y2, color, sz=16, thickness=2):
    AA = cv2.LINE_AA
    pts  = [(x1,y1), (x2,y1), (x1,y2), (x2,y2)]
    dirs = [(1,1), (-1,1), (1,-1), (-1,-1)]
    for (px, py), (dx, dy) in zip(pts, dirs):
        cv2.line(frame, (px, py), (px+dx*sz, py), color, thickness, AA)
        cv2.line(frame, (px, py), (px, py+dy*sz), color, thickness, AA)


def draw_bbox_idle(frame, box):
    """Muc tieu chua duoc track: net dut."""
    x1, y1, x2, y2 = box
    dash, gap = 8, 5
    for ax, ay, bx, by in [(x1,y1,x2,y1), (x2,y1,x2,y2),
                            (x2,y2,x1,y2), (x1,y2,x1,y1)]:
        total = int(math.hypot(bx-ax, by-ay))
        if total == 0:
            continue
        dx, dy = (bx-ax)/total, (by-ay)/total
        pos = 0
        while pos < total:
            p1 = (int(ax + dx*pos), int(ay + dy*pos))
            p2 = (int(ax + dx*min(pos+dash, total)),
                  int(ay + dy*min(pos+dash, total)))
            cv2.line(frame, p1, p2, HUD_DIM, 1, cv2.LINE_AA)
            pos += dash + gap


def draw_bbox_tracking(frame, box):
    x1, y1, x2, y2 = box
    _draw_corner_bracket(frame, x1, y1, x2, y2, HUD_BRIGHT, sz=18, thickness=2)
    _put_hud_text(frame, "TRACKING", x1, y1 - 6, HUD_BRIGHT, scale=0.40)


def draw_bbox_selected(frame, box, tid):
    x1, y1, x2, y2 = box
    _draw_corner_bracket(frame, x1, y1, x2, y2, HUD_LOCK, sz=22, thickness=2)
    pad = 4
    _draw_corner_bracket(frame, x1+pad, y1+pad, x2-pad, y2-pad,
                         HUD_LOCK, sz=10, thickness=1)
    _put_hud_text(frame, f"LOCKED  #{tid}", x1, y1 - 6,
                  HUD_LOCK, scale=0.42, thickness=1)


def draw_hud_overlay(frame, W, H, state, serial_ok, pan_deg, tilt_deg,
                     dist_cm, fps, n_targets, off_x=0, off_y=0,
                     selected_tid=None, target_xy=None):
    """Ve toan bo HUD overlay: top bar (sensor/title/seeker) + bottom bar."""
    AA = cv2.LINE_AA
    cv2.rectangle(frame, (0, 0), (W-1, H-1), HUD_DIM, 1)

    # ── Thanh ngang tren cung ────────────────────────────────────
    TOP_H = 110
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, 0), (W, TOP_H), (0, 20, 0), -1)
    cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)
    cv2.line(frame, (0, TOP_H), (W, TOP_H), HUD_MID, 1, AA)
    cv2.line(frame, (W//3, 0), (W//3, TOP_H), HUD_DIM, 1, AA)
    cv2.line(frame, (2*W//3, 0), (2*W//3, TOP_H), HUD_DIM, 1, AA)

    # TOP-LEFT: SENSOR DATA
    lx, ly = 12, 18
    _put_hud_text(frame, "SENSOR DATA", lx, ly, HUD_BRIGHT, 0.42, 1)
    cv2.line(frame, (lx, ly+3), (lx+110, ly+3), HUD_DIM, 1, AA)
    serial_str = "ONLINE" if serial_ok else "OFFLINE"
    serial_col = HUD_BRIGHT if serial_ok else HUD_WARN
    mode_col = {STATE_TRACKING: HUD_BRIGHT,
                STATE_RETURNING: (150,220,150),
                STATE_WAITING: (150,220,150)}.get(state, HUD_MID)
    for i, (txt, col) in enumerate([
        (f"Mode   : {state}", mode_col),
        (f"Serial : {serial_str}", serial_col),
        (f"FPS    : {fps:.0f}", HUD_MID),
        (f"Pan    : {pan_deg:+.1f} deg", HUD_MID),
        (f"Tilt   : {tilt_deg:+.1f} deg", HUD_MID),
    ]):
        _put_hud_text(frame, txt, lx, ly + 16*(i+1) + 2, col, 0.38)

    # TOP-CENTER: TITLE
    title = "TRACKING  SYSTEM"
    (tw, _), _ = cv2.getTextSize(title, FONT, 0.50, 1)
    _put_hud_text(frame, title, W//2 - tw//2, 22, HUD_BRIGHT, 0.50, 1)
    subtitle = ">>>>> Realtime Object Tracking <<<<<"
    (sw, _), _ = cv2.getTextSize(subtitle, FONT, 0.36, 1)
    _put_hud_text(frame, subtitle, W//2 - sw//2, 40, HUD_DIM, 0.36)
    _put_hud_text(frame, f"Targets detected : {n_targets}", W//2-70, 60, HUD_MID, 0.38)
    if state == STATE_TRACKING:
        err_col = HUD_BRIGHT if abs(off_x) < 50 and abs(off_y) < 50 else HUD_WARN
        _put_hud_text(frame, f"Error X: {off_x:+.0f}px   Y: {off_y:+.0f}px",
                      W//2-90, 78, err_col, 0.38)
    if selected_tid is not None:
        lock_txt = f"[ LOCKED  TARGET #{selected_tid} ]"
        (lw, _), _ = cv2.getTextSize(lock_txt, FONT, 0.45, 2)
        _put_hud_text(frame, lock_txt, W//2 - lw//2, 98, HUD_LOCK, 0.45, 1)

    # TOP-RIGHT: SEEKER DATA
    rx = 2*W//3 + 12
    _put_hud_text(frame, "SEEKER DATA", rx, 18, HUD_BRIGHT, 0.42, 1)
    cv2.line(frame, (rx, 21), (rx+110, 21), HUD_DIM, 1, AA)
    rows_right = [
        (f"Dist   : {dist_cm:.0f} cm", HUD_MID),
        (f"Gimbal : {pan_deg:+.0f} / {tilt_deg:+.0f}", HUD_MID),
    ]
    if target_xy:
        rows_right.insert(0, (f"Target XY: {target_xy[0]:.0f}, {target_xy[1]:.0f}", HUD_BRIGHT))
    for i, (txt, col) in enumerate(rows_right):
        _put_hud_text(frame, txt, rx, 18 + 16*(i+1) + 2, col, 0.38)

    # ── Thanh day ────────────────────────────────────────────────
    BOT_H = 44
    bot_y = H - BOT_H
    overlay2 = frame.copy()
    cv2.rectangle(overlay2, (0, bot_y), (W, H), (0, 20, 0), -1)
    cv2.addWeighted(overlay2, 0.55, frame, 0.45, 0, frame)
    cv2.line(frame, (0, bot_y), (W, bot_y), HUD_MID, 1, AA)
    mode_display = {
        STATE_IDLE: "STANDBY", STATE_TRACKING: "TRACKING",
        STATE_RETURNING: "RETURNING TO HOME",
        STATE_WAITING: "WAITING FOR TARGET",
    }.get(state, state)
    sm_txt = f"SENSOR MODE :  {mode_display}"
    (smw, _), _ = cv2.getTextSize(sm_txt, FONT, 0.46, 1)
    _put_hud_text(frame, sm_txt, W//2 - smw//2, bot_y + 16, HUD_BRIGHT, 0.46, 1)
    cmd = ("T=Track   H=Home   R=Return   U=Unlock   P=Pause   Q=Quit   Arrow=Manual   Click=Lock")
    _put_hud_text(frame, "COMMANDS: " + cmd, 12, bot_y + 32, HUD_MID, 0.42)
    cv2.rectangle(frame, (1, 1), (W-2, H-2), HUD_DIM, 1)


def draw_countdown(frame, W, H, miss_count):
    remaining = max(0, RETURN_TIMEOUT - miss_count)
    txt = f"TARGET LOST  —  RETURNING IN  {remaining}s"
    (tw, th), _ = cv2.getTextSize(txt, FONT, 0.65, 2)
    cx = (W - tw) // 2
    cy = H // 2 + 30
    overlay = frame.copy()
    cv2.rectangle(overlay, (cx-16, cy-th-12), (cx+tw+16, cy+12), (0, 10, 0), -1)
    cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)
    cv2.rectangle(frame, (cx-16, cy-th-12), (cx+tw+16, cy+12), HUD_MID, 1)
    _put_hud_text(frame, txt, cx, cy, HUD_WARN, 0.65, 2)
