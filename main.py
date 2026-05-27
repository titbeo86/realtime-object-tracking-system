
"""
main.py — Realtime Object Tracking System
RTX 3050 | Webcam 720p 30fps | YOLO + ByteTrack + Kalman + PID Gimbal
(Phien ban toi uu hoa - Cau truc OOP)
"""

from app import GimbalApp

def main():
    config = {
        "serial_port":   "COM9",
        "serial_baud":   115200,
        "serial_enable": True,
        
        "model_path":    "C:/hand_detection/best_v2.pt",
        "conf":          0.25,
        "tracker":       "bytetrack_hand.yaml",
        "max_targets":   5,
        
        "webcam_id":     0,
        "width":         1280,
        "height":        720,
        
        "window_name":   "Realtime Object Tracking System"
    }

    # Khoi tao va chay ung dung
    app = GimbalApp(config)
    app.run()


if __name__ == "__main__":
    main()