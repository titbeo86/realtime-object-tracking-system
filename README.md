# 🎯 Realtime Object Tracking System (AI Gimbal Controller)

Hệ thống giám sát và theo dõi vật thể thời gian thực sử dụng **YOLO**, **ByteTrack**, **Kalman Filter** và bộ điều khiển **PID** qua mạch **ESP32**. 

Dự án này biến một chiếc Webcam bình thường gắn trên Gimbal (Pan/Tilt) thành một "con mắt AI" có khả năng tự động khóa mục tiêu và quay bám theo vật thể cực kỳ mượt mà.

---

## ✨ Tính năng nổi bật
- **Nhận diện siêu tốc (Real-time):** Sử dụng mô hình YOLO (tối ưu hóa qua file `best_v2.pt`).
- **Theo dõi chính xác (Tracking):** Tích hợp thuật toán ByteTrack kết hợp Re-ID giúp duy trì khóa mục tiêu ngay cả khi vật thể bị che khuất tạm thời.
- **Dự đoán quỹ đạo thông minh:** Bộ lọc Kalman (Kalman Filter) giúp hệ thống đoán trước hướng đi của vật thể, triệt tiêu độ trễ.
- **Điều khiển Gimbal mượt mà:** Thuật toán PID (Proportional-Integral-Derivative) gửi góc quay chuẩn xác xuống ESP32, đảm bảo camera luôn giữ vật thể ở giữa khung hình.
- **Giao diện HUD quân sự:** Hiển thị thông số cảm biến, tâm ngắm, sai số X/Y và FPS trực tiếp trên màn hình giống như một hệ thống radar.

---

## 🛠️ Cấu trúc phần cứng (Hardware)
- Mạch xử lý trung tâm: **ESP32** (Code nạp nằm trong thư mục `firmware_esp32`).
- Cơ cấu chấp hành: Gimbal Pan/Tilt sử dụng **2 động cơ stepper** .
- Camera: Webcam tiêu chuẩn (720p).
- Máy tính chạy AI: Máy tính có card đồ họa (Khuyên dùng RTX 3050 trở lên để đạt tốc độ > 30 FPS).

---

## 💻 Hướng dẫn cài đặt & Chạy dự án

### 1. Nạp code cho ESP32
Mở file `firmware_esp32/firmware_esp32.ino` bằng phần mềm Arduino IDE và nạp vào mạch ESP32. Nhớ kiểm tra lại cổng kết nối mạch với động cơ.

### 2. Cài đặt thư viện Python
Cài đặt môi trường ảo và các thư viện cần thiết (YOLO, OpenCV, Pyserial...):
```bash
pip install ultralytics opencv-python pyserial numpy
```

### 3. Chạy chương trình
Cắm Webcam, kết nối ESP32 vào máy tính (Cập nhật lại cổng COM trong code nếu cần) và chạy lệnh:
```bash
python main.py
```

### 🎮 Phím tắt điều khiển:
- Phím `T`: Bắt đầu khóa và theo dõi mục tiêu gần tâm nhất.
- Phím `H`: Đặt vị trí hiện tại làm vị trí gốc (Home).
- Phím `R`: Ra lệnh cho Gimbal quay về vị trí Home.
- Phím `Q`: Thoát chương trình.

---

## 📝 Tác giả
- Phát triển bởi: **titbeo86**
- Liên hệ: lethai0806205@gmail.com
