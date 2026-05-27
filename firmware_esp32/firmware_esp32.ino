// ============================================================================
// HỆ THỐNG NHẬN DIỆN VÀ THEO DÕI BÀN TAY (HAND TRACKING)
// FIRMWARE ĐIỀU KHIỂN GIMBAL PAN-TILT BẰNG ESP32
//
// Phiên bản: v2.1 (Tối ưu hóa hiệu năng)
// Tính năng chính:
//  - Sử dụng Timer Interrupt 50µs để băm xung cho 2 động cơ độc lập.
//  - Tách biệt Task UART để không làm gián đoạn việc phát xung.
//  - Hỗ trợ 2 chế độ: Chạy thủ công (có accel/decel) và Tracking PID (tốc độ cố định).
//  - Tích hợp Watchdog Timer: Tự động dừng khẩn cấp nếu mất tín hiệu từ PC.
// ============================================================================

#include <math.h>

// ════════════════════════════════════════════════════════════
// PIN CONFIGURATION (Cấu hình chân kết nối ESP32 với Driver)
// ════════════════════════════════════════════════════════════
#define STEP_PAN    18  // Chân tạo xung bước cho trục Pan (trái/phải)
#define DIR_PAN     19  // Chân điều khiển hướng cho trục Pan
#define STEP_TILT   21  // Chân tạo xung bước cho trục Tilt (lên/xuống)
#define DIR_TILT    22  // Chân điều khiển hướng cho trục Tilt
#define EN_PIN       4  // Chân Enable chung cho cả 2 driver (LOW là bật)

// ════════════════════════════════════════════════════════════
// THÔNG SỐ CƠ KHÍ & BĂM XUNG (MOTOR SETTINGS)
// ════════════════════════════════════════════════════════════
#define STEPS_PER_REV      1600   // Cấu hình vi bước (Microstepping) trên driver (VD: 1/8 bước)
#define TIMER_TICK_US      50     // Chu kỳ gọi ngắt Timer (50 micro-giây)

// Tỷ số truyền bánh răng: 
// - Trục Pan: Bánh răng lớn 80 răng, nhỏ 21 răng -> Tỷ số = 80/21
// - Trục Tilt: Bánh răng lớn 60 răng, nhỏ 21 răng -> Tỷ số = 60/21
// Công thức: (STEPS_PER_REV * Tỷ_số_truyền) / 360 độ
#define PAN_STEPS_PER_DEG  16.931f   
#define TILT_STEPS_PER_DEG 12.698f   

// ── MOTION PROFILE (Đường cong gia tốc cho chế độ Thủ công/M/G) ──
#define DEF_START_DELAY  800    // (µs) Thời gian trễ ban đầu giữa 2 xung (Vận tốc thấp nhất lúc khởi động)
#define DEF_MIN_DELAY    280    // (µs) Thời gian trễ nhỏ nhất giữa 2 xung (Vận tốc cao nhất đạt được)
#define DEF_ACCEL        4      // (µs/step) Gia tốc: Mỗi bước giảm trễ đi bao nhiêu µs

// ── TÍNH NĂNG BẢO VỆ (SAFETY) ──
#define IDLE_TIMEOUT_MS  3000   // (ms) Nếu 3 giây không có lệnh nào + motor đang dừng -> Tắt Enable Driver để chống nóng
#define WATCHDOG_MS      2000   // (ms) Nếu 2 giây không nhận được lệnh từ PC (PC treo/Cáp đứt) -> Dừng motor khẩn cấp

// ════════════════════════════════════════════════════════════
// STRUCT MOTOR (Cấu trúc dữ liệu quản lý từng động cơ độc lập)
// ════════════════════════════════════════════════════════════
struct Motor {
  uint8_t stepPin, dirPin;       // Chân GPIO điều khiển
  volatile int32_t currentPos;   // Vị trí hiện tại (tính bằng số xung/bước)
  volatile int32_t stepsRemaining;// Số xung còn lại cần xuất ra
  volatile int32_t accelCount;   // Biến đếm phục vụ logic tăng/giảm tốc hình thang
  volatile int32_t currentDelay; // Độ trễ hiện tại giữa 2 lần đảo trạng thái chân STEP (µs)
  volatile int32_t usAccum;      // Bộ tích lũy thời gian từ Timer (cứ 50µs cộng thêm 1 lần)
  volatile bool    stepState;    // Trạng thái hiện tại của chân STEP (HIGH/LOW)
  volatile bool    dir;          // Chiều quay hiện tại (để cộng trừ currentPos cho đúng)
  volatile bool    tracking;     // Cờ chế độ: TRUE = Chế độ Tracking từ PC (tốc độ cố định, không gia tốc), FALSE = Chế độ chạy bằng tay (có gia tốc)
  
  int32_t startDelay, minDelay, accelStep; // Lưu trữ cấu hình giới hạn tốc độ và gia tốc
  float   stepsPerDeg;           // Hệ số chuyển đổi từ Độ sang Xung
  float   accumDeg;              // Bộ nhớ đệm giữ phần thập phân của tọa độ (chống sai số tích lũy khi làm tròn)
};

// Khởi tạo 2 biến toàn cục cho 2 trục
Motor pan  = { STEP_PAN,  DIR_PAN,  0,0,0, DEF_START_DELAY,0,false,true,false,
               DEF_START_DELAY, DEF_MIN_DELAY, DEF_ACCEL, PAN_STEPS_PER_DEG, 0.0f };
Motor tilt = { STEP_TILT, DIR_TILT, 0,0,0, DEF_START_DELAY,0,false,true,false,
               DEF_START_DELAY, DEF_MIN_DELAY, DEF_ACCEL, TILT_STEPS_PER_DEG, 0.0f };

// ════════════════════════════════════════════════════════════
// BIẾN TOÀN CỤC KHÁC
// ════════════════════════════════════════════════════════════
hw_timer_t       *gTimer      = NULL; // Con trỏ tới Timer phần cứng
unsigned long     lastCmdTime = 0;    // Lần cuối cùng nhận được lệnh (dùng cho Watchdog và Auto-Sleep)
bool              driverOn    = true; // Trạng thái chân Enable
volatile bool     prevMoving  = false;// Lưu trạng thái đang di chuyển của chu kỳ trước (để in ra chữ DONE khi dừng)

// ════════════════════════════════════════════════════════════
// TIMER INTERRUPT SERVICE ROUTINE (ISR) - HÀM NGẮT TIMER
// Chạy đều đặn mỗi 50µs ở mức ưu tiên cực cao.
// KHÔNG BAO GIỜ DÙNG Serial.print hay Delay trong hàm này!
// ════════════════════════════════════════════════════════════
static inline void IRAM_ATTR tickMotor(Motor &m) {
  if (m.stepsRemaining <= 0) return; // Nếu không còn bước nào cần đi thì bỏ qua
  
  m.usAccum += TIMER_TICK_US; // Tích lũy thêm 50µs
  
  // Logic băm xung vuông:
  // - Nếu chân STEP đang LOW: Đợi đến khi đủ currentDelay thì kéo HIGH.
  // - Nếu chân STEP đang HIGH: Kéo ngay về LOW, tính là hoàn thành 1 bước, tính toán lại Delay cho bước sau.
  if (!m.stepState) {
    if (m.usAccum >= m.currentDelay) {
      m.usAccum   = 0;
      m.stepState = true;
      digitalWrite(m.stepPin, HIGH); // Phát nửa chu kỳ xung (Cạnh lên)
    }
  } else {
    m.stepState = false;
    digitalWrite(m.stepPin, LOW);    // Kết thúc nửa chu kỳ xung (Cạnh xuống)
    m.stepsRemaining--;              // Đã đi xong 1 bước
    m.currentPos += m.dir ? 1 : -1;  // Cập nhật vị trí tuyệt đối

    // Chế độ TRACKING (do AI PC điều khiển):
    // PC gửi lệnh liên tục 50Hz, tự tính toán mượt mà rồi nên ESP32 KHÔNG cần tự tạo gia tốc.
    if (m.tracking) return;

    // Chế độ MANUAL/GOTO (Chạy thủ công bằng tay / Quay về Home):
    // Cần tạo gia tốc (Accel) hình thang để động cơ không bị giật, rớt bước.
    if (m.stepsRemaining > m.accelCount && m.currentDelay > m.minDelay) {
      // Đang tăng tốc (Giảm Delay)
      m.accelCount++;
      m.currentDelay -= m.accelStep;
      if (m.currentDelay < m.minDelay) m.currentDelay = m.minDelay;
    } else if (m.stepsRemaining <= m.accelCount) {
      // Đang giảm tốc (Tăng Delay) để chuẩn bị dừng
      m.accelCount--;
      m.currentDelay += m.accelStep;
      if (m.currentDelay > m.startDelay) m.currentDelay = m.startDelay;
    }
  }
}

// Hàm gói của ISR
void IRAM_ATTR onTimer() {
  tickMotor(pan);
  tickMotor(tilt);
}

// Khởi tạo ngắt Timer (Base clock 80MHz)
void setupTimer() {
  gTimer = timerBegin(1000000); // Đặt tần số Timer là 1MHz (1 tick = 1µs)
  timerAttachInterrupt(gTimer, &onTimer); // Gắn hàm ISR
  timerAlarm(gTimer, TIMER_TICK_US, true, 0); // Báo thức mỗi 50µs, lặp lại liên tục
}

// ════════════════════════════════════════════════════════════
// CÁC HÀM XỬ LÝ LỆNH ĐỘNG CƠ TỪ GIAO THỨC UART
// ════════════════════════════════════════════════════════════

// Lệnh GOTO/MANUAL (Di chuyển đến vị trí cụ thể với profile gia tốc)
void startMove(Motor &m, int32_t targetPos) {
  int32_t delta = targetPos - m.currentPos;
  if (delta == 0) {
    m.stepsRemaining = 0;
    return;
  }

  bool newDir = (delta > 0);

  // Nếu đang chạy cùng chiều bằng tay, chỉ cần cộng dồn số bước mới
  if (m.stepsRemaining > 0 && m.dir == newDir && !m.tracking) {
    m.stepsRemaining = abs(delta);
    return;
  }

  // Khởi tạo chuyến đi mới
  m.stepsRemaining = 0;
  delayMicroseconds(TIMER_TICK_US * 2);
  m.dir = newDir;
  digitalWrite(m.dirPin, m.dir ? HIGH : LOW);
  m.currentDelay   = m.startDelay; // Bắt đầu từ vận tốc chậm
  m.accelCount     = 0;
  m.usAccum        = 0;
  m.stepState      = false;
  m.tracking       = false;        // Đánh dấu là chạy có gia tốc
  m.stepsRemaining = abs(delta);
}

// Lệnh TRACKING 'A' (Do AI Python điều khiển, gửi ở tần số 50Hz)
void startMoveTracking(Motor &m, int32_t targetPos) {
  int32_t delta    = targetPos - m.currentPos;
  int32_t absSteps = abs(delta);

  // Bỏ qua nhiễu siêu nhỏ: nếu lệnh yêu cầu quay ít hơn 2 bước thì bỏ qua.
  if (absSteps < 2) return;

  // Giới hạn an toàn (Clamp): Không cho chạy quá 110 bước mỗi 20ms để tránh rớt bước.
  if (absSteps > 110) absSteps = 110;

  bool newDir = (delta > 0);

  // ── Tính toán Vận tốc thích ứng ──
  // Nếu sai số nhỏ: Quay rất chậm để chỉnh tinh chính xác.
  // Nếu sai số lớn: Quay nhanh để đuổi kịp bàn tay.
  int32_t trackDelay;
  if (absSteps <= 10) {
      trackDelay = 450;          // Độ trễ lớn -> Tốc độ chậm
  } else if (absSteps <= 40) {
      trackDelay = 450 - (absSteps - 10) * 170 / 30;  // Tuyến tính tăng tốc độ
  } else {
      trackDelay = 280;          // Tốc độ tối đa giới hạn
      if (trackDelay < m.minDelay) trackDelay = m.minDelay;
  }

  // Bù tỷ số truyền bánh răng: 
  // Bánh răng trục Pan to hơn nên cần phát xung nhanh hơn để quay kịp trục Tilt
  float speedScale = TILT_STEPS_PER_DEG / m.stepsPerDeg; 
  trackDelay = (int32_t)(trackDelay * speedScale);
  if (trackDelay < m.minDelay) trackDelay = m.minDelay;

  // Xử lý Blend mượt mà nếu đang chạy cùng chiều (Làm mượt bằng trung bình cộng)
  if (m.stepsRemaining > 0 && m.dir == newDir && m.tracking) {
    if (absSteps > m.stepsRemaining) {
      m.stepsRemaining = absSteps;
    }
    m.currentDelay = (m.currentDelay * 2 + trackDelay) / 3;
    return;
  }

  // Soft-brake (Phanh mềm): 
  // Nếu tay đổi hướng đột ngột, ép động cơ hãm lại một chút trước khi đảo chiều.
  if (m.stepsRemaining > 8) {
    m.stepsRemaining = 8;
    delayMicroseconds(TIMER_TICK_US * 4);
  } else {
    m.stepsRemaining = 0;
    delayMicroseconds(TIMER_TICK_US * 2);
  }
  
  // Áp dụng lệnh mới
  m.dir = newDir;
  digitalWrite(m.dirPin, m.dir ? HIGH : LOW);
  m.currentDelay   = trackDelay;
  m.accelCount     = 0;
  m.usAccum        = 0;
  m.stepState      = false;
  m.tracking       = true;       // Đánh dấu là chạy không gia tốc của ESP
  m.stepsRemaining = absSteps;
}

// ── Bật/Tắt IC Driver ──
void enableDrv() {
  if (!driverOn) {
    digitalWrite(EN_PIN, LOW); // LOW là Bật
    driverOn = true;
    delayMicroseconds(500);    // Chờ IC xả điện tĩnh
  }
}

void disableDrv() {
  if (driverOn) {
    digitalWrite(EN_PIN, HIGH); // HIGH là Tắt
    driverOn = false;
  }
}

// ════════════════════════════════════════════════════════════
// BỘ PHÂN TÍCH LỆNH TỪ SERIAL (PARSER)
// ════════════════════════════════════════════════════════════
void processCmd(const char *cmd) {
  // Loại bỏ khoảng trắng ở đầu
  while (*cmd == ' ' || *cmd == '\t') cmd++;
  if (*cmd == '\0') return;

  char c = cmd[0];         // Ký tự phân loại lệnh (Ví dụ: 'A', 'M', 'G')
  const char *p = cmd + 1; // Phần tham số phía sau
  while (*p == ' ') p++;   

  switch (c) {

    // Lệnh M: Chạy bằng tay (Manual) thêm 1 khoảng tương đối
    // Cú pháp: M [delta_pan_steps] [delta_tilt_steps]
    case 'M': case 'm': {
      const char *sp = strchr(p, ' ');
      if (!sp) { Serial.println("ERR:M"); return; }
      int32_t ps = atol(p);
      int32_t ts = atol(sp + 1);
      enableDrv(); lastCmdTime = millis();
      startMove(pan,  pan.currentPos  + ps);
      startMove(tilt, tilt.currentPos + ts);
      Serial.printf("OK M %.2f %.2f\n",
        pan.currentPos/PAN_STEPS_PER_DEG,
        tilt.currentPos/TILT_STEPS_PER_DEG);
      break;
    }

    // Lệnh A: Lệnh Tracking từ AI PC (Gửi góc lẻ delta degrees ở tần số cao 50Hz)
    // Cú pháp: A [delta_pan_deg] [delta_tilt_deg]
    case 'A': case 'a': {
      const char *sp = strchr(p, ' ');
      if (!sp) return;
      float pd = atof(p);
      float td = atof(sp + 1);
      enableDrv(); lastCmdTime = millis();

      // Cộng dồn vào biến tích lũy để tránh mất số lẻ (Ví dụ 0.1 độ * 10 lần = 1 độ)
      pan.accumDeg  += pd;
      tilt.accumDeg += td;

      // Chuyển đổi phần nguyên sang số bước nguyên (Steps)
      int32_t ps = (int32_t)roundf(pan.accumDeg  * PAN_STEPS_PER_DEG);
      int32_t ts = (int32_t)roundf(tilt.accumDeg * TILT_STEPS_PER_DEG);

      if (abs(ps) >= 2) {
        startMoveTracking(pan, pan.currentPos + ps);
        pan.accumDeg = 0.0f; // Reset sau khi đã dùng
      }
      if (abs(ts) >= 2) {
        startMoveTracking(tilt, tilt.currentPos + ts);
        tilt.accumDeg = 0.0f;
      }
      break;
    }

    // Lệnh G: Tới tọa độ tuyệt đối (GOTO - Dùng để về Home)
    // Cú pháp: G [pan_deg_target] [tilt_deg_target]
    case 'G': case 'g': {
      const char *sp = strchr(p, ' ');
      if (!sp) { Serial.println("ERR:G"); return; }
      float pd = atof(p);
      float td = atof(sp + 1);
      enableDrv(); lastCmdTime = millis();
      startMove(pan,  (int32_t)roundf(pd * PAN_STEPS_PER_DEG));
      startMove(tilt, (int32_t)roundf(td * TILT_STEPS_PER_DEG));
      Serial.printf("OK G %.2f %.2f\n", pd, td);
      break;
    }

    // Lệnh H: Đặt vị trí hiện tại làm gốc tọa độ (0,0)
    case 'H': case 'h':
      pan.stepsRemaining  = 0;
      tilt.stepsRemaining = 0;
      delayMicroseconds(TIMER_TICK_US * 3);
      pan.currentPos  = 0;
      tilt.currentPos = 0;
      Serial.println("OK H 0.00 0.00");
      break;

    // Lệnh E: Bật / Tắt Driver thủ công
    case 'E': case 'e':
      if (atoi(p)) { enableDrv();  Serial.println("OK E ON");  }
      else         { disableDrv(); Serial.println("OK E OFF"); }
      break;

    // Lệnh X: Dừng khẩn cấp
    case 'X': case 'x':
      pan.stepsRemaining  = 0;
      tilt.stepsRemaining = 0;
      Serial.println("OK X STOP");
      break;

    default:
      // Bỏ qua các lệnh không xác định
      break;
  }
}

// ════════════════════════════════════════════════════════════
// UART TASK (Core 1) — Độc lập hoàn toàn với Timer Ngắt
// ════════════════════════════════════════════════════════════
void uartTask(void *pv) {
  char          buf[64];
  uint8_t       bufIdx  = 0;
  unsigned long lastPOS = 0;

  while (true) {
    // 1. Đọc từng byte từ Buffer Serial và ghép thành chuỗi lệnh hoàn chỉnh
    while (Serial.available()) {
      char ch = Serial.read();
      if (ch == '\n' || ch == '\r') {
        if (bufIdx > 0) {
          buf[bufIdx] = '\0'; // Đóng chuỗi
          processCmd(buf);    // Đem đi xử lý
          bufIdx = 0;
        }
      } else {
        if (bufIdx < sizeof(buf) - 1) buf[bufIdx++] = ch;
      }
    }

    // 2. Phát hiện sự kiện Motor vừa dừng lại để gửi chữ "DONE" về Python (Dùng cho giao diện)
    bool mv = (pan.stepsRemaining > 0 || tilt.stepsRemaining > 0);
    if (prevMoving && !mv) {
      Serial.printf("DONE %.2f %.2f\n",
        pan.currentPos  / PAN_STEPS_PER_DEG,
        tilt.currentPos / TILT_STEPS_PER_DEG);
    }
    prevMoving = mv;

    // 3. WATCHDOG TIMER: Tự vệ khi mất kết nối PC
    // Nếu quá 2 giây không nhận được lệnh điều khiển mà hệ thống đang chạy tracking -> Cắt động cơ.
    if (mv && (pan.tracking || tilt.tracking) &&
        (millis() - lastCmdTime > WATCHDOG_MS)) {
      pan.stepsRemaining  = 0;
      tilt.stepsRemaining = 0;
      Serial.println("WATCHDOG: No cmd 2s, stopped");
    }

    // 4. FEEDBACK: Gửi tọa độ thực tế về PC mỗi 200ms
    if (millis() - lastPOS > 200) {
      Serial.printf("POS %.2f %.2f %s\n",
        pan.currentPos  / PAN_STEPS_PER_DEG,
        tilt.currentPos / TILT_STEPS_PER_DEG,
        mv ? "MOVING" : "IDLE");
      lastPOS = millis();
    }

    // Nhường CPU 3ms cho FreeRTOS dọn rác (Watchdog Core 1)
    vTaskDelay(pdMS_TO_TICKS(3));  
  }
}

// ════════════════════════════════════════════════════════════
// HÀM KHỞI TẠO (SETUP)
// ════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);

  // Setup I/O Motor
  pinMode(STEP_PAN,  OUTPUT); pinMode(DIR_PAN,  OUTPUT);
  pinMode(STEP_TILT, OUTPUT); pinMode(DIR_TILT, OUTPUT);
  pinMode(EN_PIN,    OUTPUT);
  digitalWrite(EN_PIN, LOW); // Mặc định bật driver

  // Khởi động Timer tạo xung
  setupTimer();

  // Khởi chạy Task UART trên Core 1 riêng biệt để giảm nghẽn
  xTaskCreatePinnedToCore(uartTask, "uart", 4096, NULL, 2, NULL, 1);

  Serial.println("READY");
}

// ════════════════════════════════════════════════════════════
// HÀM LẶP CHÍNH (LOOP)
// ════════════════════════════════════════════════════════════
void loop() {
  // Logic tiết kiệm điện (Auto Sleep):
  // Nếu động cơ đã đứng yên (không còn bước nào) và thời gian rảnh lớn hơn IDLE_TIMEOUT (3 giây)
  // -> Tắt Driver để giảm sinh nhiệt cho Động cơ và Mạch.
  if (driverOn &&
      (millis() - lastCmdTime > IDLE_TIMEOUT_MS) &&
      pan.stepsRemaining  == 0 &&
      tilt.stepsRemaining == 0) {
    disableDrv();
    Serial.println("SLEEP"); // PC nhận được chữ SLEEP sẽ biết ESP đang nghỉ
  }
  
  delay(100); // Loop chính không làm gì cả, mọi việc đã có ngắt Timer và Task Uart lo.
}
