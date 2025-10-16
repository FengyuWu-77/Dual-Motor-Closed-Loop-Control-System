#include <Arduino.h>
#include <math.h>   // fabsf()

/* ======================== Pins (Motor A) ======================== */
const uint8_t PIN_PWMA = 5;     // PWM (OC0B -> PD5)
const uint8_t PIN_AIN1 = 8;     // DIR1
const uint8_t PIN_AIN2 = 9;     // DIR2
const uint8_t PIN_STBY = 7;     // Enable (HIGH)

/* ======================== Pins (Motor B) ======================== */
const uint8_t PIN_PWMB = 10;    // PWM (OC1B -> PB2)
const uint8_t PIN_BIN1 = A0;    // DIR1
const uint8_t PIN_BIN2 = A1;    // DIR2

/* ======================== Encoders ============================== */
// Motor A encoder (外部中断)
const uint8_t PIN_ENC1_A = 2;   // INT0 (D2)
const uint8_t PIN_ENC1_B = 3;   // INT1 (D3)
// Motor B encoder 使用 Pin-Change 中断（PCINT20: D4, PCINT22: D6）
const uint8_t PIN_ENC2_A = 4;   // D4 (PD4 -> PCINT20)
const uint8_t PIN_ENC2_B = 6;   // D6 (PD6 -> PCINT22)

// 若测得转向为负，可改为 -1 快速翻转方向
const int ENCODER1_DIR = -1;    // +1 or -1
const int ENCODER2_DIR = -1;

/* ======================== Timing & control ====================== */
constexpr float   DT_CTRL_S         = 0.01f;   // 10 ms 固定步长（Timer2 触发）
constexpr uint16_t PRINT_PERIOD_MS  = 100;     // 打印 10 Hz
unsigned long lastPrint = 0;

/* ======================== Encoder totals ======================== */
volatile long enc1_total = 0;   // x4 解码累计脉冲(+/-)
volatile long enc2_total = 0;

/* ======================== Boot (open-loop) ====================== */
float U_BOOT = 20.0f;                     // 启动占空比 (0..255)
unsigned long BOOT_MS = 200;              // 启动持续时间 (ms)
volatile bool in_boot1 = true, in_boot2 = true;
volatile unsigned long boot1_until = 0, boot2_until = 0;

/* ======================== Shared PI ============================= */
float KP = 0.0018f;
float KI = 0.00003f;

/* 目标转速（counts/s） */
volatile float target1_cps = 200.0f;
volatile float target2_cps = 300.0f;

/* PI 状态（各自）*/
volatile float e1_prev = 0.0f, e2_prev = 0.0f;
volatile float u1_prev = 0.0f, u2_prev = 0.0f;

/* 限幅/限斜率 */
const float UMAX = 255.0f;               // PWM 上限
float DU_MAX_PER_SEC =400.0f;           // 每秒最大 PWM 变化量

/* x4 scale：把“每个沿”换回“完整四相周期”的计数 */
const float ENC_SCALE = 1.0f / 4.0f;

/* ======================== 5点滑动平均（每路各一套） ============ */
constexpr uint8_t SMA_N = 5;
float sma1_buf[SMA_N] = {0}, sma2_buf[SMA_N] = {0};
uint8_t sma1_idx = 0, sma2_idx = 0;
uint8_t sma1_cnt = 0, sma2_cnt = 0;
float sma1_sum = 0.0f, sma2_sum = 0.0f;

/* 最新平滑速度（供控制与打印） */
volatile float cps1_meas = 0.0f;
volatile float cps2_meas = 0.0f;

/* 上一次累计计数（用于10ms增量） */
volatile long last1_total = 0, last2_total = 0;

/* ======================== Sample 快照 =========================== */
struct Sample {
  uint32_t t;    // 时间戳(ms)
  float meas;    // 平滑速度(cps)
  float target;  // 斜坡后目标(cps)
  float u;       // 控制输出(PWM)
};
volatile Sample lastSampleA;
volatile Sample lastSampleB;

/* ======================== Soft Stop 参数/状态 =================== */
float SOFTSTOP_ERR_CPS        = 150.0f;   // 误差阈值（cps）
unsigned int SOFTSTOP_TIME_MS = 1500;      // 连续触发时间（ms）
float SOFTSTOP_RAMP_CPS_S     = 1200.0f;  // 软停斜坡（cps/s）→ 0
float SAT_MARGIN              = 5.0f;     // 认为“接近饱和”的裕度（PWM）
inline unsigned int ss_needed_ticks() { return (SOFTSTOP_TIME_MS + 9) / 10; }
volatile uint16_t satTicks1 = 0, satTicks2 = 0;
volatile bool softstop1 = false, softstop2 = false;

/* ============ 软停报警（一次性事件，ISR 只挂起，loop 打印） ==== */
struct SoftStopAlarm {
  uint32_t t;
  uint8_t  motor;   // 1 or 2
  float    target;
  float    meas;
  float    u;
};
volatile bool alarm_ss1_pending = false;
volatile bool alarm_ss2_pending = false;
volatile SoftStopAlarm alarm_ss1, alarm_ss2;

/* ======================== PCINT 解码（Motor B） ================== */
volatile uint8_t enc2_prev = 0;
const int8_t QDEC_TAB[16] = {
  0, +1, -1, 0,
  -1, 0,  0, +1,
  +1, 0,  0, -1,
   0, -1, +1, 0
};
inline uint8_t enc2_read_AB() {
  uint8_t pind = PIND;
  uint8_t A = (pind >> 4) & 0x1; // D4 -> PD4
  uint8_t B = (pind >> 6) & 0x1; // D6 -> PD6
  return (A << 1) | B;           // bit1=A, bit0=B
}
ISR(PCINT2_vect) {
  uint8_t nowAB = enc2_read_AB();
  uint8_t idx   = ((enc2_prev & 0x3) << 2) | (nowAB & 0x3);
  int8_t  step  = QDEC_TAB[idx];
  if (step != 0) {
    enc2_total += ENCODER2_DIR * step;
    enc2_prev = nowAB;
  }
}

/* ======================== Motor set ============================= */
void motorA_set(float u) {
  float uu = u;
  if (uu >  UMAX) uu =  UMAX;
  if (uu < -UMAX) uu = -UMAX;

  if (uu >= 0.0f) {
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);
    analogWrite(PIN_PWMA, (int)(uu + 0.5f));
  } else {
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, HIGH);
    analogWrite(PIN_PWMA, (int)(-uu + 0.5f));
  }
}
void motorB_set(float u) {
  float uu = u;
  if (uu >  UMAX) uu =  UMAX;
  if (uu < -UMAX) uu = -UMAX;

  if (uu >= 0.0f) {
    digitalWrite(PIN_BIN1, HIGH);
    digitalWrite(PIN_BIN2, LOW);
    analogWrite(PIN_PWMB, (int)(uu + 0.5f));
  } else {
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, HIGH);
    analogWrite(PIN_PWMB, (int)(-uu + 0.5f));
  }
}

/* ======================== 目标斜坡器 ============================ */
volatile float tgt1_cmd = 0.0f, tgt2_cmd = 0.0f; // 斜坡后的目标
float ramp_update(float target_in, float last_cmd, float rate_cps_per_s) {
  float max_step = rate_cps_per_s * DT_CTRL_S; // 每拍允许最大变化
  float diff = target_in - last_cmd;
  if (diff >  max_step) return last_cmd + max_step;
  if (diff < -max_step) return last_cmd - max_step;
  return target_in;
}

/* ======================== 控制器一步（增量式 PI） ================ */
/*float controller_step(float meas_cps, float target_cmd,
                      volatile float* e_prev, volatile float* u_prev) {
  float e  = target_cmd - meas_cps;
  float du = KP * e + KI * e * DT_CTRL_S;      // 增量式 PI
  float u_cmd = *u_prev + du;

  // 幅值饱和
  if (u_cmd >  UMAX) u_cmd =  UMAX;
  if (u_cmd < -UMAX) u_cmd = -UMAX;

  // 斜率限制
  float du_max = DU_MAX_PER_SEC * DT_CTRL_S;
  if (u_cmd > *u_prev + du_max) u_cmd = *u_prev + du_max;
  if (u_cmd < *u_prev - du_max) u_cmd = *u_prev - du_max;

  *e_prev = e;
  *u_prev = u_cmd;
  return u_cmd;
}*/
float controller_step(float meas_cps, float target_cmd,
                      volatile float* e_prev, volatile float* u_prev) {
  float e  = target_cmd - meas_cps;
  float du = KP * e + KI * e * DT_CTRL_S;      // 增量式 PI
  float u_cmd = *u_prev + du;

  // 幅值饱和
  if (u_cmd >  UMAX) u_cmd =  UMAX;
  if (u_cmd < -UMAX) u_cmd = -UMAX;

  // 斜率限制
  float du_max = DU_MAX_PER_SEC * DT_CTRL_S;
  if (u_cmd > *u_prev + du_max) u_cmd = *u_prev + du_max;
  if (u_cmd < *u_prev - du_max) u_cmd = *u_prev - du_max;

  *e_prev = e;
  *u_prev = u_cmd;
  return u_cmd;
}
/* ======================== CLI ============================= */
void process_line(const char* line) {
  String cmd = String(line);
  String low = cmd; low.toLowerCase(); cmd.trim();

  if (low.startsWith("set t1")) {
    String num = cmd.substring(low.indexOf("set t1")+6); num.trim();
    target1_cps = num.toFloat();
    e1_prev = 0.0f;
    Serial.print(F("OK t1=")); Serial.println(target1_cps,2);
  }
  else if (low.startsWith("set t2")) {
    String num = cmd.substring(low.indexOf("set t2")+6); num.trim();
    target2_cps = num.toFloat();
    e2_prev = 0.0f;
    Serial.print(F("OK t2=")); Serial.println(target2_cps,2);
  }
  else if (low.startsWith("set kp")) {
    String num = cmd.substring(low.indexOf("set kp")+6); num.trim();
    KP = num.toFloat();
    e1_prev = e2_prev = 0.0f;
    Serial.print(F("OK KP=")); Serial.println(KP,6);
  }
  else if (low.startsWith("set ki")) {
    String num = cmd.substring(low.indexOf("set ki")+6); num.trim();
    KI = num.toFloat();
    e1_prev = e2_prev = 0.0f;
    Serial.print(F("OK KI=")); Serial.println(KI,6);
  }
  else if (low.startsWith("set bootu")) {
    String num = cmd.substring(low.indexOf("set bootu")+9); num.trim();
    U_BOOT = num.toFloat();
    Serial.print(F("OK U_BOOT=")); Serial.println(U_BOOT,1);
  }
  else if (low.startsWith("set boott")) {
    String num = cmd.substring(low.indexOf("set boott")+9); num.trim();
    BOOT_MS = (unsigned long) num.toInt();
    Serial.print(F("OK BOOT_MS=")); Serial.println(BOOT_MS);
  }
  else if (low.startsWith("set dumax")) {
    String num = cmd.substring(low.indexOf("set dumax")+9); num.trim();
    DU_MAX_PER_SEC = num.toFloat();
    Serial.print(F("OK DU_MAX/s=")); Serial.println(DU_MAX_PER_SEC,1);
  }
  // SoftStop 参数与清除
  else if (low.startsWith("set ss_err")) {
    String num = cmd.substring(low.indexOf("set ss_err")+10); num.trim();
    SOFTSTOP_ERR_CPS = num.toFloat();
    Serial.print(F("OK SS_ERR_CPS=")); Serial.println(SOFTSTOP_ERR_CPS,1);
  }
  else if (low.startsWith("set ss_time")) {
    String num = cmd.substring(low.indexOf("set ss_time")+11); num.trim();
    SOFTSTOP_TIME_MS = (unsigned int) num.toInt();
    Serial.print(F("OK SS_TIME_MS=")); Serial.println(SOFTSTOP_TIME_MS);
  }
  else if (low.startsWith("set ss_ramp")) {
    String num = cmd.substring(low.indexOf("set ss_ramp")+11); num.trim();
    SOFTSTOP_RAMP_CPS_S = num.toFloat();
    Serial.print(F("OK SS_RAMP=")); Serial.println(SOFTSTOP_RAMP_CPS_S,1);
  }
  else if (low.startsWith("set sat_margin")) {
    String num = cmd.substring(low.indexOf("set sat_margin")+14); num.trim();
    SAT_MARGIN = num.toFloat();
    Serial.print(F("OK SAT_MARGIN=")); Serial.println(SAT_MARGIN,1);
  }
  else if (low.startsWith("ss clear")) {
    softstop1 = softstop2 = false; satTicks1 = satTicks2 = 0;
    Serial.println(F("OK softstop cleared (both)"));
  }
  else if (low.startsWith("ss1 clear")) {
    softstop1 = false; satTicks1 = 0;
    Serial.println(F("OK softstop1 cleared"));
  }
  else if (low.startsWith("ss2 clear")) {
    softstop2 = false; satTicks2 = 0;
    Serial.println(F("OK softstop2 cleared"));
  }
  else if (low.startsWith("status")) {
    noInterrupts();
    float c1 = cps1_meas, c2 = cps2_meas;
    float t1 = tgt1_cmd,  t2 = tgt2_cmd;
    bool b1  = in_boot1,  b2 = in_boot2;
    interrupts();
    Serial.print(F("STAT,t=")); Serial.print(millis());
    Serial.print(F(",boot1="));  Serial.print(b1 ? 1 : 0);
    Serial.print(F(",boot2="));  Serial.print(b2 ? 1 : 0);
    Serial.print(F(",kp="));     Serial.print(KP,6);
    Serial.print(F(",ki="));     Serial.print(KI,6);
    Serial.print(F(",t1="));     Serial.print(t1,2);
    Serial.print(F(",c1="));     Serial.print(c1,2);
    Serial.print(F(",t2="));     Serial.print(t2,2);
    Serial.print(F(",c2="));     Serial.println(c2,2);
  }
  else if (low.length()) {
    Serial.println(F("Unknown. Use: 'set t1 <v>', 'set t2 <v>', 'set kp <v>', 'set ki <v>', 'set bootu <v>', 'set boott <ms>', 'set dumax <v>', 'set ss_err <cps>', 'set ss_time <ms>', 'set ss_ramp <cps/s>', 'set sat_margin <pwm>', 'ss clear|ss1 clear|ss2 clear', 'status'"));
  }
}

void serial_cli_nonblock() {
  static char buf[64];
  static uint8_t idx = 0;
  static unsigned long last_rx = 0;
  unsigned long now = millis();

  while (Serial.available()) {
    char c = (char)Serial.read();
    last_rx = now;
    if (c == '\r' || c == '\n') {
      if (idx > 0) { buf[idx] = '\0'; process_line(buf); idx = 0; }
      continue;
    }
    if (idx < sizeof(buf) - 1) buf[idx++] = c;
    else { buf[idx] = '\0'; process_line(buf); idx = 0; }
  }
  if (idx > 0 && (now - last_rx) > 200) { buf[idx] = '\0'; process_line(buf); idx = 0; }
}

/* ======================== 外部中断（Motor A） ==================== */
void isrEnc1A() {
  bool A = digitalRead(PIN_ENC1_A);
  bool B = digitalRead(PIN_ENC1_B);
  enc1_total += ENCODER1_DIR * ((A == B) ? +1 : -1);
}
void isrEnc1B() {
  bool A = digitalRead(PIN_ENC1_A);
  bool B = digitalRead(PIN_ENC1_B);
  enc1_total += ENCODER1_DIR * ((A != B) ? +1 : -1);
}

/* ======================== Timer2: 10 ms CTC ===================== */
void setup_timer2_10ms() {
  // CTC 模式
  TCCR2A = 0;
  TCCR2B = 0;
  TCCR2A |= (1 << WGM21);            // CTC
  // 预分频 1024：16MHz/1024 = 15625 Hz，10ms ≈ 156 计数
  OCR2A = 155;                        // ≈9.93ms，误差<1%
  TCCR2B |= (1 << CS22) | (1 << CS20);// CS22=1,CS21=0,CS20=1 -> 1024
  TIMSK2 |= (1 << OCIE2A);            // 允许比较匹配中断
}

/* ======================== Setup ============================ */
void setup() {
  Serial.begin(57600);
  Serial.setTimeout(5);

  // Driver pins
  pinMode(PIN_PWMA, OUTPUT);
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_PWMB, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, HIGH);

  // Encoder pins
  pinMode(PIN_ENC1_A, INPUT_PULLUP);
  pinMode(PIN_ENC1_B, INPUT_PULLUP);
  pinMode(PIN_ENC2_A, INPUT_PULLUP);
  pinMode(PIN_ENC2_B, INPUT_PULLUP);

  // Motor A：外部中断
  attachInterrupt(digitalPinToInterrupt(PIN_ENC1_A), isrEnc1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC1_B), isrEnc1B, CHANGE);

  // Motor B：PCINT2 组
  enc2_prev = enc2_read_AB();
  PCICR  |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT20) | (1 << PCINT22);

  // 启动前馈
  unsigned long t = millis();
  in_boot1 = in_boot2 = true;
  boot1_until = t + BOOT_MS;
  boot2_until = t + BOOT_MS;
  u1_prev = U_BOOT; motorA_set(U_BOOT);
  u2_prev = U_BOOT; motorB_set(U_BOOT);

  // 初始化累计计数
  noInterrupts();
  last1_total = enc1_total;
  last2_total = enc2_total;
  interrupts();

  // 初始化斜坡目标为当前目标
  tgt1_cmd = target1_cps;
  tgt2_cmd = target2_cps;

  // 启动 Timer2：10ms 控制节拍
  setup_timer2_10ms();

  Serial.println(F("Dual-motor PI (10ms ISR + 5pt SMA + SoftStop alarms; 10Hz print; lastSampleA/B)."));
}

/* ======================== 10 ms 控制 ISR ======================== */
ISR(TIMER2_COMPA_vect, ISR_NOBLOCK) {
  // 1) 瞬时速度
  long t1 = enc1_total;   // 在本 ISR 中其他中断屏蔽，读取是原子的
  long t2 = enc2_total;

  long d1 = t1 - last1_total; last1_total = t1;
  long d2 = t2 - last2_total; last2_total = t2;

  float cps1_inst = (d1 / DT_CTRL_S) * ENC_SCALE;
  float cps2_inst = (d2 / DT_CTRL_S) * ENC_SCALE;

  // 2) 5 点滑动平均（O(1) 更新）
  // A
  float old1 = (sma1_cnt < SMA_N) ? 0.0f : sma1_buf[sma1_idx];
  sma1_sum += (cps1_inst - old1);
  sma1_buf[sma1_idx] = cps1_inst;
  sma1_idx = (sma1_idx + 1) % SMA_N;
  if (sma1_cnt < SMA_N) sma1_cnt++;
  cps1_meas = sma1_sum / (float)sma1_cnt;

  // B
  float old2 = (sma2_cnt < SMA_N) ? 0.0f : sma2_buf[sma2_idx];
  sma2_sum += (cps2_inst - old2);
  sma2_buf[sma2_idx] = cps2_inst;
  sma2_idx = (sma2_idx + 1) % SMA_N;
  if (sma2_cnt < SMA_N) sma2_cnt++;
  cps2_meas = sma2_sum / (float)sma2_cnt;

  // 3) 统一缓存一次时间戳
  uint32_t t_ms = millis();

  // 4) 目标斜坡（正常跟随目标）
  constexpr float RAMP_CPS_PER_S = 800.0f;
  tgt1_cmd = ramp_update(target1_cps, tgt1_cmd, RAMP_CPS_PER_S);
  tgt2_cmd = ramp_update(target2_cps, tgt2_cmd, RAMP_CPS_PER_S);

  // 5) Soft Stop 触发判据（非启动期才计数），fabsf()
  if (!in_boot1) {
    bool satA   = (fabsf(u1_prev) >= (UMAX - SAT_MARGIN));
    bool bigErr = (fabsf(tgt1_cmd - cps1_meas) >= SOFTSTOP_ERR_CPS);
    if (!softstop1 && satA && bigErr) {
      if (satTicks1 < 0xFFFF) satTicks1++;
      if (satTicks1 >= ss_needed_ticks()) {
        softstop1 = true;
        // 上升沿：挂起一次报警（不在ISR打印）
        alarm_ss1.t      = t_ms;
        alarm_ss1.motor  = 1;
        alarm_ss1.target = tgt1_cmd;
        alarm_ss1.meas   = cps1_meas;
        alarm_ss1.u      = u1_prev;
        alarm_ss1_pending = true;
      }
    } else {
      if (satTicks1 > 0) satTicks1--; // 抗抖衰减
    }
  } else {
    satTicks1 = 0;
  }

  if (!in_boot2) {
    bool satB   = (fabsf(u2_prev) >= (UMAX - SAT_MARGIN));
    bool bigErr = (fabsf(tgt2_cmd - cps2_meas) >= SOFTSTOP_ERR_CPS);
    if (!softstop2 && satB && bigErr) {
      if (satTicks2 < 0xFFFF) satTicks2++;
      if (satTicks2 >= ss_needed_ticks()) {
        softstop2 = true;
        alarm_ss2.t      = t_ms;
        alarm_ss2.motor  = 2;
        alarm_ss2.target = tgt2_cmd;
        alarm_ss2.meas   = cps2_meas;
        alarm_ss2.u      = u2_prev;
        alarm_ss2_pending = true;
      }
    } else {
      if (satTicks2 > 0) satTicks2--;
    }
  } else {
    satTicks2 = 0;
  }

  // 6) 执行软停：把目标斜坡向 0 收敛
  if (softstop1) {
    tgt1_cmd = ramp_update(0.0f, tgt1_cmd, SOFTSTOP_RAMP_CPS_S);
  }
  if (softstop2) {
    tgt2_cmd = ramp_update(0.0f, tgt2_cmd, SOFTSTOP_RAMP_CPS_S);
  }

  // 7) 启动期 or 闭环控制
  // A
  if (in_boot1 && (long)(t_ms - boot1_until) < 0) {
    motorA_set(U_BOOT); u1_prev = U_BOOT;
  } else {
    if (in_boot1) { in_boot1 = false; e1_prev = tgt1_cmd - cps1_meas; u1_prev = U_BOOT; }
    float u1 = controller_step(cps1_meas, tgt1_cmd, &e1_prev, &u1_prev);
    motorA_set(u1);
  }

  // B
  if (in_boot2 && (long)(t_ms - boot2_until) < 0) {
    motorB_set(U_BOOT); u2_prev = U_BOOT;
  } else {
    if (in_boot2) { in_boot2 = false; e2_prev = tgt2_cmd - cps2_meas; u2_prev = U_BOOT; }
    float u2 = controller_step(cps2_meas, tgt2_cmd, &e2_prev, &u2_prev);
    motorB_set(u2);
  }

  // 8) 快照（逐字段写入 volatile）
  lastSampleA.t      = t_ms;
  lastSampleA.meas   = cps1_meas;
  lastSampleA.target = tgt1_cmd;
  lastSampleA.u      = u1_prev;

  lastSampleB.t      = t_ms;
  lastSampleB.meas   = cps2_meas;
  lastSampleB.target = tgt2_cmd;
  lastSampleB.u      = u2_prev;
}

/* ======================== Loop ============================= */
void loop() {
  serial_cli_nonblock();

  unsigned long now = millis();
  if (now - lastPrint >= PRINT_PERIOD_MS) {
    lastPrint = now;

    Sample sA, sB;
    noInterrupts();
    sA.t      = lastSampleA.t;
    sA.meas   = lastSampleA.meas;
    sA.target = lastSampleA.target;
    sA.u      = lastSampleA.u;

    sB.t      = lastSampleB.t;
    sB.meas   = lastSampleB.meas;
    sB.target = lastSampleB.target;
    sB.u      = lastSampleB.u;
    interrupts();

    // CSV：CTL,t_ms,t1,c1,u1,t2,c2,u2,kp,ki
    Serial.print(F("CTL,"));
    Serial.print(sA.t);         Serial.print(',');
    Serial.print(sA.target,1);  Serial.print(',');
    Serial.print(sA.meas,1);    Serial.print(',');
    Serial.print(sA.u,0);       Serial.print(',');
    Serial.print(sB.target,1);  Serial.print(',');
    Serial.print(sB.meas,1);    Serial.print(',');
    Serial.print(sB.u,0);       Serial.print(',');
    Serial.print(KP,4);         Serial.print(',');
    Serial.println(KI,5);
  }

  // 事件驱动报警（一次性，不在 ISR 打印）
if (alarm_ss1_pending || alarm_ss2_pending) {
  SoftStopAlarm s1, s2;
  bool p1, p2;

  noInterrupts();
  p1 = alarm_ss1_pending;
  p2 = alarm_ss2_pending;

  // 从 volatile 源逐字段拷贝到本地
  if (p1) {
    s1.t      = alarm_ss1.t;
    s1.motor  = alarm_ss1.motor;
    s1.target = alarm_ss1.target;
    s1.meas   = alarm_ss1.meas;
    s1.u      = alarm_ss1.u;
    alarm_ss1_pending = false;
  }
  if (p2) {
    s2.t      = alarm_ss2.t;
    s2.motor  = alarm_ss2.motor;
    s2.target = alarm_ss2.target;
    s2.meas   = alarm_ss2.meas;
    s2.u      = alarm_ss2.u;
    alarm_ss2_pending = false;
  }
  interrupts();

  if (p1) {
    Serial.print(F("ALARM,code=SOFTSTOP, motor=1, t="));
    Serial.print(s1.t);
    Serial.print(F(", target=")); Serial.print(s1.target,1);
    Serial.print(F(", meas="));   Serial.print(s1.meas,1);
    Serial.print(F(", u="));      Serial.println(s1.u,0);
  }
  if (p2) {
    Serial.print(F("ALARM,code=SOFTSTOP, motor=2, t="));
    Serial.print(s2.t);
    Serial.print(F(", target=")); Serial.print(s2.target,1);
    Serial.print(F(", meas="));   Serial.print(s2.meas,1);
    Serial.print(F(", u="));      Serial.println(s2.u,0);
  }
}
}