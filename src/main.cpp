#include <Arduino.h>

/* ======================== Pins (Motor A) ======================== */
const uint8_t PIN_PWMA = 5;     // PWM (OC0B -> PD5)
const uint8_t PIN_AIN1 = 8;     // DIR1
const uint8_t PIN_AIN2 = 9;     // DIR2
const uint8_t PIN_STBY = 7;     // Enable (HIGH)

/* ======================== Pins (Motor B) ======================== */
const uint8_t PIN_PWMB = 10;    // PWM (OC1B -> PB2)  *** PB2 as requested ***
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

/* ======================== Timing ================================ */
unsigned long lastHB   = 0;               // heartbeat
unsigned long last10ms = 0;               // 10 ms 采样触发参考
unsigned long lastCtrl = 0;               // 100 ms 控制触发参考
const unsigned long SAMPLE10_DT_MS = 10;  // 速度采样名义窗口 10 ms
const unsigned long CONTROL_DT_MS  = 100; // 控制/打印窗口 100 ms

/* ======================== Encoder counts ======================== */
volatile long enc1_total = 0;   // x4 解码累计脉冲(+/-)
volatile long enc2_total = 0;
long last1_total_10ms = 0;
long last2_total_10ms = 0;

/* ======================== Boot (open-loop) ====================== */
float U_BOOT = 20.0f;                     // 启动占空比 (0..255)
unsigned long BOOT_MS = 200;              // 启动持续时间 (ms)
// 两路各自的 boot 状态
bool in_boot1 = true, in_boot2 = true;
unsigned long boot1_until = 0, boot2_until = 0;

/* ======================== Shared PI ============================= */
float KP = 0.014f;
float KI = 0.0009f;

/* 目标转速（counts/s） */
float target1_cps = 600.0f;
float target2_cps = 900.0f;

/* 上一拍误差（各自）*/
float e1_prev = 0.0f;
float e2_prev = 0.0f;

const float UMAX = 255.0f;     // PWM 上限

/* 50ms 平均速度（各自）*/
float cps1_meas = 0.0f;
float cps2_meas = 0.0f;

/* ======================== Slew limit ============================ */
float DU_MAX_PER_SEC = 800.0f; // 每秒最大变化量（PWM 数/秒）
float u1_prev = 0.0f;
float u2_prev = 0.0f;

/* ======================== x4 scale ============================== */
const float ENC_SCALE = 1.0f / 4.0f;

/* ======================== 滑动平均窗口 ========================== */
#define N_WIN 10
float win1_buf[N_WIN] = {0}, win2_buf[N_WIN] = {0};
uint8_t win1_idx = 0, win2_idx = 0;
uint8_t win1_cnt = 0, win2_cnt = 0;

/* ======================== Encoder ISRs (Motor A, ext INT) ======= */
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

/* ======================== Motor B encoder via PCINT ============== */
/* x4 解码查表：prev(AB) -> now(AB) */
volatile uint8_t enc2_prev = 0;
const int8_t QDEC_TAB[16] = {
  0, +1, -1, 0,
  -1, 0,  0, +1,
  +1, 0,  0, -1,
   0, -1, +1, 0
};
// 读取 D4/D6 的 A/B 位
inline uint8_t enc2_read_AB() {
  uint8_t pind = PIND;
  uint8_t A = (pind >> 4) & 0x1; // D4 -> PD4
  uint8_t B = (pind >> 6) & 0x1; // D6 -> PD6
  return (A << 1) | B;           // bit1=A, bit0=B
}
// PORTD 的 PCINT (PCINT[23:16])
ISR(PCINT2_vect) {
  uint8_t nowAB = enc2_read_AB();
  uint8_t idx   = ((enc2_prev & 0x3) << 2) | (nowAB & 0x3);
  int8_t  step  = QDEC_TAB[idx];          // -1 / 0 / +1
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

/* ======================== CLI ============================= */
void process_line(const char* line) {
  String cmd = String(line);
  String low = cmd; low.toLowerCase(); cmd.trim();

  // targets per motor
  if (low.startsWith("set t1")) {
    String num = cmd.substring(low.indexOf("set t1")+6); num.trim();
    target1_cps = num.toFloat();
    e1_prev = target1_cps - cps1_meas;
    Serial.print(F("OK t1=")); Serial.println(target1_cps,2);
  }
  else if (low.startsWith("set t2")) {
    String num = cmd.substring(low.indexOf("set t2")+6); num.trim();
    target2_cps = num.toFloat();
    e2_prev = target2_cps - cps2_meas;
    Serial.print(F("OK t2=")); Serial.println(target2_cps,2);
  }
  // shared PI
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
  else if (low.startsWith("status")) {
    Serial.print(F("STAT,t=")); Serial.print(millis());
    Serial.print(F(",boot1="));  Serial.print(in_boot1 ? 1 : 0);
    Serial.print(F(",boot2="));  Serial.print(in_boot2 ? 1 : 0);
    Serial.print(F(",kp="));     Serial.print(KP,6);
    Serial.print(F(",ki="));     Serial.print(KI,6);
    Serial.print(F(",t1="));     Serial.print(target1_cps,2);
    Serial.print(F(",c1="));     Serial.print(cps1_meas,2);
    Serial.print(F(",t2="));     Serial.print(target2_cps,2);
    Serial.print(F(",c2="));     Serial.println(cps2_meas,2);
  }
  else if (low.length()) {
    Serial.println(F("Unknown. Use: 'set t1 <v>', 'set t2 <v>', 'set kp <v>', 'set ki <v>', 'set bootu <v>', 'set boott <ms>', 'set dumax <v>', 'status'"));
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

  // Motor B：PCINT2 组 (PORTD: PCINT16..23)，仅关心 D4(PCINT20) 与 D6(PCINT22)
  // 初始化 enc2_prev
  enc2_prev = enc2_read_AB();
  // 允许 PCINT2 组
  PCICR  |= (1 << PCIE2);
  // 使能 PCINT20 与 PCINT22
  PCMSK2 |= (1 << PCINT20) | (1 << PCINT22);

  unsigned long t = millis();
  lastHB = last10ms = lastCtrl = t;

  // —— 两路启动前馈：先给初始占空比，等 BOOT_MS 后切闭环 ——
  in_boot1 = in_boot2 = true;
  boot1_until = t + BOOT_MS;
  boot2_until = t + BOOT_MS;
  u1_prev = U_BOOT; motorA_set(U_BOOT);
  u2_prev = U_BOOT; motorB_set(U_BOOT);

  // 初始化 10ms 参考计数
  noInterrupts();
  last1_total_10ms = enc1_total;
  last2_total_10ms = enc2_total;
  interrupts();

  Serial.println(F("Dual-motor PI ready (x4 quad; A: ext-INT, B: PCINT; non-blocking CLI; boot->closed loop; slew limit; KP/KI shared)."));
}

/* ======================== 10 ms sampling =================== */
void sample_maybe_10ms() {
  unsigned long now = millis();
  if (now - last10ms < SAMPLE10_DT_MS) return;

  unsigned long dt_ms = now - last10ms;
  last10ms = now;

  long total1, total2;
  noInterrupts(); total1 = enc1_total; total2 = enc2_total; interrupts();

  long d1 = total1 - last1_total_10ms; last1_total_10ms = total1;
  long d2 = total2 - last2_total_10ms; last2_total_10ms = total2;

  // 异常大 dt：丢弃本次并清窗口
  if (dt_ms > 100) {
    win1_idx = win2_idx = 0;
    win1_cnt = win2_cnt = 0;
    for (uint8_t i=0;i<N_WIN;i++){ win1_buf[i]=0; win2_buf[i]=0; }
    cps1_meas = cps2_meas = 0.0f;
    return;
  }

  float dt = dt_ms * 0.001f;
  float cps1 = (d1 / dt) * ENC_SCALE;
  float cps2 = (d2 / dt) * ENC_SCALE;

  // 滑动平均
  win1_buf[win1_idx] = cps1; win1_idx = (win1_idx + 1) % N_WIN; if (win1_cnt < N_WIN) win1_cnt++;
  win2_buf[win2_idx] = cps2; win2_idx = (win2_idx + 1) % N_WIN; if (win2_cnt < N_WIN) win2_cnt++;

  float sum1 = 0, sum2 = 0;
  for (uint8_t i=0;i<win1_cnt;i++) sum1 += win1_buf[i];
  for (uint8_t i=0;i<win2_cnt;i++) sum2 += win2_buf[i];

  cps1_meas = (win1_cnt>0) ? (sum1 / win1_cnt) : 0.0f;
  cps2_meas = (win2_cnt>0) ? (sum2 / win2_cnt) : 0.0f;
}

/* ======================== 100 ms control =================== */
void control_one(bool &in_boot, unsigned long boot_until,
                 float target, float &cps_meas,
                 float &e_prev, float &u_prev,
                 void (*motor_set_fn)(float),
                 int /*boot_flag*/, float &u_applied_out)
{
  unsigned long now = millis();

  // 启动阶段：维持 U_BOOT，不做闭环
  if (in_boot && (long)(now - boot_until) < 0) {
    u_prev = U_BOOT;
    motor_set_fn(U_BOOT);
    u_applied_out = u_prev;
    return;
  } else if (in_boot) {
    // 启动结束：切闭环（防跳变）
    in_boot = false;
    e_prev = target - cps_meas;
    // u_prev 保持为当前输出（U_BOOT）
  }

  // 增量式 PI
  float dt = CONTROL_DT_MS * 0.001f; // 0.1 s
  float e  = target - cps_meas;
  float du_pi = KP * e + KI * e * dt;

  float u_cmd = u_prev + du_pi;

  // 幅值饱和
  if (u_cmd >  UMAX) u_cmd =  UMAX;
  if (u_cmd < -UMAX) u_cmd = -UMAX;

  // 斜率限制
  float du_max = DU_MAX_PER_SEC * dt;
  if (u_cmd > u_prev + du_max) u_cmd = u_prev + du_max;
  if (u_cmd < u_prev - du_max) u_cmd = u_prev - du_max;

  e_prev = e;
  motor_set_fn(u_cmd);
  u_prev = u_cmd;
  u_applied_out = u_cmd;
}

void control_maybe_100ms() {
  unsigned long now = millis();
  if (now - lastCtrl < CONTROL_DT_MS) return;
  lastCtrl += CONTROL_DT_MS;   // 尽量等周期

  float u1 = 0.0f, u2 = 0.0f;
  control_one(in_boot1, boot1_until, target1_cps, cps1_meas, e1_prev, u1_prev, motorA_set, -1, u1);
  control_one(in_boot2, boot2_until, target2_cps, cps2_meas, e2_prev, u2_prev, motorB_set, -1, u2);

  // 简洁输出：CTL,t,t1,c1,u1,t2,c2,u2,kp,ki
  Serial.print(F("CTL,"));
  Serial.print(now);            Serial.print(',');
  Serial.print(target1_cps,1);  Serial.print(',');
  Serial.print(cps1_meas,1);    Serial.print(',');
  Serial.print(u1,0);           Serial.print(',');
  Serial.print(target2_cps,1);  Serial.print(',');
  Serial.print(cps2_meas,1);    Serial.print(',');
  Serial.print(u2,0);           Serial.print(',');
  Serial.print(KP,4);           Serial.print(',');
  Serial.println(KI,4);
}

/* ======================== Loop ============================= */
void loop() {
  serial_cli_nonblock();
  sample_maybe_10ms();
  control_maybe_100ms();

  unsigned long now = millis();
  if (now - lastHB >= 500) {
    lastHB = now;
    // 可选心跳
    // Serial.print(F("HB t=")); Serial.println(now);
  }
}