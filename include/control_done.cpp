#include <Arduino.h>

// ===== Pins (Motor A on TB6612FNG) =====
const uint8_t PIN_PWMA = 5;     // PWM
const uint8_t PIN_AIN1 = 8;     // DIR1
const uint8_t PIN_AIN2 = 9;     // DIR2
const uint8_t PIN_STBY = 7;     // Enable (HIGH)

// ===== Encoder (Motor A) =====
const uint8_t PIN_ENC_A = 2;    // INT0 (D2)
const uint8_t PIN_ENC_B = 3;    // INT1 (D3)

// 若测得转向为负，可改为 -1 快速翻转方向
const int ENCODER_DIR = -1;     // +1 或 -1

// ===== Timing =====
unsigned long lastHB   = 0;               // heartbeat
unsigned long last10ms = 0;               // 10 ms 采样
unsigned long lastCtrl = 0;               // 50 ms 控制
const unsigned long SAMPLE10_DT_MS = 10;  // 速度采样窗口 10 ms
const unsigned long CONTROL_DT_MS  = 50;  // 控制/打印窗口 50 ms

// ===== Encoder count =====
volatile long enc_total = 0;              // x4解码累计脉冲(+/-)
long last_total_10ms = 0;

// ===== 启动前馈（open-loop boot） =====
float U_BOOT = 10.0f;                     // 启动占空比 (0..255)
unsigned long BOOT_MS = 300;              // 启动持续时间 (ms)
bool in_boot = true;                      // 启动阶段标志
unsigned long boot_until = 0;             // 启动截止时间

// ===== PI controller (增量式) =====
float KP = 0.0095f;
float KI = 0.002f;
float target_cps = 300.0f;                 // counts/s 目标(保持旧单位)
float e_prev = 0.0f;                       // 上一拍误差（增量式用）
const float UMAX = 255.0f;                 // PWM 最大
float cps_meas = 0.0f;                     // 50 ms 平均速度

// ===== 斜率限制（slew rate limit）=====
float DU_MAX_PER_SEC = 800.0f;            // 每秒最大变化量（PWM 数/秒）
float u_prev = 0.00f;                       // 上一拍实际下发的 PWM

// ===== x4 解码比例：回到旧单位 =====
const float ENC_SCALE = 1.0f / 4.0f;

// ===== 滑动平均缓冲区 (存5个10 ms值) =====
#define N_WIN 5
float win_buf[N_WIN] = {0};
uint8_t win_idx = 0;
uint8_t win_count = 0;

// ====== Encoder ISR (x4 解码) ======
void isrEncA() {
  bool A = digitalRead(PIN_ENC_A);
  bool B = digitalRead(PIN_ENC_B);
  // A 变化时：A==B 正向一步，A!=B 反向一步
  enc_total += ENCODER_DIR * ((A == B) ? +1 : -1);
}
void isrEncB() {
  bool A = digitalRead(PIN_ENC_A);
  bool B = digitalRead(PIN_ENC_B);
  // B 变化时：A!=B 正向一步，A==B 反向一步
  enc_total += ENCODER_DIR * ((A != B) ? +1 : -1);
}

// ===== Utility: set motor =====
void motor_set(float u) {
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

// ===== CLI =====
void serial_cli() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();  String low = cmd; low.toLowerCase();

  if (low.startsWith("set target")) {
    String num = cmd.substring(cmd.indexOf("set target")+10); num.trim();
    target_cps = num.toFloat();
    Serial.print(F("OK target_cps=")); Serial.println(target_cps,3);
  }
  else if (low.startsWith("set kp")) {
    String num = cmd.substring(cmd.indexOf("set kp")+6); num.trim();
    KP = num.toFloat();
    Serial.print(F("OK KP=")); Serial.println(KP,6);
  }
  else if (low.startsWith("set ki")) {
    String num = cmd.substring(cmd.indexOf("set ki")+6); num.trim();
    KI = num.toFloat();
    Serial.print(F("OK KI=")); Serial.println(KI,6);
  }
  else if (low.startsWith("set bootu")) {
    String num = cmd.substring(cmd.indexOf("set bootu")+9); num.trim();
    U_BOOT = num.toFloat();
    Serial.print(F("OK U_BOOT=")); Serial.println(U_BOOT,1);
  }
  else if (low.startsWith("set boott")) {
    String num = cmd.substring(cmd.indexOf("set boott")+9); num.trim();
    BOOT_MS = (unsigned long) num.toInt();
    Serial.print(F("OK BOOT_MS=")); Serial.println(BOOT_MS);
  }
  else if (low.startsWith("set dumax")) {
    String num = cmd.substring(cmd.indexOf("set dumax")+9); num.trim();
    DU_MAX_PER_SEC = num.toFloat();
    Serial.print(F("OK DU_MAX_PER_SEC=")); Serial.println(DU_MAX_PER_SEC,1);
  }
  else if (low.startsWith("status")) {
    Serial.print(F("STAT,t=")); Serial.print(millis());
    Serial.print(F(",boot="));  Serial.print(in_boot ? 1 : 0);
    Serial.print(F(",kp="));    Serial.print(KP,6);
    Serial.print(F(",ki="));    Serial.print(KI,6);
    Serial.print(F(",target="));Serial.print(target_cps,2);
    Serial.print(F(",cps="));   Serial.print(cps_meas,2);
    Serial.print(F(",e_prev="));Serial.print(e_prev,4);
    Serial.print(F(",dumax/s=")); Serial.println(DU_MAX_PER_SEC,1);
  }
  else {
    Serial.println(F("Unknown cmd. Use: 'set target <v>', 'set kp <v>', 'set ki <v>', 'set bootu <v>', 'set boott <ms>', 'set dumax <v>', 'status'"));
  }
}

void setup() {
  Serial.begin(57600);

  // Driver pins
  pinMode(PIN_PWMA, OUTPUT);
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, HIGH);

  // Encoder pins
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), isrEncA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), isrEncB, CHANGE);

  unsigned long t = millis();
  lastHB = last10ms = lastCtrl = t;

  // —— 启动前馈阶段：先给初始占空比，等 BOOT_MS 后切闭环 ——
  in_boot = true;
  boot_until = t + BOOT_MS;
  u_prev = U_BOOT;             // 斜率限制的初值 = 启动占空比
  motor_set(U_BOOT);

  // 初始化 10ms 参考计数
  noInterrupts();
  last_total_10ms = enc_total;
  interrupts();

  Serial.println(F("PI controller ready (x4 quad, boot->closed loop, slew limit, incremental PI)."));
  Serial.print(F("Init target=")); Serial.println(target_cps);
}

// ===== 10 ms 采样：推入滑动窗口 =====
void sample_10ms() {
  long total;
  noInterrupts(); total = enc_total; interrupts();

  long delta = total - last_total_10ms;
  last_total_10ms = total;

  float dt = SAMPLE10_DT_MS * 0.001f;
  // x4 解码后回到旧单位（1/4）
  float cps10 = (delta / dt) * ENC_SCALE;

  win_buf[win_idx] = cps10;
  win_idx = (win_idx + 1) % N_WIN;
  if (win_count < N_WIN) win_count++;

  // 计算 50ms 平均
  float sum = 0.0f;
  for (uint8_t i=0;i<win_count;i++) sum += win_buf[i];
  cps_meas = sum / win_count;
}

// ===== 50 ms PI 控制（增量式 + 斜率限制 + 饱和）=====
void control_step() {
  unsigned long now = millis();

  // 启动阶段：维持 U_BOOT，不做闭环；同时维持 u_prev
  if (in_boot) {
    if ((long)(now - boot_until) < 0) {
      u_prev = U_BOOT;
      motor_set(U_BOOT);
      return;
    } else {
      // 启动结束：切闭环（防跳变）
      in_boot = false;
      e_prev = target_cps - cps_meas;  // 以当前误差为起点
      // u_prev 保持为当前输出（U_BOOT），后续按增量式逐步调整
    }
  }

  // —— 增量式 PI ——
  float dt    = CONTROL_DT_MS * 0.001f;     // 0.05 s
  float e     = target_cps - cps_meas;      // 当前误差
 
  float du_pi = KP * e + KI * e * dt;      // PI 增量

  // 先在上一拍输出基础上加增量
  float u_cmd = u_prev + du_pi;

  // 幅值饱和（保护执行器）
  if (u_cmd >  UMAX) u_cmd =  UMAX;
  if (u_cmd < -UMAX) u_cmd = -UMAX;

  // 斜率限制：|u_k - u_{k-1}| <= DU_MAX_PER_SEC * dt
  float du_max = DU_MAX_PER_SEC * dt;
  if (u_cmd > u_prev + du_max) u_cmd = u_prev + du_max;
  if (u_cmd < u_prev - du_max) u_cmd = u_prev - du_max;

  // 更新误差记忆
  e_prev = e;

  // 下发并记录
  motor_set(u_cmd);
  u_prev = u_cmd;

  // 打印（50ms）
  Serial.print(F("CTL,"));
  Serial.print(now);         Serial.print(',');
  Serial.print(in_boot ? -1 : 1);  Serial.print(','); // -1=boot, 1=closed
  Serial.print(target_cps);  Serial.print(',');
  Serial.print(cps_meas,2);  Serial.print(',');
  Serial.print(KP,4);        Serial.print(',');
  Serial.print(KI,4);        Serial.print(',');
  Serial.print(e_prev,4);    Serial.print(',');
  Serial.println(u_prev,1);  // 当前实际PWM
}

void loop() {
  serial_cli();
  unsigned long now = millis();

  // 心跳（可关）
  if (now - lastHB >= 100) {
    lastHB = now;
    // Serial.print(F("HB t=")); Serial.print(now);
    // Serial.print(F(" boot=")); Serial.println(in_boot ? 1 : 0);
  }
  // 10 ms 采样
  if (now - last10ms >= SAMPLE10_DT_MS) {
    last10ms += SAMPLE10_DT_MS;
    sample_10ms();
  }
  // 50 ms 控制与打印
  if (now - lastCtrl >= CONTROL_DT_MS) {
    lastCtrl += CONTROL_DT_MS;
    control_step();
  }
}