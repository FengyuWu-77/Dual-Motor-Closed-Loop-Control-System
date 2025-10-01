#include <Arduino.h>

// ===== Pins (Motor A on TB6612FNG) =====
const uint8_t PIN_PWMA = 5;     // PWM
const uint8_t PIN_AIN1 = 8;     // DIR1
const uint8_t PIN_AIN2 = 9;     // DIR2
const uint8_t PIN_STBY = 7;     // Enable (HIGH)

// ===== Encoder (Motor A) =====
const uint8_t PIN_ENC_A = 2;    // INT0
const uint8_t PIN_ENC_B = 3;    // INT1   <-- 另一通道也用外部中断

// ===== Timing =====
unsigned long lastHB   = 0;               // heartbeat 100 ms
unsigned long last10ms = 0;               // 10 ms 采样速度
unsigned long lastCtrl = 0;               // 50 ms 控制与串口
const unsigned long SAMPLE10_DT_MS = 10;  // 速度采样窗口 10 ms
const unsigned long CONTROL_DT_MS  = 50;  // 控制/打印窗口 50 ms

// ===== Encoder count =====
volatile long enc_total = 0;              // x4解码累计脉冲(+/-)
long last_total_10ms = 0;

// ===== Open-loop boot =====
const uint8_t PWM_BOOT = 0;                // 启动时 0 占空比

// ===== PI controller =====
float KP = 0.04f;
float KI = 0.00f;
float target_cps = 300.0f;                 // counts/s 目标(保持旧单位)
float integ_e = 0.0f;
const float UMAX = 255.0f;                 // PWM 最大
float cps_meas = 0.0f;                     // 50 ms 平均速度

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
  enc_total += (A == B) ? +1 : -1;
}
void isrEncB() {
  bool A = digitalRead(PIN_ENC_A);
  bool B = digitalRead(PIN_ENC_B);
  enc_total += (A != B) ? +1 : -1;
}

// ===== Utility: set motor =====
void motor_set(float u) {
  if (u >  UMAX) u =  UMAX;
  if (u < -UMAX) u = -UMAX;

  if (u >= 0.0f) {
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);
    analogWrite(PIN_PWMA, (int)(u + 0.5f));
  } else {
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, HIGH);
    analogWrite(PIN_PWMA, (int)(-u + 0.5f));
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
  else if (low.startsWith("status")) {
    Serial.print(F("STAT,t=")); Serial.print(millis());
    Serial.print(F(",kp="));     Serial.print(KP,6);
    Serial.print(F(",ki="));     Serial.print(KI,6);
    Serial.print(F(",target=")); Serial.print(target_cps,2);
    Serial.print(F(",cps="));    Serial.print(cps_meas,2);
    Serial.print(F(",integ="));  Serial.println(integ_e,4);
  }
  else {
    Serial.println(F("Unknown cmd. Use: 'set target <v>', 'set kp <v>', 'set ki <v>', 'status'"));
  }
}

void setup() {
  Serial.begin(57600);
  pinMode(PIN_PWMA, OUTPUT);
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, HIGH);
  motor_set(0);

  // Encoder pins
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), isrEncA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), isrEncB, CHANGE);

  lastHB = last10ms = lastCtrl = millis();
  Serial.println(F("PI controller ready. Commands: set target / set kp / set ki / status"));
  Serial.print(F("Init target=")); Serial.println(target_cps);
  noInterrupts();
  last_total_10ms = enc_total;  // 初始化10ms参考计数
  interrupts();
}

// ===== 10 ms 采样：推入滑动窗口 =====
void sample_10ms() {
  long total;
  noInterrupts(); total = enc_total; interrupts();

  long delta = total - last_total_10ms;
  last_total_10ms = total;

  float dt = SAMPLE10_DT_MS * 0.001f;
  float cps10 = (delta / dt) * ENC_SCALE;      // 转回旧单位

  win_buf[win_idx] = cps10;
  win_idx = (win_idx + 1) % N_WIN;
  if (win_count < N_WIN) win_count++;

  // 计算平均
  float sum = 0.0f;
  for (uint8_t i=0;i<win_count;i++) sum += win_buf[i];
  cps_meas = sum / win_count;
}

// ===== 50 ms PI 控制 =====
void control_step() {
  float e = target_cps - cps_meas;
  float dt = CONTROL_DT_MS * 0.001f;

  float integ_next = integ_e + e * dt;
  float u_unsat = KP * e + KI * integ_next;

  float u = u_unsat;
  if (u >  UMAX) u =  UMAX;
  if (u < -UMAX) u = -UMAX;

  bool pushing_high = (u >=  UMAX-1e-6) && (e>0.0f);
  bool pushing_low  = (u <= -UMAX+1e-6) && (e<0.0f);
  if (!(pushing_high || pushing_low)) integ_e = integ_next;

  motor_set(u);
}

void loop() {
  serial_cli();
  unsigned long now = millis();

  // 心跳
  if (now - lastHB >= 100) {
    lastHB = now;
    Serial.print(F("HB t=")); Serial.println(now);
  }
  // 10 ms 采样
  if (now - last10ms >= SAMPLE10_DT_MS) {
    last10ms = now;
    sample_10ms();
  }
  // 50 ms 控制与打印
  if (now - lastCtrl >= CONTROL_DT_MS) {
    lastCtrl = now;
    control_step();
    Serial.print(F("CTL,"));
    Serial.print(now);         Serial.print(',');
    Serial.print(target_cps);  Serial.print(',');
    Serial.print(cps_meas,2);  Serial.print(',');
    Serial.print(KP,4);        Serial.print(',');
    Serial.print(KI,4);        Serial.print(',');
    Serial.println(integ_e,4);
  }
}