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
unsigned long last10ms = 0;               // 10 ms 采样触发参考
unsigned long lastCtrl = 0;               // 50 ms 控制触发参考
const unsigned long SAMPLE10_DT_MS = 10;  // 速度采样名义窗口 10 ms
const unsigned long CONTROL_DT_MS  = 100;  // 控制/打印窗口 50 ms

// ===== Encoder count =====
volatile long enc_total = 0;              // x4 解码累计脉冲(+/-)
long last_total_10ms = 0;

// ===== 启动前馈（open-loop boot） =====
float U_BOOT = 20.0f;                     // 启动占空比 (0..255)
unsigned long BOOT_MS = 200;              // 启动持续时间 (ms)
bool in_boot = true;                      // 启动阶段标志
unsigned long boot_until = 0;             // 启动截止时间

// ===== PI controller (增量式) =====
float KP = 0.014f;
float KI = 0.0009f;
float target_cps = 300.0f;                 // counts/s 目标(保持旧单位)
float e_prev = 0.0f;                       // 上一拍误差（增量式用）
const float UMAX = 255.0f;                 // PWM 最大
float cps_meas = 0.0f;                     // 50 ms 平均速度

// ===== 斜率限制（slew rate limit）=====
float DU_MAX_PER_SEC = 800.0f;            // 每秒最大变化量（PWM 数/秒）
float u_prev = 0.0f;                       // 上一拍实际下发的 PWM

// ===== x4 解码比例：回到旧单位 =====
const float ENC_SCALE = 1.0f / 4.0f;

// ===== 滑动平均缓冲区 (存5个10 ms值) =====
#define N_WIN 10
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

// ====== 命令处理（非阻塞）======
void process_line(const char* line) {
  String cmd = String(line);
  String low = cmd; low.toLowerCase(); cmd.trim();

  if (low.startsWith("set target")) {
    String num = cmd.substring(low.indexOf("set target")+10); num.trim();
    target_cps = num.toFloat();
    // 软切换：以当前误差为起点，避免突变
    e_prev = target_cps - cps_meas;
    Serial.print(F("OK target_cps=")); Serial.println(target_cps,3);
  }
  else if (low.startsWith("set kp")) {
    String num = cmd.substring(low.indexOf("set kp")+6); num.trim();
    KP = num.toFloat();
    e_prev = 0.0f; // 改增益后清误差记忆，避免瞬时抖动
    Serial.print(F("OK KP=")); Serial.println(KP,6);
  }
  else if (low.startsWith("set ki")) {
    String num = cmd.substring(low.indexOf("set ki")+6); num.trim();
    KI = num.toFloat();
    e_prev = 0.0f;
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
  else if (low.length()) {
    Serial.println(F("Unknown cmd. Use: 'set target <v>', 'set kp <v>', 'set ki <v>', 'set bootu <v>', 'set boott <ms>', 'set dumax <v>', 'status'"));
  }
}

void serial_cli_nonblock() {
  static char buf[64];
  static uint8_t idx = 0;
  static unsigned long last_rx = 0;
  unsigned long now = millis();

  // 读所有可用字符
  while (Serial.available()) {
    char c = (char)Serial.read();
    last_rx = now;

    // 统一把 \r 和 \n 都当作“结束”
    if (c == '\r' || c == '\n') {
      if (idx > 0) {
        buf[idx] = '\0';
        process_line(buf);
        idx = 0;
      }
      continue;
    }

    if (idx < sizeof(buf) - 1) {
      buf[idx++] = c;
    } else {
      // 缓冲满了也提交一次，避免卡死
      buf[idx] = '\0';
      process_line(buf);
      idx = 0;
    }
  }

  // 超时自动提交（例如监视器不发换行）
  if (idx > 0 && (now - last_rx) > 200) {
    buf[idx] = '\0';
    process_line(buf);
    idx = 0;
  }
}
void setup() {
  Serial.begin(57600);
  Serial.setTimeout(5); // 防止任何阻塞式读被长时间卡住

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

  Serial.println(F("PI controller ready (x4 quad, non-blocking CLI, robust sampling, boot->closed loop, slew limit, incremental PI)."));
  Serial.print(F("Init target=")); Serial.println(target_cps);
}

// ===== 10 ms 采样：用真实 dt，并处理大 dt 的异常 =====
void sample_maybe_10ms() {
  unsigned long now = millis();
  if (now - last10ms < SAMPLE10_DT_MS) return;

  unsigned long dt_ms = now - last10ms;
  last10ms = now;

  long total;
  noInterrupts(); total = enc_total; interrupts();

  long delta = total - last_total_10ms;
  last_total_10ms = total;

  // 如果 dt 异常大（例如串口/其他占用导致），丢弃这次样本并重置平滑窗口，避免巨峰值
  if (dt_ms > 100) {
    win_idx = 0; win_count = 0;
    for (uint8_t i=0;i<N_WIN;i++) win_buf[i] = 0;
    cps_meas = 0.0f;
    return;
  }

  float dt = dt_ms * 0.001f;
  float cps = (delta / dt) * ENC_SCALE;

  // 推入窗口并求平均
  win_buf[win_idx] = cps;
  win_idx = (win_idx + 1) % N_WIN;
  if (win_count < N_WIN) win_count++;

  float sum = 0.0f;
  for (uint8_t i=0;i<win_count;i++) sum += win_buf[i];
  cps_meas = (win_count > 0) ? (sum / win_count) : 0.0f;
}

// ===== 50 ms PI 控制（增量式 + 斜率限制 + 饱和）=====
void control_maybe_50ms() {
  unsigned long now = millis();
  if (now - lastCtrl < CONTROL_DT_MS) return;
  lastCtrl += CONTROL_DT_MS;   // 尽量等周期

  // 启动阶段：维持 U_BOOT，不做闭环；同时维持 u_prev
  if (in_boot) {
    if ((long)(now - boot_until) < 0) {
      u_prev = U_BOOT;
      motor_set(U_BOOT);
      // 打印
      Serial.print(F("CTL,"));
      Serial.print(now);         Serial.print(',');
      Serial.print(-1);          Serial.print(','); // -1=boot
      Serial.print(target_cps);  Serial.print(',');
      Serial.print(cps_meas,2);  Serial.print(',');
      Serial.print(KP,4);        Serial.print(',');
      Serial.print(KI,4);        Serial.print(',');
      Serial.print(e_prev,4);    Serial.print(',');
      Serial.println(u_prev,1);
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
  float du_pi = KP * e + KI * e * dt;       // PI 增量（简单形式）

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
  Serial.print(1);           Serial.print(','); // 1=closed
  Serial.print(target_cps);  Serial.print(',');
  Serial.print(cps_meas,2);  Serial.print(',');
  Serial.print(KP,4);        Serial.print(',');
  Serial.print(KI,4);        Serial.print(',');
  Serial.print(e_prev,4);    Serial.print(',');
  Serial.println(u_prev,1);  // 当前实际PWM
}

void loop() {
  // 非阻塞串口 CLI
  serial_cli_nonblock();

  // 速度采样（名义 10ms，一定不会阻塞；内部用真实 dt 并做异常保护）
  sample_maybe_10ms();

  // 控制（50ms）
  control_maybe_50ms();

  // 心跳（可关）
  unsigned long now = millis();
  if (now - lastHB >= 500) {
    lastHB = now;
    // Serial.print(F("HB t=")); Serial.println(now);
  }
}