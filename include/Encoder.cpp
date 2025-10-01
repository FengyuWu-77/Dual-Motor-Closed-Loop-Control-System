#include <Arduino.h>

// 电机A
const uint8_t PIN_PWMA = 5;     // A 路 PWM
const uint8_t PIN_AIN1 = 8;     // A 路方向
const uint8_t PIN_AIN2 = 9;     // A 路方向
const uint8_t PIN_STBY = 7;     // 使能 (高电平)

/*** ---- 编码器（电机A） ---- ***/
const uint8_t PIN_ENC_A = 2;    // A 相 → 外部中断（INT0）
const uint8_t PIN_ENC_B = 4;    // B 相 → 普通输入（判方向）

/*** ---- 计时 ---- ***/
unsigned long lastHB   = 0;     // 100ms 心跳
unsigned long lastSpd  = 0;     // 10ms 速度结算

/*** ---- 编码器计数 ---- ***/
volatile long enc_total = 0;    // 累计计数（有正负，方向已含在内）
long last_total = 0;            // 上次结算时的累计值

// ====== 配置：固定占空比与方向 ======
const uint8_t PWM_DUTY = 80;   // 0~255，先用 ~47% 占空比，按需调整
const bool MOTOR_FWD = true;    // true=正转 (AIN1=H, AIN2=L)，false=反转

void isrEncA() {
  // B 相电平决定增/减；若与你实际方向相反，把 +1/-1 对调即可
  bool b = digitalRead(PIN_ENC_B);
  enc_total += (b ? -1 : +1);
}

void setup() {
  Serial.begin(9600);

  // TB6612FNG 管脚初始化
  pinMode(PIN_PWMA, OUTPUT);
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);

  // 使能驱动
  digitalWrite(PIN_STBY, HIGH);

  // 设置方向
  if (MOTOR_FWD) {
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);
  } else {
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, HIGH);
  }

  // 给定固定占空比，电机空载运转
  analogWrite(PIN_PWMA, PWM_DUTY);

  // 编码器输入，上拉
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  // A 相任何跳变计一次
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), isrEncA, RISING);

  lastHB  = millis();
  lastSpd = lastHB;

  Serial.println(F("Motor open-loop running; encoder logging started."));
  Serial.print(F("PWM duty=")); Serial.println(PWM_DUTY);
}

void loop() {
  unsigned long now = millis();

  // 1) 心跳：每 100ms 打印一次
  if (now - lastHB >= 100) {
    lastHB = now;
    Serial.print(F("HB t="));
    Serial.println(now);
  }

  // 2) 速度结算：每 10ms 计算 Δcount 与 cps（counts per second）
  if (now - lastSpd >= 50) {
    unsigned long dt_ms = now - lastSpd;
    lastSpd = now;
    float dt = dt_ms * 0.001f;   // s

    long total;
    noInterrupts();              // 原子读取累计计数
    total = enc_total;
    interrupts();

    long delta = total - last_total;
    last_total = total;

    float cps = delta / dt;      // counts per second

    // 打印：时间戳, 累计计数, 本周期Δcount, 速度(cps)
    Serial.print(F("ENC,"));
    Serial.print(now);           Serial.print(',');
    Serial.print(total);         Serial.print(',');
    Serial.print(delta);         Serial.print(',');
    Serial.println(cps, 2);
  }

  // 开环空载运行，无其它控制
}