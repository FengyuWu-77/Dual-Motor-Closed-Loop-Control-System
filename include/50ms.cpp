#include <Arduino.h>

// ===== Pins (Motor A on TB6612FNG) =====
const uint8_t PIN_PWMA = 5;     // PWM
const uint8_t PIN_AIN1 = 8;     // DIR1
const uint8_t PIN_AIN2 = 9;     // DIR2
const uint8_t PIN_STBY = 7;     // Enable (HIGH)

// ===== Encoder (Motor A) =====
const uint8_t PIN_ENC_A = 2;    // INT0
const uint8_t PIN_ENC_B = 4;    // normal input

// ===== Timing =====
unsigned long lastHB = 0;                 // optional heartbeat (keep or disable)
unsigned long lastSample = 0;             // 10 ms speed sample
unsigned long lastCtrl   = 0;             // 50 ms control & print

const unsigned long SAMPLE_DT_MS  = 10;   // 10 ms raw cps window
const unsigned long CONTROL_DT_MS = 50;   // 50 ms control/print period

// ===== Encoder count =====
volatile long enc_total = 0;              // accumulated counts (+/-)
long last_total_10ms = 0;                 // for 10 ms delta

// ===== Open-loop boot (keep) =====
const uint8_t PWM_BOOT = 0;               // start closed-loop from stop (0 duty)

// ===== PI controller =====
float KP = 0.04f;                         // start small, then tune up
float KI = 0.00f;                         // add small integral at last
float target_cps = 300.0f;                // counts per second target
float integ_e = 0.0f;                     // integral of error (sum_e)
const float UMAX = 255.0f;                // output saturation (|u| <= 255)

// ===== Measured speed (counts per second) =====
float cps_meas = 0.0f;                    // 50 ms sliding average of cps

// ===== 5-sample (5x10ms) sliding window =====
float cps_buf[5] = {0,0,0,0,0};
uint8_t buf_idx = 0;
uint8_t buf_count = 0;
float cps_sum = 0.0f;                     // running sum for O(1) average

// ====== Encoder ISR ======
// Count one tick per rising edge; B decides direction
void isrEncA() {
  bool b = digitalRead(PIN_ENC_B);
  enc_total += (b ? -1 : +1);
}

// ===== Utility: set motor by signed command u =====
// u in [-255, 255], sign = direction, abs = PWM duty
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
  cmd.trim();
  String low = cmd; low.toLowerCase();

  if (low.startsWith("set target")) {
    int sp = low.indexOf("set target");
    String num = cmd.substring(sp + 10); num.trim();
    target_cps = num.toFloat();
    Serial.print(F("OK target_cps=")); Serial.println(target_cps, 3);
  }
  else if (low.startsWith("set kp")) {
    int sp = low.indexOf("set kp");
    String num = cmd.substring(sp + 6); num.trim();
    KP = num.toFloat();
    Serial.print(F("OK KP=")); Serial.println(KP, 6);
  }
  else if (low.startsWith("set ki")) {
    int sp = low.indexOf("set ki");
    String num = cmd.substring(sp + 6); num.trim();
    KI = num.toFloat();
    Serial.print(F("OK KI=")); Serial.println(KI, 6);
  }
  else if (low.startsWith("status")) {
    Serial.print(F("STAT, t=")); Serial.print(millis());
    Serial.print(F(", kp="));     Serial.print(KP, 6);
    Serial.print(F(", ki="));     Serial.print(KI, 6);
    Serial.print(F(", target=")); Serial.print(target_cps, 2);
    Serial.print(F(", cps="));    Serial.print(cps_meas, 2);
    Serial.print(F(", integ="));  Serial.print(integ_e, 4);
    Serial.println();
  }
  else {
    Serial.println(F("Unknown cmd. Use: 'set target <v>', 'set kp <v>', 'set ki <v>', 'status'"));
  }
}

void setup() {
  Serial.begin(57600);

  // Driver pins
  pinMode(PIN_PWMA, OUTPUT);
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, HIGH);   // enable

  // Start stopped (closed-loop will take over)
  digitalWrite(PIN_AIN1, LOW);
  digitalWrite(PIN_AIN2, LOW);
  analogWrite(PIN_PWMA, PWM_BOOT);

  // Encoder pins
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), isrEncA, RISING);

  unsigned long t = millis();
  lastHB     = t;
  lastSample = t;
  lastCtrl   = t;

  // init last_total_10ms to current count to avoid first big delta
  noInterrupts();
  last_total_10ms = enc_total;
  interrupts();

  Serial.println(F("PI controller ready. Commands: 'set target <v>', 'set kp <v>', 'set ki <v>', 'status'"));
  Serial.print(F("Init target=")); Serial.println(target_cps);
}

// ===== 10 ms sampling =====
void sample_10ms() {
  long total;
  noInterrupts();
  total = enc_total;
  interrupts();

  long delta = total - last_total_10ms;
  last_total_10ms = total;

  // cps over 10 ms window
  const float dt = SAMPLE_DT_MS * 0.001f; // 0.010 s
  float cps10 = delta / dt;

  // sliding window: update running sum and buffer
  if (buf_count < 5) {
    cps_sum += cps10;
    cps_buf[buf_idx] = cps10;
    buf_idx = (buf_idx + 1) % 5;
    buf_count++;
  } else {
    // replace oldest
    float old = cps_buf[buf_idx];
    cps_sum -= old;
    cps_buf[buf_idx] = cps10;
    cps_sum += cps10;
    buf_idx = (buf_idx + 1) % 5;
  }
}

// ===== 50 ms control step (use 5x10ms average) =====
void control_step_50ms() {
  // use average of available samples (up to 5)
  if (buf_count > 0) {
    cps_meas = cps_sum / buf_count;
  } else {
    cps_meas = 0.0f;
  }

  // PI control with conditional anti-windup
  float e = target_cps - cps_meas;

  // tentative integrate
  float dt = CONTROL_DT_MS * 0.001f; // 0.050 s
  float integ_next = integ_e + e * dt;

  float u_unsat = KP * e + KI * integ_next;

  // Saturate
  float u = u_unsat;
  if (u >  UMAX) u =  UMAX;
  if (u < -UMAX) u = -UMAX;

  // Anti-windup: freeze integral if pushing deeper into saturation
  bool pushing_high = (u >=  UMAX - 1e-6) && (e > 0.0f);
  bool pushing_low  = (u <= -UMAX + 1e-6) && (e < 0.0f);
  if (!(pushing_high || pushing_low)) {
    integ_e = integ_next;
  }

  // apply output
  motor_set(u);

  // telemetry @ 50 ms
  Serial.print(F("CTL,"));
  Serial.print(millis());     Serial.print(',');
  Serial.print(target_cps);   Serial.print(',');
  Serial.print(cps_meas, 2);  Serial.print(',');
  Serial.print(KP, 4);        Serial.print(',');
  Serial.print(KI, 4);        Serial.print(',');
  Serial.println(integ_e, 4);
}

void loop() {
  // CLI
  serial_cli();

  unsigned long now = millis();

  // 10 ms speed sampling
  if (now - lastSample >= SAMPLE_DT_MS) {
    lastSample += SAMPLE_DT_MS; // resist jitter accumulation
    sample_10ms();
  }

  // 50 ms control + print
  if (now - lastCtrl >= CONTROL_DT_MS) {
    lastCtrl += CONTROL_DT_MS;
    control_step_50ms();
  }

  // (optional) heartbeat, keep slower if you like
  if (now - lastHB >= 500) {
    lastHB += 500;
    // Serial.println(F("HB"));
  }
}