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
unsigned long lastHB   = 0;         // heartbeat 100 ms
unsigned long lastCtrl = 0;         // control & speed update 50 ms
const unsigned long CONTROL_DT_MS = 20;

// ===== Encoder count =====
volatile long enc_total = 0;        // accumulated counts (+/-)
long last_total = 0;

// ===== Open-loop boot (keep) =====
const uint8_t PWM_BOOT = 0;         // start closed-loop from stop (0 duty)

// ===== PI controller =====
float KP = 0.04f;                   // start small, then tune up
float KI = 0.00f;                   // add small integral at last
float target_cps = 300.0f;          // counts per second target
float integ_e = 0.0f;               // integral of error (sum_e)
const float UMAX = 255.0f;          // output saturation (|u| <= 255)

// ===== Measured speed (counts per second) =====
float cps_meas = 0.0f;

// ====== Encoder ISR ======
// Count one tick per rising edge; B decides direction
void isrEncA() {
  bool b = digitalRead(PIN_ENC_B);
  enc_total += (b ? -1 : +1);
}

// ===== Utility: set motor by signed command u =====
// u in [-255, 255], sign = direction, abs = PWM duty
void motor_set(float u) {
  // saturate hard for safety
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
// Commands (case-insensitive):
//   set target <value>
//   set kp <value>
//   set ki <value>
//   status
void serial_cli() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  // normalize to lower-case for startsWith / parsing
  String low = cmd;
  low.toLowerCase();

  if (low.startsWith("set target")) {
    // after "set target"
    int sp = low.indexOf("set target");
    String num = cmd.substring(sp + 10); // keep original spacing for number
    num.trim();
    float v = num.toFloat();
    target_cps = v;
    Serial.print(F("OK target_cps=")); Serial.println(target_cps, 3);
  }
  else if (low.startsWith("set kp")) {
    int sp = low.indexOf("set kp");
    String num = cmd.substring(sp + 6);
    num.trim();
    float v = num.toFloat();
    KP = v;
    Serial.print(F("OK KP=")); Serial.println(KP, 6);
  }
  else if (low.startsWith("set ki")) {
    int sp = low.indexOf("set ki");
    String num = cmd.substring(sp + 6);
    num.trim();
    float v = num.toFloat();
    KI = v;
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

  lastHB   = millis();
  lastCtrl = lastHB;

  Serial.println(F("PI controller ready. Commands: 'set target <v>', 'set kp <v>', 'set ki <v>', 'status'"));
  Serial.print(F("Init target=")); Serial.println(target_cps);
}

// ===== Control step (every CONTROL_DT_MS) =====
void control_step() {
  // 1) read counts atomically
  long total;
  noInterrupts();
  total = enc_total;
  interrupts();

  // 2) compute delta & cps
  long delta = total - last_total;
  last_total = total;

  float dt = CONTROL_DT_MS * 0.001f;      // seconds
  cps_meas = delta / dt;                   // counts per second

  // 3) PI control with anti-windup
  float e = target_cps - cps_meas;

  // tentative integrate (conditional anti-windup: only integrate if not pushing further into saturation)
  // We'll compute u_unsat using current integ_e, then check saturation direction
  float integ_next = integ_e + e * dt;

  float u_unsat = KP * e + KI * integ_next;

  // Saturate
  float u = u_unsat;
  if (u >  UMAX) u =  UMAX;
  if (u < -UMAX) u = -UMAX;

  // Anti-windup (conditional integration):
  // If saturated and error drives further into the same saturation side, cancel this integration.
  bool pushing_high = (u >=  UMAX - 1e-6) && (e > 0.0f);
  bool pushing_low  = (u <= -UMAX + 1e-6) && (e < 0.0f);
  if (!(pushing_high || pushing_low)) {
    // safe to commit integral
    integ_e = integ_next;
  }
  // else: freeze integral this step

  // 4) apply output
  motor_set(u);
}

void loop() {
  // 0) CLI first
  serial_cli();

  // 1) heartbeat
  unsigned long now = millis();
  if (now - lastHB >= 100) {
    lastHB = now;
    Serial.print(F("HB t="));
    Serial.println(now);
  }

  // 2) control loop @ 50 ms
  if (now - lastCtrl >= CONTROL_DT_MS) {
    lastCtrl = now;
    control_step();

    // optional: stream concise telemetry to log
    Serial.print(F("CTL,"));
    Serial.print(now);         Serial.print(',');
    Serial.print(target_cps);  Serial.print(',');
    Serial.print(cps_meas, 2); Serial.print(',');
    Serial.print(KP, 4);       Serial.print(',');
    Serial.print(KI, 4);       Serial.print(',');
    Serial.println(integ_e, 4);
  }
}