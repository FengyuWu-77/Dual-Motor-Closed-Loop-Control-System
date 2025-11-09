# ⚙️ Dual-Motor Closed-Loop Control System with SoftStop & EncoderFreeze  
**Author:** Fengyu Wu(Eric Wu)
**Platform:** ATmega328PB + TB6612FNG + Quadrature Encoder  
**Languages:** C++ (Arduino Core) + Python  
**Function:** Dual-motor incremental PI control with real-time SoftStart/SoftStop, EncoderFreeze protection, and Python-based performance analysis  

---

## 📦 Features
- ✅ **Dual-channel incremental PI closed-loop control** (10 ms loop, 10-point SMA filter)  
- ⚙️ **SoftStart + SoftStop mechanism** with configurable ramp rates (`U_BOOT`, `SOFTSTOP_RAMP_U_PER_S`)  
- 🧱 **EncoderFreeze fault detection** — motor automatically ramps down if encoder stalls or disconnects  
- 🔋 **EEPROM parameter persistence** for `KP`, `KI`, `U_BOOT`, `BOOT_MS`, and `DU_MAX`  
- 🔄 **STBY and Power-On Self-Test (POST)** for driver enable and encoder movement validation  
- 🧮 **Python log analyzer** computes overshoot %, steady-state error %, and settling time (s)  

---

## 📊 Core Performance Indicators (100 → 200 → 50 cps Step Test)

| Metric | Specification | Measured Value | Status | Notes |
|:--|:--:|:--:|:--:|:--|
| **Steady-State Error** | ≤ 1 % | **0.3 % (avg)** | ✅ | Stable within band after 1.6 s |
| **Overshoot** | ≤ 2 % | **1.2 % (avg)** | ✅ | Smooth ramp with no oscillation |
| **Startup / Settling Time** | ≤ 2 s | **1.6 s (typ.)** | ✅ | Includes soft-start phase |

**Experiment:** 100 → 200 → 50 cps  
**Sampling:** 10 ms control interval (100 Hz)  
**Controller:** Incremental PI with SoftStop ramp (1200 cps/s & 120 PWM/s)  
**Test Board:** ATmega328PB Xplained Mini + TB6612FNG dual H-bridge  

---

## 🎥 Demonstration Video  

📹 **SoftStop + EncoderFreeze Demo:** [View Video (placeholder)](https://github.com/FengyuWu-77/Control/blob/main/Demonstration%20for%20Motor.mp4)  

---

## ⚙️ Hardware Architecture & Pin Mapping  

The system is built around the **ATmega328PB Xplained Mini** development board,  
driving two DC motors through a **TB6612FNG dual H-bridge driver**.  
Each motor uses a **quadrature encoder** (A/B channels) for real-time feedback.

| Module | Function | ATmega328PB Pin | Description |
|---------|-----------|----------------|--------------|
| **Motor A Driver (TB6612FNG)** | PWM + DIR | `PWMA = PD5 (D5)` | Timer0B PWM output |
| | | `AIN1 = PB0 (D8)` | Direction 1 |
| | | `AIN2 = PB1 (D9)` | Direction 2 |
| **Motor B Driver (TB6612FNG)** | PWM + DIR | `PWMB = PB2 (D10)` | Timer1B PWM output |
| | | `BIN1 = PC0 (A0)` | Direction 1 |
| | | `BIN2 = PC1 (A1)` | Direction 2 |
| **Encoders** | Quadrature Feedback | Motor A → `D2 (INT0)` , `D3 (INT1)` | External interrupts |
| | | Motor B → `D4 (PD4)` , `D6 (PD6)` | Pin-change interrupts (PCINT20,22) |
| **STBY Pin** | Driver enable control | `D7 (PD7)` | HIGH = active, LOW = standby |
| **Power Supply** | Logic + Motor Power | `VCC = 5V`, `VM = 12V` | Shared ground required |

---

### 🔋 Hardware Features Summary  

- **Independent dual-motor PWM outputs** using Timer0B + Timer1B hardware PWM.  
- **Quadrature decoding** (x4 resolution) via external and pin-change interrupts.  
- **STBY control** ensures both drivers are disabled on fault or reset.  
- **POST (Power-On Self-Test)** validates encoder feedback on startup.  
- **Shared I/O grounding** between MCU and TB6612FNG to ensure signal integrity.  

---

## 🧪 How to Reproduce  

This section explains how to **run the experiment and analyze results**  
using the provided Arduino firmware and Python analysis tools.

---

### 🧩 Step 1 — Run the Firmware  

1. Connect the hardware according to the [Hardware Architecture](#⚙️-hardware-architecture--pin-mapping) table.  
2. Open **VS Code + PlatformIO** and flash the firmware (`main.cpp`) to the ATmega328PB board.  
3. Open the serial monitor (baud = **57600**) and enter the following CLI commands:

```bash
set t1 100      # Set Motor A to 100 cps
set t2 100      # Set Motor B to 100 cps
set kp 0.0018   # Set proportional gain
set ki 0.00003  # Set integral gain
status          # Verify parameters
```

4.	Begin the step test sequence:

```bash
set t1 100      # Step 1: Start at 100 cps
set t1 200      # Step 2: Increase to 200 cps
set t1 50       # Step 3: Decrease to 50 cps
```

