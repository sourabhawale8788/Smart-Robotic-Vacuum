# 🤖 Smart Robotic Vacuum Cleaner with Object Detection
**Academic Project — 2024–2025**

A fully autonomous indoor robotic vacuum cleaner built with a **Raspberry Pi + ESP32-CAM** platform, combining bare-metal embedded C firmware with a Python/OpenCV computer vision pipeline for real-time obstacle detection and dynamic path re-routing.

---

## 📌 Table of Contents
- [System Architecture](#system-architecture)
- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Software Stack](#software-stack)
- [Project Structure](#project-structure)
- [How It Works](#how-it-works)
- [Setup & Installation](#setup--installation)
- [UART Communication Protocol](#uart-communication-protocol)
- [Pin Configuration](#pin-configuration)
- [Performance Benchmarks](#performance-benchmarks)
- [Team](#team)

---

## 🏗️ System Architecture

```
┌──────────────────────────────────────────────────────┐
│                  RASPBERRY PI 4B                     │
│                                                      │
│   ┌─────────────┐       ┌──────────────────────┐    │
│   │  ESP32-CAM  │──USB──│  Python/OpenCV        │    │
│   │  MJPEG Feed │       │  Vision Pipeline      │    │
│   └─────────────┘       │  - Object Detection   │    │
│                          │  - Path Re-routing    │    │
│                          └──────────┬───────────┘    │
│                                     │ UART /dev/ttyS0│
└─────────────────────────────────────┼────────────────┘
                                      │
                        ┌─────────────▼──────────────┐
                        │         ESP32 (Main MCU)    │
                        │                             │
                        │  ┌─────────┐ ┌──────────┐  │
                        │  │  Motor  │ │  Sensor  │  │
                        │  │ Control │ │  Fusion  │  │
                        │  │ (PWM)   │ │ US + IR  │  │
                        │  └────┬────┘ └────┬─────┘  │
                        └───────┼────────────┼────────┘
                                │            │
                         ┌──────┴──┐   ┌────┴────┐
                         │L298N    │   │HC-SR04  │
                         │H-Bridge │   │GP2Y0A   │
                         └──┬──┬───┘   └─────────┘
                            │  │
                         [M1] [M2]  DC Motors
```

---

## ✨ Features

| Feature | Implementation |
|---|---|
| Autonomous Navigation | Bare-metal ESP32 PWM motor control (no RTOS, no Arduino) |
| Real-time Obstacle Detection | Python + OpenCV on Raspberry Pi |
| Sensor Fusion | Ultrasonic (HC-SR04) + IR proximity (GPIO interrupt-driven) |
| Vision–Actuator Bridge | UART serial at 115200 baud |
| Path Re-routing | Dynamic OpenCV contour + depth heuristic |
| Collision Response Latency | < 30 ms (verified via oscilloscope) |
| Camera Feed | ESP32-CAM MJPEG stream over USB serial |
| Motor Drive | PWM @ 1 kHz, H-bridge L298N |

---

## 🔧 Hardware Requirements

| Component | Model | Qty |
|---|---|---|
| Single-Board Computer | Raspberry Pi 4B (2GB+) | 1 |
| Camera + Wireless MCU | ESP32-CAM (AI-Thinker) | 1 |
| Main Microcontroller | ESP32 DevKit V1 | 1 |
| Motor Driver | L298N H-Bridge | 2 |
| DC Gear Motors | 12V, 150–300 RPM | 4 |
| Ultrasonic Sensor | HC-SR04 | 3 |
| IR Proximity Sensor | GP2Y0A21YK0F | 4 |
| Power Supply | 12V LiPo / Li-Ion battery | 1 |
| Voltage Regulator | LM2596 (5V for RPi, 3.3V for ESP) | 2 |
| Chassis | Custom acrylic / 3D-printed | 1 |

---

## 💻 Software Stack

**Raspberry Pi (Vision Host)**
- Python 3.10+
- OpenCV 4.8
- PySerial 3.5
- NumPy 1.26

**ESP32 Firmware**
- ESP-IDF v5.x (bare-metal C, no Arduino framework)
- FreeRTOS (bundled with ESP-IDF)
- LEDC peripheral (PWM motor control)
- GPIO interrupt handlers
- UART driver

---

## 📁 Project Structure

```
smart-vacuum/
├── README.md
├── requirements.txt
│
├── src/
│   ├── rpi/                          # Raspberry Pi Python code
│   │   ├── main.py                   # Entry point & main loop
│   │   ├── obstacle_detector.py      # OpenCV vision pipeline
│   │   ├── uart_bridge.py            # UART serial communication
│   │   └── navigation.py             # Path planning & re-routing
│   │
│   └── esp32/                        # ESP32 bare-metal firmware
│       ├── CMakeLists.txt
│       ├── main/
│       │   ├── main.c                # Firmware entry point
│       │   └── CMakeLists.txt
│       └── components/
│           ├── motor_control/
│           │   ├── motor_control.c   # PWM-based DC motor driver
│           │   └── motor_control.h
│           ├── sensor_fusion/
│           │   ├── sensor_fusion.c   # Ultrasonic + IR fusion
│           │   └── sensor_fusion.h
│           └── uart_protocol/
│               ├── uart_protocol.c   # Command parser & dispatcher
│               └── uart_protocol.h
│
└── docs/
    └── wiring_diagram.md             # Pin-level wiring reference
```

---

## ⚙️ How It Works

### 1. Boot Sequence
1. ESP32 boots, initializes PWM channels, GPIO interrupt handlers, and UART.
2. Raspberry Pi starts Python main loop, opens UART port to ESP32, and starts ESP32-CAM MJPEG stream.

### 2. Vision Pipeline (Raspberry Pi)
```
ESP32-CAM MJPEG frame
        │
        ▼
  Resize → BGR→HSV
        │
        ▼
  Background subtraction (MOG2)
        │
        ▼
  Contour detection (findContours)
        │
        ▼
  Obstacle bounding boxes + depth estimate
        │
        ▼
  Navigation decision → UART command
```

### 3. Sensor Fusion (ESP32)
- HC-SR04 triggers via GPIO, echo pulse measured with hardware timer → distance in cm.
- IR sensors read analog via ADC, converted to distance using Sharp GP2Y curve.
- Weighted fusion: `fused_dist = 0.6 * ultrasonic + 0.4 * IR` (adjustable).
- If fused distance < threshold, interrupt fires → emergency STOP command overrides UART.

### 4. Motor Control (ESP32)
- 4 DC motors driven via 2× L298N H-bridge.
- ESP32 LEDC peripheral generates PWM at 1 kHz on 4 channels (ENA, ENB × 2 bridges).
- Direction pins (IN1–IN4) toggled via GPIO for forward/reverse/turn.
- Commands: `FWD`, `REV`, `LEFT`, `RIGHT`, `STOP`, `SPD:<0-255>`.

### 5. UART Communication
- Raspberry Pi sends ASCII command frames: `CMD:<command>\n`
- ESP32 parses in ISR context, dispatches to motor controller.
- ESP32 sends telemetry back: `TEL:<dist_front>,<dist_left>,<dist_right>,<speed>\n`
- Round-trip latency < 30 ms verified on oscilloscope.

---

## 🚀 Setup & Installation

### Raspberry Pi Setup
```bash
# Clone repository
git clone https://github.com/<your-username>/smart-vacuum.git
cd smart-vacuum

# Install Python dependencies
pip3 install -r requirements.txt

# Set UART port (check with: ls /dev/tty*)
export VACUUM_UART_PORT=/dev/ttyS0
export VACUUM_CAM_PORT=/dev/ttyUSB0

# Run
python3 src/rpi/main.py
```

### ESP32 Firmware Setup
```bash
# Install ESP-IDF v5.x
# https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/

cd src/esp32
idf.py set-target esp32
idf.py menuconfig     # set UART pins, baud rate if needed
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

---

## 🔌 UART Communication Protocol

| Direction | Frame Format | Example |
|---|---|---|
| RPi → ESP32 | `CMD:<cmd>\n` | `CMD:FWD\n` |
| RPi → ESP32 | `CMD:SPD:<0-255>\n` | `CMD:SPD:180\n` |
| ESP32 → RPi | `TEL:<f>,<l>,<r>,<spd>\n` | `TEL:42,80,15,180\n` |
| ESP32 → RPi | `EVT:COLLISION\n` | Emergency alert |

**Command Set:**

| Command | Action |
|---|---|
| `FWD` | Move forward |
| `REV` | Move reverse |
| `LEFT` | Turn left |
| `RIGHT` | Turn right |
| `STOP` | Full stop |
| `SPD:<n>` | Set PWM duty 0–255 |
| `CLEAN:ON` | Enable vacuum motor relay |
| `CLEAN:OFF` | Disable vacuum motor relay |

---

## 📍 Pin Configuration

### ESP32 DevKit V1

| Pin | Function | Connected To |
|---|---|---|
| GPIO 18 | Motor A PWM (ENA) | L298N #1 ENA |
| GPIO 19 | Motor B PWM (ENB) | L298N #1 ENB |
| GPIO 21 | Motor IN1 | L298N #1 IN1 |
| GPIO 22 | Motor IN2 | L298N #1 IN2 |
| GPIO 23 | Motor IN3 | L298N #1 IN3 |
| GPIO 25 | Motor IN4 | L298N #1 IN4 |
| GPIO 26 | HC-SR04 TRIG (Front) | HC-SR04 |
| GPIO 27 | HC-SR04 ECHO (Front) | HC-SR04 |
| GPIO 32 | IR Sensor Left | GP2Y0A |
| GPIO 33 | IR Sensor Right | GP2Y0A |
| GPIO 34 | IR Sensor Front-Left | GP2Y0A |
| GPIO 35 | IR Sensor Front-Right | GP2Y0A |
| GPIO 16 | UART2 RX | Raspberry Pi TX |
| GPIO 17 | UART2 TX | Raspberry Pi RX |

---

## 📊 Performance Benchmarks

| Metric | Value | Method |
|---|---|---|
| Obstacle detection latency | ~45 ms | Python time.perf_counter() |
| UART round-trip | < 5 ms | Logic analyzer |
| Motor response (UART→PWM) | < 30 ms | Oscilloscope (CH1=UART, CH2=PWM) |
| Sensor fusion cycle | ~10 ms | ESP32 hardware timer |
| Camera FPS (320×240) | ~15 FPS | OpenCV frame counter |
| Minimum avoid distance | 20 cm | Tuned empirically |

---

## 👥 Team

| Name | Role |
|---|---|
| [Your Name] | Embedded Firmware (ESP32) |
| [Member 2] | Computer Vision (Raspberry Pi) |
| [Member 3] | Hardware & Power Systems |
| [Member 4] | Navigation Algorithm & Testing |

**Faculty Guide:** [Guide Name], [Department], [Institution]

---

## 📄 License
MIT License — see [LICENSE](LICENSE) for details.
