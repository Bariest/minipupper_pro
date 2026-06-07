# 🐶 Mini Pupper ESP32-S3

ESP32-S3 firmware for Mini Pupper quadruped robot with real-time web control interface.

![Mini Pupper](https://img.shields.io/badge/ESP32-S3-blue)
![License](https://img.shields.io/badge/license-MIT-green)

## ✨ Features

- 🌐 **Web Control Interface** - Control your robot from any browser
- 🤖 **Multiple Movement Modes**
  - Walking (Forward, Backward, Left, Right)
  - Turning (Left/Right rotation)
  - Special moves (Twerk, Jump)
  - Dynamic poses (Roll, Pitch, Stretch)
- ⚙️ **Real-time Servo Calibration** - Fine-tune each servo offset via web UI
- 📡 **WiFi Access Point** - Direct connection without router
- 🎮 **Adjustable Parameters**
  - Gait period (speed control)
  - Step height
  - Stride length
  - Tilt angle
- 💾 **Persistent Settings** - All calibrations saved to NVS flash

## 🔧 Hardware Requirements

- **Microcontroller**: ESP32-S3
- **Servos**: 12x SCSCL serial bus servos (Feetech/Waveshare)
- **Power**: Suitable power supply for servos (typically 7.4V LiPo)
- **Frame**: Quadruped robot chassis

### Pin Configuration

| Component | GPIO Pin |
|-----------|----------|
| Servo TX  | GPIO 4   |
| Servo RX  | GPIO 5   |
| Power Enable | GPIO 8 |

## 📥 Installation

### Prerequisites

- ESP-IDF v6.0 or later
- USB cable for flashing

### Build and Flash

```bash
# Clone repository
git clone https://github.com/Bariest/minipupperesp.git
cd minipupperesp

# Configure (optional)
idf.py menuconfig

# Build
idf.py build

# Flash to ESP32-S3
idf.py -p COM... flash

# Monitor serial output
idf.py -p COM... monitor
