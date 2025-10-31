# STM32F103 with VL53L1X and IMU BNO055

**Date:** 5/2025
**Author:** Phat Nguyen Tan

This repository is part of my final-year graduation project.  
The firmware runs on an **STM32F103C8T6 (Bluepill)** and communicates with a **Raspberry Pi** over UART.

---

## 🎯 Purpose

The STM32 board collects sensor data and sends it to the Raspberry Pi for robot navigation:

- Read distance from **VL53L1X** ToF sensor (object distance measurement)
- Read **yaw (heading)** from **BNO055** IMU
- Transmit sensor data to Raspberry Pi via UART using a custom frame format
- Receive commands from Raspberry Pi to:
  - Reset the STM32
  - Request sensor data frames
- Choose UART communication mode
- Store and retrieve **BNO055 calibration data** from Flash memory

---

## 🧩 Hardware

| Device | Role |
|--------|------|
| STM32F103C8T6 (Bluepill) | Main microcontroller |
| VL53L1X | Time-of-Flight distance sensor |
| BNO055 | IMU for yaw/heading |
| Raspberry Pi | Companion computer |

---

## 📡 Communication

UART communication with selectable mode:

| UART Mode | Supported |
|----------|----------|
| Polling | ✅ |
| Interrupt | ✅ |
| DMA | ✅ |

---

## ⚙️ Features

- VL53L1X distance reading
- BNO055 yaw reading
- Custom UART data frames
- Reset on command
- UART: Polling / IT / DMA selectable
- Flash read/write for IMU calibration data

---

## 📂 Source Code

- **Inc folder:** <https://github.com/Phat-sv/VL53L1X_and_BNO055_STM32/tree/main/Core/Inc>  
- **Src folder:** <https://github.com/Phat-sv/VL53L1X_and_BNO055_STM32/tree/main/Core/Src>

---

## 🤖 Demo Video

▶️ **Robot demonstration:** <https://www.youtube.com/watch?v=gQMT6rC0Xig>

▶️ **Other information:** <https://github.com/Phat-sv/Capstone_Cleaning_robot/blob/main/README.md>

---

## 📎 Notes

- This code is part of a graduation project
- Focused on functionality and clarity, not optimized for production
