# IoT Backend Device Simulation via ESP32 (C++)

This project implements a complete simulation of an IoT device backend system using an **ESP32** microcontroller. It communicates securely with a cloud server via HTTPS and real-time WebSocket (Socket.IO) connections, simulates fault conditions using a **Poisson process**, and provides **audible and visual alerts** using onboard peripherals.

---

## 📜 Overview

This device acts as a backend simulator that:
- Connects to a backend server via **HTTPS** to fetch or reuse a session.
- Establishes a secure **Socket.IO** connection for bidirectional messaging.
- Measures distance via **HC-SR04 ultrasonic sensor**.
- Detects and simulates faults based on **Poisson-distributed anomalies**.
- Uses **NeoPixel RGB LED** for visual alerts.
- Drives a **MAX98357A I²S audio amplifier** for audio alerts.

---

## 🧪 Key Features

- ✅ **Session Initialization**: Secure HTTPS request to `/device/deviceopen` endpoint.
- 🔄 **Real-Time Communication**: WebSocket with `ping/pong` heartbeats.
- 📏 **Sensor Integration**: HC-SR04 for real-world proximity measurement.
- ⚠️ **Fault Detection**: Uses Poisson randomization to trigger fault levels.
- 🔊 **Audio Alerting**: Error tones via MAX98357A and speaker.
- 💡 **LED Feedback**: Status indication (yellow=boot, green=OK, red=error).

---

## 🧰 Hardware Used

- ESP32 (LILYGO T‑Relay or equivalent)
- HC-SR04 Ultrasonic Distance Sensor
- MAX98357A I²S Audio Amplifier
- 8Ω Speaker
- WS2812 NeoPixel RGB LED
- 5V Power Supply
- Jumper wires, breadboard

---

## 📂 Files

| File | Description |
|------|-------------|
| `BackEndIOTCPP.cpp` | Main C++ code for ESP32-based simulation |
| `BackEndIoTArdunio.pdf` | Full system explanation, diagrams, and methodology |

---

## 🔁 Fault Levels (`Xn` Logic)

The system maps distance measurements into a fault scale:

| `Xn` Value | Condition | Action |
|------------|-----------|--------|
| 1 | Marginal | Log warning, light LED |
| 2 | Critical | Suggest manual reboot |
| 3 | Unsafe | Auto reboot + error tone |
| 4 / 0 | Catastrophic | Play error tone + system shutdown |

---

## 🧪 How Faults Are Triggered

Every 5 seconds, a `ping` is sent. On `pong`, if simulation is active, a **Poisson-distributed** random value `k` is drawn with λ=5:

```math
P(k; λ) = (e^-λ * λ^k) / k!

