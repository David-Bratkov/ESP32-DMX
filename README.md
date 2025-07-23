# ESP32 DMX LED Controller

A multithreaded ESP32 project for controlling RGBW LED strips via DMX512, with built-in web control, OTA firmware updates, and flexible lighting modes like **pulse** and **manual override**. Designed to drive up to **3 Adafruit PCA9685 PWM controllers**.

---

## 🔧 Features

* ✅ **DMX512 input** via UART2 (starting at channel `220`)
* ✅ **Controls up to 12 RGBW LED strips** via I2C PWM (48 channels)
* ✅ **Web server (HTTP API)** for remote control, override, and color setting
* ✅ **Pulse effect** with dynamic brightness and color support
* ✅ **Manual mode** to override DMX input
* ✅ **Wi-Fi configuration** via captive portal (WiFiManager)
* ✅ **OTA firmware updates** via ArduinoOTA
* ✅ **Failsafe fallback** to last known good DMX values
* ✅ **Multithreaded task separation** (DMX vs Network)

---

## 📦 Hardware Requirements

* ESP32
* 1–3x Adafruit PCA9685 16-channel PWM drivers
* RGBW LED strips (12 max)
* RS485 → UART converter (DMX input)
* Status LED (optional) on GPIO 13
* Button (optional) on GPIO 0 for Wi-Fi config reset

---

## 📚 Library Dependencies

Install via Arduino Library Manager:

* `esp_dmx` (ESP-IDF native DMX support)
* `Adafruit PWM Servo Driver`
* `WiFiManager`
* `ArduinoOTA`
* `ArduinoJson`

---

## 🌐 Web API

### POST `/led`

Control color, pulse, or manual mode via HTTP POST (port `800`).

#### JSON Payload:

```json
{
  "multi": true,
  "pulse": false,
  "manual": true,
  "pulseSpeed": 2,
  "channel": 5,
  "red": 128,
  "green": 64,
  "blue": 255,
  "white": 0
}
```

| Field                     | Type  | Description                                    |
| ------------------------- | ----- | ---------------------------------------------- |
| `multi`                   | bool  | Set all LED strips if true                     |
| `channel`                 | int   | Strip index (0–47) if `multi` is false         |
| `red, green, blue, white` | 0–255 | RGBW values                                    |
| `pulse`                   | bool  | Enables pulsing effect and disables DMX        |
| `manual`                  | bool  | Enables manual color override and disables DMX |
| `pulseSpeed`              | int   | (Optional) Speed of pulse animation            |

---

## ⚙️ Behavior Overview

* DMX is read continuously unless **pulse** or **manual** mode is active
* In case of DMX timeout, last valid values are reused
* Pulse mode uses color from last HTTP POST to animate brightness
* Wi-Fi captive portal opens if saved credentials are missing (via button or boot)
* OTA updates available once device is online

---

## 📡 OTA Updates

* Available over network using **Arduino IDE** or PlatformIO
* Device will appear as a network port
* OTA handles update progress and error reporting

---

## 🧪 Debug Options

Enable `#define DEBUG_MODE true` to print:

* Incoming DMX values
* Invalid channel data
* Debug logs for mode switching

---

## 🔌 Pin Configuration

| Function    | GPIO    |
| ----------- | ------- |
| DMX RX      | 16      |
| DMX TX      | 17      |
| DMX RTS     | 21      |
| I2C SDA/SCL | Default |
| Status LED  | 13      |
| Button      | 0       |

---

## 🚀 Flashing Instructions

1. Connect ESP32 via USB
2. Open VSCode with PlatformIO or Arduino IDE
3. Flash as normal (press/hold BOOT if required)
4. Once booted, ESP32 will host a Wi-Fi config portal if no credentials are saved

---

## 📋 Notes

* Each **LED strip** uses 4 DMX channels (R/G/B/W)
* **DMX universe starts at channel 220** for this setup
* **LED strip limit**: 12 total (48 DMX channels)
* Web server runs on **port 800**

---

## ✅ TODO (as of now)

* [x] Fix DMX flickering (✅ resolved via `esp_dmx`)
* [ ] Improve fading interpolation for smoother transitions
* [ ] Add GET `/status` endpoint for current mode/state

---