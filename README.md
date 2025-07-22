# ESP32 DMX LED Controller

This project reads DMX512 data using an ESP32 and controls up to 3 Adafruit PCA9685 16-channel PWM controllers to drive RGBW LED strips. It includes a web server for manual color control and OTA updates.

## Features

- Reads DMX512 input (starting at channel 220)
- Controls 12 RGBW LED strips (48 channels total)
- Manual override via HTTP API
- Pulse mode with adjustable speed
- Wi-Fi config using API (WiFiManager)
- OTA firmware updates (ArduinoOTA)

## Requirements

### Hardware

- ESP32
- 1–3x Adafruit PCA9685 16-channel PWM controllers
- RGBW LED strips
- RS485 to UART converter (for DMX input)
- Optional: status LED on GPIO 13

### Libraries

Install via Arduino Library Manager:

- WiFiManager
- ArduinoJson
- ArduinoOTA
- Adafruit PWM Servo Driver
- A DMX library (custom or SparkFunDMX)

---

Modify `dmx.cpp` as follows:
```c++
#include <dmx.h>
#include <Arduino.h>

#define DMX_SERIAL_INPUT_PIN    GPIO_NUM_16 // pin for dmx rx
#define DMX_SERIAL_OUTPUT_PIN   GPIO_NUM_17 // pin for dmx tx
#define DMX_SERIAL_IO_PIN       GPIO_NUM_21 // pin for dmx rx/tx change

#define DMX_UART_NUM            UART_NUM_2  // dmx uart
#define HEALTHY_TIME            500         // timeout in ms 
#define BUF_SIZE                1024        //  buffer size for rx events
#define DMX_CORE                1           // select the core the rx/tx thread should run on
#define DMX_IGNORE_THREADSAFETY 0           // set to 1 to disable all threadsafe mechanisms
```

---

## Web API

### POST `/led`

Update colors for one or all channels. Also used to enable pulse or manual mode.

#### JSON Format

```json
{
  "multi": true,
  "pulse": false,
  "manual": true,
  "pulseSpeed": 2,
  "channel": 5,
  //Color values range from 0-255
  "red": 128,
  "green": 64,
  "blue": 255,
  "white": 0
}

```

- `multi`: If `true`, updates all strips.
- `channel`: Channel index (0–47) if `multi` is `false`.
- `pulse`: Enables automatic pulsing (disables DMX).
- `manual`: Enables manual mode (disables DMX).
- `pulseSpeed`: A multiplier that determines how quickly the pulse effect runs.
---

## Behavior

- DMX is read continuously unless manual or pulse mode is active.
- If DMX is not detected, the last known values are held.
- Button on GPIO 0 starts/stops Wi-Fi config portal (captive mode).
- Web server runs on port `800`.

---

## OTA Updates

Once connected to Wi-Fi, upload firmware using ArduinoOTA. The ESP32 will appear as a network port in the Arduino IDE.

---

## Flash Instructions

1. Connect the ESP32 via USB.
2. Open VSCode
3. Install PlatformIO
4. Plug the ESP32 using USB
5. Use the built in flash button located on the top right

---

## Notes

- My DMX LEDs address starts at 220. 
- Each LED strip uses 4 channels: Red, Green, Blue, White.

---

## TODO

- Figure out why the LED lights are flashing
- Improve DMX fading behavior