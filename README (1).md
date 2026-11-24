# ESP32-S3 Environmental Monitoring Station

Presentation Link : https://drive.google.com/file/d/1AuJ6CJ_ycUY9XcgFCacMq2VpDqquatQu/view?usp=drive_link

A room environment monitor built on an **ESP32-S3** running **FreeRTOS**. It reads temperature, humidity, light and air pressure, sends them over **MQTT**, and shows them on a **live web dashboard**. The dashboard also replays what happens *inside* the chip: every function call for every reading, timed in microseconds, across both CPU cores.

**Author:** Mudit Agrawal (2501010283), 

---

## Features

- Live readings of **temperature, humidity, light level and air pressure**, with plain labels such as "Comfortable" or "Dim"
- **4 FreeRTOS tasks on 2 cores**: networking on core 0, never delaying sensing on core 1
- Every FreeRTOS feature is used for a real job: **queues, mutex, binary semaphore + interrupt, task notifications, event group, software timer, priorities**
- **Function trace**: each reading carries the list of functions that ran for it, with µs timings, replayed step by step on an animated board map
- **Two-way control** from the browser: LED colour, reading interval, text message, ping (round-trip delay) and restart
- **BOOT button interrupt** triggers an instant reading
- **Raw MQTT viewer**: every message the computer receives, as JSON
- **Offline detection** using the MQTT last-will message
- **Demo mode** and **test mode** to try the dashboard without the board

---

## Project structure

```
esp32-station/
├── firmware/
│   ├── platformio.ini      # board + libraries
│   └── src/
│       └── main.cpp        # all firmware (tasks, sensors, MQTT)
├── index.html              # live web dashboard (open in a browser)
├── sensor_test/
│   └── main.cpp            # temporary sketch to check sensor wiring
└── README.md
```

---

## Hardware

| Component | Measures / does | Interface |
|---|---|---|
| ESP32-S3 DevKitC-1 | Dual-core MCU with Wi-Fi, runs everything | USB |
| BMP280 | Temperature (°C) and pressure (hPa) | I2C |
| DHT22 (or DHT11) | Humidity (%), backup temperature | 1 data wire |
| LDR + 10 kΩ resistor | Light level, 0–100 % | Analog (ADC1) |
| BOOT button (on board) | Interrupt for an instant reading | GPIO 0 |
| RGB LED (on board) | Colour set from the dashboard | GPIO 48 |

### Wiring

| Sensor pin | ESP32-S3 pin |
|---|---|
| BMP280 VCC / GND | 3V3 / GND |
| BMP280 SDA | **GPIO 8** |
| BMP280 SCL | **GPIO 9** |
| DHT VCC / GND | 3V3 / GND |
| DHT DATA | **GPIO 5** (bare 4-pin sensor: add 10 kΩ from DATA to VCC) |
| LDR | 3V3 → LDR → **GPIO 6** → 10 kΩ → GND |

**Pins to avoid:** 0 (BOOT button), 19–20 (USB), 26–32 (flash), 43–44 (Serial), 48 (LED). Analog sensors must use **GPIO 1–10** (ADC1), because ADC2 does not work while Wi-Fi is on.

---

## How it works

```
Sensors ──► SensorTask ──► dataQueue ──► MqttTask ──► MQTT broker ──► Web dashboard
 (core 1)                                 (core 0)        ▲                │
BOOT ISR ──► buttonSem ──► ButtonTask ──► outQueue ───────┘                │
                               │ notify                                     │
                               ▼                                            │
                           SensorTask        CommandTask ◄── cmdQueue ◄─────┘ (commands)
```

### FreeRTOS concepts used

| Concept | Name in code | Its job |
|---|---|---|
| Tasks | SensorTask, MqttTask, CommandTask, ButtonTask | Each job runs as its own small program |
| Two cores | `xTaskCreatePinnedToCore` | Core 0: MqttTask + Wi-Fi. Core 1: the other tasks |
| Priorities | ButtonTask 3, SensorTask 2, others 1 | A button press is handled first (pre-emption) |
| Queues | `dataQueue`, `cmdQueue`, `outQueue` | Pass readings, commands and replies between tasks |
| Mutex | `serialMutex` | Only one task prints to Serial at a time |
| Interrupt (ISR) | `buttonISR` | Runs the instant BOOT is pressed |
| Binary semaphore | `buttonSem` | The ISR gives it; ButtonTask wakes and does the work |
| Task notifications | `xTaskNotifyGive`, `xTaskNotify` | ButtonTask → SensorTask ("read now"), timer → MqttTask ("send report") |
| Event group | `netEvents` | `WIFI_OK`, `MQTT_OK` bits; SensorTask waits for Wi-Fi |
| Software timer | `sysTimer` (5 s) | Snapshot of task states, stacks and CPU load |
| Idle hooks | `idleHook0/1` | Approximate CPU load per core |

---

## MQTT topics

All topics start with the prefix `iitp/priyanshi/envstation`. The ESP32 uses `broker.hivemq.com:1883`; the browser uses `wss://broker.hivemq.com:8884/mqtt`.

| Topic | Direction | Contents |
|---|---|---|
| `/telemetry` | ESP32 → browser | Each reading: values + function trace |
| `/system` | ESP32 → browser | Every 5 s: heap, uptime, Wi-Fi, queues, CPU, mutex, semaphore, event bits, task states |
| `/button` | ESP32 → browser | Each BOOT press: count, interrupt-to-task delay, trace |
| `/ack` | ESP32 → browser | Reply to each command, with trace |
| `/event` | ESP32 → browser | Connects and reconnects |
| `/status` | ESP32 → browser | `online` / `offline` (retained, last will) |
| `/cmd` | browser → ESP32 | Commands |

Example reading:

```json
{"seq":12,"why":"timer","t":27.4,"h":58,"light":55,"p":1006.9,
 "tr":[["readTemperature","S",20479982,1060], ..., ["mqtt.publish","M",20488103,-1]]}
```

A trace step is `[function, who, start µs, µs taken]`. Who: `S` SensorTask, `M` MqttTask, `C` CommandTask, `K` ButtonTask, `I` interrupt. Taken `-1` = still running, `-2` = not timed.

### Commands

| `cmd` | Extra field | Action |
|---|---|---|
| `led` | `color` (e.g. `"#ff0000"`) | Sets the on-board RGB LED |
| `interval` | `ms` (500–60000) | Changes how often SensorTask reads |
| `msg` | `text` | Prints the text on the Serial Monitor |
| `ping` | – | Replies at once (round-trip time) |
| `restart` | – | Replies, then restarts after 1 s |

---

## Setup

### 1. Firmware

1. Install **VS Code** and the **PlatformIO IDE** extension.
2. Create a new project: board **Espressif ESP32-S3-DevKitC-1**, framework **Arduino**.
3. Copy `platformio.ini` and `src/main.cpp` into the project. Libraries download automatically:
   - PubSubClient, ArduinoJson, DHT sensor library, Adafruit Unified Sensor, Adafruit BMP280
4. In `main.cpp`, set your Wi-Fi:
   ```cpp
   const char* WIFI_SSID = "YOUR_WIFI_NAME";
   const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";
   ```
   Use a **2.4 GHz** network. A phone hotspot works best; college Wi-Fi often blocks port 1883.
5. Set the DHT type: `#define DHT_TYPE DHT22` (white sensor) or `DHT11` (blue sensor).
6. **Build** (✓), **Upload** (→), then open the **Serial Monitor** (115200 baud).

You should see:

```
BMP280 found
MqttTask: Wi-Fi OK, IP 192.168.x.x
MqttTask: MQTT connected, listening for commands
SensorTask: reading #1 (timer) | Temp 27.3 C | Humidity 61 % | Light 48 % | Pressure 1006.2 hPa
```

### 2. Dashboard

1. Open `index.html` in Chrome (laptop needs internet).
2. Check the **topic prefix** matches `PREFIX` in `main.cpp`.
3. Click **Connect live**. The badge turns green: **Live · ESP32 online**.
4. Press the **BOOT** button on the board to see the interrupt path on the map.

### Dashboard modes

| Open as | Shows |
|---|---|
| `index.html` | Demo data (labelled "Demo data") until **Connect live** is clicked |
| `index.html#test` | Demo data with the green live look, labelled "(test mode)", for testing the page without the board |
| After **Connect live** | Real data from the ESP32 |

---

## Troubleshooting

| Problem | Likely cause | Fix |
|---|---|---|
| Stuck at `waiting for Wi-Fi...` | Wrong name/password or 5 GHz network | Fix credentials, use 2.4 GHz |
| `MQTT connect failed` repeats | Network blocks port 1883 | Use a phone hotspot |
| `BMP280 NOT found` | Wiring | Check SDA = 8, SCL = 9, 3V3, GND; try CSB → 3V3, SDO → GND |
| Humidity `nan` | DHT wiring or wrong type | Check DATA on GPIO 5; match `DHT_TYPE` to your sensor |
| Light stuck at 0 % or 100 % | LDR divider wrong | Check the 10 kΩ from GPIO 6 to GND |
| Serial OK but dashboard empty | Old `index.html`, wrong prefix, or not connected | Use the latest file, match the prefix, click Connect live |
| "No reply in 6 s" for commands | Board offline | Check the Serial Monitor and the badge |

To check each sensor on its own, temporarily replace `src/main.cpp` with `sensor_test/main.cpp`. It scans I2C, reads the chip ID, and tries the DHT as both DHT22 and DHT11.

---

## Limitations

- Light is a relative 0–100 % (an LDR is not calibrated in lux).
- DHT22 gives a new value at most every 2 s.
- CPU load is approximate (idle-tick counting).
- Task states are a snapshot every 5 s.
- The public broker has no login or encryption: fine for a demo, not for real use. Keep the prefix unique.
- No long-term storage; the browser keeps the last 90 values.

## Future scope

- Gas sensor (MQ-135 / SGP30) for air quality
- Private broker with login and TLS
- Database for long-term history
- Alerts when a value leaves its healthy range
- Battery operation with deep sleep

---

## References

- [FreeRTOS](https://www.freertos.org)
- [Espressif ESP32-S3 documentation](https://docs.espressif.com)
- [PubSubClient](https://github.com/knolleary/pubsubclient)
- [ArduinoJson](https://arduinojson.org)
- [Adafruit DHT library](https://github.com/adafruit/DHT-sensor-library)
- [Adafruit BMP280 library](https://github.com/adafruit/Adafruit_BMP280_Library)
- [MQTT.js](https://github.com/mqttjs/MQTT.js)
