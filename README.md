# Raspberry Pi Pico W Environmental Monitor  
(M5Stack ENV III / SHT30 → MQTT → Home Assistant)

This project turns a **Raspberry Pi Pico 2 W** into a Wi-Fi environmental monitor that reads **temperature and humidity** from an **M5Stack ENV III** sensor (SHT30) and publishes values to an **MQTT broker**.  
It includes **Home Assistant MQTT Discovery**, so sensors appear automatically in HA with no manual YAML configuration.

---

## Features
- Wi-Fi + MQTT publishing
- Home Assistant auto-discovery
- Temperature (°F / °C)
- Relative Humidity (%)
- Default reporting interval: 5 minutes
- Watchdog + reconnect logic (recovers from Wi-Fi/MQTT drops)
- Lightweight, optimized MicroPython code

---

## Hardware

| Item                         | Example/Model | Notes                          |
|------------------------------|---------------|--------------------------------|
| Microcontroller              | Raspberry Pi Pico 2 W | Built-in Wi-Fi |
| Sensor                       | M5Stack ENV III (SHT30 + QMP6988) | Code uses SHT30 |
| Cable                        | Grove / Dupont jumper wires | 4-wire I²C |
| Power                        | 5V USB | From HA server, wall adapter, or hub |

---

## Wiring (Pico 2 W → ENV III)

| ENV III Pin | Wire Color | Pico 2 W Pin | Function |
|-------------|------------|--------------|----------|
| VCC         | Red        | 3V3 (Pin 36) | 3.3V Power |
| GND         | Black      | GND (Pin 38) | Ground |
| SDA         | White      | GP0 (Pin 1)  | I²C0 SDA |
| SCL         | Yellow     | GP1 (Pin 2)  | I²C0 SCL |

---

## Setup

### 1. Flash MicroPython
- Download latest [MicroPython UF2 for Pico W](https://micropython.org/download/rp2-pico-w/)
- Hold BOOTSEL, plug Pico into USB, drag the `.uf2` onto the RPI-RP2 drive

### 2. Install Thonny
- [Download Thonny](https://thonny.org/)  
- Select interpreter: MicroPython (Raspberry Pi Pico)

### 3. Clone Repo & Upload
```bash
git clone https://github.com/yourusername/pico-env-monitor.git

	•	Upload main.py and secrets.py to the Pico via Thonny

⸻

Configuration

Create secrets.py on the Pico with your settings:

WIFI_SSID = "YourWiFiSSID"
WIFI_PASSWORD = "YourWiFiPassword"

MQTT_BROKER = "10.0.1.68"
MQTT_PORT = 1883
MQTT_SSL = False
MQTT_USER = "mqtt_user"
MQTT_PASSWORD = "yourpassword"

DEVICE_NAME = "Environmental Monitor"

# Publish interval in seconds (default = 300 → every 5 minutes)
PUBLISH_INTERVAL_SEC = 300


⸻

MQTT Topics

With DEVICE_NAME = "Environmental Monitor" → slug = environmental_monitor
	•	Discovery (retained):
	•	homeassistant/sensor/environmental_monitor/temperature_f/config
	•	homeassistant/sensor/environmental_monitor/humidity/config
	•	Availability (retained):
	•	environmental_monitor/status → online / offline
	•	Data:
	•	environmental_monitor/temperature_f → {"time": 1696182955054, "value": 72.3}
	•	environmental_monitor/humidity     → {"time": 1696182955054, "value": 55.1}

⸻

Home Assistant

Enable MQTT Discovery
	•	Settings → Devices & Services → MQTT → Enable Discovery

Entities created automatically
	•	Rack Temperature (°F)
	•	Rack Humidity (%)

⸻

Example Automation (Kasa outlet control)

Turn on a Kasa outlet when rack temperature > 80°F, except between 00:00–09:00.

alias: Rack Cooling Control
trigger:
  - platform: numeric_state
    entity_id: sensor.rack_temperature
    above: 80
condition:
  - condition: not
    conditions:
      - condition: time
        after: '00:00:00'
        before: '09:00:00'
action:
  - service: switch.turn_on
    target:
      entity_id: switch.kasa_outlet
mode: single


⸻

Testing

Listen to the Pico’s MQTT messages:

mosquitto_sub -h 10.0.1.68 -u mqtt_user -P yourpassword -v -t 'environmental_monitor/#'

You should see:

environmental_monitor/status online
environmental_monitor/temperature_f {"time":1696182955054,"value":72.3}
environmental_monitor/humidity {"time":1696182955054,"value":55.1}


⸻

Troubleshooting
	•	Entities show “Unavailable”:
	•	Ensure Pico published at least one retained state
	•	Verify environmental_monitor/status = online
	•	Gaps in HA graph:
	•	Watchdog + reconnect ensures Pico won’t stall
	•	Check Wi-Fi signal & broker logs
	•	Change interval:
	•	Edit PUBLISH_INTERVAL_SEC in secrets.py

⸻

License

MIT License
Copyright (c) 2025 Nikolay Georgiev

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the “Software”), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

---

Do you also want me to generate the `LICENSE` file separately (plain MIT text), or just keep it embedded in the README?