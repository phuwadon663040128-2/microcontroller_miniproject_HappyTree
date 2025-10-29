# ESP32 Telemetry Bridge

Firmware that bridges STM32 UART telemetry to the FastAPI gateway over Wi-Fi.

## Features
- Consumes newline-delimited JSON payloads from STM32 via UART (default 115200 baud).
- Wraps the payload with device metadata and an ISO-8601 timestamp obtained from NTP.
- POSTs the combined payload to the FastAPI `/telemetry` endpoint with retry logic.
- Sends `OK` / `ERR` acknowledgements back to STM32 for lightweight flow control.
- Uses the ESP32 USB serial port for diagnostics while speaking to STM32 over a configurable secondary UART.

## Hardware Wiring
| STM32 (Nucleo F411) | ESP32 DevKit | Notes |
| --- | --- | --- |
| PA11 / USART6_TX | GPIO16 (RX2) | 3.3 V logic only |
| PA12 / USART6_RX | GPIO17 (TX2) | Optional for ACK feedback |
| GND | GND | Must share common ground |

Default pins map the STM32 UART to the ESP32's second hardware serial port (`Serial1`).
You can override them via PlatformIO build flags, for example:

```ini
build_flags = -DSTM32_RX_PIN=14 -DSTM32_TX_PIN=27
```

> **Warning:** ESP32 GPIOs are 3.3 V tolerant only. Use a level shifter if the STM32 pins expose 5 V logic.

## Credentials Setup
1. Copy `include/credentials.sample.h` to `include/credentials.h`.
2. Fill in your hotspot SSID/password and FastAPI endpoint URL.

```cpp
constexpr char WIFI_SSID[] = "MyHotspot";
constexpr char WIFI_PASSWORD[] = "SuperSecret";
constexpr char TELEMETRY_URL[] = "http://192.168.137.1:8000/telemetry";
constexpr char DEVICE_ID[] = "stm32-node-1";
```

## Build & Flash (PlatformIO)

```bash
# Optional: create a virtualenv first (python -m venv .venv && source .venv/bin/activate)
pip install platformio intelhex
pio run -d esp32_bridge
pio run -d esp32_bridge -t upload
pio device monitor -d esp32_bridge
```

The included `platformio.ini` targets the `esp32dev` board. Adjust it if you are using a different ESP32 module.

### Arduino IDE Alternative
- Add the ESP8266 board package via *Boards Manager*.
- Create a new sketch and copy `src/main.cpp` content.
- Place `credentials.h` next to the sketch.
- Select your board/port and upload.

## Runtime Behaviour
- Reconnects Wi-Fi every 10 seconds until successful.
- Synchronises time using `pool.ntp.org` every minute.
- Retries HTTP POST up to three times with exponential backoff.
- Prints diagnostic logs over the USB serial console (`Serial`) at 115200 baud.

## Expected STM32 Payload Format
Send newline-terminated JSON objects containing metric key/value pairs, for example:

```json
{"temperature_c": 23.7, "humidity_pct": 45.4}
```

The firmware wraps this fragment into the full payload expected by the FastAPI service.
