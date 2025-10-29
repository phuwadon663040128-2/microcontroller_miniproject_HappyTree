# End-to-End Setup Guide

This guide explains how to transmit telemetry from the STM32 board through an ESP8266 Wi-Fi bridge into the FastAPI dashboard.

## 1. STM32 Firmware
1. Keep **USART2** (`PA2/PA3`) at 115200 baud for the PC debug console.
2. Enable **USART6** at 115200 baud, 8-N-1 for telemetry. On the Nucleo-F411, this maps to `PA11` (TX to ESP8266) and `PA12` (RX for ACKs).
3. Serialize telemetry as JSON and terminate each frame with a newline (`\n`). Example (HAL pseudo-code):
   ```c
   printf("{\"temperature_c\": %.2f, \"humidity_pct\": %.2f}\n", temp, humidity);
   ```
4. After writing a frame, block (with timeout) waiting for a short `OK` response from the ESP8266. If `ERR` is received or no response arrives within 100 ms, resend the frame.
5. Ensure the STM32 pins expose **3.3 V logic**; add a level shifter if required.

## 2. ESP8266 Bridge
1. Copy `esp8266_bridge/include/credentials.sample.h` ➜ `credentials.h` and edit the values.
2. Flash the firmware:
   ```bash
   pip install platformio
   pio run -d esp8266_bridge
   pio run -d esp8266_bridge -t upload
   ```
3. Monitor logs:
   ```bash
   pio device monitor -d esp8266_bridge
   ```
4. Wire STM32 `PA11` (USART6_TX) ➜ ESP8266 `GPIO13` (RX after `Serial.swap()`), and optionally `PA12` (USART6_RX) ➜ `GPIO15` for acknowledgements.
5. Power the ESP8266 from a stable 3.3 V source (>= 300 mA burst).

## 3. FastAPI Gateway
1. Create a virtual environment and install dependencies:
   ```bash
   cd fastapi_gateway
   python -m venv .venv
   source .venv/bin/activate
   pip install -r requirements.txt
   ```
2. Run the server:
   ```bash
   uvicorn app.main:app --host 0.0.0.0 --port 8000
   ```
3. Open <http://localhost:8000> (or the hotspot IP) to watch the live telemetry.

## 4. Network Topology
- Enable a computer hotspot and note its IP (e.g. `192.168.137.1`).
- Connect the ESP8266 to the hotspot via credentials in `credentials.h`.
- Keep the FastAPI server running on the same computer; ensure firewall allows inbound port 8000.

## 5. Validation Checklist
- **FastAPI**: `curl http://localhost:8000/healthz` returns `{"status":"ok", ...}`.
- **ESP8266**: Serial monitor prints `[HTTP] Delivered (202)` when packets arrive.
- **STM32**: Receives `OK` after each frame, `ERR` indicates resend required.
- **Dashboard**: New payloads appear instantly in the browser.

## 6. Optional Enhancements
- Persist telemetry in a database (PostgreSQL, InfluxDB) for historical analysis.
- Add TLS/HTTPS via a reverse proxy (Caddy, Nginx).
- Extend STM32 packets with diagnostics (RSSI, battery voltage) as additional metrics.
