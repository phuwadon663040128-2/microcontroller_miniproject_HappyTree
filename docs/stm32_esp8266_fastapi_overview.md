# STM32 ➔ ESP32 ➔ FastAPI Data Flow Overview

## High-Level Flow
1. **STM32 (Producer)**
   - Collects sensor data or computed metrics.
   - Frames data as newline-delimited JSON (e.g. `{ "temperature": 24.5, "units": "C" }`).
   - Streams payloads over USART6 (PA11/PA12) at 115200 baud to the ESP32 bridge.
2. **ESP32 (Bridge)**
   - Reads UART frames from STM32 and validates each JSON payload.
   - Connects to the computer's Wi-Fi hotspot as a station client.
   - Issues HTTPS-safe HTTP POST requests to the FastAPI endpoint with the JSON payload.
3. **Python FastAPI Service (Consumer)**
   - Exposes a `/telemetry` endpoint to receive JSON payloads.
   - Persists the most recent sample in memory and broadcasts it to connected browsers via Server-Sent Events (SSE).
   - Provides a lightweight web UI to visualize incoming data in real time.

## Data Contract
- **Transport:** HTTP POST from ESP32 to FastAPI.
- **Endpoint:** `/telemetry`
- **Content-Type:** `application/json`
- **Payload Schema:**
  ```json
  {
    "device_id": "stm32-node-1",
    "timestamp": "2025-10-09T12:34:56Z",
    "metrics": {
      "temperature_c": 24.5,
      "humidity_pct": 43.2
    }
  }
  ```
- ESP32 adds the `timestamp` when forwarding.
- STM32 must provide `device_id` and metric key/value pairs.

## Error Handling Strategy
- STM32 retransmits frames if it does not receive an "OK" acknowledgement from the ESP32 within 2 seconds (up to 3 retries with short backoff).
- ESP32 immediately returns "ERR" if a serial frame is truncated or exceeds 256 bytes.
- ESP32 retries HTTP POST up to three times with an exponential backoff (0.5 s, 1 s, 2 s).
- FastAPI responds with HTTP 202 on acceptance, 400 on validation issues.

## Hardware & Connectivity Notes
- **Debug Console:** USART2 on `PA2/PA3` remains dedicated to the ST-Link virtual COM port for logs.
- **Telemetry Link:** USART6 on `PA11 (TX)` ➔ ESP32 `RX2` and `PA12 (RX)` ⇦ ESP32 `TX2` for ACK feedback.
- Common ground between STM32, ESP32, and USB-UART if used.
- **Voltage Levels:** Use a 3.3 V logic-level UART; add a level shifter if the STM32 board exposes 5 V TTL.
- **Power:** Ensure ESP32 receives a stable 3.3 V supply up to 500 mA peak.

## Assumptions
- ESP32 firmware is built with the Arduino core (C++), using the `WiFi` and `HTTPClient` libraries.
- FastAPI app runs on the same computer that hosts the Wi-Fi hotspot (default `http://192.168.137.1:8000`).
- STM32 firmware leverages STM32Cube low-level drivers; USART2 remains dedicated to the PC console while USART6 carries telemetry to the ESP32.
- TLS termination is optional for lab setups; configure HTTPS in production.

## Next Steps
1. Implement the FastAPI service skeleton (`fastapi_gateway/`).
2. Develop the ESP32 firmware bridge (`esp32_bridge/`).
3. Integrate UART transmission in the STM32 project and test end-to-end.
