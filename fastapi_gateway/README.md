# FastAPI Telemetry Gateway

This service receives telemetry JSON payloads from the ESP8266 bridge, stores the latest sample in memory, and streams real-time updates to the browser via Server-Sent Events (SSE).

## Features
- `POST /telemetry`: ingest telemetry from the ESP8266.
- `GET /telemetry/latest`: fetch the most recent telemetry sample.
- `GET /telemetry/stream`: real-time SSE stream for dashboards.
- `/`: Web dashboard that displays the latest payload.



uvicorn app.main:app --reload --host 0.0.0.0 --port 8000


The dashboard becomes available at <http://localhost:8000>. Expose port 8000 to the LAN if the ESP8266 is on Wi-Fi.

## Environment Variables
- `FASTAPI_ALLOW_ORIGINS` *(optional)*: comma-separated list of origins, defaults to `*`.


Open the dashboard or `GET /telemetry/latest` to verify the payload.
