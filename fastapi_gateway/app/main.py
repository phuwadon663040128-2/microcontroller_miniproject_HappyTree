import asyncio
from datetime import datetime
from pathlib import Path
from typing import AsyncGenerator, Optional

from fastapi import Depends, FastAPI, HTTPException, Request, status
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from sse_starlette.sse import EventSourceResponse

try:
    from .schemas import TelemetryPayload, TelemetryResponse
except ImportError:  # pragma: no cover - fallback when running as a script
    from schemas import TelemetryPayload, TelemetryResponse

BASE_DIR = Path(__file__).resolve().parent


class TelemetryBroker:
    """In-memory store and fan-out for telemetry samples."""

    def __init__(self) -> None:
        self._subscribers: set[asyncio.Queue[TelemetryPayload]] = set()
        self._lock = asyncio.Lock()
        self._latest: Optional[TelemetryPayload] = None

    async def publish(self, payload: TelemetryPayload) -> None:
        async with self._lock:
            self._latest = payload
            if not self._subscribers:
                return
            for queue in list(self._subscribers):
                await queue.put(payload)

    async def subscribe(self) -> asyncio.Queue[TelemetryPayload]:
        queue: asyncio.Queue[TelemetryPayload] = asyncio.Queue(maxsize=10)
        async with self._lock:
            self._subscribers.add(queue)
        return queue

    async def unsubscribe(self, queue: asyncio.Queue[TelemetryPayload]) -> None:
        async with self._lock:
            self._subscribers.discard(queue)

    async def latest(self) -> Optional[TelemetryPayload]:
        async with self._lock:
            return self._latest


broker = TelemetryBroker()

app = FastAPI(title="STM32 Telemetry Gateway", version="0.1.0")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

static_dir = BASE_DIR / "static"
app.mount("/static", StaticFiles(directory=static_dir), name="static")

templates = Jinja2Templates(directory=str(BASE_DIR / "templates"))


async def get_broker() -> TelemetryBroker:
    return broker


@app.post("/telemetry", response_model=TelemetryResponse, status_code=status.HTTP_202_ACCEPTED)
async def ingest_telemetry(
    payload: TelemetryPayload,
    broker: TelemetryBroker = Depends(get_broker),
) -> TelemetryResponse:
    """Receive telemetry samples from the ESP32 bridge."""

    await broker.publish(payload)
    response = TelemetryResponse(status="accepted", received_at=datetime.utcnow(), payload=payload)
    return response


@app.get("/telemetry/latest", response_model=TelemetryResponse)
async def latest_sample(broker: TelemetryBroker = Depends(get_broker)) -> TelemetryResponse:
    latest = await broker.latest()
    if latest is None:
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="No telemetry received yet")
    return TelemetryResponse(status="ok", received_at=datetime.utcnow(), payload=latest)


async def telemetry_event_stream(
    broker: TelemetryBroker,
) -> AsyncGenerator[str, None]:
    queue = await broker.subscribe()
    try:
        latest = await broker.latest()
        if latest is not None:
            yield f"data: {latest.json()}\n\n"
        while True:
            payload = await queue.get()
            yield f"data: {payload.json()}\n\n"
    except asyncio.CancelledError:
        raise
    finally:
        await broker.unsubscribe(queue)


@app.get("/telemetry/stream")
async def stream(broker: TelemetryBroker = Depends(get_broker)) -> EventSourceResponse:
    return EventSourceResponse(telemetry_event_stream(broker))


@app.get("/", response_class=HTMLResponse)
async def index(request: Request, broker: TelemetryBroker = Depends(get_broker)) -> HTMLResponse:
    latest = await broker.latest()
    return templates.TemplateResponse(
        "index.html",
        {
            "request": request,
            "latest": latest.dict() if latest else None,
        },
    )


@app.get("/healthz")
async def healthcheck() -> JSONResponse:
    return JSONResponse({"status": "ok", "time": datetime.utcnow().isoformat()})
