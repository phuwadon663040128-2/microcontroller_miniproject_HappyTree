from datetime import datetime, timezone
from typing import Dict, Optional

from pydantic import BaseModel, Field


class TelemetryPayload(BaseModel):
    device_id: str = Field(..., description="Unique identifier for the STM32 node")
    timestamp: datetime = Field(default_factory=lambda: datetime.now(timezone.utc))
    metrics: Dict[str, float] = Field(..., description="Key/value metric pairs, e.g. temperature_c")
    metadata: Optional[Dict[str, str]] = Field(
        default=None, description="Optional string metadata such as firmware version"
    )


class TelemetryResponse(BaseModel):
    status: str
    received_at: datetime
    payload: TelemetryPayload
