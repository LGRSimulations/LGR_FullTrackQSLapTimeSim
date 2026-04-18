from typing import Any

from pydantic import BaseModel, Field


class LapRunRequest(BaseModel):
    parameter_overrides: dict[str, Any] = Field(default_factory=dict)
    track_file_path: str | None = None


class LiftCoastRequest(BaseModel):
    power_limits_kw: list[float] = Field(default_factory=lambda: [10, 20, 30, 40, 50])
    energy_target_kwh: float = 0.5
    dt: float = 0.05
    parameter_overrides: dict[str, Any] = Field(default_factory=dict)
