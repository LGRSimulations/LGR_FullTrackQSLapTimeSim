from typing import Any

from pydantic import BaseModel, Field

from app.security.config_overrides import ConfigOverrides


class LapRunRequest(BaseModel):
    parameters: dict[str, float | int | bool] | None = Field(default=None)
    overrides: ConfigOverrides = Field(default_factory=ConfigOverrides)


class LiftCoastRequest(BaseModel):
    power_limits_kw: list[float] = Field(default_factory=lambda: [10, 20, 30, 40, 50])
    energy_target_kwh: float = 0.5
    dt: float = 0.05
    parameter_overrides: dict[str, Any] = Field(default_factory=dict)


class TyreVerifyRequest(BaseModel):
    lat_dataset: str = Field(min_length=1, max_length=64)
    long_dataset: str = Field(min_length=1, max_length=64)
    model_variant: str = Field(default="tyre_peak_load_clamp", max_length=64)
    rmse_threshold_pct: float = Field(default=12.0, ge=0.0, le=100.0)
    base_mu: float = Field(default=1.0, ge=0.0, le=5.0)


class SweepRequest(BaseModel):
    param: str = Field(min_length=1, max_length=64)
    values: str = Field(min_length=1, max_length=512)
    steps: int = Field(default=5, ge=2, le=50)
    parameters: dict[str, float | int | bool] | None = Field(default=None)
    overrides: ConfigOverrides = Field(default_factory=ConfigOverrides)


class ChatSource(BaseModel):
    file: str
    section: str


class ChatRequest(BaseModel):
    question: str = Field(min_length=1, max_length=1000)
    history: list[dict] = Field(default_factory=list)


class ChatResponse(BaseModel):
    answer: str
    sources: list[ChatSource]
