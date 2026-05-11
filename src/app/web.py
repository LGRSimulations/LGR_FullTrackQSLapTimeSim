import httpx
from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from app.paths import lessons_dir, static_dir
from app.schemas import LapRunRequest, LiftCoastRequest, SweepRequest, TyreVerifyRequest, ChatRequest, ChatResponse
from app.services.lap_service import get_config, get_parameters, metadata, run_lap
from app.services.lift_coast_service import run_lift_coast
from app.services.sweep_service import run_sweep
from app.services.tyre_service import run_tyre_verify
from app.services.chat_service import chat


def create_app() -> FastAPI:
    app = FastAPI(title="LGR Sim Workbench", version="0.1.0")

    static_root = static_dir()
    lessons_root = lessons_dir()
    app.mount("/static", StaticFiles(directory=str(static_root)), name="static")
    app.mount("/lessons", StaticFiles(directory=str(lessons_root)), name="lessons")

    @app.get("/")
    def index() -> FileResponse:
        return FileResponse(static_root / "index.html")

    @app.get("/api/health")
    def health() -> dict:
        return {"status": "ok"}

    @app.get("/api/datasets")
    def list_datasets() -> dict:
        from app.security.dataset_registry import (
            list_track_ids,
            list_tyre_lateral_ids,
            list_tyre_longitudinal_ids,
        )
        return {
            "tracks": list_track_ids(),
            "tyre_lateral": list_tyre_lateral_ids(),
            "tyre_longitudinal": list_tyre_longitudinal_ids(),
        }

    @app.get("/api/metadata")
    def get_metadata() -> dict:
        return metadata()

    @app.get("/api/parameters")
    def get_parameters_endpoint() -> dict:
        return get_parameters()

    @app.get("/api/config")
    def get_config_endpoint() -> dict:
        cfg = get_config()
        return {
            "debug_mode": False,
            "full_telemetry_mode": bool(cfg.get("full_telemetry_mode", True)),
            "solver": {
                "use_rollover_speed_cap": bool(cfg.get("solver", {}).get("use_rollover_speed_cap", True)),
                "max_brake_decel_g": float(cfg.get("solver", {}).get("max_brake_decel_g", 2.0)),
            },
            "ambient_conditions": {
                "air_density": float(cfg.get("ambient_conditions", {}).get("air_density", 1.225)),
            },
        }

    @app.post("/api/lap/run")
    def run_lap_endpoint(req: LapRunRequest) -> dict:
        return run_lap(parameters=req.parameters, overrides=req.overrides)

    @app.post("/api/tyre/verify")
    def tyre_verify_endpoint(req: TyreVerifyRequest) -> dict:
        return run_tyre_verify(
            lat_dataset=req.lat_dataset,
            long_dataset=req.long_dataset,
            model_variant=req.model_variant,
            rmse_threshold_pct=req.rmse_threshold_pct,
            base_mu=req.base_mu,
        )

    @app.post("/api/sweep/run")
    def run_sweep_endpoint(req: SweepRequest) -> dict:
        return run_sweep(
            param=req.param,
            values=req.values,
            steps=req.steps,
            parameters=req.parameters,
            overrides=req.overrides,
        )

    @app.post("/api/lift-coast/run")
    def run_lift_coast_endpoint(req: LiftCoastRequest) -> dict:
        return run_lift_coast(
            power_limits_kw=req.power_limits_kw,
            energy_target_kwh=req.energy_target_kwh,
            dt=req.dt,
            parameter_overrides=req.parameter_overrides,
        )

    @app.post("/api/chat")
    def chat_endpoint(req: ChatRequest) -> ChatResponse:
        try:
            return chat(question=req.question, history=req.history)
        except FileNotFoundError:
            raise HTTPException(status_code=503, detail="Chat is not configured.")
        except httpx.HTTPError as exc:
            raise HTTPException(status_code=502, detail=str(exc))

    return app
