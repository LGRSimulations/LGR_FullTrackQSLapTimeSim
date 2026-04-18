from pathlib import Path

from fastapi import FastAPI
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from app.schemas import LapRunRequest, LiftCoastRequest
from app.services.lap_service import metadata, run_lap
from app.services.lift_coast_service import run_lift_coast


def create_app() -> FastAPI:
    app = FastAPI(title="LGR Sim Workbench", version="0.1.0")

    static_root = Path(__file__).resolve().parent / "static"
    app.mount("/static", StaticFiles(directory=str(static_root)), name="static")

    @app.get("/")
    def index() -> FileResponse:
        return FileResponse(static_root / "index.html")

    @app.get("/api/health")
    def health() -> dict:
        return {"status": "ok"}

    @app.get("/api/metadata")
    def get_metadata() -> dict:
        return metadata()

    @app.post("/api/lap/run")
    def run_lap_endpoint(req: LapRunRequest) -> dict:
        return run_lap(parameter_overrides=req.parameter_overrides, track_file_path=req.track_file_path)

    @app.post("/api/lift-coast/run")
    def run_lift_coast_endpoint(req: LiftCoastRequest) -> dict:
        return run_lift_coast(
            power_limits_kw=req.power_limits_kw,
            energy_target_kwh=req.energy_target_kwh,
            dt=req.dt,
            parameter_overrides=req.parameter_overrides,
        )

    return app
