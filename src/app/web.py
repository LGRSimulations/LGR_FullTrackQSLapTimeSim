import os
os.environ.setdefault("MPLBACKEND", "Agg")

import httpx
from fastapi import Depends, FastAPI, HTTPException, Request
from fastapi.responses import FileResponse, RedirectResponse
from fastapi.staticfiles import StaticFiles
from starlette.middleware.sessions import SessionMiddleware

from app.auth import build_oauth_client, load_auth_config, local_auth_bypass_email, register_auth_routes, require_user
from app.paths import lessons_dir, static_dir
from app.schemas import LapRunRequest, LiftCoastRequest, SweepRequest, TyreVerifyRequest, ChatRequest, ChatResponse
from app.services.lap_service import get_config, get_parameters, metadata, run_lap
from app.services.lift_coast_service import run_lift_coast
from app.services.sweep_service import run_sweep
from app.services.tyre_service import run_tyre_verify
from app.services.chat_service import chat


def _basename(path: str) -> str:
    if not path:
        return ""
    return path.replace("\\", "/").rsplit("/", 1)[-1]


def create_app() -> FastAPI:
    app = FastAPI(title="LGR Sim Workbench", version="0.1.0")

    auth_cfg = load_auth_config()
    oauth = build_oauth_client(auth_cfg)

    app.add_middleware(
        SessionMiddleware,
        secret_key=auth_cfg.session_secret,
        https_only=auth_cfg.cookie_secure,
        same_site="lax",
    )

    static_root = static_dir()
    lessons_root = lessons_dir()
    app.mount("/static", StaticFiles(directory=str(static_root)), name="static")
    app.mount("/lessons", StaticFiles(directory=str(lessons_root)), name="lessons")

    register_auth_routes(app, auth_cfg, oauth, static_root / "login.html")

    @app.get("/")
    def index(request: Request):
        if not request.session.get("email") and not local_auth_bypass_email():
            return RedirectResponse(url="/login", status_code=303)
        return FileResponse(static_root / "index.html")

    @app.get("/api/health")
    def health() -> dict:
        return {"status": "ok"}

    @app.get("/api/me")
    def me(email: str = Depends(require_user)) -> dict:
        return {"email": email}

    @app.get("/api/datasets")
    def list_datasets(email: str = Depends(require_user)) -> dict:
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
    def get_metadata(email: str = Depends(require_user)) -> dict:
        return metadata()

    @app.get("/api/parameters")
    def get_parameters_endpoint(email: str = Depends(require_user)) -> dict:
        return get_parameters()

    @app.get("/api/config")
    def get_config_endpoint(email: str = Depends(require_user)) -> dict:
        from app.security.dataset_registry import (
            get_track_label,
            get_tyre_lateral_label,
            get_tyre_longitudinal_label,
            lookup_powertrain_label_by_path,
            lookup_track_id_by_path,
            lookup_tyre_lateral_id_by_path,
            lookup_tyre_longitudinal_id_by_path,
        )
        cfg = get_config()
        max_brake_decel_g = cfg.get("solver", {}).get("max_brake_decel_g")

        powertrain_path = cfg.get("powertrain", {}).get("powertrain", "")
        track_path      = cfg.get("track", {}).get("file_path", "")
        tyre_lat_path   = cfg.get("tyre_model", {}).get("file_path_lateral", "")
        tyre_long_path  = cfg.get("tyre_model", {}).get("file_path_longit",  "")

        track_id    = lookup_track_id_by_path(track_path)
        tyre_lat_id = lookup_tyre_lateral_id_by_path(tyre_lat_path)
        tyre_long_id = lookup_tyre_longitudinal_id_by_path(tyre_long_path)

        powertrain_label = lookup_powertrain_label_by_path(powertrain_path) or _basename(powertrain_path)
        track_label    = get_track_label(track_id) if track_id else _basename(track_path)
        tyre_lat_label = get_tyre_lateral_label(tyre_lat_id) if tyre_lat_id else _basename(tyre_lat_path)
        tyre_long_label = get_tyre_longitudinal_label(tyre_long_id) if tyre_long_id else _basename(tyre_long_path)

        return {
            "full_telemetry_mode": bool(cfg.get("full_telemetry_mode", True)),
            "solver": {
                "use_rollover_speed_cap": bool(cfg.get("solver", {}).get("use_rollover_speed_cap", True)),
                "max_brake_decel_g": None if max_brake_decel_g is None else float(max_brake_decel_g),
            },
            "ambient_conditions": {
                "air_density": float(cfg.get("ambient_conditions", {}).get("air_density", 1.225)),
            },
            "datasets": {
                "powertrain":         {"label": powertrain_label},
                "track":              {"id": track_id,    "label": track_label},
                "tyre_lateral":       {"id": tyre_lat_id, "label": tyre_lat_label},
                "tyre_longitudinal":  {"id": tyre_long_id, "label": tyre_long_label},
            },
        }

    @app.post("/api/lap/run")
    def run_lap_endpoint(req: LapRunRequest, email: str = Depends(require_user)) -> dict:
        return run_lap(parameters=req.parameters, overrides=req.overrides)

    @app.get("/api/track/datasets")
    def list_track_datasets(email: str = Depends(require_user)) -> list[dict[str, str]]:
        from app.security.dataset_registry import list_tracks
        return list_tracks()

    @app.get("/api/tyre/datasets")
    def list_tyre_datasets(email: str = Depends(require_user)) -> dict:
        from app.security.dataset_registry import list_tyre_lateral, list_tyre_longitudinal
        return {
            "lateral": list_tyre_lateral(),
            "longitudinal": list_tyre_longitudinal(),
        }

    @app.post("/api/tyre/verify")
    def tyre_verify_endpoint(req: TyreVerifyRequest, email: str = Depends(require_user)) -> dict:
        return run_tyre_verify(
            lat_dataset=req.lat_dataset,
            long_dataset=req.long_dataset,
            model_variant=req.model_variant,
            rmse_threshold_pct=req.rmse_threshold_pct,
            base_mu=req.base_mu,
            mu_multiplier=req.mu_multiplier,
        )

    @app.post("/api/sweep/run")
    def run_sweep_endpoint(req: SweepRequest, email: str = Depends(require_user)) -> dict:
        return run_sweep(
            param=req.param,
            values=req.values,
            steps=req.steps,
            parameters=req.parameters,
            overrides=req.overrides,
        )

    @app.post("/api/lift-coast/run")
    def run_lift_coast_endpoint(req: LiftCoastRequest, email: str = Depends(require_user)) -> dict:
        return run_lift_coast(
            power_limits_kw=req.power_limits_kw,
            energy_target_kwh=req.energy_target_kwh,
            dt=req.dt,
            parameter_overrides=req.parameter_overrides,
        )

    @app.post("/api/chat")
    def chat_endpoint(req: ChatRequest, email: str = Depends(require_user)) -> ChatResponse:
        try:
            return chat(question=req.question, history=req.history)
        except FileNotFoundError:
            raise HTTPException(status_code=503, detail="Chat is not configured.")
        except httpx.HTTPError as exc:
            raise HTTPException(status_code=502, detail=str(exc))

    return app
