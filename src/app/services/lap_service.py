import json
from pathlib import Path

from fastapi import HTTPException

from app.paths import config_path, parameters_path


def _load_base_config() -> dict:
    with open(config_path(), "r", encoding="utf-8") as f:
        return json.load(f)


def _load_base_parameters() -> dict:
    with open(parameters_path(), "r", encoding="utf-8") as f:
        data = json.load(f)
    data.pop("_comments", None)
    return data


def get_parameters() -> dict:
    return _load_base_parameters()


def get_config() -> dict:
    return _load_base_config()


def run_lap(parameters: dict | None = None, overrides=None) -> dict:
    from app.security.config_overrides import ConfigOverrides, build_config
    from simulator.simulator import run_lap_time_simulation
    from track.track import load_track
    from vehicle.vehicle import create_vehicle

    overrides = overrides if overrides is not None else ConfigOverrides()
    try:
        cfg = build_config(_load_base_config(), overrides)
    except ValueError as exc:
        raise HTTPException(status_code=422, detail=str(exc))

    if parameters is not None:
        cfg["vehicle_parameters"] = parameters

    try:
        vehicle = create_vehicle(cfg)
    except KeyError as e:
        raise HTTPException(status_code=422, detail=f"Missing parameter: {e}")
    track = load_track(cfg["track"]["file_path"], cfg.get("debug_mode", False))
    result = run_lap_time_simulation(track, vehicle, cfg, display=False)

    distances_m = [float(p.distance) for p in track.points]
    speeds_kmh = [float(s) * 3.6 for s in result.final_speeds]
    g_lat = [0.0] + [float(g) for g in result.g_lat_channel]
    g_long = [0.0] + [float(g) for g in result.g_long_channel]

    max_abs_g_lat = max((abs(g) for g in g_lat), default=0.0)
    max_abs_g_long = max((abs(g) for g in g_long), default=0.0)

    return {
        "lap_time_s": float(result.lap_time),
        "track_file_path": Path(cfg["track"]["file_path"]).name,
        "points": int(len(track.points)),
        "max_abs_g_lat": max_abs_g_lat,
        "max_abs_g_long": max_abs_g_long,
        "diagnostics": {
            "fallback_rate": float(result.diagnostics.get("fallback_rate", 0.0)),
            "corner_fallback_count": int(sum(1 for x in result.diagnostics.get("corner_fallback_used", []) if x)),
        },
        "telemetry": {
            "distances_m": distances_m,
            "speeds_kmh": speeds_kmh,
            "g_lat": g_lat,
            "g_long": g_long,
        },
    }


def metadata() -> dict:
    cfg = _load_base_config()
    params_json = _load_base_parameters()

    return {
        "app_name": "LGR Sim Workbench",
        "model_scope": "Quasi-static lap-time simulator (production v1)",
        "deferred_parameters": ["suspension_stiffness", "damping_coefficient"],
        "track_default": cfg.get("track", {}).get("file_path"),
        "parameter_file_sections": list(params_json.keys()),
    }
