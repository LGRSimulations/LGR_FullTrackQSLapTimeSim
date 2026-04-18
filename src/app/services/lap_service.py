import copy
import json
from pathlib import Path

from app.paths import config_path, resolve_track_path


def _load_base_config() -> dict:
    with open(config_path(), "r", encoding="utf-8") as f:
        return json.load(f)


def _apply_aliases(overrides: dict) -> dict:
    alias_map = {
        "aero_cp": "aero_centre_of_pressure",
    }
    fixed = {}
    for key, value in overrides.items():
        fixed[alias_map.get(key, key)] = value
    return fixed


def run_lap(parameter_overrides: dict | None = None, track_file_path: str | None = None) -> dict:
    # Local imports keep startup light and avoid importing heavy libs for metadata endpoints.
    from simulator.simulator import run_lap_time_simulation
    from track.track import load_track
    from vehicle.vehicle import create_vehicle

    cfg = _load_base_config()
    cfg = copy.deepcopy(cfg)

    if track_file_path:
        cfg.setdefault("track", {})["file_path"] = resolve_track_path(track_file_path)
    else:
        cfg.setdefault("track", {})["file_path"] = resolve_track_path(cfg["track"]["file_path"])

    vehicle = create_vehicle(cfg)

    for key, value in _apply_aliases(parameter_overrides or {}).items():
        if hasattr(vehicle.params, key):
            setattr(vehicle.params, key, value)

    track = load_track(cfg["track"]["file_path"], cfg.get("debug_mode", False))
    result = run_lap_time_simulation(track, vehicle, cfg, display=False)

    max_abs_g_lat = max([abs(float(g)) for g in result.g_lat_channel], default=0.0)
    max_abs_g_long = max([abs(float(g)) for g in result.g_long_channel], default=0.0)

    return {
        "lap_time_s": float(result.lap_time),
        "track_file_path": cfg["track"]["file_path"],
        "points": int(len(track.points)),
        "max_abs_g_lat": max_abs_g_lat,
        "max_abs_g_long": max_abs_g_long,
        "diagnostics": {
            "fallback_rate": float(result.diagnostics.get("fallback_rate", 0.0)),
            "corner_fallback_count": int(sum(1 for x in result.diagnostics.get("corner_fallback_used", []) if x)),
        },
    }


def metadata() -> dict:
    cfg = _load_base_config()
    params_file = Path(config_path()).with_name("parameters.json")
    with open(params_file, "r", encoding="utf-8") as f:
        params_json = json.load(f)

    return {
        "app_name": "LGR Sim Workbench",
        "model_scope": "Quasi-static lap-time simulator (production v1)",
        "deferred_parameters": ["suspension_stiffness", "damping_coefficient"],
        "track_default": cfg.get("track", {}).get("file_path"),
        "parameter_file_sections": list(params_json.keys()),
    }
