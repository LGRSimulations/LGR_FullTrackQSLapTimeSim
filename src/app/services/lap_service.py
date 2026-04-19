import copy
import json

from app.paths import config_path, parameters_path, resolve_track_path


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


def run_lap(parameters: dict | None = None, config: dict | None = None) -> dict:
    from simulator.simulator import run_lap_time_simulation
    from track.track import load_track
    from vehicle.vehicle import create_vehicle

    if config is None:
        cfg = copy.deepcopy(_load_base_config())
    else:
        cfg = copy.deepcopy(config)

    if parameters is not None:
        cfg["vehicle_parameters"] = parameters

    track_file_path = cfg.get("track", {}).get("file_path", "datasets/tracks/FSUK.txt")
    cfg.setdefault("track", {})["file_path"] = resolve_track_path(track_file_path)

    vehicle = create_vehicle(cfg)
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
    params_json = _load_base_parameters()

    return {
        "app_name": "LGR Sim Workbench",
        "model_scope": "Quasi-static lap-time simulator (production v1)",
        "deferred_parameters": ["suspension_stiffness", "damping_coefficient"],
        "track_default": cfg.get("track", {}).get("file_path"),
        "parameter_file_sections": list(params_json.keys()),
    }
