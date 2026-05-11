"""Allowlist of permitted dataset files. The web layer accepts short IDs only;
the IDs map to concrete paths beneath `datasets/`. Anything else is rejected."""
from __future__ import annotations

from pathlib import Path

from app.paths import repo_root

_TRACKS: dict[str, str] = {
    "FSUK": "datasets/tracks/FSUK.txt",
    "SkidpadF26": "datasets/tracks/SkidpadF26.txt",
    "StraightLineTrack": "datasets/tracks/StraightLineTrack.txt",
}

_TYRE_LATERAL: dict[str, str] = {
    "round_8_12psi": "datasets/vehicle/tyre_data/round_8_12_psi_lateral_load_tyredata_parsed.csv",
}

_TYRE_LONGITUDINAL: dict[str, str] = {
    "round_6_12psi": "datasets/vehicle/tyre_data/round_6_12_psi_longit_load_tyredata_parsed.csv",
}


def _lookup(table: dict[str, str], dataset_id: str, kind: str) -> Path:
    if not isinstance(dataset_id, str) or not dataset_id:
        raise ValueError(f"{kind} id must be a non-empty string")
    if dataset_id not in table:
        raise ValueError(f"Unknown {kind} id: {dataset_id!r}")
    candidate = repo_root() / table[dataset_id]
    resolved = candidate.resolve()
    root = repo_root().resolve()
    if root not in resolved.parents and resolved != root:
        raise ValueError(f"Resolved {kind} path escapes repo root")
    return resolved


def get_track_path(track_id: str) -> Path:
    return _lookup(_TRACKS, track_id, "track")


def get_tyre_lateral_path(dataset_id: str) -> Path:
    return _lookup(_TYRE_LATERAL, dataset_id, "tyre lateral")


def get_tyre_longitudinal_path(dataset_id: str) -> Path:
    return _lookup(_TYRE_LONGITUDINAL, dataset_id, "tyre longitudinal")


def list_track_ids() -> list[str]:
    return sorted(_TRACKS.keys())


def list_tyre_lateral_ids() -> list[str]:
    return sorted(_TYRE_LATERAL.keys())


def list_tyre_longitudinal_ids() -> list[str]:
    return sorted(_TYRE_LONGITUDINAL.keys())
