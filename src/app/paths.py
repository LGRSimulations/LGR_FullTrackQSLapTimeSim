import os
import sys
from pathlib import Path


def app_root() -> Path:
    if getattr(sys, "frozen", False):
        return Path(sys.executable).resolve().parent
    return Path(__file__).resolve().parents[2]


def repo_root() -> Path:
    root = app_root()
    if (root / "config.json").exists():
        return root
    return Path(__file__).resolve().parents[2]


def config_path() -> Path:
    return repo_root() / "config.json"


def parameters_path() -> Path:
    return repo_root() / "parameters.json"


def static_dir() -> Path:
    return Path(__file__).resolve().parent / "static"


def resolve_track_path(track_path: str) -> str:
    p = Path(track_path)
    if p.is_absolute():
        return str(p)
    resolved = repo_root() / p
    if resolved.exists():
        return str(resolved)
    return str(Path(os.path.abspath(track_path)))
