import os

import pytest

from app.auth import AuthConfig, load_auth_config


def test_load_auth_config_returns_values_from_env(monkeypatch):
    monkeypatch.setenv("GOOGLE_CLIENT_ID", "cid")
    monkeypatch.setenv("GOOGLE_CLIENT_SECRET", "csec")
    monkeypatch.setenv("SESSION_SECRET", "ssec")
    monkeypatch.setenv("APP_BASE_URL", "http://localhost:8000")
    monkeypatch.delenv("COOKIE_SECURE", raising=False)
    cfg = load_auth_config()
    assert cfg.google_client_id == "cid"
    assert cfg.google_client_secret == "csec"
    assert cfg.session_secret == "ssec"
    assert cfg.app_base_url == "http://localhost:8000"
    assert cfg.cookie_secure is False


def test_load_auth_config_respects_cookie_secure_true(monkeypatch):
    monkeypatch.setenv("GOOGLE_CLIENT_ID", "cid")
    monkeypatch.setenv("GOOGLE_CLIENT_SECRET", "csec")
    monkeypatch.setenv("SESSION_SECRET", "ssec")
    monkeypatch.setenv("APP_BASE_URL", "https://x.fly.dev")
    monkeypatch.setenv("COOKIE_SECURE", "true")
    cfg = load_auth_config()
    assert cfg.cookie_secure is True


def test_load_auth_config_raises_if_any_required_var_missing(monkeypatch):
    for missing in ("GOOGLE_CLIENT_ID", "GOOGLE_CLIENT_SECRET", "SESSION_SECRET", "APP_BASE_URL"):
        monkeypatch.setenv("GOOGLE_CLIENT_ID", "cid")
        monkeypatch.setenv("GOOGLE_CLIENT_SECRET", "csec")
        monkeypatch.setenv("SESSION_SECRET", "ssec")
        monkeypatch.setenv("APP_BASE_URL", "http://localhost:8000")
        monkeypatch.delenv(missing)
        with pytest.raises(RuntimeError, match=missing):
            load_auth_config()


def test_load_auth_config_strips_trailing_slash_from_base_url(monkeypatch):
    monkeypatch.setenv("GOOGLE_CLIENT_ID", "cid")
    monkeypatch.setenv("GOOGLE_CLIENT_SECRET", "csec")
    monkeypatch.setenv("SESSION_SECRET", "ssec")
    monkeypatch.setenv("APP_BASE_URL", "http://localhost:8000/")
    cfg = load_auth_config()
    assert cfg.app_base_url == "http://localhost:8000"


def test_auth_config_redirect_uri_is_callback_under_base_url(monkeypatch):
    monkeypatch.setenv("GOOGLE_CLIENT_ID", "cid")
    monkeypatch.setenv("GOOGLE_CLIENT_SECRET", "csec")
    monkeypatch.setenv("SESSION_SECRET", "ssec")
    monkeypatch.setenv("APP_BASE_URL", "https://x.fly.dev")
    cfg = load_auth_config()
    assert cfg.redirect_uri == "https://x.fly.dev/auth/callback"
