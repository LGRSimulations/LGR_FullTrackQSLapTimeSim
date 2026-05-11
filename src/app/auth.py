"""Google OAuth gate. One module owns env-loading, the Authlib client,
the FastAPI session dependency, and the route factory."""
from __future__ import annotations

import os
from dataclasses import dataclass

from authlib.integrations.starlette_client import OAuth, OAuthError
from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import FileResponse, RedirectResponse


_REQUIRED_ENV = ("GOOGLE_CLIENT_ID", "GOOGLE_CLIENT_SECRET", "SESSION_SECRET", "APP_BASE_URL")


@dataclass(frozen=True)
class AuthConfig:
    google_client_id: str
    google_client_secret: str
    session_secret: str
    app_base_url: str
    cookie_secure: bool

    @property
    def redirect_uri(self) -> str:
        return f"{self.app_base_url}/auth/callback"


def load_auth_config() -> AuthConfig:
    """Read the required env vars or fail loudly. Strips a trailing slash from APP_BASE_URL."""
    missing = [v for v in _REQUIRED_ENV if not os.environ.get(v)]
    if missing:
        raise RuntimeError(
            f"Missing required environment variables for auth: {', '.join(missing)}"
        )
    base = os.environ["APP_BASE_URL"].rstrip("/")
    cookie_secure = os.environ.get("COOKIE_SECURE", "false").lower() in ("1", "true", "yes")
    return AuthConfig(
        google_client_id=os.environ["GOOGLE_CLIENT_ID"],
        google_client_secret=os.environ["GOOGLE_CLIENT_SECRET"],
        session_secret=os.environ["SESSION_SECRET"],
        app_base_url=base,
        cookie_secure=cookie_secure,
    )


def build_oauth_client(cfg: AuthConfig) -> OAuth:
    oauth = OAuth()
    oauth.register(
        name="google",
        client_id=cfg.google_client_id,
        client_secret=cfg.google_client_secret,
        server_metadata_url="https://accounts.google.com/.well-known/openid-configuration",
        client_kwargs={"scope": "openid email profile"},
    )
    return oauth


def require_user(request: Request) -> str:
    """FastAPI dependency. Returns the user's email if signed in, else 401."""
    email = request.session.get("email") if hasattr(request, "session") else None
    if not email:
        raise HTTPException(status_code=401, detail="Authentication required")
    return email


def register_auth_routes(app: FastAPI, cfg: AuthConfig, oauth: OAuth, login_html_path) -> None:
    """Attach /login, /auth/callback, /logout to the given app."""

    @app.get("/login")
    def login_page() -> FileResponse:
        return FileResponse(str(login_html_path))

    @app.get("/auth/start")
    async def auth_start(request: Request):
        return await oauth.google.authorize_redirect(request, cfg.redirect_uri)

    @app.get("/auth/callback")
    async def auth_callback(request: Request):
        try:
            token = await oauth.google.authorize_access_token(request)
        except OAuthError as exc:
            raise HTTPException(status_code=400, detail=f"OAuth error: {exc.error}")
        userinfo = token.get("userinfo")
        if not userinfo or not userinfo.get("email"):
            raise HTTPException(status_code=400, detail="Google did not return an email")
        if not userinfo.get("email_verified", False):
            raise HTTPException(status_code=403, detail="Email is not verified by Google")
        request.session["email"] = userinfo["email"]
        return RedirectResponse(url="/", status_code=303)

    @app.post("/logout")
    def logout(request: Request) -> RedirectResponse:
        request.session.clear()
        return RedirectResponse(url="/login", status_code=303)
