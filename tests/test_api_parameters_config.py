import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from app.services.lap_service import get_parameters, get_config
from fastapi.testclient import TestClient
from app.web import create_app


def test_get_parameters_excludes_comments():
    data = get_parameters()
    assert "_comments" not in data
    assert "general" in data
    assert "aerodynamics" in data
    assert "geometry" in data
    assert "vehicle_dynamics" in data
    assert "drivetrain" in data


def test_get_config_has_expected_keys():
    data = get_config()
    assert "track" in data
    assert "powertrain" in data
    assert "tyre_model" in data


def test_http_get_config_sanitised_shape():
    client = TestClient(create_app())
    resp = client.get("/api/config")
    assert resp.status_code == 200
    data = resp.json()
    assert set(data.keys()) == {"debug_mode", "full_telemetry_mode", "solver", "ambient_conditions"}
    assert data["debug_mode"] is False
    assert "use_rollover_speed_cap" in data["solver"]
    assert "max_brake_decel_g" in data["solver"]
    assert "air_density" in data["ambient_conditions"]
    assert "track" not in data
    assert "powertrain" not in data
    assert "tyre_model" not in data
    assert "vehicle_parameters" not in data


def test_http_get_parameters():
    client = TestClient(create_app())
    resp = client.get("/api/parameters")
    assert resp.status_code == 200
    data = resp.json()
    assert "_comments" not in data
    assert "general" in data


def test_http_get_config_returns_200():
    client = TestClient(create_app())
    resp = client.get("/api/config")
    assert resp.status_code == 200


def test_http_post_lap_run_null_body():
    client = TestClient(create_app())
    resp = client.post("/api/lap/run", json={})
    assert resp.status_code == 200
    data = resp.json()
    assert "lap_time_s" in data
