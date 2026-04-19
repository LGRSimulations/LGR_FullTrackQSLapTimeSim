import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from app.services.lap_service import get_parameters, get_config


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


from fastapi.testclient import TestClient
from app.web import create_app


def test_http_get_parameters():
    client = TestClient(create_app())
    resp = client.get("/api/parameters")
    assert resp.status_code == 200
    data = resp.json()
    assert "_comments" not in data
    assert "general" in data


def test_http_get_config():
    client = TestClient(create_app())
    resp = client.get("/api/config")
    assert resp.status_code == 200
    data = resp.json()
    assert "track" in data
    assert "powertrain" in data


def test_http_post_lap_run_null_body():
    client = TestClient(create_app())
    resp = client.post("/api/lap/run", json={})
    assert resp.status_code == 200
    data = resp.json()
    assert "lap_time_s" in data
