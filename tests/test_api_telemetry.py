import pytest
from fastapi.testclient import TestClient
from app.web import create_app


@pytest.fixture
def client():
    return TestClient(create_app())


def test_run_lap_returns_telemetry_key(client):
    res = client.post('/api/lap/run', json={})
    assert res.status_code == 200
    assert 'telemetry' in res.json()


def test_telemetry_arrays_all_same_length(client):
    res = client.post('/api/lap/run', json={})
    telem = res.json()['telemetry']
    n = len(telem['distances_m'])
    assert n > 0
    assert len(telem['speeds_kmh']) == n
    assert len(telem['g_lat']) == n
    assert len(telem['g_long']) == n


def test_telemetry_distances_start_at_zero(client):
    res = client.post('/api/lap/run', json={})
    telem = res.json()['telemetry']
    assert telem['distances_m'][0] == pytest.approx(0.0)
    assert telem['distances_m'][-1] > 0


def test_telemetry_speeds_positive(client):
    res = client.post('/api/lap/run', json={})
    telem = res.json()['telemetry']
    assert all(s > 0 for s in telem['speeds_kmh'])
