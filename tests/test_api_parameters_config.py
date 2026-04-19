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
