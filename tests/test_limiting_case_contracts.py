import copy
import json
import os
import sys
import unittest

sys.path.insert(0, os.path.abspath("src"))

from simulator.util.calcSpeedProfile import (
    _longitudinal_budget_scale_from_lateral_demand,
    backward_pass,
    forward_pass,
)
from simulator.util.vehicleDynamics import find_vehicle_state_at_point
from track.track import Track, TrackPoint
from vehicle.vehicle import create_vehicle


def _build_straight_track(ds=10.0, n_points=4):
    points = []
    for i in range(n_points):
        points.append(
            TrackPoint(
                distance=float(i * ds),
                x=float(i * ds),
                y=0.0,
                z=0.0,
                curvature=0.0,
                heading=0.0,
                elevation_angle=0.0,
            )
        )
    return Track(points, is_closed=False)


class LimitingCaseContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with open("config.json", "r") as f:
            cls.base_config = json.load(f)

    def _create_vehicle(self):
        cfg = copy.deepcopy(self.base_config)
        cfg.setdefault("ab_testing", {})["model_variant"] = "baseline"
        cfg.setdefault("solver", {})["use_rollover_speed_cap"] = True
        return create_vehicle(cfg), cfg

    def test_near_zero_curvature_returns_straight_cap_and_zero_angles(self):
        vehicle, _ = self._create_vehicle()
        result = find_vehicle_state_at_point(
            curvature=0.0,
            vehicle=vehicle,
            straight_line_speed_cap=37.5,
        )

        self.assertTrue(result["success"])
        self.assertAlmostEqual(result["v_car"], 37.5, places=6)
        self.assertAlmostEqual(result["a_steer"], 0.0, places=6)
        self.assertAlmostEqual(result["a_sideslip"], 0.0, places=6)

    def test_longitudinal_budget_scale_bounds_and_endpoints(self):
        self.assertAlmostEqual(_longitudinal_budget_scale_from_lateral_demand(0.0, 5000.0), 1.0, places=8)
        self.assertAlmostEqual(_longitudinal_budget_scale_from_lateral_demand(5000.0, 5000.0), 0.0, places=8)
        self.assertAlmostEqual(_longitudinal_budget_scale_from_lateral_demand(7000.0, 5000.0), 0.0, places=8)

        mid = _longitudinal_budget_scale_from_lateral_demand(2500.0, 5000.0)
        self.assertGreater(mid, 0.0)
        self.assertLess(mid, 1.0)

    def test_zero_powertrain_torque_cannot_accelerate_forward_pass(self):
        vehicle, cfg = self._create_vehicle()
        track = _build_straight_track(ds=10.0, n_points=4)
        point_speeds = [20.0, 20.0, 20.0, 20.0]

        original_get_torque = vehicle.power_unit.get_torque
        try:
            vehicle.power_unit.get_torque = lambda rpm, throttle=1.0: 0.0
            speeds, diagnostics = forward_pass(track, vehicle, point_speeds, cfg)
        finally:
            vehicle.power_unit.get_torque = original_get_torque

        self.assertTrue(all(abs(v) <= 1e-9 for v in diagnostics["forward_powertrain_force"]))
        self.assertLessEqual(float(speeds[1]), float(speeds[0]) + 1e-9)
        self.assertLessEqual(float(speeds[2]), float(speeds[1]) + 1e-9)
        self.assertLessEqual(float(speeds[3]), float(speeds[2]) + 1e-9)

    def test_zero_speed_backward_pass_stays_zero(self):
        vehicle, cfg = self._create_vehicle()
        track = _build_straight_track(ds=10.0, n_points=4)
        point_speeds = [0.0, 0.0, 0.0, 0.0]

        speeds, diagnostics = backward_pass(track, vehicle, point_speeds, cfg)
        self.assertTrue(all(abs(float(v)) <= 1e-9 for v in speeds))
        self.assertEqual(len(diagnostics["backward_limiting_mode"]), len(point_speeds))


if __name__ == "__main__":
    unittest.main()