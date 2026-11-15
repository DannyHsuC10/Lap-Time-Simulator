import unittest
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch

import numpy as np

from lap_sim_lab.analysis import AttitudeSweep, sweep_tires
from lap_sim_lab.car import Car
from lap_sim_lab.ggv import build_ggv
from lap_sim_lab.planner import (
    AeroTireLimits, SegmentSpeedPath, integrate_forward, max_entry_speed_for_braking,
)
from lap_sim_lab.settings import Settings
from lap_sim_lab.simulation import simulate
from lap_sim_lab.tire import FrictionEllipse
from lap_sim_lab.timeout_recovery import describe_model_changes, lower_model_choices, timeout_suggestions


class SolverTests(unittest.TestCase):
    def test_tire_sweep_holds_static_loads_and_uses_ay_only_for_force_demand(self):
        settings = Settings(
            tire_model="ellipse", ellipse_mu_x=1.0, ellipse_mu_y=1.0,
            aero_path=Path("missing-aero.json"), mass=300.0, wheelbase=2.0, cg_from_front=0.8,
        )
        baseline = sweep_tires(settings, AttitudeSweep())
        corner = sweep_tires(settings, AttitudeSweep(speed_kmh=144, heave_mm=20, ax_g=2, ay_g=0.6))
        expected = 300.0 * 9.81 * np.array([0.3, 0.3, 0.2, 0.2])
        np.testing.assert_allclose(baseline.fy_peak[0], expected)
        np.testing.assert_allclose(corner.fy_peak, baseline.fy_peak)
        np.testing.assert_allclose(corner.fx_drive, 0.8 * baseline.fx_drive)

    def test_ggv_point_mass_slices_follow_lateral_grip_and_speed_range(self):
        settings = Settings(
            tire_model="ellipse", load_model="point_mass", aero_enabled=False,
            ellipse_mu_y=1.5, max_speed=20,
        )
        slices = build_ggv(settings)
        self.assertEqual([part.speed_kmh for part in slices], [0, 18, 36, 54, 72])
        for part in slices:
            np.testing.assert_allclose(part.ay_g[[0, 20, -1]], [-1.47, 0, 1.47], atol=1e-4)
            self.assertTrue(np.isfinite(part.drive_g).all() and np.isfinite(part.brake_g).all())
            self.assertGreaterEqual(part.drive_g[20], part.drive_g[0])
            self.assertLess(part.brake_g[20], part.brake_g[0])

    def test_lateral_capacity_respects_axle_yaw_balance_and_weakest_wheel(self):
        car = Car(Settings(wheelbase=2.0, cg_from_front=1.0), preload_motor=False)
        limits = AeroTireLimits(FrictionEllipse(1.0, 1.0), car)
        loads = np.array([100.0, 300.0, 600.0, 600.0])
        fy = limits.balanced_lateral_limits(loads, np.zeros(4))
        np.testing.assert_allclose(fy, [100.0, 300.0, 200.0, 200.0])
        self.assertAlmostEqual(car.lf * fy[:2].sum(), car.lr * fy[2:].sum())
        self.assertIsNone(limits.fx_limits_for_loads_and_fy(loads, limits.lateral_forces(900, loads)))
        self.assertIsNotNone(limits.fx_limits_for_loads_and_fy(loads, fy))
        unloaded = np.array([0.0, 0.0, 600.0, 600.0])
        self.assertFalse(limits.balanced_lateral_limits(unloaded, np.zeros(4)).any())
        self.assertIsNone(limits.fx_limits_for_loads_and_fy(unloaded, limits.lateral_forces(100, unloaded)))

    def test_timeout_recovery_lowers_each_selected_model(self):
        settings = Settings(
            tire_model="superellipse", motor_model="lp03",
            brake_model="hydraulic", curvature_model="smooth",
        )
        changes = lower_model_choices(settings)
        self.assertEqual(changes, {
            "tire_model": "ellipse", "load_model": "point_mass", "motor_model": "curve",
            "brake_model": "leverage", "aero_model": "quadratic", "curvature_model": "raw",
        })
        replace(settings, **changes).validate()
        self.assertIn("超橢圓 >> 摩擦橢圓", describe_model_changes(settings, changes))

    def test_timeout_recovery_keeps_point_mass_compatible(self):
        settings = Settings(
            tire_model="table", ellipse_source="fit", yaw_correction=True,
            aero_enabled=False,
        )
        changes = lower_model_choices(settings)
        self.assertEqual(changes["tire_model"], "ellipse")
        self.assertEqual(changes["ellipse_source"], "fixed")
        self.assertIs(changes["yaw_correction"], False)
        intermediate = replace(settings, **changes)
        intermediate.validate()
        self.assertEqual(lower_model_choices(intermediate), {"rolling_model": "off"})
        lowest = replace(intermediate, rolling_model="off")
        lowest.validate()
        self.assertEqual(lower_model_choices(lowest), {})
        self.assertIn("3 輪降到 2 輪", timeout_suggestions(lowest))
        self.assertIn("0.5 m 加大到 1 m", timeout_suggestions(lowest))
        self.assertNotIn("掃描次數", timeout_suggestions(replace(lowest, solver="distance")))

    def test_time_limit_checked_while_reading_track(self):
        with patch("lap_sim_lab.simulation.monotonic", side_effect=(10.0, 10.0, 11.0)):
            with self.assertRaisesRegex(TimeoutError, "1 秒"):
                simulate(Settings(time_limit_s=1))

    def test_feasible_braking_upper_bound_does_not_drift(self):
        with patch("lap_sim_lab.planner.brake_decel_at_speed", return_value=5.0):
            upper = 10.0
            for _ in range(30):
                upper = max_entry_speed_for_braking(10.0, 0.5, 0.0, upper, None, None)
            self.assertEqual(upper, 10.0)
            upper = 10.0
            for _ in range(30):
                upper = max_entry_speed_for_braking(9.9, 0.5, 0.0, upper, None, None)
            self.assertEqual(upper, 10.0)

    def test_forward_integration_respects_initial_speed_and_force_limits(self):
        car = Car(Settings(solver="distance"), preload_motor=False)
        model = AeroTireLimits(FrictionEllipse(1.0, 1.0), car)
        s = np.arange(21.0)
        zeros = np.zeros_like(s)
        track = SegmentSpeedPath(s, zeros, s, s, zeros, s, zeros, zeros, s[-1])
        caps = np.full_like(s, 12.0)
        caps[-1] = 3.0
        with (
            patch("lap_sim_lab.planner.drive_accel_at_speed", return_value=4.0),
            patch("lap_sim_lab.planner.brake_decel_at_speed", return_value=3.0),
        ):
            speed = integrate_forward(track, caps, model, None, 0.0)
        ax = np.diff(speed**2) / 2
        self.assertEqual(speed[0], 0.0)
        self.assertLessEqual(speed[-1], caps[-1] + 1e-9)
        self.assertGreaterEqual(ax.min(), -3.0 - 1e-9)
        self.assertLessEqual(ax.max(), 4.0 + 1e-9)


if __name__ == "__main__":
    unittest.main()
