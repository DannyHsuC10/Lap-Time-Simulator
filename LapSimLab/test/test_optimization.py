from dataclasses import replace
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from lap_sim_lab.optimization import PARAMETERS, build_ranges, optimize_lap
from lap_sim_lab.settings import Settings
from lap_sim_lab.simulation import InfeasibleLapError, simulate


def fixed_margins(**changes):
    return {p.key: changes.get(p.key, 0) for p in PARAMETERS}


class OptimizationTests(unittest.TestCase):
    def test_default_margins_follow_vehicle_and_aero_baselines(self):
        settings = Settings(mass=375, tire_utilization=0.82, gear_ratio=8.5, gear_efficiency=0.91,
                            brake_leverage=15)
        ranges = build_ranges(settings, {"CL0": -3.2, "CD0": 0.6}, {})
        expected = {"mass": 37.5, "tire_utilization": 0.01, "gear_ratio": 0.85, "gear_efficiency": 0.01,
                    "brake_leverage": 1.5, "CL0": 0.32, "CD0": 0.06}
        for bound in ranges:
            margin = expected[bound.parameter.key]
            self.assertAlmostEqual(bound.lower, bound.baseline - margin)
            self.assertAlmostEqual(bound.upper, bound.baseline + margin)

    def test_single_parameter_times_keep_other_parameters_at_baseline(self):
        settings = Settings()
        trials = []

        def sample_lap(candidate, *, include_signals=True, **kwargs):
            mass_change = (settings.mass - candidate.mass) / 32
            brake_change = settings.brake_leverage - candidate.brake_leverage
            if not include_signals:
                trials.append((mass_change, brake_change))
            return SimpleNamespace(lap_time=80 - mass_change + brake_change**2 - 4 * mass_change * brake_change)

        with patch("lap_sim_lab.optimization.simulate", side_effect=sample_lap):
            result = optimize_lap(settings, margins=fixed_margins(mass=32, brake_leverage=1), max_evaluations=5)

        self.assertEqual(result.settings.mass, 288)
        self.assertEqual(result.settings.brake_leverage, 11.6)
        self.assertEqual(result.baseline_time, 80)
        self.assertEqual(result.simulation.lap_time, 76)
        self.assertEqual(result.single_parameter_times["mass"], 79)
        self.assertEqual(result.single_parameter_times["brake_leverage"], 81)
        self.assertEqual(result.single_parameter_times["gear_ratio"], 80)
        self.assertEqual(result.equivalent_gain, 1)
        self.assertEqual(set(result.equivalent_changes), {"mass"})
        self.assertEqual(result.evaluations, 5)
        self.assertEqual(len(trials), 6)
        self.assertEqual(trials[-1], (0, 1))

    def test_equivalent_changes_match_smallest_positive_gain_excluding_gear_ratio(self):
        settings = Settings(tire_utilization=0.8, gear_efficiency=0.8)
        margins = fixed_margins(mass=32, tire_utilization=0.1, gear_efficiency=0.1, gear_ratio=1)

        def sample_lap(candidate, **kwargs):
            mass_change = (settings.mass - candidate.mass) / 32
            return SimpleNamespace(lap_time=80 - 4 * mass_change * abs(mass_change)
                                   - 0.1 * (candidate.tire_utilization - settings.tire_utilization)
                                   - 2.5 * (candidate.gear_efficiency - settings.gear_efficiency)
                                   - 0.001 * (candidate.gear_ratio - settings.gear_ratio))

        with patch("lap_sim_lab.optimization.simulate", side_effect=sample_lap):
            result = optimize_lap(settings, margins=margins)

        self.assertAlmostEqual(result.equivalent_gain, 0.01)
        self.assertAlmostEqual(result.baseline_time - result.single_parameter_times["gear_ratio"], 0.001)
        self.assertEqual(set(result.equivalent_changes), {"mass", "tire_utilization", "gear_efficiency"})
        self.assertAlmostEqual(result.equivalent_changes["mass"].value, settings.mass - 1.6, delta=0.01)
        self.assertAlmostEqual(result.equivalent_changes["gear_efficiency"].value, 0.804, delta=0.0001)
        for key, change in result.equivalent_changes.items():
            isolated_lap = sample_lap(replace(settings, **{key: change.value})).lap_time
            self.assertEqual(change.lap_time, isolated_lap)
            self.assertGreaterEqual(result.baseline_time - isolated_lap, result.equivalent_gain)
            self.assertAlmostEqual(result.baseline_time - isolated_lap, result.equivalent_gain, delta=0.0001)

    def test_equivalent_target_requires_a_feasible_baseline_and_positive_eligible_gain(self):
        settings = Settings()
        margins = fixed_margins(gear_ratio=1)
        for baseline_feasible in (True, False):
            with self.subTest(baseline_feasible=baseline_feasible):
                def sample_lap(candidate, **kwargs):
                    if not baseline_feasible and candidate.gear_ratio <= settings.gear_ratio:
                        raise InfeasibleLapError
                    return SimpleNamespace(lap_time=80 - candidate.gear_ratio)

                with patch("lap_sim_lab.optimization.simulate", side_effect=sample_lap):
                    result = optimize_lap(settings, margins=margins)
                self.assertIsNone(result.equivalent_gain)
                self.assertEqual(result.equivalent_changes, {})

    def test_equivalent_selection_preserves_all_search_and_single_parameter_results(self):
        settings = Settings(tire_utilization=0.8)
        margins = fixed_margins(mass=32, brake_leverage=1, tire_utilization=0.1, CL0=0.2, gear_ratio=1)

        def sample_lap(candidate, *, aero_coefficients, **kwargs):
            return SimpleNamespace(lap_time=80 + (candidate.mass - settings.mass) / 32
                                   + 0.05 * (candidate.brake_leverage - settings.brake_leverage)
                                   - 5 * (candidate.tire_utilization - settings.tire_utilization)
                                   + 10 * (aero_coefficients["CL0"] + 2.4)
                                   - 0.002 * (candidate.gear_ratio - settings.gear_ratio))

        selected = {"mass", "tire_utilization", "CL0"}
        with patch("lap_sim_lab.optimization.simulate", side_effect=sample_lap):
            default = optimize_lap(settings, margins=margins)
            subset = optimize_lap(settings, margins=margins, equivalent_keys=selected)
            empty = optimize_lap(settings, margins=margins, equivalent_keys=())
            gear = optimize_lap(settings, margins=margins, equivalent_keys={"gear_ratio"})

        self.assertAlmostEqual(default.equivalent_gain, 0.05)
        self.assertAlmostEqual(subset.equivalent_gain, 0.5)
        self.assertEqual(subset.equivalent_keys, selected)
        self.assertEqual(set(subset.equivalent_changes), selected)
        self.assertIsNone(empty.equivalent_gain)
        self.assertEqual(empty.equivalent_changes, {})
        self.assertAlmostEqual(gear.equivalent_gain, 0.002)
        self.assertEqual(set(gear.equivalent_changes), {"gear_ratio"})
        for result in (subset, empty, gear):
            self.assertEqual(result.single_parameter_times, default.single_parameter_times)
            self.assertEqual(result.simulation.lap_time, default.simulation.lap_time)
            self.assertEqual(result.evaluations, default.evaluations)

    def test_lp03_caps_voltage_and_reports_its_individual_lap(self):
        settings = Settings(motor_model="lp03", dc_voltage=580)
        margins = fixed_margins(dc_voltage=200, tire_utilization=2, gear_efficiency=2)

        def sample_lap(candidate, **kwargs):
            return SimpleNamespace(lap_time=100 - candidate.dc_voltage / 10)

        with patch("lap_sim_lab.optimization.simulate", side_effect=sample_lap):
            result = optimize_lap(settings, margins=margins)
            curve = optimize_lap(replace(settings, motor_model="curve"), max_evaluations=1)

        bounds = {bound.parameter.key: bound for bound in result.ranges}
        voltage = bounds["dc_voltage"]
        self.assertEqual((voltage.lower, voltage.upper), (400, 600))
        self.assertEqual(result.settings.dc_voltage, 600)
        self.assertEqual(result.baseline_time - result.single_parameter_times["dc_voltage"], 2)
        for key in ("tire_utilization", "gear_efficiency"):
            self.assertEqual((bounds[key].lower, bounds[key].upper), (0, 1))
        self.assertNotIn("dc_voltage", {bound.parameter.key for bound in curve.ranges})
        self.assertNotIn("dc_voltage", curve.single_parameter_times)

    def test_search_can_refine_back_toward_a_clipped_baseline(self):
        def sample_lap(candidate, **kwargs):
            return SimpleNamespace(lap_time=60 + (candidate.tire_utilization - 0.94)**2)

        with patch("lap_sim_lab.optimization.simulate", side_effect=sample_lap):
            result = optimize_lap(Settings(), margins=fixed_margins(tire_utilization=0.1))

        self.assertAlmostEqual(result.settings.tire_utilization, 0.94, delta=0.005)

    def test_selected_models_produce_reproducible_complete_laps(self):
        for model in ("ellipse", "superellipse", "table"):
            with self.subTest(tire_model=model):
                settings = Settings(tire_model=model, step_m=3.0, aero_enabled=False,
                                    brake_model="hydraulic", solver="distance", yaw_correction=True)
                result = optimize_lap(settings, margins=fixed_margins(mass=32), max_evaluations=3)
                best = result.settings
                self.assertEqual(best.tire_model, model)
                self.assertEqual((best.load_model, best.brake_model, best.aero_model, best.solver),
                                 ("point_mass", "leverage", "quadratic", "lap"))
                self.assertTrue(best.aero_enabled)
                self.assertFalse(best.yaw_correction)
                self.assertEqual(result.evaluations, 3)
                self.assertLessEqual(result.simulation.lap_time, result.baseline_time)
                self.assertTrue(result.simulation.signals)
                self.assertEqual(result.simulation.lap_time, simulate(best).lap_time)
                self.assertEqual(result.single_parameter_times["mass"], result.simulation.lap_time)

    def test_lap_and_optimization_use_minimal_quadratic_aero_json(self):
        settings = Settings(tire_model="ellipse", load_model="point_mass", aero_model="quadratic", step_m=4.0)
        coefficients = {"CL0": -3.2, "CD0": 1.45}
        with TemporaryDirectory() as folder:
            path = Path(folder) / "aero.json"
            path.write_text(json.dumps(coefficients), encoding="utf-8")
            settings = replace(settings, aero_path=path)
            lap = simulate(settings)
            result = optimize_lap(settings, margins=fixed_margins(mass=32), max_evaluations=3)

        for simulation in (lap, result.simulation):
            np.testing.assert_allclose(simulation.plan.cl, -3.2)
            np.testing.assert_allclose(simulation.plan.cd, 1.45)
        self.assertEqual(result.settings.aero_path, path)
        self.assertEqual(result.aero_coefficients, coefficients)

    def test_aero_margins_use_json_baselines_and_only_vary_selected_coefficients(self):
        coefficients = {"CL0": -3.2, "CD0": 0.15, "rho": 1.18, "area": 1.25}

        def sample_lap(candidate, *, aero_coefficients, **kwargs):
            self.assertEqual(aero_coefficients["rho"], coefficients["rho"])
            self.assertEqual(aero_coefficients["area"], coefficients["area"])
            return SimpleNamespace(lap_time=80 + aero_coefficients["CL0"] + 2 * aero_coefficients["CD0"])

        with TemporaryDirectory() as folder:
            path = Path(folder) / "aero.json"
            path.write_text(json.dumps(coefficients), encoding="utf-8")
            with patch("lap_sim_lab.optimization.simulate", side_effect=sample_lap):
                result = optimize_lap(Settings(aero_path=path), margins=fixed_margins(CL0=0.2, CD0=0.3),
                                      max_evaluations=5)
            self.assertEqual(json.loads(path.read_text(encoding="utf-8")), coefficients)

        ranges = {bound.parameter.key: bound for bound in result.ranges}
        self.assertEqual(ranges["CL0"].baseline, -3.2)
        self.assertAlmostEqual(ranges["CL0"].lower, -3.4)
        self.assertAlmostEqual(ranges["CL0"].upper, -3.0)
        self.assertEqual(ranges["CD0"].baseline, 0.15)
        self.assertEqual(ranges["CD0"].lower, 0)
        self.assertAlmostEqual(ranges["CD0"].upper, 0.45)
        self.assertAlmostEqual(result.parameter_value("CL0"), -3.4)
        self.assertEqual(result.parameter_value("CD0"), 0)
        self.assertAlmostEqual(result.baseline_time - result.single_parameter_times["CL0"], 0.2)
        self.assertAlmostEqual(result.baseline_time - result.single_parameter_times["CD0"], 0.3)


if __name__ == "__main__":
    unittest.main()
