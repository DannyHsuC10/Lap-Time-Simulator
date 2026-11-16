import unittest
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import patch

from lap_sim_lab.optimization import PARAMETERS, optimize_lap
from lap_sim_lab.settings import Settings
from lap_sim_lab.simulation import simulate


def fixed_margins(**changes):
    return {p.key: changes.get(p.key, 0) for p in PARAMETERS}


class OptimizationTests(unittest.TestCase):
    def test_single_parameter_times_keep_other_parameters_at_baseline(self):
        settings = Settings(aero_cl0=-2.4, aero_cd0=2.0)
        trials = []

        def sample_lap(candidate, *, include_signals=True, **kwargs):
            mass_change = (settings.mass - candidate.mass) / 32
            drag_change = (settings.aero_cd0 - candidate.aero_cd0) / 0.25
            if not include_signals:
                trials.append((mass_change, drag_change))
            return SimpleNamespace(lap_time=80 - mass_change + drag_change**2 - 4 * mass_change * drag_change)

        with patch("lap_sim_lab.optimization.simulate", side_effect=sample_lap):
            result = optimize_lap(settings, margins=fixed_margins(mass=32, aero_cd0=0.25), max_evaluations=5)

        self.assertEqual(result.settings.mass, 288)
        self.assertEqual(result.settings.aero_cd0, 1.75)
        self.assertEqual(result.baseline_time, 80)
        self.assertEqual(result.simulation.lap_time, 76)
        self.assertEqual(result.single_parameter_times["mass"], 79)
        self.assertEqual(result.single_parameter_times["aero_cd0"], 81)
        self.assertEqual(result.single_parameter_times["gear_ratio"], 80)
        self.assertEqual(result.evaluations, 5)
        self.assertEqual(len(trials), 6)
        self.assertEqual(trials[-1], (0, 1))

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


if __name__ == "__main__":
    unittest.main()
