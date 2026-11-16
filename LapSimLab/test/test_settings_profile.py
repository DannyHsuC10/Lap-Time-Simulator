import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from lap_sim_lab.settings import Settings
from lap_sim_lab.settings_profile import load_profile, save_profile


class SettingsProfileTests(unittest.TestCase):
    def test_round_trip_retains_inactive_inputs_and_relative_file_references(self):
        with TemporaryDirectory() as folder:
            root = Path(folder)
            path = root / "setup.json"
            track_path = root / "tracks" / "circuit.csv"
            save_profile(path, {
                "track_path": track_path,
                "tire_path": Settings().tire_path,
                "aero_enabled": False,
                "mass": "",
                "gear_efficiency": "0.95",
                "passes": "5",
                "tire_model": "table",
                "brake_model": "leverage",
                "front_caliper_diameter": "0.028",
            })
            stored = json.loads(path.read_text(encoding="utf-8"))["settings"]
            self.assertEqual(Path(stored["track_path"]), Path("tracks/circuit.csv"))
            restored = load_profile(path)
            self.assertEqual(restored, {
                "track_path": track_path,
                "tire_path": Settings().tire_path,
                "aero_enabled": False,
                "mass": "",
                "gear_efficiency": 0.95,
                "passes": 5,
                "tire_model": "table",
                "brake_model": "leverage",
                "front_caliper_diameter": 0.028,
            })

    def test_old_profiles_discard_retired_fields_and_keep_active_inputs(self):
        with TemporaryDirectory() as folder:
            path = Path(folder) / "input.json"
            old_settings = {"aero_path": "custom_aero.json", "aero_cl0": -8, "aero_cd0": 0.3,
                            "rolling_model": "off", "rolling_coefficient": 0.024, "closed_lap": True}
            path.write_text(json.dumps({"schema_version": 1, "settings": old_settings}), encoding="utf-8")
            restored = load_profile(path)
            self.assertEqual(restored, {"aero_path": path.parent / "custom_aero.json", "rolling_coefficient": 0.024})

            save_profile(path, dict(old_settings, **restored))
            self.assertEqual(json.loads(path.read_text(encoding="utf-8"))["settings"],
                             {"aero_path": "custom_aero.json", "rolling_coefficient": 0.024})

    def test_rejects_other_json_files_and_unknown_model_options(self):
        with TemporaryDirectory() as folder:
            path = Path(folder) / "input.json"
            for data in ({"mu_x": 1.5, "mu_y": 1.8},
                         {"schema_version": 1, "settings": {"tire_model": "unknown"}}):
                path.write_text(json.dumps(data), encoding="utf-8")
                with self.subTest(data=data), self.assertRaises(ValueError):
                    load_profile(path)
