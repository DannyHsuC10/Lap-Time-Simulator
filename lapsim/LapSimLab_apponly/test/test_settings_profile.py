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
                "aero_cl0": "-2.4",
                "aero_cd0": "",
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
                "aero_cl0": -2.4,
                "aero_cd0": None,
                "passes": 5,
                "tire_model": "table",
                "brake_model": "leverage",
                "front_caliper_diameter": 0.028,
            })

    def test_rejects_other_json_files_and_unknown_model_options(self):
        with TemporaryDirectory() as folder:
            path = Path(folder) / "input.json"
            for data in ({"mu_x": 1.5, "mu_y": 1.8},
                         {"schema_version": 1, "settings": {"tire_model": "unknown"}}):
                path.write_text(json.dumps(data), encoding="utf-8")
                with self.subTest(data=data), self.assertRaises(ValueError):
                    load_profile(path)
