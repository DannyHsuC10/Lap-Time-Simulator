"""Package the browser app and the shared LapSimLab solver for static hosting."""

import argparse
from dataclasses import fields
import json
from pathlib import Path
import shutil
import sys
from zipfile import ZIP_DEFLATED, ZipFile


WEB = Path(__file__).resolve().parent
PROJECT = WEB.parent
sys.path.insert(0, str(PROJECT))

from lap_sim_lab.settings import MODEL_CHOICES, Settings


MODULES = (
    "__init__", "aero", "brake", "car", "ggv", "motor", "planner", "settings",
    "settings_profile", "signals", "simulation", "suspension", "tire", "track",
)


def build(destination):
    defaults = Settings()
    values = {}
    data_files = set()
    for field in fields(defaults):
        value = getattr(defaults, field.name)
        if isinstance(value, Path):
            data_files.add(value)
            value = value.name
        values[field.name] = value

    destination.mkdir(parents=True, exist_ok=True)
    for name in ("index.html", "style.css", "app.js", "worker.js", "charts.js"):
        shutil.copyfile(WEB / name, destination / name)
    schema = {"defaults": values, "models": MODEL_CHOICES}
    (destination / "settings.json").write_text(json.dumps(schema, ensure_ascii=False), encoding="utf-8")
    with ZipFile(destination / "solver.zip", "w", ZIP_DEFLATED) as bundle:
        for name in MODULES:
            path = Path("lap_sim_lab") / f"{name}.py"
            bundle.write(PROJECT / path, path.as_posix())
        bundle.write(WEB / "bridge.py", "bridge.py")
        for source in sorted(data_files):
            bundle.write(source, f"input_data/{source.name}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("destination", type=Path)
    build(parser.parse_args().destination)
