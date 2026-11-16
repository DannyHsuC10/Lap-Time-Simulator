"""Translate browser requests into calls to the desktop application's solver."""

from dataclasses import replace
import json
from pathlib import Path

from lap_sim_lab.planner import GRAVITY
from lap_sim_lab.settings import Settings
from lap_sim_lab.simulation import simulate, write_csv


def read_settings(values_json, files_json):
    values = json.loads(values_json)
    paths = {key: Path(path) for key, path in json.loads(files_json).items()}
    return replace(Settings(), **values, **paths)


def run_lap(settings, progress):
    result = simulate(settings, progress)
    track, plan = result.track, result.plan
    output = {
        "lapTime": result.lap_time,
        "distance": track.s.tolist(),
        "time": result.time.tolist(),
        "x": track.x.tolist(),
        "y": track.y.tolist(),
        "topSpeed": float(plan.speed.max() * 3.6),
        "peakLateral": float(abs(result.lateral_acceleration).max() / GRAVITY),
        "overspeedPoints": result.overspeed_points,
        "converged": plan.converged,
        "signals": {
            key: {"label": signal.label, "unit": signal.unit, "values": signal.values.tolist()}
            for key, signal in result.signals.items()
        },
    }
    return result, json.dumps(output, ensure_ascii=False, allow_nan=False)


def export_csv(result):
    path = Path("/lap-export.csv")
    try:
        write_csv(path, result)
        with path.open(encoding="utf-8", newline="") as stream:
            return stream.read()
    finally:
        path.unlink(missing_ok=True)
