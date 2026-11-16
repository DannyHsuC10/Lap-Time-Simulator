"""Translate browser requests into calls to the desktop application's solver."""

from dataclasses import fields, replace
import json
from pathlib import Path

from lap_sim_lab.ggv import build_ggv
from lap_sim_lab.planner import GRAVITY
from lap_sim_lab.settings import Settings
from lap_sim_lab.simulation import simulate, write_csv


def read_settings(values_json, files_json):
    defaults = Settings()
    values = json.loads(values_json)
    files = json.loads(files_json)
    overrides = {}
    for field in fields(defaults):
        key = field.name
        default = getattr(defaults, key)
        if isinstance(default, Path):
            if key in files:
                overrides[key] = Path(files[key])
        elif key in values:
            value = values[key]
            if type(default) is int and type(value) is not int:
                raise ValueError(f"{key} 須為整數。")
            overrides[key] = value
    return replace(defaults, **overrides)


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
        "converged": bool(plan.converged),
        "signals": {
            key: {"label": signal.label, "unit": signal.unit, "values": signal.values.tolist()}
            for key, signal in result.signals.items()
        },
    }
    return result, json.dumps(output, ensure_ascii=False, allow_nan=False)


def run_ggv(settings):
    slices = build_ggv(settings)
    return json.dumps([
        {"speed": float(row.speed_kmh), "ay": row.ay_g.tolist(),
         "drive": row.drive_g.tolist(), "brake": row.brake_g.tolist()}
        for row in slices
    ], allow_nan=False)


def export_csv(result, keys_json):
    path = Path("/lap-export.csv")
    try:
        write_csv(path, result, signal_keys=json.loads(keys_json))
        return path.read_text(encoding="utf-8", newline="")
    finally:
        path.unlink(missing_ok=True)
