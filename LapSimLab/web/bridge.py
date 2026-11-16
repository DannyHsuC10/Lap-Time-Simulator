"""Translate browser requests into calls to the desktop application's solver."""

from dataclasses import replace
import json
from pathlib import Path

from lap_sim_lab.analysis import AttitudeSweep, sweep_aero, sweep_tires
from lap_sim_lab.car import Car
from lap_sim_lab.ggv import build_ggv
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


def inspect_curve(tool, values_json, files_json, conditions_json):
    settings = read_settings(values_json, files_json)
    settings.validate()
    if tool == "motor":
        curve = Car(replace(settings, aero_enabled=False), preload_motor=False).motor_curve
        payload = {"rpm": curve.rpm.tolist(), "torque": curve.torque.tolist(),
                   "voltage": settings.dc_voltage if settings.motor_model == "lp03" else None}
    elif tool == "ggv":
        payload = [
            {"speed": float(row.speed_kmh), "ay": row.ay_g.tolist(),
             "drive": row.drive_g.tolist(), "brake": row.brake_g.tolist()}
            for row in build_ggv(settings, speed_samples=13)
        ]
    else:
        sweep = AttitudeSweep(**json.loads(conditions_json))
        if tool == "aero":
            angles, rows = sweep_aero(settings, sweep)
            payload = {"angles": angles.tolist(), "signals": {key: row.tolist() for key, row in rows.items()},
                       "axis": sweep.axis, "fixed": sweep.fixed_deg, "enabled": settings.aero_enabled}
        elif tool == "tire":
            result = sweep_tires(settings, sweep)
            payload = {"angles": result.angles.tolist(), "pointMass": result.point_mass,
                       "peaks": result.fy_peak.tolist(), "drive": result.fx_drive.tolist(),
                       "brake": result.fx_brake.tolist(), "failures": int((~result.feasible).sum()),
                       "ay": sweep.ay_g}
        else:
            raise ValueError(f"無法辨識曲線工具「{tool}」。")
    return json.dumps(payload, ensure_ascii=False, allow_nan=False)


def export_csv(result):
    path = Path("/lap-export.csv")
    try:
        write_csv(path, result)
        with path.open(encoding="utf-8", newline="") as stream:
            return stream.read()
    finally:
        path.unlink(missing_ok=True)
