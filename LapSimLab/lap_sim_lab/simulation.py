from dataclasses import dataclass, field, replace
import csv
from pathlib import Path
from time import monotonic

import numpy as np

from .car import Car
from .planner import (
    AeroTireLimits, BRAKING_SPEED_TOLERANCE, GRAVITY, PowerBrakeSystemLimit, SegmentSpeedPath, SpeedPlan,
    build_friction_limited_speed_plan, build_segment_speed_path, lap_time_vector,
)
from .settings import Settings
from .signals import Signal, build_signals
from .tire import build_tire_model
from .track import read_sections, smooth_curvature


@dataclass(frozen=True)
class Result:
    track: SegmentSpeedPath
    plan: SpeedPlan
    time: np.ndarray
    signals: dict[str, Signal] = field(default_factory=dict)
    overspeed_points: int = 0

    @property
    def lateral_acceleration(self):
        return self.plan.speed**2 * self.track.curvature

    @property
    def lap_time(self):
        return float(self.time[-1])


class InfeasibleLapError(ValueError):
    """The input was valid, but the speed plan could not cover the track."""


def simulate(settings: Settings, progress=None, *, include_signals=True, check_cancelled=None) -> Result:
    settings.validate()
    limit = settings.time_limit_s
    deadline = monotonic() + limit if limit else None

    def check_deadline():
        if check_cancelled is not None:
            check_cancelled()
        if deadline is not None and monotonic() >= deadline:
            raise TimeoutError(f"已達 {limit:g} 秒計算上限。")

    def report(message):
        check_deadline()
        if progress is not None:
            progress(message)

    report("讀取賽道、輪胎與馬達資料")
    sections = read_sections(settings.track_path, settings.track_width, settings.step_m, check_deadline=check_deadline)
    track = build_segment_speed_path(sections)
    if not np.isfinite(track.s[-1]) or track.s[-1] <= 0:
        raise ValueError("賽道路徑須有有限的正長度。")
    if settings.solver == "lap" and settings.closed_lap:
        gap = float(np.hypot(track.x[-1] - track.x[0], track.y[-1] - track.y[0]))
        if gap > 0.1:
            raise ValueError(f"首尾相距 {gap:.2f} m，無法套用封閉圈首尾等速。")
    if settings.curvature_model == "smooth":
        track = replace(track, curvature=smooth_curvature(track.s, track.curvature, settings.curvature_sigma_m))
    check_deadline()
    car = Car(settings)
    check_deadline()
    tire = build_tire_model(settings)
    limits = AeroTireLimits(tire, car)
    power = PowerBrakeSystemLimit(car)
    plan = build_friction_limited_speed_plan(
        track,
        limits,
        power,
        max_straight_speed=settings.max_speed,
        safety_factor=settings.safety_factor,
        passes=settings.passes,
        progress=report,
        check_deadline=check_deadline,
    )
    if not all(np.isfinite(values).all() for values in (
        plan.speed, plan.acceleration, plan.wheel_loads, plan.theta,
        plan.heave, plan.downforce, plan.drag, plan.cop, plan.cl, plan.cd,
        plan.total_normal_load, plan.rolling_resistance, plan.total_rolling_resistance,
    )):
        raise ValueError("模擬結果含有非有限值，請確認輸入參數及模型資料。")
    stopped = (plan.speed[:-1] + plan.speed[1:]) <= 2e-9
    if np.any(stopped & (np.diff(track.s) > 1e-9)):
        raise InfeasibleLapError("有賽道區段的規劃速度為零，無法完成計時。請確認速度上限及車輛參數。")
    time = lap_time_vector(track.s, plan.speed)
    signals = {}
    if include_signals:
        report("整理逐點訊號")
        signals = build_signals(track, plan, time, limits, power, check_deadline=check_deadline)
        if not all(np.isfinite(signal.values).all() for signal in signals.values()):
            raise ValueError("逐點訊號含有非有限值，請確認輸入參數及模型資料。")
    overspeed_points = int(np.count_nonzero(plan.speed > plan.lateral_speed_limit + BRAKING_SPEED_TOLERANCE))
    check_deadline()
    return Result(track, plan, time, signals, overspeed_points)


def write_csv(path: Path, result: Result):
    track, plan = result.track, result.plan
    summary_keys = (
        "distance_m", "time_s", "x_m", "y_m", "curvature_1_m",
        "speed_kmh", "lateral_speed_limit_kmh", "ax_g", "ay_g", "phase",
        "fz_FL_N", "fz_FR_N", "fz_RL_N", "fz_RR_N",
        "heave_mm", "roll_deg", "pitch_deg", "downforce_N", "drag_N",
        "CL", "CD", "cop_x_m", "cop_y_m", "cop_z_m",
    )
    summary_values = (
        track.s, result.time, track.x, track.y, track.curvature,
        plan.speed * 3.6, plan.lateral_speed_limit * 3.6,
        plan.acceleration / GRAVITY, result.lateral_acceleration / GRAVITY, plan.phase,
        *plan.wheel_loads.T, plan.heave * 1000,
        np.degrees(plan.theta[:, 0]), np.degrees(plan.theta[:, 2]),
        plan.downforce, plan.drag, plan.cl, plan.cd, *plan.cop.T,
    )
    columns = dict(zip(summary_keys, summary_values))
    for key, signal in result.signals.items():
        columns[key] = signal.values
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(columns)
        writer.writerows(zip(*columns.values()))
