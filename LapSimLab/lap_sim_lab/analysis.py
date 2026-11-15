"""Fixed-condition attitude sweeps using the LapSim aero and tire models."""

from dataclasses import dataclass, replace
from math import isfinite

import numpy as np

from .car import Car
from .planner import AeroTireLimits, GRAVITY, camber_from_roll
from .settings import Settings
from .tire import build_tire_model


@dataclass(frozen=True)
class AttitudeSweep:
    axis: str = "Roll"
    extent_deg: float = 3.0
    fixed_deg: float = 0.0
    speed_kmh: float = 72.0
    heave_mm: float = 0.0
    ax_g: float = 0.0
    ay_g: float = 0.0

    def poses(self):
        if self.axis not in ("Roll", "Pitch"):
            raise ValueError("掃描角度須為 Roll 或 Pitch。")
        if not 0 < self.extent_deg < 90 or not abs(self.fixed_deg) < 90:
            raise ValueError("掃描幅度須介於 0° 與 90°，固定角的絕對值須小於 90°。")
        if not all(map(isfinite, (self.speed_kmh, self.heave_mm, self.ax_g, self.ay_g))) or self.speed_kmh < 0:
            raise ValueError("車速須為非負數，且車速、Heave 和加速度都須為有限值。")
        angles = np.linspace(-self.extent_deg, self.extent_deg, 121)
        roll = angles if self.axis == "Roll" else np.full_like(angles, self.fixed_deg)
        pitch = angles if self.axis == "Pitch" else np.full_like(angles, self.fixed_deg)
        return angles, zip(np.radians(roll), np.radians(pitch))


AERO_SIGNALS = ("F_down", "F_drag", "F_FL", "F_FR", "F_RL", "F_RR", "CL", "CD")


def sweep_aero(settings: Settings, sweep: AttitudeSweep):
    angles, poses = sweep.poses()
    aero = Car(settings, preload_motor=False).aero
    if aero is None:
        return angles, {key: np.zeros_like(angles) for key in (*AERO_SIGNALS, "front", "rear")}
    with np.errstate(over="raise", invalid="raise"):
        rows = [
            aero.calculate_from_modal(sweep.speed_kmh / 3.6, sweep.heave_mm / 1000, pitch, roll)
            for roll, pitch in poses
        ]
    data = {key: np.array([row[key] for row in rows]) for key in AERO_SIGNALS}
    data["front"] = data["F_FL"] + data["F_FR"]
    data["rear"] = data["F_RL"] + data["F_RR"]
    return angles, data


@dataclass(frozen=True)
class TireSweepResult:
    angles: np.ndarray
    fy_peak: np.ndarray
    fx_drive: np.ndarray
    fx_brake: np.ndarray
    feasible: np.ndarray
    point_mass: bool


def sweep_tires(settings: Settings, sweep: AttitudeSweep) -> TireSweepResult:
    if not 0 < sweep.extent_deg < 90 or not isfinite(sweep.ay_g):
        raise ValueError("Roll 掃描幅度須介於 0° 與 90°，橫向加速度須為有限值。")
    angles = np.linspace(-sweep.extent_deg, sweep.extent_deg, 121)
    car = Car(replace(settings, aero_enabled=False), preload_motor=False)
    limits = AeroTireLimits(build_tire_model(settings), car)
    point_mass = settings.load_model == "point_mass"
    weight = car.m * GRAVITY
    total_fy = weight * sweep.ay_g
    if point_mass:
        fy_max = limits.tire_model.peak_forces(weight)[1]
        fx = limits.tire_model.fx_limits(total_fy, fz=weight)
        shape = (len(angles), 1)
        return TireSweepResult(
            angles, np.full(shape, fy_max), np.full(shape, max(fx, default=0.0)),
            np.full(shape, min(fx, default=0.0)), np.full(len(angles), bool(fx)), point_mass,
        )

    loads = limits.suspension_state({"F_down": 0.0}, ax=0.0, ay=0.0).loads
    lateral_demand = limits.lateral_forces(total_fy, loads)
    fy_peak, fx_drive, fx_brake = (np.empty((len(angles), 4)) for _ in range(3))
    feasible = np.empty(len(angles), dtype=bool)

    with np.errstate(over="raise", invalid="raise"):
        for index, roll in enumerate(np.radians(angles)):
            camber = camber_from_roll(np.array([roll, roll, 0.0, 0.0]))
            fy_peak[index] = limits.balanced_lateral_limits(loads, camber)
            bounds = limits.fx_limits_for_loads_and_fy(loads, lateral_demand, camber)
            feasible[index] = bounds is not None
            if bounds is None:
                fx_brake[index] = fx_drive[index] = 0.0
            else:
                fx_brake[index], fx_drive[index] = bounds

    return TireSweepResult(angles, fy_peak, fx_drive, fx_brake, feasible, point_mass)
