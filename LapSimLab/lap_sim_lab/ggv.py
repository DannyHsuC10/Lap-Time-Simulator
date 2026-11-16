"""Vehicle acceleration envelopes at fixed speeds."""

from dataclasses import dataclass
from time import monotonic

import numpy as np

from .car import Car
from .localization import localize
from .planner import (
    GRAVITY, AeroTireLimits, PowerBrakeSystemLimit, brake_decel_at_lateral_accel,
    drive_accel_at_lateral_accel,
)
from .settings import Settings
from .tire import build_tire_model


@dataclass(frozen=True)
class GGVSlice:
    speed_kmh: float
    ay_g: np.ndarray
    drive_g: np.ndarray
    brake_g: np.ndarray


def build_ggv(settings: Settings, *, speed_samples: int = 5) -> list[GGVSlice]:
    """Sample both turn directions at evenly spaced speeds."""
    settings.validate()
    if speed_samples < 2:
        raise ValueError(localize("GGV 至少需要兩個車速切片。"))
    limit = settings.time_limit_s
    deadline = monotonic() + limit if limit else None

    def check_deadline():
        if deadline is not None and monotonic() >= deadline:
            raise TimeoutError(localize("GGV 已達 {limit:g} 秒計算上限。", limit=limit))

    car = Car(settings)
    tires = AeroTireLimits(build_tire_model(settings), car)
    power = PowerBrakeSystemLimit(car)
    slices = []
    for speed in np.linspace(0.0, settings.max_speed, speed_samples):
        check_deadline()
        left = lateral_limit(speed, 1, tires, settings.safety_factor)
        right = lateral_limit(speed, -1, tires, settings.safety_factor)
        ay_samples = np.r_[np.linspace(-right, 0, 21)[:-1], np.linspace(0, left, 21)]
        drive, brake = [], []
        for ay in ay_samples:
            check_deadline()
            drive.append(drive_accel_at_lateral_accel(speed, ay, tires, power))
            brake.append(-brake_decel_at_lateral_accel(speed, ay, tires, power))
        slices.append(GGVSlice(
            speed * 3.6, ay_samples / GRAVITY, np.array(drive) / GRAVITY, np.array(brake) / GRAVITY,
        ))
    check_deadline()
    return slices


def lateral_limit(speed, direction, tires, safety_factor):
    """Use the lap solver's zero-longitudinal-acceleration cornering criterion."""
    def feasible(ay):
        state = tires.wheel_load_state(speed, 0.0, direction * ay)
        return tires.car.m * ay <= safety_factor * tires.available_lateral_force(state)

    low, high = 0.0, GRAVITY
    while feasible(high):
        low, high = high, high * 2
        if not np.isfinite(high):
            raise ValueError(localize("無法取得有限的 GGV 側向加速度上限，請檢查車輛參數。"))
    for _ in range(16):
        candidate = (low + high) / 2
        if feasible(candidate):
            low = candidate
        else:
            high = candidate
    return low
