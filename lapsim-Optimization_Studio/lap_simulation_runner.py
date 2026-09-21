from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
from typing import Callable, Mapping

import numpy as np

import theoretical_lap_simulator as lapsim
from car import Car_simplify


@dataclass(frozen=True)
class LapSimulationConfig:
    segment_csv: Path = lapsim.DEFAULT_SEGMENT_CSV
    tire_map: Path = lapsim.DEFAULT_TIRE_MAP
    track_width: float = 3.0
    ds: float = 0.5
    max_straight_speed: float = lapsim.DEFAULT_MAX_STRAIGHT_SPEED
    safety_factor: float = 0.98
    aero_enabled: bool = True
    fz_samples: int = 25
    tire_fx_samples: int = 301
    attitude_iterations: int = 1
    speed_plan_passes: int = 3
    lateral_speed_iterations: int = 14
    accel_limit_iterations: int = 6
    brake_entry_iterations: int = 6
    speed_cache_decimals: int = 1
    curvature_cache_decimals: int = 4
    include_rolling_resistance: bool = True
    collect_diagnostics: bool = False


@dataclass(frozen=True)
class LapSimulationResult:
    lap_time_s: float
    track_length_m: float
    track_points: int
    min_speed_mps: float
    max_speed_mps: float
    min_accel_mps2: float
    max_accel_mps2: float
    min_wheel_load_n: float
    max_wheel_load_n: float


@dataclass(frozen=True)
class SpeedOnlyPlan:
    lateral_speed_limit: np.ndarray
    speed: np.ndarray
    acceleration: np.ndarray


@lru_cache(maxsize=8)
def cached_track(segment_csv: str, track_width: float, ds: float) -> lapsim.SegmentSpeedPath:
    segment_track = lapsim.SegmentTrack(
        csv_file=Path(segment_csv),
        track_width=track_width,
        ds=ds,
    )
    return build_segment_speed_path(segment_track)


@lru_cache(maxsize=4)
def cached_tire_map(map_path: str) -> lapsim.TireForceMap:
    return lapsim.TireForceMap.load(Path(map_path))


def build_segment_speed_path(segment_track: lapsim.SegmentTrack) -> lapsim.SegmentSpeedPath:
    x_parts = []
    y_parts = []
    inner_x_parts = []
    inner_y_parts = []
    outer_x_parts = []
    outer_y_parts = []
    curvature_parts = []

    for index, geometry in enumerate(segment_track.section_geometries):
        start = 0 if index == 0 else 1
        section = geometry.section
        curvature = 0.0

        if np.isfinite(section.signed_radius) and abs(section.signed_radius) > 1e-9:
            curvature = 1.0 / float(section.signed_radius)

        x_parts.append(geometry.x[start:])
        y_parts.append(geometry.y[start:])
        inner_x_parts.append(geometry.inner_x[start:])
        inner_y_parts.append(geometry.inner_y[start:])
        outer_x_parts.append(geometry.outer_x[start:])
        outer_y_parts.append(geometry.outer_y[start:])
        curvature_parts.append(
            np.full(len(geometry.x[start:]), curvature, dtype=float)
        )

    x = np.concatenate(x_parts)
    y = np.concatenate(y_parts)
    ds = np.hypot(
        np.diff(x, prepend=x[0]),
        np.diff(y, prepend=y[0]),
    )

    return lapsim.SegmentSpeedPath(
        x=x,
        y=y,
        s=np.cumsum(ds),
        inner_x=np.concatenate(inner_x_parts),
        inner_y=np.concatenate(inner_y_parts),
        outer_x=np.concatenate(outer_x_parts),
        outer_y=np.concatenate(outer_y_parts),
        curvature=np.concatenate(curvature_parts),
        length=float(sum(section.length for section in segment_track.sections)),
    )


def speed_to_acceleration(s: np.ndarray, speed: np.ndarray) -> np.ndarray:
    accel = np.zeros_like(speed)
    ds = np.diff(s)
    valid = ds > 1e-9
    segment_accel = np.zeros(len(speed) - 1, dtype=float)
    segment_accel[valid] = (
        speed[1:][valid] ** 2 - speed[:-1][valid] ** 2
    ) / (2.0 * ds[valid])

    accel[:-1] += segment_accel
    accel[1:] += segment_accel
    accel[1:-1] *= 0.5
    return accel


def lap_time_from_speed(s: np.ndarray, speed: np.ndarray) -> float:
    ds = np.diff(s)
    segment_speed = 0.5 * (speed[:-1] + speed[1:])
    valid = (ds > 1e-9) & (segment_speed > 1e-9)
    return float(np.sum(ds[valid] / segment_speed[valid]))


def build_tire_fx_lookup_from_map(
    tire_map: lapsim.TireForceMap,
    normal_load_per_tire: float,
    samples: int,
) -> lapsim.TireFxLimitLookup:
    alpha_grid, kappa_grid = np.meshgrid(
        tire_map.alpha_axis_deg,
        tire_map.kappa_axis,
        indexing="ij",
    )
    result = tire_map.evaluate(
        fz=np.full_like(alpha_grid, normal_load_per_tire, dtype=float),
        alpha_deg=alpha_grid,
        kappa=kappa_grid,
        outputs=("FX", "FY"),
    )
    fx_grid = np.asarray(result["FX"], dtype=float)
    fy_grid = np.asarray(result["FY"], dtype=float)
    boundary = lapsim.fx_fy_boundary(fx_grid, fy_grid)

    fy_axis = np.linspace(float(np.min(fy_grid)), float(np.max(fy_grid)), samples)
    brake_fx_axis = np.empty_like(fy_axis)
    drive_fx_axis = np.empty_like(fy_axis)

    for index, target_fy in enumerate(fy_axis):
        brake_fx, drive_fx = lapsim.solve_fx_limits_on_boundary(boundary, target_fy)
        brake_fx_axis[index] = brake_fx
        drive_fx_axis[index] = drive_fx

    return lapsim.TireFxLimitLookup(
        fy_axis=fy_axis,
        brake_fx_axis=brake_fx_axis,
        drive_fx_axis=drive_fx_axis,
    )


@lru_cache(maxsize=16)
def cached_tire_limit_axes(
    map_path: str,
    fz_min: float,
    fz_max: float,
    fz_samples: int,
    tire_fx_samples: int,
) -> tuple[np.ndarray, np.ndarray, tuple[lapsim.TireFxLimitLookup, ...]]:
    tire_map = cached_tire_map(map_path)

    if abs(fz_max - fz_min) < 1e-6:
        fz_axis = np.array([fz_min], dtype=float)
    else:
        fz_axis = np.linspace(fz_min, fz_max, max(int(fz_samples), 2))

    max_fy_axis = np.array(
        [lapsim.pure_lateral_limit_at_fz(tire_map, fz) for fz in fz_axis],
        dtype=float,
    )
    fx_lookups = tuple(
        build_tire_fx_lookup_from_map(tire_map, fz, samples=tire_fx_samples)
        for fz in fz_axis
    )

    return fz_axis, max_fy_axis, fx_lookups


def build_cached_tire_limit_map(
    car: Car_simplify,
    config: LapSimulationConfig,
) -> lapsim.AeroTireLimitMap:
    map_path = str(Path(config.tire_map).resolve())
    tire_map = cached_tire_map(map_path)
    aero_config = lapsim.AeroConfig(enabled=config.aero_enabled)
    static_fz = car.m * lapsim.GRAVITY / 4.0
    mean_max_fz = lapsim.tire_normal_load_per_tire(
        speed=config.max_straight_speed,
        static_fz_per_tire=static_fz,
        aero=car.aero,
        aero_config=aero_config,
    )
    fz_min = max(float(tire_map.fz_axis[0]), 100.0)
    fz_max = min(float(tire_map.fz_axis[-1]), max(static_fz, mean_max_fz) * 2.5)
    fz_axis, max_fy_axis, fx_lookups = cached_tire_limit_axes(
        map_path,
        round(fz_min, 9),
        round(fz_max, 9),
        config.fz_samples,
        config.tire_fx_samples,
    )

    return lapsim.AeroTireLimitMap(
        fz_axis=fz_axis,
        max_fy_axis=max_fy_axis,
        fx_lookups=fx_lookups,
        tire_map=tire_map,
        static_fz_per_tire=static_fz,
        car=car,
        aero=car.aero,
        aero_config=aero_config,
        attitude_iterations=config.attitude_iterations,
    )


@dataclass(frozen=True)
class WheelLoadFastResult:
    loads: np.ndarray
    drag: float


def fast_wheel_load_state(
    tire_limit_map: lapsim.AeroTireLimitMap,
    speed: float,
    ax: float,
    ay: float,
) -> WheelLoadFastResult:
    car = tire_limit_map.car
    aero_result = tire_limit_map.aero_result_from_attitude(speed, 0.0, 0.0, 0.0)

    k_heave_front = wheel_rate_from_cache(car, 0)
    k_heave_rear = wheel_rate_from_cache(car, 1)
    k_roll_front = wheel_rate_from_cache(car, 2)
    k_roll_rear = wheel_rate_from_cache(car, 3)
    pitch_stiffness = k_heave_front + k_heave_rear
    roll_stiffness = k_roll_front + k_roll_rear
    front_ratio = car.lr / car.l

    loads = None
    for _ in range(max(int(tire_limit_map.attitude_iterations), 1)):
        aero = 0.0 if not tire_limit_map.aero_config.enabled else float(aero_result["F_down"])
        total_vertical_force = car.m * tire_limit_map.gravity + aero

        front_static = total_vertical_force * front_ratio
        rear_static = total_vertical_force - front_static

        pitch_moment = car.m * ax * car.h_cg
        pitch_front = pitch_moment * k_heave_front / pitch_stiffness
        pitch_rear = pitch_moment * k_heave_rear / pitch_stiffness
        front_load = front_static - pitch_front / car.l
        rear_load = rear_static + pitch_rear / car.l

        roll_moment = car.m * ay * car.h_cg
        roll_front = roll_moment * k_roll_front / roll_stiffness
        roll_rear = roll_moment * k_roll_rear / roll_stiffness
        front_transfer = roll_front / car.tf
        rear_transfer = roll_rear / car.tr

        loads = np.array(
            [
                front_load / 2.0 - front_transfer,
                front_load / 2.0 + front_transfer,
                rear_load / 2.0 - rear_transfer,
                rear_load / 2.0 + rear_transfer,
            ],
            dtype=float,
        )

        k_roll_axle_front = 2.0 * k_roll_front * (car.tf / 2.0) ** 2
        k_roll_axle_rear = 2.0 * k_roll_rear * (car.tr / 2.0) ** 2
        qh = aero / pitch_stiffness
        qp = pitch_moment / pitch_stiffness
        qr = roll_moment / (k_roll_axle_front + k_roll_axle_rear)
        aero_result = tire_limit_map.aero_result_from_attitude(speed, qh, qp, qr)

    return WheelLoadFastResult(
        loads=np.maximum(loads, 1.0),
        drag=float(aero_result["F_drag"]),
    )


def wheel_rate_from_cache(car: Car_simplify, shock_index: int) -> float:
    cache = getattr(car, "_lap_runner_wheel_rate_cache", None)
    if cache is None:
        cache = {}
        setattr(car, "_lap_runner_wheel_rate_cache", cache)

    if shock_index not in cache:
        cache[shock_index] = lapsim.wheel_rate_from_kc(car, shock_index)
    return float(cache[shock_index])


def apply_car_parameters(car: Car_simplify, parameters: Mapping[str, float]) -> Car_simplify:
    for name, value in parameters.items():
        if not hasattr(car, name):
            raise AttributeError(f"Car_simplify has no parameter {name!r}")
        setattr(car, name, float(value))

    if hasattr(car, "wheel_pos"):
        car.wheel_pos = np.array(
            [
                [car.lf, car.tf / 2.0, 0.0],
                [car.lf, -car.tf / 2.0, 0.0],
                [-car.lr, car.tr / 2.0, 0.0],
                [-car.lr, -car.tr / 2.0, 0.0],
            ],
            dtype=float,
        )

    if hasattr(car, "aero") and hasattr(car.aero, "track"):
        car.aero.track = (car.tf + car.tr) / 2.0

    if hasattr(car, "_lap_runner_wheel_rate_cache"):
        delattr(car, "_lap_runner_wheel_rate_cache")

    return car


def build_aero_lateral_speed_profile_fast(
    track: lapsim.SegmentSpeedPath,
    tire_limit_map: lapsim.AeroTireLimitMap,
    max_straight_speed: float,
    curvature_floor: float,
    safety_factor: float,
    iterations: int,
) -> np.ndarray:
    curvature = np.abs(np.asarray(track.curvature, dtype=float))
    speed = np.full_like(curvature, max_straight_speed, dtype=float)

    for index, kappa in enumerate(curvature):
        if kappa <= curvature_floor:
            continue

        low = 0.0
        high = max_straight_speed
        for _ in range(max(int(iterations), 1)):
            mid = 0.5 * (low + high)
            ay = mid * mid * kappa
            loads = fast_wheel_load_state(tire_limit_map, mid, 0.0, ay).loads
            fy_need_total = tire_limit_map.car.m * ay
            fy_cap_total = float(np.sum(tire_limit_map.max_fy_for_loads(loads)))
            if fy_need_total <= fy_cap_total * safety_factor:
                low = mid
            else:
                high = mid

        speed[index] = low

    return speed


def drive_accel_at_speed_fast(
    speed: float,
    curvature: float,
    tire_limit_map: lapsim.AeroTireLimitMap,
    power_brake_limit: lapsim.PowerBrakeSystemLimit,
    config: LapSimulationConfig,
    cache: dict[tuple[str, float, float], float],
) -> float:
    key = (
        "drive",
        round(float(speed), config.speed_cache_decimals),
        round(float(curvature), config.curvature_cache_decimals),
    )
    if key in cache:
        return cache[key]

    ay = speed * speed * curvature
    vehicle_mass = tire_limit_map.car.m
    low = 0.0
    high = 35.0

    for _ in range(max(int(config.accel_limit_iterations), 1)):
        candidate_ax = 0.5 * (low + high)
        load_state = fast_wheel_load_state(tire_limit_map, speed, candidate_ax, ay)
        loads = load_state.loads
        fy_wheels = lapsim.distribute_lateral_force(vehicle_mass * ay, loads)
        _, drive_fx = tire_limit_map.fx_limits_for_loads_and_fy(loads, fy_wheels)
        loaded_radius = np.maximum(
            tire_limit_map.car.R_tire - loads / tire_limit_map.car.K_tire,
            1e-9,
        )
        rolling_resistance = 0.0
        if config.include_rolling_resistance:
            rolling_resistance = float(
                np.sum(
                    lapsim.estimate_rolling_resistance_force(
                        tire_limit_map.tire_map,
                        loads,
                        loaded_radius,
                    )
                )
            )
        drive_force = power_brake_limit.scale_limited_drive_force(
            speed,
            drive_fx,
            loaded_radius=loaded_radius,
        )
        capacity_ax = max(
            (float(np.sum(drive_force)) - load_state.drag - rolling_resistance)
            / vehicle_mass,
            0.0,
        )

        if candidate_ax <= capacity_ax:
            low = candidate_ax
        else:
            high = candidate_ax

    cache[key] = low
    return low


def brake_decel_at_speed_fast(
    speed: float,
    curvature: float,
    tire_limit_map: lapsim.AeroTireLimitMap,
    power_brake_limit: lapsim.PowerBrakeSystemLimit,
    config: LapSimulationConfig,
    cache: dict[tuple[str, float, float], float],
) -> float:
    key = (
        "brake",
        round(float(speed), config.speed_cache_decimals),
        round(float(curvature), config.curvature_cache_decimals),
    )
    if key in cache:
        return cache[key]

    ay = speed * speed * curvature
    vehicle_mass = tire_limit_map.car.m
    low = 0.0
    high = 35.0

    for _ in range(max(int(config.accel_limit_iterations), 1)):
        candidate_decel = 0.5 * (low + high)
        load_state = fast_wheel_load_state(tire_limit_map, speed, -candidate_decel, ay)
        loads = load_state.loads
        fy_wheels = lapsim.distribute_lateral_force(vehicle_mass * ay, loads)
        brake_fx, _ = tire_limit_map.fx_limits_for_loads_and_fy(loads, fy_wheels)
        loaded_radius = np.maximum(
            tire_limit_map.car.R_tire - loads / tire_limit_map.car.K_tire,
            1e-9,
        )
        rolling_resistance = 0.0
        if config.include_rolling_resistance:
            rolling_resistance = float(
                np.sum(
                    lapsim.estimate_rolling_resistance_force(
                        tire_limit_map.tire_map,
                        loads,
                        loaded_radius,
                    )
                )
            )
        brake_force = power_brake_limit.scale_limited_brake_force(
            brake_fx,
            loaded_radius=loaded_radius,
        )
        capacity_decel = max(
            (float(-np.sum(brake_force)) + load_state.drag + rolling_resistance)
            / vehicle_mass,
            0.0,
        )

        if candidate_decel <= capacity_decel:
            low = candidate_decel
        else:
            high = candidate_decel

    cache[key] = low
    return low


def max_entry_speed_for_braking_fast(
    exit_speed: float,
    ds: float,
    curvature: float,
    upper_speed: float,
    tire_limit_map: lapsim.AeroTireLimitMap,
    power_brake_limit: lapsim.PowerBrakeSystemLimit,
    config: LapSimulationConfig,
    cache: dict[tuple[str, float, float], float],
) -> float:
    low = float(exit_speed)
    high = float(max(upper_speed, exit_speed))

    for _ in range(max(int(config.brake_entry_iterations), 1)):
        mid = 0.5 * (low + high)
        decel = brake_decel_at_speed_fast(
            mid,
            curvature,
            tire_limit_map,
            power_brake_limit,
            config,
            cache,
        )
        reachable_exit_square = mid * mid - 2.0 * decel * ds

        if reachable_exit_square <= exit_speed * exit_speed:
            low = mid
        else:
            high = mid

    return low


def build_friction_limited_speed_only(
    track: lapsim.SegmentSpeedPath,
    tire_limit_map: lapsim.AeroTireLimitMap,
    power_brake_limit: lapsim.PowerBrakeSystemLimit,
    max_straight_speed: float = lapsim.DEFAULT_MAX_STRAIGHT_SPEED,
    curvature_floor: float = 1e-5,
    safety_factor: float = 0.98,
    passes: int = 3,
    config: LapSimulationConfig = LapSimulationConfig(),
) -> SpeedOnlyPlan:
    lateral_speed_limit = build_aero_lateral_speed_profile_fast(
        track=track,
        tire_limit_map=tire_limit_map,
        max_straight_speed=max_straight_speed,
        curvature_floor=curvature_floor,
        safety_factor=safety_factor,
        iterations=config.lateral_speed_iterations,
    )
    speed = lateral_speed_limit.copy()
    accel_cache: dict[tuple[str, float, float], float] = {}

    total_passes = max(int(passes), 1)
    for _pass_index in range(total_passes):
        for index in range(len(speed) - 1):
            ds = max(float(track.s[index + 1] - track.s[index]), 1e-6)
            accel = drive_accel_at_speed_fast(
                speed[index],
                track.curvature[index],
                tire_limit_map,
                power_brake_limit,
                config,
                accel_cache,
            )
            reachable = np.sqrt(max(speed[index] ** 2 + 2.0 * accel * ds, 0.0))
            speed[index + 1] = min(speed[index + 1], reachable)

        for index in range(len(speed) - 2, -1, -1):
            ds = max(float(track.s[index + 1] - track.s[index]), 1e-6)
            speed[index] = min(
                speed[index],
                max_entry_speed_for_braking_fast(
                    exit_speed=speed[index + 1],
                    ds=ds,
                    curvature=track.curvature[index],
                    upper_speed=speed[index],
                    tire_limit_map=tire_limit_map,
                    power_brake_limit=power_brake_limit,
                    config=config,
                    cache=accel_cache,
                ),
            )

    return SpeedOnlyPlan(
        lateral_speed_limit=lateral_speed_limit,
        speed=speed,
        acceleration=speed_to_acceleration(track.s, speed),
    )


def run_lap_simulation(
    car: Car_simplify | None = None,
    parameters: Mapping[str, float] | None = None,
    config: LapSimulationConfig = LapSimulationConfig(),
    car_factory: Callable[[], Car_simplify] = Car_simplify,
    verbose: bool = False,
) -> LapSimulationResult:
    original_log_step = getattr(lapsim, "log_step", None)
    if not verbose and original_log_step is not None:
        lapsim.log_step = lambda _message: None

    try:
        lapsim.ACCEL_CACHE.clear()
        car = car_factory() if car is None else car
        if parameters:
            apply_car_parameters(car, parameters)

        track = cached_track(
            str(Path(config.segment_csv).resolve()),
            config.track_width,
            config.ds,
        )
        power_brake_limit = lapsim.PowerBrakeSystemLimit(car)
        tire_limit_map = build_cached_tire_limit_map(
            car=car,
            config=config,
        )
        speed_plan = build_friction_limited_speed_only(
            track=track,
            tire_limit_map=tire_limit_map,
            power_brake_limit=power_brake_limit,
            max_straight_speed=config.max_straight_speed,
            safety_factor=config.safety_factor,
            passes=config.speed_plan_passes,
            config=config,
        )
        min_wheel_load_n = float("nan")
        max_wheel_load_n = float("nan")

        lap_time_s = lap_time_from_speed(track.s, speed_plan.speed)

        return LapSimulationResult(
            lap_time_s=lap_time_s,
            track_length_m=track.length,
            track_points=len(track.x),
            min_speed_mps=float(np.min(speed_plan.speed)),
            max_speed_mps=float(np.max(speed_plan.speed)),
            min_accel_mps2=float(np.min(speed_plan.acceleration)),
            max_accel_mps2=float(np.max(speed_plan.acceleration)),
            min_wheel_load_n=min_wheel_load_n,
            max_wheel_load_n=max_wheel_load_n,
        )
    finally:
        if original_log_step is not None:
            lapsim.log_step = original_log_step


def lap_time_cost(parameters: Mapping[str, float]) -> float:
    return run_lap_simulation(parameters=parameters).lap_time_s
