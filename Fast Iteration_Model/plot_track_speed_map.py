from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection
from scipy.spatial import ConvexHull

from aero.Aero import ModalAeroModel
from car import Car
from data.Simulation_Logger import SimulationLogger
from Pacejka_tire_model.tire_mapping_tool import TireForceMap
from power_brake_sys.brake import brake as brake_torque
from power_brake_sys.motor import Throttle_command
from Suspension_sys.steady_state.sus_load import wheel_load_stiffness
from track.track_segments import SegmentTrack
from Vehicle_status import Vehicle_state


BASE_DIR = Path(__file__).resolve().parent
DEFAULT_SEGMENT_CSV = BASE_DIR / "track" / "fsae_A_2025.csv"
DEFAULT_TIRE_MAP = BASE_DIR / "Pacejka_tire_model" / "tire_force_map.pkl"
DEFAULT_PLANNED_OUTPUT = BASE_DIR / "track" / "track_speed_map.png"
DEFAULT_LAP_DATA_OUTPUT = BASE_DIR / "track" / "track_speed_map_lap_data.pkl"
DEFAULT_VEHICLE_MASS = 320.0
DEFAULT_MAX_STRAIGHT_SPEED = 30.0
GRAVITY = 9.81
CG_HEIGHT = 0.3016451511
WHEELBASE = 1.53
LF = 0.7498400000000001
LR = WHEELBASE - LF
FRONT_STATIC_RATIO = LR / WHEELBASE
TRACK_FRONT = 1.28
TRACK_REAR = 1.24
K_HEAVE_FRONT = 56000.0
K_HEAVE_REAR = 44800.0
K_ROLL_FRONT = 80000.0
K_ROLL_REAR = 64000.0

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

ACCEL_CACHE: dict[tuple[int, str, float, float], float] = {}


@dataclass(frozen=True)
class TireLateralLimit:
    normal_load_per_tire: float
    max_lateral_force_per_tire: float
    mu_y: float
    lateral_accel_limit: float


@dataclass(frozen=True)
class AeroConfig:
    qh: float = 0.0
    qp: float = 0.0
    qr: float = 0.0
    enabled: bool = True


class PowerBrakeSystemLimit:
    """Full-throttle and full-brake force limits from power_brake_sys."""

    def __init__(self, car: Car):
        self.car = car

    def drive_accel_limit(self, speed: float) -> float:
        omega = np.full(4, max(speed, 0.0) / self.car.R_tire, dtype=float)
        wheel_torque = np.asarray(
            Throttle_command(omega, self.car, cmd=1.0),
            dtype=float,
        )
        drive_force = np.maximum(wheel_torque / self.car.R_tire, 0.0)
        return float(np.sum(drive_force) / self.car.m)

    def brake_decel_limit(self, speed: float) -> float:
        wheel_torque = np.asarray(
            brake_torque(max(speed, 1e-3), self.car),
            dtype=float,
        )
        brake_force = np.maximum(-wheel_torque / self.car.R_tire, 0.0)
        return float(np.sum(brake_force) / self.car.m)


@dataclass(frozen=True)
class SegmentSpeedPath:
    x: np.ndarray
    y: np.ndarray
    s: np.ndarray
    inner_x: np.ndarray
    inner_y: np.ndarray
    outer_x: np.ndarray
    outer_y: np.ndarray
    curvature: np.ndarray
    length: float


@dataclass(frozen=True)
class TireFxLimitLookup:
    fy_axis: np.ndarray
    brake_fx_axis: np.ndarray
    drive_fx_axis: np.ndarray

    @classmethod
    def from_tire_map(
        cls,
        map_path: str | Path,
        normal_load_per_tire: float,
        samples: int = 301,
    ) -> "TireFxLimitLookup":
        tire_map = TireForceMap.load(map_path)
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
        boundary = fx_fy_boundary(fx_grid, fy_grid)

        fy_axis = np.linspace(float(np.min(fy_grid)), float(np.max(fy_grid)), samples)
        brake_fx_axis = np.empty_like(fy_axis)
        drive_fx_axis = np.empty_like(fy_axis)

        for index, target_fy in enumerate(fy_axis):
            brake_fx, drive_fx = solve_fx_limits_on_boundary(boundary, target_fy)
            brake_fx_axis[index] = brake_fx
            drive_fx_axis[index] = drive_fx

        return cls(
            fy_axis=fy_axis,
            brake_fx_axis=brake_fx_axis,
            drive_fx_axis=drive_fx_axis,
        )

    def limits_at_fy(self, fy_per_tire: float) -> tuple[float, float]:
        fy = float(
            np.clip(fy_per_tire, float(self.fy_axis[0]), float(self.fy_axis[-1]))
        )
        brake_fx = float(np.interp(fy, self.fy_axis, self.brake_fx_axis))
        drive_fx = float(np.interp(fy, self.fy_axis, self.drive_fx_axis))
        return brake_fx, drive_fx


@dataclass(frozen=True)
class AeroTireLimitMap:
    fz_axis: np.ndarray
    max_fy_axis: np.ndarray
    fx_lookups: tuple[TireFxLimitLookup, ...]
    static_fz_per_tire: float
    aero: ModalAeroModel
    aero_config: AeroConfig
    gravity: float = GRAVITY

    @classmethod
    def build(
        cls,
        map_path: str | Path,
        vehicle_mass: float = DEFAULT_VEHICLE_MASS,
        max_speed: float = DEFAULT_MAX_STRAIGHT_SPEED,
        gravity: float = GRAVITY,
        aero: ModalAeroModel | None = None,
        aero_config: AeroConfig = AeroConfig(),
        fz_samples: int = 25,
    ) -> "AeroTireLimitMap":
        tire_map = TireForceMap.load(map_path)
        if aero is None:
            aero = default_aero_model()

        static_fz = vehicle_mass * gravity / 4.0
        mean_max_fz = tire_normal_load_per_tire(
            speed=max_speed,
            static_fz_per_tire=static_fz,
            aero=aero,
            aero_config=aero_config,
        )
        fz_min = max(float(tire_map.fz_axis[0]), 100.0)
        fz_max = min(float(tire_map.fz_axis[-1]), max(static_fz, mean_max_fz) * 2.5)

        if abs(fz_max - fz_min) < 1e-6:
            fz_axis = np.array([fz_min], dtype=float)
        else:
            fz_axis = np.linspace(fz_min, fz_max, max(int(fz_samples), 2))

        max_fy_axis = np.array(
            [pure_lateral_limit_at_fz(tire_map, fz) for fz in fz_axis],
            dtype=float,
        )
        fx_lookups = tuple(
            TireFxLimitLookup.from_tire_map(map_path, fz)
            for fz in fz_axis
        )

        return cls(
            fz_axis=fz_axis,
            max_fy_axis=max_fy_axis,
            fx_lookups=fx_lookups,
            static_fz_per_tire=static_fz,
            aero=aero,
            aero_config=aero_config,
            gravity=gravity,
        )

    def fz_at_speed(self, speed: float) -> float:
        loads = self.wheel_loads(speed=speed, ax=0.0, ay=0.0)
        return float(np.mean(loads))

    def wheel_loads(self, speed: float, ax: float, ay: float) -> np.ndarray:
        return np.maximum(
            wheel_load_stiffness(
                mass=DEFAULT_VEHICLE_MASS,
                g=self.gravity,
                aero=self.downforce_at_speed(speed),
                ax=ax,
                ay=ay,
                cg_height=CG_HEIGHT,
                wheelbase=WHEELBASE,
                front_ratio=FRONT_STATIC_RATIO,
                track_front=TRACK_FRONT,
                track_rear=TRACK_REAR,
                k_heave_front=K_HEAVE_FRONT,
                k_heave_rear=K_HEAVE_REAR,
                k_roll_front=K_ROLL_FRONT,
                k_roll_rear=K_ROLL_REAR,
            ),
            1.0,
        )

    def downforce_at_speed(self, speed: float) -> float:
        if not self.aero_config.enabled:
            return 0.0
        result = self.aero.calculate_from_modal(
            speed,
            self.aero_config.qh,
            self.aero_config.qp,
            self.aero_config.qr,
        )
        return float(result["F_down"])

    def max_fy_at_speed(self, speed: float) -> float:
        fz = np.clip(self.fz_at_speed(speed), self.fz_axis[0], self.fz_axis[-1])
        return float(np.interp(fz, self.fz_axis, self.max_fy_axis))

    def max_fy_for_loads(self, loads: np.ndarray) -> np.ndarray:
        clipped = np.clip(loads, self.fz_axis[0], self.fz_axis[-1])
        return np.interp(clipped, self.fz_axis, self.max_fy_axis)

    def fx_limits_at_fz_and_fy(self, fz: float, fy: float) -> tuple[float, float]:
        fz = float(np.clip(fz, self.fz_axis[0], self.fz_axis[-1]))

        if len(self.fz_axis) == 1:
            return self.fx_lookups[0].limits_at_fy(fy)

        upper = int(np.searchsorted(self.fz_axis, fz, side="right"))
        upper = min(max(upper, 1), len(self.fz_axis) - 1)
        lower = upper - 1

        low_fz = float(self.fz_axis[lower])
        high_fz = float(self.fz_axis[upper])
        ratio = 0.0 if high_fz == low_fz else (fz - low_fz) / (high_fz - low_fz)

        low_brake, low_drive = self.fx_lookups[lower].limits_at_fy(fy)
        high_brake, high_drive = self.fx_lookups[upper].limits_at_fy(fy)

        brake = low_brake + ratio * (high_brake - low_brake)
        drive = low_drive + ratio * (high_drive - low_drive)
        return float(brake), float(drive)

    def fx_limits_for_loads_and_fy(
        self,
        loads: np.ndarray,
        fy_per_wheel: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        brake = np.empty(4, dtype=float)
        drive = np.empty(4, dtype=float)
        for index, (fz, fy) in enumerate(zip(loads, fy_per_wheel)):
            brake[index], drive[index] = self.fx_limits_at_fz_and_fy(fz, fy)
        return brake, drive


@dataclass(frozen=True)
class SpeedPlan:
    lateral_speed_limit: np.ndarray
    speed: np.ndarray
    acceleration: np.ndarray
    phase: np.ndarray
    fz_per_tire: np.ndarray
    wheel_loads: np.ndarray
    downforce: np.ndarray
    drive_system_accel_limit: np.ndarray
    brake_system_decel_limit: np.ndarray


def load_tire_lateral_limit(
    map_path: str | Path = DEFAULT_TIRE_MAP,
    vehicle_mass: float | None = None,
    gravity: float = 9.81,
    normal_load_per_tire: float | None = None,
) -> TireLateralLimit:
    """
    Read tire_force_map.pkl and estimate the pure lateral limit at static load.

    The map is a single-tire table.  For a first-pass speed map we use static
    per-tire load and the largest absolute FY available at kappa closest to 0.
    """

    tire_map = TireForceMap.load(map_path)

    if normal_load_per_tire is None:
        if vehicle_mass is None:
            vehicle_mass = DEFAULT_VEHICLE_MASS
        normal_load_per_tire = vehicle_mass * gravity / 4.0

    alpha_grid = np.asarray(tire_map.alpha_axis_deg, dtype=float)
    kappa_zero = float(tire_map.kappa_axis[np.argmin(np.abs(tire_map.kappa_axis))])
    fz_grid = np.full_like(alpha_grid, normal_load_per_tire, dtype=float)
    kappa_grid = np.full_like(alpha_grid, kappa_zero, dtype=float)

    result = tire_map.evaluate(
        fz=fz_grid,
        alpha_deg=alpha_grid,
        kappa=kappa_grid,
        outputs=("FY",),
    )
    fy = np.asarray(result["FY"], dtype=float)
    max_lateral_force = float(np.max(np.abs(fy)))
    mu_y = max_lateral_force / float(normal_load_per_tire)

    return TireLateralLimit(
        normal_load_per_tire=float(normal_load_per_tire),
        max_lateral_force_per_tire=max_lateral_force,
        mu_y=float(mu_y),
        lateral_accel_limit=float(mu_y * gravity),
    )


def default_aero_model() -> ModalAeroModel:
    """Return the same fixed-attitude aero parameters used by car.Car."""

    return ModalAeroModel(
        rho=1.225,
        area=1.2,
        CL0=-2.4,
        CD0=1.8,
        Kh=-5.0,
        Kp=20.0,
        Kr=5.0,
        Khh=50.0,
        Kpp=30.0,
        Kd_h=0.0,
        Kd_p=0.0,
        Kd_r=0.0,
        front_balance=0.45,
        K_balance_pitch=0.20,
        K_balance_roll=0.10,
        wheelbase=1.53,
        track=(1.28 + 1.24) / 2.0,
        cop_z=0.0,
    )


def tire_normal_load_per_tire(
    speed: float,
    static_fz_per_tire: float,
    aero: ModalAeroModel,
    aero_config: AeroConfig,
) -> float:
    if not aero_config.enabled:
        return static_fz_per_tire

    result = aero.calculate_from_modal(
        speed,
        aero_config.qh,
        aero_config.qp,
        aero_config.qr,
    )
    return float(static_fz_per_tire + result["F_down"] / 4.0)


def pure_lateral_limit_at_fz(tire_map: TireForceMap, fz: float) -> float:
    alpha_grid = np.asarray(tire_map.alpha_axis_deg, dtype=float)
    kappa_zero = float(tire_map.kappa_axis[np.argmin(np.abs(tire_map.kappa_axis))])
    result = tire_map.evaluate(
        fz=np.full_like(alpha_grid, fz, dtype=float),
        alpha_deg=alpha_grid,
        kappa=np.full_like(alpha_grid, kappa_zero, dtype=float),
        outputs=("FY",),
    )
    return float(np.max(np.abs(result["FY"])))


def build_speed_profile(
    track: SegmentSpeedPath,
    tire_limit: TireLateralLimit,
    max_straight_speed: float = 30.0,
    curvature_floor: float = 1e-5,
    safety_factor: float = 1.0,
) -> np.ndarray:
    """Convert track curvature into a tire-limited speed profile."""

    curvature = np.abs(np.asarray(track.curvature, dtype=float))
    speed = np.full_like(curvature, max_straight_speed, dtype=float)

    corner_mask = curvature > curvature_floor
    speed[corner_mask] = np.sqrt(
        safety_factor
        * tire_limit.lateral_accel_limit
        / curvature[corner_mask]
    )

    return np.minimum(speed, max_straight_speed)


def build_aero_lateral_speed_profile(
    track: SegmentSpeedPath,
    tire_limit_map: AeroTireLimitMap,
    vehicle_mass: float = DEFAULT_VEHICLE_MASS,
    max_straight_speed: float = DEFAULT_MAX_STRAIGHT_SPEED,
    curvature_floor: float = 1e-5,
    safety_factor: float = 0.98,
) -> np.ndarray:
    curvature = np.abs(np.asarray(track.curvature, dtype=float))
    speed = np.full_like(curvature, max_straight_speed, dtype=float)

    for index, kappa in enumerate(curvature):
        if kappa <= curvature_floor:
            continue

        low = 0.0
        high = max_straight_speed
        for _ in range(30):
            mid = 0.5 * (low + high)
            ay = mid * mid * kappa
            loads = tire_limit_map.wheel_loads(speed=mid, ax=0.0, ay=ay)
            fy_need_total = vehicle_mass * ay
            fy_cap_total = float(np.sum(tire_limit_map.max_fy_for_loads(loads)))
            if fy_need_total <= fy_cap_total * safety_factor:
                low = mid
            else:
                high = mid

        speed[index] = low

    return speed


def build_friction_limited_speed_plan(
    track: SegmentSpeedPath,
    tire_limit_map: AeroTireLimitMap,
    power_brake_limit: PowerBrakeSystemLimit,
    vehicle_mass: float = DEFAULT_VEHICLE_MASS,
    max_straight_speed: float = DEFAULT_MAX_STRAIGHT_SPEED,
    curvature_floor: float = 1e-5,
    safety_factor: float = 0.98,
    passes: int = 3,
) -> SpeedPlan:
    """
    Build a speed plan with lateral, acceleration, and braking tire limits.

    Lateral demand is converted to per-tire FY, then the tire map is queried for
    remaining positive/negative FX.  A forward pass handles exit acceleration,
    and a backward pass pulls braking zones back from lower-speed corners.
    """

    lateral_speed_limit = build_aero_lateral_speed_profile(
        track=track,
        tire_limit_map=tire_limit_map,
        vehicle_mass=vehicle_mass,
        max_straight_speed=max_straight_speed,
        curvature_floor=curvature_floor,
        safety_factor=safety_factor,
    )
    speed = lateral_speed_limit.copy()

    for _ in range(max(int(passes), 1)):
        for index in range(len(speed) - 1):
            ds = max(float(track.s[index + 1] - track.s[index]), 1e-6)
            accel = drive_accel_at_speed(
                speed[index],
                track.curvature[index],
                tire_limit_map,
                power_brake_limit,
                vehicle_mass,
            )
            reachable = np.sqrt(max(speed[index] ** 2 + 2.0 * accel * ds, 0.0))
            speed[index + 1] = min(speed[index + 1], reachable)

        for index in range(len(speed) - 2, -1, -1):
            ds = max(float(track.s[index + 1] - track.s[index]), 1e-6)
            speed[index] = min(
                speed[index],
                max_entry_speed_for_braking(
                    exit_speed=speed[index + 1],
                    ds=ds,
                    curvature=track.curvature[index],
                    upper_speed=speed[index],
                    tire_limit_map=tire_limit_map,
                    power_brake_limit=power_brake_limit,
                    vehicle_mass=vehicle_mass,
                ),
            )

    acceleration = speed_to_acceleration(track.s, speed)
    phase = np.full(len(speed), "hold", dtype=object)
    phase[acceleration > 0.05] = "accelerate"
    phase[acceleration < -0.05] = "brake"
    fz_per_tire = np.array([tire_limit_map.fz_at_speed(v) for v in speed], dtype=float)
    downforce = np.array([tire_limit_map.downforce_at_speed(v) for v in speed], dtype=float)
    drive_system_accel_limit = np.array(
        [power_brake_limit.drive_accel_limit(v) for v in speed],
        dtype=float,
    )
    brake_system_decel_limit = np.array(
        [power_brake_limit.brake_decel_limit(v) for v in speed],
        dtype=float,
    )
    wheel_loads = np.array(
        [
            tire_limit_map.wheel_loads(
                speed=float(v),
                ax=float(ax),
                ay=float(v * v * kappa),
            )
            for v, ax, kappa in zip(speed, acceleration, track.curvature)
        ],
        dtype=float,
    )

    return SpeedPlan(
        lateral_speed_limit=lateral_speed_limit,
        speed=speed,
        acceleration=acceleration,
        phase=phase,
        fz_per_tire=fz_per_tire,
        wheel_loads=wheel_loads,
        downforce=downforce,
        drive_system_accel_limit=drive_system_accel_limit,
        brake_system_decel_limit=brake_system_decel_limit,
    )


def drive_accel_at_speed(
    speed: float,
    curvature: float,
    tire_limit_map: AeroTireLimitMap,
    power_brake_limit: PowerBrakeSystemLimit,
    vehicle_mass: float,
) -> float:
    key = (
        id(tire_limit_map),
        id(power_brake_limit),
        "drive",
        round(float(speed), 2),
        round(float(curvature), 5),
    )
    if key in ACCEL_CACHE:
        return ACCEL_CACHE[key]

    ay = speed * speed * curvature
    low = 0.0
    high = 35.0

    for _ in range(10):
        candidate_ax = 0.5 * (low + high)
        loads = tire_limit_map.wheel_loads(speed=speed, ax=candidate_ax, ay=ay)
        fy_wheels = distribute_lateral_force(vehicle_mass * ay, loads)
        _, drive_fx = tire_limit_map.fx_limits_for_loads_and_fy(loads, fy_wheels)
        tire_capacity_ax = max(float(np.sum(drive_fx)) / vehicle_mass, 0.0)
        system_capacity_ax = power_brake_limit.drive_accel_limit(speed)
        capacity_ax = min(tire_capacity_ax, system_capacity_ax)

        if candidate_ax <= capacity_ax:
            low = candidate_ax
        else:
            high = candidate_ax

    ACCEL_CACHE[key] = low
    return low


def brake_decel_at_speed(
    speed: float,
    curvature: float,
    tire_limit_map: AeroTireLimitMap,
    power_brake_limit: PowerBrakeSystemLimit,
    vehicle_mass: float,
) -> float:
    key = (
        id(tire_limit_map),
        id(power_brake_limit),
        "brake",
        round(float(speed), 2),
        round(float(curvature), 5),
    )
    if key in ACCEL_CACHE:
        return ACCEL_CACHE[key]

    ay = speed * speed * curvature
    low = 0.0
    high = 35.0

    for _ in range(10):
        candidate_decel = 0.5 * (low + high)
        loads = tire_limit_map.wheel_loads(speed=speed, ax=-candidate_decel, ay=ay)
        fy_wheels = distribute_lateral_force(vehicle_mass * ay, loads)
        brake_fx, _ = tire_limit_map.fx_limits_for_loads_and_fy(loads, fy_wheels)
        tire_capacity_decel = max(float(-np.sum(brake_fx)) / vehicle_mass, 0.0)
        system_capacity_decel = power_brake_limit.brake_decel_limit(speed)
        capacity_decel = min(tire_capacity_decel, system_capacity_decel)

        if candidate_decel <= capacity_decel:
            low = candidate_decel
        else:
            high = candidate_decel

    ACCEL_CACHE[key] = low
    return low


def distribute_lateral_force(total_fy: float, loads: np.ndarray) -> np.ndarray:
    load_sum = float(np.sum(loads))
    if load_sum <= 1e-9:
        return np.full(4, total_fy / 4.0, dtype=float)
    return total_fy * loads / load_sum


def max_entry_speed_for_braking(
    exit_speed: float,
    ds: float,
    curvature: float,
    upper_speed: float,
    tire_limit_map: AeroTireLimitMap,
    power_brake_limit: PowerBrakeSystemLimit,
    vehicle_mass: float,
) -> float:
    """Solve the highest current speed that can brake to exit_speed over ds."""

    low = float(exit_speed)
    high = float(max(upper_speed, exit_speed))

    for _ in range(10):
        mid = 0.5 * (low + high)
        decel = brake_decel_at_speed(
            mid,
            curvature,
            tire_limit_map,
            power_brake_limit,
            vehicle_mass,
        )
        reachable_exit_square = mid * mid - 2.0 * decel * ds

        if reachable_exit_square <= exit_speed * exit_speed:
            low = mid
        else:
            high = mid

    return low


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
    """Integrate lap time from distance and speed arrays."""

    ds = np.diff(s)
    segment_speed = 0.5 * (speed[:-1] + speed[1:])
    valid = (ds > 1e-9) & (segment_speed > 1e-9)

    return float(np.sum(ds[valid] / segment_speed[valid]))


def export_lap_log(
    track: SegmentSpeedPath,
    speed_plan: SpeedPlan,
    tire_limit_map: AeroTireLimitMap,
    car: Car,
    output_path: str | Path = DEFAULT_LAP_DATA_OUTPUT,
) -> Path:
    """Store one planned lap as Vehicle_state samples through SimulationLogger."""

    logger = SimulationLogger()
    heading = np.arctan2(np.gradient(track.y), np.gradient(track.x))
    time = lap_time_vector(track.s, speed_plan.speed)

    for index, (x, y, s_pos, speed, ax, kappa) in enumerate(
        zip(
            track.x,
            track.y,
            track.s,
            speed_plan.speed,
            speed_plan.acceleration,
            track.curvature,
        )
    ):
        state = Vehicle_state(car)
        state.t = float(time[index])

        if index == 0:
            state.dt = 0.0
        else:
            state.dt = float(time[index] - time[index - 1])

        yaw = float(heading[index])
        ay = float(speed * speed * kappa)
        loads = np.asarray(speed_plan.wheel_loads[index], dtype=float)
        total_fy = car.m * ay
        total_fx = car.m * float(ax)

        state.s = np.array([float(s_pos), 0.0, 0.0])
        state.s_world = np.array([float(x), float(y), 0.0])
        state.theta[3] = yaw
        state.v = np.array([float(speed), 0.0, 0.0])
        state.a = np.array([float(ax), ay, 0.0])
        state.omega[3] = float(speed * kappa)
        state.v_world = np.array(
            [float(speed * np.cos(yaw)), float(speed * np.sin(yaw)), 0.0]
        )
        state.update_world_state()

        state.N = loads.copy()
        state.Fz = loads.copy()
        state.loaded_radius = np.maximum(
            car.R_tire - loads / car.K_tire,
            0.0,
        )
        state.omega_w = np.full(4, float(speed / car.R_tire), dtype=float)

        state.Fy = distribute_lateral_force(total_fy, loads)
        state.Fx = distribute_lateral_force(total_fx, loads)
        state.F_cg = np.array([total_fx, total_fy, -float(np.sum(loads))])

        aero_result = tire_limit_map.aero.calculate_from_modal(
            float(speed),
            tire_limit_map.aero_config.qh,
            tire_limit_map.aero_config.qp,
            tire_limit_map.aero_config.qr,
        )
        state.F_aero = np.array(
            [0.0, 0.0, -float(aero_result["F_down"])],
            dtype=float,
        )
        state.COP = np.array(
            [
                float(aero_result["COP_x"]),
                float(aero_result["COP_y"]),
                tire_limit_map.aero.cop_z,
            ],
            dtype=float,
        )
        state.aero_force = state.F_aero.copy()
        state.aero_F_down = float(aero_result["F_down"])
        state.aero_F_drag = float(aero_result["F_drag"])
        state.aero_CL = float(aero_result["CL"])
        state.aero_CD = float(aero_result["CD"])
        state.aero_COP = state.COP.copy()

        if ax > 0.05:
            state.gas_cmd = 1.0
            state.brake_cmd = 0.0
        elif ax < -0.05:
            state.gas_cmd = 0.0
            state.brake_cmd = 1.0
        else:
            state.gas_cmd = 0.0
            state.brake_cmd = 0.0

        logger.add_state(state)

    logger.set_metadata(
        source="plot_track_speed_map.py",
        description="Planned full-lap speed map exported as Vehicle_state samples.",
        track_csv=str(DEFAULT_SEGMENT_CSV),
        tire_map=str(DEFAULT_TIRE_MAP),
        samples=len(track.x),
        lap_time_s=float(time[-1]),
        vehicle_mass_kg=float(car.m),
        note="Unavailable model fields are left at Vehicle_state defaults, mostly 0.0.",
    )

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    logger.save(output_path)
    return output_path


def lap_time_vector(s: np.ndarray, speed: np.ndarray) -> np.ndarray:
    time = np.zeros_like(speed, dtype=float)
    ds = np.diff(s)
    segment_speed = 0.5 * (speed[:-1] + speed[1:])
    dt = np.zeros_like(ds, dtype=float)
    valid = (ds > 1e-9) & (segment_speed > 1e-9)
    dt[valid] = ds[valid] / segment_speed[valid]
    time[1:] = np.cumsum(dt)
    return time


def solve_fx_limits_on_slice(
    fx_grid: np.ndarray,
    fy_grid: np.ndarray,
    target_fy: float,
) -> tuple[float, float]:
    candidates: list[float] = []
    rows, cols = fy_grid.shape

    for row in range(rows):
        for col in range(cols - 1):
            append_fx_crossing(
                fx_grid,
                fy_grid,
                target_fy,
                (row, col),
                (row, col + 1),
                candidates,
            )

    for row in range(rows - 1):
        for col in range(cols):
            append_fx_crossing(
                fx_grid,
                fy_grid,
                target_fy,
                (row, col),
                (row + 1, col),
                candidates,
            )

    if not candidates:
        return 0.0, 0.0

    return float(np.min(candidates)), float(np.max(candidates))


def fx_fy_boundary(fx_grid: np.ndarray, fy_grid: np.ndarray) -> np.ndarray:
    points = np.column_stack((fx_grid.ravel(), fy_grid.ravel()))
    points = np.unique(points, axis=0)
    hull = ConvexHull(points)
    return points[np.append(hull.vertices, hull.vertices[0])]


def solve_fx_limits_on_boundary(
    boundary: np.ndarray,
    target_fy: float,
) -> tuple[float, float]:
    candidates: list[float] = []

    for index in range(len(boundary) - 1):
        fx_a, fy_a = boundary[index]
        fx_b, fy_b = boundary[index + 1]

        if fy_a == fy_b:
            continue
        if not (min(fy_a, fy_b) <= target_fy <= max(fy_a, fy_b)):
            continue

        ratio = (target_fy - fy_a) / (fy_b - fy_a)
        candidates.append(float(fx_a + ratio * (fx_b - fx_a)))

    if not candidates:
        return 0.0, 0.0

    return float(np.min(candidates)), float(np.max(candidates))


def append_fx_crossing(
    fx_grid: np.ndarray,
    fy_grid: np.ndarray,
    target_fy: float,
    index_a: tuple[int, int],
    index_b: tuple[int, int],
    candidates: list[float],
) -> None:
    fy_a = float(fy_grid[index_a])
    fy_b = float(fy_grid[index_b])

    if fy_a == fy_b:
        return
    if not (min(fy_a, fy_b) <= target_fy <= max(fy_a, fy_b)):
        return

    ratio = (target_fy - fy_a) / (fy_b - fy_a)
    fx_a = float(fx_grid[index_a])
    fx_b = float(fx_grid[index_b])
    candidates.append(fx_a + ratio * (fx_b - fx_a))


def build_segment_speed_path(segment_track: SegmentTrack) -> SegmentSpeedPath:
    """Expand track_segments.py section geometry and section curvature."""

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

    return SegmentSpeedPath(
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


def plot_speed_map(
    track: SegmentSpeedPath,
    speed_plan: SpeedPlan,
    tire_limit_map: AeroTireLimitMap,
    output_path: str | Path = DEFAULT_PLANNED_OUTPUT,
) -> Path:
    """Draw the Fx/Fy-limited speed plan and save it."""

    speed = speed_plan.speed
    points = np.column_stack((track.x, track.y))
    segments = np.stack((points[:-1], points[1:]), axis=1)

    line_collection = LineCollection(
        segments,
        cmap="turbo",
        norm=plt.Normalize(float(np.min(speed)), float(np.max(speed))),
    )
    line_collection.set_array(speed[:-1])
    line_collection.set_linewidth(3.0)

    fig, (ax, ax_speed) = plt.subplots(
        2,
        1,
        figsize=(11, 10),
        gridspec_kw={"height_ratios": [3.0, 1.0]},
    )
    ax.add_collection(line_collection)
    ax.plot(track.inner_x, track.inner_y, color="0.25", linewidth=0.8, alpha=0.65)
    ax.plot(track.outer_x, track.outer_y, color="0.25", linewidth=0.8, alpha=0.65)
    ax.scatter(track.x[0], track.y[0], color="black", s=35, label="Start")

    cbar = fig.colorbar(line_collection, ax=ax)
    cbar.set_label("Speed [m/s]")

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_title(
        "Aero Fx/Fy Speed Map "
        f"(wheel load={np.min(speed_plan.wheel_loads):.0f}-{np.max(speed_plan.wheel_loads):.0f} N)"
    )
    ax.grid(True, linestyle=":", linewidth=0.6)
    ax.legend(loc="best")

    phase_colors = {
        "accelerate": "tab:green",
        "brake": "tab:red",
        "hold": "0.35",
    }
    for phase_name, color in phase_colors.items():
        mask = speed_plan.phase == phase_name
        ax_speed.scatter(
            track.s[mask],
            speed[mask],
            s=8,
            color=color,
            label=phase_name,
            alpha=0.8,
        )

    ax_speed.plot(
        track.s,
        speed_plan.lateral_speed_limit,
        color="0.45",
        linewidth=1.0,
        linestyle="--",
        label="lateral limit",
    )
    ax_speed.set_xlabel("Distance [m]")
    ax_speed.set_ylabel("Speed [m/s]")
    ax_speed.grid(True, linestyle=":", linewidth=0.6)
    ax_speed.legend(ncol=4, loc="best")
    fig.tight_layout()

    output_path = Path(output_path)
    fig.savefig(output_path, dpi=180)
    plt.show()
    return output_path


def main() -> None:
    segment_track = SegmentTrack(
        csv_file=DEFAULT_SEGMENT_CSV,
        track_width=3.0,
        ds=0.5,
    )
    track = build_segment_speed_path(segment_track)
    car = Car()
    power_brake_limit = PowerBrakeSystemLimit(car)
    static_tire_limit = load_tire_lateral_limit()
    tire_limit_map = AeroTireLimitMap.build(
        map_path=DEFAULT_TIRE_MAP,
        vehicle_mass=DEFAULT_VEHICLE_MASS,
        max_speed=DEFAULT_MAX_STRAIGHT_SPEED,
        aero_config=AeroConfig(enabled=True),
    )
    speed_plan = build_friction_limited_speed_plan(
        track=track,
        tire_limit_map=tire_limit_map,
        power_brake_limit=power_brake_limit,
        max_straight_speed=DEFAULT_MAX_STRAIGHT_SPEED,
        safety_factor=0.98,
    )
    planned_lap_time = lap_time_from_speed(track.s, speed_plan.speed)
    planned_output_path = plot_speed_map(track, speed_plan, tire_limit_map)
    lap_data_output_path = export_lap_log(
        track=track,
        speed_plan=speed_plan,
        tire_limit_map=tire_limit_map,
        car=car,
    )

    print("Tire-limited speed map complete")
    print(f"Track points: {len(track.x)}")
    print(f"Track length: {track.length:.2f} m")
    print(f"Static Fz per tire: {static_tire_limit.normal_load_per_tire:.1f} N")
    print(f"Static max lateral force per tire: {static_tire_limit.max_lateral_force_per_tire:.1f} N")
    print(f"Static estimated mu_y: {static_tire_limit.mu_y:.3f}")
    print(f"Aero Fz lookup range per tire: {tire_limit_map.fz_axis[0]:.1f} - {tire_limit_map.fz_axis[-1]:.1f} N")
    print(
        f"Lateral-only speed range: "
        f"{np.min(speed_plan.lateral_speed_limit):.2f} - "
        f"{np.max(speed_plan.lateral_speed_limit):.2f} m/s"
    )
    print(f"Planned speed range: {np.min(speed_plan.speed):.2f} - {np.max(speed_plan.speed):.2f} m/s")
    print(f"Acceleration range: {np.min(speed_plan.acceleration):.2f} - {np.max(speed_plan.acceleration):.2f} m/s^2")
    print(
        f"Drive system accel limit range: "
        f"{np.min(speed_plan.drive_system_accel_limit):.2f} - "
        f"{np.max(speed_plan.drive_system_accel_limit):.2f} m/s^2"
    )
    print(
        f"Brake system decel limit range: "
        f"{np.min(speed_plan.brake_system_decel_limit):.2f} - "
        f"{np.max(speed_plan.brake_system_decel_limit):.2f} m/s^2"
    )
    print(f"Planned mean Fz per tire range: {np.min(speed_plan.fz_per_tire):.1f} - {np.max(speed_plan.fz_per_tire):.1f} N")
    print(f"Planned steady-state wheel load range: {np.min(speed_plan.wheel_loads):.1f} - {np.max(speed_plan.wheel_loads):.1f} N")
    print(f"Planned downforce range: {np.min(speed_plan.downforce):.1f} - {np.max(speed_plan.downforce):.1f} N")
    print(f"Fx/Fy planned lap time: {planned_lap_time:.3f} s")
    print(f"Saved Fx/Fy planned map: {planned_output_path}")
    print(f"Saved planned lap data: {lap_data_output_path}")


if __name__ == "__main__":
    main()
