from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection
from scipy.spatial import ConvexHull, cKDTree

from aero.Aero import ModalAeroModel
from car import Car_simplify
from data.Simulation_Logger import SimulationLogger
from Pacejka_tire_maping.tire_mapping_tool import TireForceMap
from power_brake_sys.brake import brake as brake_torque
from power_brake_sys.motor import Motor_output_torque, Throttle_command
from Suspension_sys.sus_load import wheel_load_stiffness
from track.track_segments import SegmentTrack
from Vehicle_status import Vehicle_state

Car = Car_simplify

BASE_DIR = Path(__file__).resolve().parent
DEFAULT_SEGMENT_CSV = BASE_DIR / "track" / "fsae_A_2025.csv"
DEFAULT_TIRE_MAP = BASE_DIR / "Pacejka_tire_maping" / "tire_force_map.pkl"
DEFAULT_PLANNED_OUTPUT = BASE_DIR / "data" / "track_speed_map.png"
DEFAULT_LAP_DATA_OUTPUT = BASE_DIR / "data" / "data.pkl"
DEFAULT_MAX_STRAIGHT_SPEED = 30.0
DEFAULT_SLIP_FZ_CACHE_RESOLUTION = 5.0
LATERAL_SPEED_MAX_ITERATIONS = 12
LATERAL_SPEED_TOLERANCE = 0.02
ACCELERATION_MAX_ITERATIONS = 10
ACCELERATION_TOLERANCE = 0.1
BRAKING_SPEED_MAX_ITERATIONS = 10
BRAKING_SPEED_TOLERANCE = 0.06
SPEED_PLAN_CONVERGENCE_TOLERANCE = 0.05
GRAVITY = 9.81

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

ACCEL_CACHE: dict[tuple[int, str, float, float], float] = {}


def log_step(message: str) -> None:
    print(f"[theoretical_lap_simulator] {message}", flush=True)


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
        self.max_brake_torque = -np.asarray(
            brake_torque(1.0, self.car),
            dtype=float,
        )

    def drive_accel_limit(self, speed: float) -> float:
        drive_force = self.max_drive_force_distribution(speed)
        return float(np.sum(drive_force) / self.car.m)

    def max_drive_torque_distribution(
        self,
        speed: float,
        loaded_radius: np.ndarray | None = None,
    ) -> np.ndarray:
        omega = np.full(4, max(speed, 0.0) / self.car.R_tire, dtype=float)
        return np.maximum(
            np.asarray(Throttle_command(omega, self.car, cmd=self.car.T_rate), dtype=float),
            0.0,
        )

    def max_drive_force_distribution(
        self,
        speed: float,
        loaded_radius: np.ndarray | None = None,
    ) -> np.ndarray:
        if loaded_radius is None:
            loaded_radius = np.full(4, self.car.R_tire, dtype=float)
        loaded_radius = np.maximum(np.asarray(loaded_radius, dtype=float), 1e-9)
        wheel_torque = self.max_drive_torque_distribution(speed, loaded_radius)
        return wheel_torque / loaded_radius

    def scale_limited_drive_force(
        self,
        speed: float,
        tire_drive_fx_limit: np.ndarray,
        loaded_radius: np.ndarray | None = None,
    ) -> np.ndarray:
        base_force = self.max_drive_force_distribution(speed, loaded_radius)
        active = base_force > 1e-9

        if not np.any(active):
            return np.zeros(4, dtype=float)

        scale = float(
            np.min(
                np.divide(
                    tire_drive_fx_limit[active],
                    base_force[active],
                    out=np.zeros_like(base_force[active]),
                    where=base_force[active] > 1e-9,
                )
            )
        )
        scale = float(np.clip(scale, 0.0, 1.0))
        return base_force * scale

    def distribute_drive_force(
        self,
        speed: float,
        total_fx: float,
        loaded_radius: np.ndarray | None = None,
    ) -> np.ndarray:
        base_force = self.max_drive_force_distribution(speed, loaded_radius)
        base_sum = float(np.sum(base_force))

        if base_sum <= 1e-9:
            return np.zeros(4, dtype=float)

        scale = max(float(total_fx), 0.0) / base_sum
        return base_force * scale

    def drive_command_from_force(
        self,
        speed: float,
        total_fx: float,
        loaded_radius: np.ndarray | None = None,
    ) -> float:
        base_force = self.max_drive_force_distribution(speed, loaded_radius)
        base_sum = float(np.sum(base_force))

        if base_sum <= 1e-9:
            return 0.0

        return float(np.clip(max(float(total_fx), 0.0) / base_sum, 0.0, 1.0))

    def max_brake_torque_distribution(self) -> np.ndarray:
        return self.max_brake_torque.copy()

    def max_brake_force_distribution(
        self,
        loaded_radius: np.ndarray | None = None,
    ) -> np.ndarray:
        if loaded_radius is None:
            loaded_radius = np.full(4, self.car.R_tire, dtype=float)
        loaded_radius = np.maximum(np.asarray(loaded_radius, dtype=float), 1e-9)
        return -self.max_brake_torque_distribution() / loaded_radius

    def scale_limited_brake_force(
        self,
        tire_brake_fx_limit: np.ndarray,
        loaded_radius: np.ndarray | None = None,
    ) -> np.ndarray:
        base_force = self.max_brake_force_distribution(loaded_radius)
        base_capacity = -base_force
        tire_capacity = np.maximum(-np.asarray(tire_brake_fx_limit, dtype=float), 0.0)
        active = base_capacity > 1e-9

        if not np.any(active):
            return np.zeros(4, dtype=float)

        scale = float(
            np.min(
                np.divide(
                    tire_capacity[active],
                    base_capacity[active],
                    out=np.zeros_like(base_capacity[active]),
                    where=base_capacity[active] > 1e-9,
                )
            )
        )
        scale = float(np.clip(scale, 0.0, 1.0))
        return base_force * scale

    def distribute_brake_force(
        self,
        total_fx: float,
        loaded_radius: np.ndarray | None = None,
    ) -> np.ndarray:
        base_force = self.max_brake_force_distribution(loaded_radius)
        base_sum = float(np.sum(-base_force))

        if base_sum <= 1e-9:
            return np.zeros(4, dtype=float)

        scale = max(-float(total_fx), 0.0) / base_sum
        return base_force * scale

    def brake_command_from_force(
        self,
        total_fx: float,
        loaded_radius: np.ndarray | None = None,
    ) -> float:
        base_force = self.max_brake_force_distribution(loaded_radius)
        base_sum = float(np.sum(-base_force))

        if base_sum <= 1e-9:
            return 0.0

        return float(np.clip(max(-float(total_fx), 0.0) / base_sum, 0.0, 1.0))

    def brake_decel_limit(self, speed: float) -> float:
        brake_force = self.max_brake_force_distribution()
        return float(np.sum(-brake_force) / self.car.m)


@dataclass(frozen=True)
class WheelLoadState:
    loads: np.ndarray
    theta: np.ndarray
    heave: float
    aero_result: dict[str, float]


@dataclass(frozen=True)
class TireSlipSolution:
    sa_deg: np.ndarray
    sl: np.ndarray
    mx: np.ndarray
    mz: np.ndarray
    rrt: np.ndarray
    fx_error: np.ndarray
    fy_error: np.ndarray


@dataclass(frozen=True)
class TireSlipLookupSurface:
    fz: float
    fx: np.ndarray
    fy: np.ndarray
    mx: np.ndarray
    mz: np.ndarray
    rrt: np.ndarray
    fx_scale: float
    fy_scale: float
    tree: cKDTree


class TireSlipLookup:
    """Nearest force lookup for repeated tire slip solves during lap export."""

    def __init__(
        self,
        tire_map: TireForceMap,
        fz_cache_resolution: float = DEFAULT_SLIP_FZ_CACHE_RESOLUTION,
    ):
        self.tire_map = tire_map
        self.fz_cache_resolution = float(fz_cache_resolution)
        self.alpha_grid, self.kappa_grid = np.meshgrid(
            tire_map.alpha_axis_deg,
            tire_map.kappa_axis,
            indexing="ij",
        )
        self.alpha_flat = self.alpha_grid.ravel()
        self.kappa_flat = self.kappa_grid.ravel()
        self._surface_cache: dict[float, TireSlipLookupSurface] = {}

    def solve(
        self,
        fz: float,
        target_fx: float,
        target_fy: float,
    ) -> dict[str, float]:
        surface = self._surface_for_fz(fz)
        point = np.array(
            [[target_fx / surface.fx_scale, target_fy / surface.fy_scale]],
            dtype=float,
        )
        _, flat_index = surface.tree.query(point, k=1)
        index = int(np.ravel(flat_index)[0])

        return {
            "FZ": surface.fz,
            "FX": float(surface.fx[index]),
            "FY": float(surface.fy[index]),
            "MX": float(surface.mx[index]),
            "MZ": float(surface.mz[index]),
            "RRT": float(surface.rrt[index]),
            "SA_deg": float(self.alpha_flat[index]),
            "SL": float(self.kappa_flat[index]),
        }

    def _surface_for_fz(self, fz: float) -> TireSlipLookupSurface:
        cache_key = self._cache_key(fz)
        if cache_key not in self._surface_cache:
            self._surface_cache[cache_key] = self._build_surface(cache_key)
        return self._surface_cache[cache_key]

    def _cache_key(self, fz: float) -> float:
        fz = float(fz)
        if self.fz_cache_resolution <= 0.0:
            return fz
        return round(fz / self.fz_cache_resolution) * self.fz_cache_resolution

    def _build_surface(self, fz: float) -> TireSlipLookupSurface:
        fz_grid = np.full_like(self.alpha_grid, fz, dtype=float)
        result = self.tire_map.evaluate(
            fz=fz_grid,
            alpha_deg=self.alpha_grid,
            kappa=self.kappa_grid,
            outputs=("FX", "FY", "MX", "MZ", "RRT"),
        )
        fx = np.asarray(result["FX"], dtype=float).ravel()
        fy = np.asarray(result["FY"], dtype=float).ravel()
        fx_scale = max(float(np.ptp(fx)), 1.0)
        fy_scale = max(float(np.ptp(fy)), 1.0)
        points = np.column_stack((fx / fx_scale, fy / fy_scale))

        return TireSlipLookupSurface(
            fz=fz,
            fx=fx,
            fy=fy,
            mx=np.asarray(result["MX"], dtype=float).ravel(),
            mz=np.asarray(result["MZ"], dtype=float).ravel(),
            rrt=np.asarray(result["RRT"], dtype=float).ravel(),
            fx_scale=fx_scale,
            fy_scale=fy_scale,
            tree=cKDTree(points),
        )


@dataclass(frozen=True)
class SegmentSpeedPath:
    x: np.ndarray
    y: np.ndarray
    s: np.ndarray
    inner_x: np.ndarray
    inner_y: np.ndarray
    outer_x: np.ndarray
    outer_y: np.ndarray
    curvature_raw: np.ndarray
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
        tire_map: TireForceMap,
        normal_load_per_tire: float,
        samples: int = 301,
    ) -> "TireFxLimitLookup":
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
    tire_map: TireForceMap
    static_fz_per_tire: float
    car: Car
    aero: ModalAeroModel
    aero_config: AeroConfig
    gravity: float = GRAVITY
    attitude_iterations: int = 3

    @classmethod
    def build(
        cls,
        map_path: str | Path,
        car: Car,
        max_speed: float = DEFAULT_MAX_STRAIGHT_SPEED,
        gravity: float = GRAVITY,
        aero: ModalAeroModel | None = None,
        aero_config: AeroConfig = AeroConfig(),
        fz_samples: int = 25,
        tire_map: TireForceMap | None = None,
    ) -> "AeroTireLimitMap":
        if tire_map is None:
            tire_map = TireForceMap.load(map_path)
        if aero is None:
            aero = car.aero

        static_fz = car.m * gravity / 4.0
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
            TireFxLimitLookup.from_tire_map(tire_map, fz)
            for fz in fz_axis
        )

        return cls(
            fz_axis=fz_axis,
            max_fy_axis=max_fy_axis,
            fx_lookups=fx_lookups,
            tire_map=tire_map,
            static_fz_per_tire=static_fz,
            car=car,
            aero=aero,
            aero_config=aero_config,
            gravity=gravity,
            attitude_iterations=3,
        )

    def fz_at_speed(self, speed: float) -> float:
        loads = self.wheel_loads(speed=speed, ax=0.0, ay=0.0)
        return float(np.mean(loads))

    def wheel_loads(self, speed: float, ax: float, ay: float) -> np.ndarray:
        return self.wheel_load_state(speed=speed, ax=ax, ay=ay).loads

    def wheel_load_state(
        self,
        speed: float,
        ax: float,
        ay: float,
        yaw: float = 0.0,
    ) -> WheelLoadState:
        aero_result = self.aero_result_from_attitude(speed, 0.0, 0.0, 0.0)
        load_state = None

        for _ in range(max(int(self.attitude_iterations), 1)):
            load_state = wheel_load_stiffness(
                mass=self.car.m,
                g=self.gravity,
                aero=0.0 if not self.aero_config.enabled else float(aero_result["F_down"]),
                ax=ax,
                ay=ay,
                cg_height=self.car.h_cg,
                wheelbase=self.car.l,
                front_ratio=self.car.lr / self.car.l,
                track_front=self.car.tf,
                track_rear=self.car.tr,
                k_heave_front=self.car.k_heave_front,
                k_heave_rear=self.car.k_heave_rear,
                k_roll_front=self.car.k_roll_front,
                k_roll_rear=self.car.k_roll_rear,
                yaw=yaw,
                return_state=True,
            )
            aero_result = self.aero_result_from_attitude(
                speed,
                load_state.qh,
                load_state.qp,
                load_state.qr,
            )

        return WheelLoadState(
            loads=np.maximum(load_state.loads, 1.0),
            theta=load_state.theta,
            heave=float(load_state.heave),
            aero_result=aero_result,
        )

    def aero_result_from_attitude(
        self,
        speed: float,
        qh: float,
        qp: float,
        qr: float,
    ) -> dict[str, float]:
        if not self.aero_config.enabled:
            return {
                "F_down": 0.0,
                "F_drag": 0.0,
                "F_FL": 0.0,
                "F_FR": 0.0,
                "F_RL": 0.0,
                "F_RR": 0.0,
                "COP_x": 0.0,
                "COP_y": 0.0,
                "CL": 0.0,
                "CD": 0.0,
                "qh": qh,
                "qp": qp,
                "qr": qr,
            }

        return self.aero.calculate_from_modal(speed, qh, qp, qr)

    def downforce_at_speed(self, speed: float) -> float:
        return float(self.wheel_load_state(speed=speed, ax=0.0, ay=0.0).aero_result["F_down"])

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
    wheel_loads: np.ndarray
    theta: np.ndarray
    heave: np.ndarray
    downforce: np.ndarray
    drag: np.ndarray
    cop: np.ndarray
    cl: np.ndarray
    cd: np.ndarray


def load_tire_lateral_limit(
    map_path: str | Path = DEFAULT_TIRE_MAP,
    car: Car | None = None,
    gravity: float = 9.81,
    normal_load_per_tire: float | None = None,
    tire_map: TireForceMap | None = None,
) -> TireLateralLimit:
    """
    Read tire_force_map.pkl and estimate the pure lateral limit at static load.

    The map is a single-tire table.  For a first-pass speed map we use static
    per-tire load and the largest absolute FY available at kappa closest to 0.
    """

    if tire_map is None:
        tire_map = TireForceMap.load(map_path)

    if normal_load_per_tire is None:
        if car is None:
            raise ValueError("car is required when normal_load_per_tire is not given")
        normal_load_per_tire = car.m * gravity / 4.0

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
        for _ in range(LATERAL_SPEED_MAX_ITERATIONS):
            if high - low <= LATERAL_SPEED_TOLERANCE:
                break
            mid = 0.5 * (low + high)
            ay = mid * mid * kappa
            loads = tire_limit_map.wheel_loads(speed=mid, ax=0.0, ay=ay)
            fy_need_total = tire_limit_map.car.m * ay
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

    log_step("building aero lateral speed limit")
    lateral_speed_limit = build_aero_lateral_speed_profile(
        track=track,
        tire_limit_map=tire_limit_map,
        max_straight_speed=max_straight_speed,
        curvature_floor=curvature_floor,
        safety_factor=safety_factor,
    )
    speed = lateral_speed_limit.copy()

    total_passes = max(int(passes), 1)
    for pass_index in range(total_passes):
        speed_before_pass = speed.copy()
        log_step(f"speed plan pass {pass_index + 1}/{total_passes}: forward acceleration limit")
        for index in range(len(speed) - 1):
            ds = max(float(track.s[index + 1] - track.s[index]), 1e-6)
            accel = drive_accel_at_speed(
                speed[index],
                track.curvature[index],
                tire_limit_map,
                power_brake_limit,
            )
            reachable = np.sqrt(max(speed[index] ** 2 + 2.0 * accel * ds, 0.0))
            speed[index + 1] = min(speed[index + 1], reachable)

        log_step(f"speed plan pass {pass_index + 1}/{total_passes}: backward braking limit")
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
                ),
            )

        pass_speed_change = float(np.max(np.abs(speed_before_pass - speed)))
        if pass_speed_change <= SPEED_PLAN_CONVERGENCE_TOLERANCE:
            log_step(
                f"speed plan converged after pass {pass_index + 1} "
                f"(max change {pass_speed_change:.3f} m/s)"
            )
            break

    log_step("calculating speed plan acceleration and phase")
    acceleration = speed_to_acceleration(track.s, speed)
    phase = np.full(len(speed), "hold", dtype=object)
    phase[acceleration > 0.05] = "accelerate"
    phase[acceleration < -0.05] = "brake"
    log_step("calculating wheel load and aero attitude history")
    wheel_states = [
        tire_limit_map.wheel_load_state(
            speed=float(v),
            ax=float(ax),
            ay=float(v * v * kappa),
        )
        for v, ax, kappa in zip(speed, acceleration, track.curvature)
    ]
    wheel_loads = np.array([state.loads for state in wheel_states], dtype=float)
    theta = np.array([state.theta for state in wheel_states], dtype=float)
    heave = np.array([state.heave for state in wheel_states], dtype=float)
    downforce = np.array(
        [state.aero_result["F_down"] for state in wheel_states],
        dtype=float,
    )
    drag = np.array(
        [state.aero_result["F_drag"] for state in wheel_states],
        dtype=float,
    )
    cop = np.array(
        [
            [
                state.aero_result["COP_x"],
                state.aero_result["COP_y"],
                tire_limit_map.aero.cop_z,
            ]
            for state in wheel_states
        ],
        dtype=float,
    )
    cl = np.array([state.aero_result["CL"] for state in wheel_states], dtype=float)
    cd = np.array([state.aero_result["CD"] for state in wheel_states], dtype=float)

    return SpeedPlan(
        lateral_speed_limit=lateral_speed_limit,
        speed=speed,
        acceleration=acceleration,
        phase=phase,
        wheel_loads=wheel_loads,
        theta=theta,
        heave=heave,
        downforce=downforce,
        drag=drag,
        cop=cop,
        cl=cl,
        cd=cd,
    )


def drive_accel_at_speed(
    speed: float,
    curvature: float,
    tire_limit_map: AeroTireLimitMap,
    power_brake_limit: PowerBrakeSystemLimit,
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
    vehicle_mass = tire_limit_map.car.m
    low = 0.0
    high = 35.0

    for _ in range(ACCELERATION_MAX_ITERATIONS):
        if high - low <= ACCELERATION_TOLERANCE:
            break
        candidate_ax = 0.5 * (low + high)
        load_state = tire_limit_map.wheel_load_state(speed=speed, ax=candidate_ax, ay=ay)
        loads = load_state.loads
        fy_wheels = distribute_lateral_force(vehicle_mass * ay, loads)
        _, drive_fx = tire_limit_map.fx_limits_for_loads_and_fy(loads, fy_wheels)
        loaded_radius = np.maximum(
            tire_limit_map.car.R_tire - loads / tire_limit_map.car.K_tire,
            1e-9,
        )
        aero_drag = float(load_state.aero_result["F_drag"])
        rolling_resistance = float(
            np.sum(
                estimate_rolling_resistance_force(
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
            (float(np.sum(drive_force)) - aero_drag - rolling_resistance)
            / vehicle_mass,
            0.0,
        )

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
    vehicle_mass = tire_limit_map.car.m
    low = 0.0
    high = 35.0

    for _ in range(ACCELERATION_MAX_ITERATIONS):
        if high - low <= ACCELERATION_TOLERANCE:
            break
        candidate_decel = 0.5 * (low + high)
        load_state = tire_limit_map.wheel_load_state(
            speed=speed,
            ax=-candidate_decel,
            ay=ay,
        )
        loads = load_state.loads
        fy_wheels = distribute_lateral_force(vehicle_mass * ay, loads)
        brake_fx, _ = tire_limit_map.fx_limits_for_loads_and_fy(loads, fy_wheels)
        loaded_radius = np.maximum(
            tire_limit_map.car.R_tire - loads / tire_limit_map.car.K_tire,
            1e-9,
        )
        aero_drag = float(load_state.aero_result["F_drag"])
        rolling_resistance = float(
            np.sum(
                estimate_rolling_resistance_force(
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
            (float(-np.sum(brake_force)) + aero_drag + rolling_resistance)
            / vehicle_mass,
            0.0,
        )

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


def steering_angle_from_curvature(car: Car, curvature: float) -> float:
    if not np.isfinite(curvature):
        return 0.0

    denominator = 1.0 - car.lr * curvature
    if abs(denominator) < 1e-9:
        denominator = np.copysign(1e-9, denominator)

    return float(np.arctan((car.l * curvature) / denominator))


def rolling_resistance_force_from_rrt(
    rrt: np.ndarray,
    loaded_radius: np.ndarray,
) -> np.ndarray:
    loaded_radius = np.maximum(np.asarray(loaded_radius, dtype=float), 1e-9)
    return np.abs(np.asarray(rrt, dtype=float)) / loaded_radius


def estimate_rolling_resistance_force(
    tire_map: TireForceMap,
    loads: np.ndarray,
    loaded_radius: np.ndarray,
) -> np.ndarray:
    result = tire_map.evaluate(
        fz=np.asarray(loads, dtype=float),
        alpha_deg=np.zeros(4, dtype=float),
        kappa=np.zeros(4, dtype=float),
        outputs=("RRT",),
    )
    return rolling_resistance_force_from_rrt(result["RRT"], loaded_radius)


def solve_tire_slip_from_force_map(
    tire_map: TireForceMap,
    fz: np.ndarray,
    fx: np.ndarray,
    fy: np.ndarray,
    slip_lookup: TireSlipLookup | None = None,
) -> TireSlipSolution:
    sa_vehicle = np.zeros(4, dtype=float)
    sl = np.zeros(4, dtype=float)
    mx = np.zeros(4, dtype=float)
    mz = np.zeros(4, dtype=float)
    rrt = np.zeros(4, dtype=float)
    fx_error = np.zeros(4, dtype=float)
    fy_error = np.zeros(4, dtype=float)
    fy_to_map_sign = np.array([1.0, -1.0, 1.0, -1.0])
    sa_from_map_sign = np.array([-1.0, 1.0, -1.0, 1.0])
    mx_from_map_sign = -fy_to_map_sign
    mz_from_map_sign = fy_to_map_sign

    for index in range(4):
        solution = solve_single_tire_slip_from_force_map(
            tire_map=tire_map,
            fz=float(fz[index]),
            target_fx=float(fx[index]),
            target_fy=float(fy[index] * fy_to_map_sign[index]),
            slip_lookup=slip_lookup,
        )
        sa_vehicle[index] = solution["SA_deg"] * sa_from_map_sign[index]
        sl[index] = solution["SL"]
        mx[index] = solution["MX"] * mx_from_map_sign[index]
        mz[index] = solution["MZ"] * mz_from_map_sign[index]
        rrt[index] = solution["RRT"]
        fx_error[index] = solution["FX"] - float(fx[index])
        fy_error[index] = solution["FY"] * fy_to_map_sign[index] - float(fy[index])

    return TireSlipSolution(
        sa_deg=sa_vehicle,
        sl=sl,
        mx=mx,
        mz=mz,
        rrt=rrt,
        fx_error=fx_error,
        fy_error=fy_error,
    )


def solve_single_tire_slip_from_force_map(
    tire_map: TireForceMap,
    fz: float,
    target_fx: float,
    target_fy: float,
    slip_lookup: TireSlipLookup | None = None,
) -> dict[str, float]:
    if slip_lookup is not None:
        return slip_lookup.solve(fz, target_fx, target_fy)

    alpha_grid, kappa_grid = np.meshgrid(
        tire_map.alpha_axis_deg,
        tire_map.kappa_axis,
        indexing="ij",
    )
    fz_grid = np.full_like(alpha_grid, fz, dtype=float)
    result = tire_map.evaluate(
        fz=fz_grid,
        alpha_deg=alpha_grid,
        kappa=kappa_grid,
        outputs=("FX", "FY", "MX", "MZ", "RRT"),
    )
    fx_grid = np.asarray(result["FX"], dtype=float)
    fy_grid = np.asarray(result["FY"], dtype=float)
    fx_scale = max(float(np.ptp(fx_grid)), 1.0)
    fy_scale = max(float(np.ptp(fy_grid)), 1.0)
    error = ((fx_grid - target_fx) / fx_scale) ** 2 + (
        (fy_grid - target_fy) / fy_scale
    ) ** 2
    flat_index = int(np.argmin(error))
    index = np.unravel_index(flat_index, error.shape)

    return {
        "FZ": fz,
        "FX": float(fx_grid[index]),
        "FY": float(fy_grid[index]),
        "MX": float(result["MX"][index]),
        "MZ": float(result["MZ"][index]),
        "RRT": float(result["RRT"][index]),
        "SA_deg": float(alpha_grid[index]),
        "SL": float(kappa_grid[index]),
    }


def max_entry_speed_for_braking(
    exit_speed: float,
    ds: float,
    curvature: float,
    upper_speed: float,
    tire_limit_map: AeroTireLimitMap,
    power_brake_limit: PowerBrakeSystemLimit,
) -> float:
    """Solve the highest current speed that can brake to exit_speed over ds."""

    low = float(exit_speed)
    high = float(max(upper_speed, exit_speed))

    for _ in range(BRAKING_SPEED_MAX_ITERATIONS):
        if high - low <= BRAKING_SPEED_TOLERANCE:
            break
        mid = 0.5 * (low + high)
        decel = brake_decel_at_speed(
            mid,
            curvature,
            tire_limit_map,
            power_brake_limit,
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
    power_brake_limit: PowerBrakeSystemLimit,
    car: Car,
    output_path: str | Path = DEFAULT_LAP_DATA_OUTPUT,
    slip_fz_cache_resolution: float = DEFAULT_SLIP_FZ_CACHE_RESOLUTION,
) -> Path:
    """Store one planned lap as Vehicle_state samples through SimulationLogger."""

    log_step("exporting lap log: reusing tire map")
    logger = SimulationLogger()
    tire_map = tire_limit_map.tire_map
    slip_lookup = TireSlipLookup(
        tire_map,
        fz_cache_resolution=slip_fz_cache_resolution,
    )
    log_step("exporting lap log: preparing kinematic histories")
    heading = np.arctan2(np.gradient(track.y), np.gradient(track.x))
    time = lap_time_vector(track.s, speed_plan.speed)

    static_loads = tire_limit_map.wheel_loads(speed=0.0, ax=0.0, ay=0.0)
    static_loaded_radius = np.maximum(car.R_tire - static_loads / car.K_tire, 0.0)
    loaded_radius_history = np.maximum(
        car.R_tire - speed_plan.wheel_loads / car.K_tire,
        0.0,
    )
    # Flat road: wheel-center height follows loaded radius; upward is positive.
    zu_history = loaded_radius_history - static_loaded_radius
    vzu_history = gradient_by_time(zu_history, time)
    azu_history = gradient_by_time(vzu_history, time)

    theta_history = speed_plan.theta.copy()
    theta_history[:, 3] = heading
    s_history = np.column_stack(
        (
            track.s,
            np.zeros_like(track.s),
            speed_plan.heave,
        )
    )
    v_history = np.column_stack(
        (
            speed_plan.speed,
            np.zeros_like(speed_plan.speed),
            gradient_by_time(speed_plan.heave, time),
        )
    )
    a_history = np.column_stack(
        (
            speed_plan.acceleration,
            speed_plan.speed * speed_plan.speed * track.curvature,
            gradient_by_time(v_history[:, 2], time),
        )
    )
    omega_history = gradient_by_time(theta_history, time)
    alpha_history = gradient_by_time(omega_history, time)

    total_samples = len(track.x)
    progress_step = max(total_samples // 10, 1)
    log_step(f"exporting lap log: solving tire slips for {total_samples} samples")
    for index, (x, y, speed, ax, kappa) in enumerate(
        zip(
            track.x,
            track.y,
            speed_plan.speed,
            speed_plan.acceleration,
            track.curvature,
        )
    ):
        if index == 0 or (index + 1) % progress_step == 0 or index + 1 == total_samples:
            progress = 100.0 * (index + 1) / max(total_samples, 1)
            log_step(
                f"exporting lap log: sample {index + 1}/{total_samples} "
                f"({progress:.0f}%)"
            )

        state = Vehicle_state(car)
        state.t = float(time[index])

        ay = float(speed * speed * kappa)
        loads = np.asarray(speed_plan.wheel_loads[index], dtype=float)
        total_fy = car.m * ay
        motion_fx = car.m * float(ax)
        aero_drag = float(speed_plan.drag[index])
        theta = theta_history[index].copy()
        aero_cop = speed_plan.cop[index]
        steer_angle = steering_angle_from_curvature(car, float(kappa))

        state.s = s_history[index].copy()
        state.s_world = np.array([float(x), float(y), 0.0])
        state.theta = theta
        state.v = v_history[index].copy()
        state.a = a_history[index].copy()
        state.omega = omega_history[index].copy()
        state.alpha = alpha_history[index].copy()
        state.update_world_state()
        state.zs = Vehicle_state.body_to_corner(state.s, state.theta, car)
        state.vzs = Vehicle_state.body_to_corner(state.v, state.omega, car)
        state.azs = Vehicle_state.body_to_corner(state.a, state.alpha, car)

        state.N = loads.copy()
        state.Fz = loads.copy()
        state.Fs = loads.copy()
        state.Ft = loads.copy()
        state.loaded_radius = loaded_radius_history[index].copy()
        state.zu = zu_history[index].copy()
        state.vzu = vzu_history[index].copy()
        state.azu = azu_history[index].copy()
        state.omega_w = speed / np.maximum(state.loaded_radius, 1e-9)
        state.steer = np.array([steer_angle, steer_angle, 0.0, 0.0], dtype=float)

        state.Fy = distribute_lateral_force(total_fy, loads)
        rolling_resistance = estimate_rolling_resistance_force(
            tire_map=tire_map,
            loads=loads,
            loaded_radius=state.loaded_radius,
        )
        total_tire_fx = motion_fx + aero_drag + float(np.sum(rolling_resistance))

        for _ in range(2):
            if total_tire_fx > 0.05:
                state.Fx = power_brake_limit.distribute_drive_force(
                    speed=float(speed),
                    total_fx=total_tire_fx,
                    loaded_radius=state.loaded_radius,
                )
            elif total_tire_fx < -0.05:
                state.Fx = power_brake_limit.distribute_brake_force(
                    total_fx=total_tire_fx,
                    loaded_radius=state.loaded_radius,
                )
            else:
                state.Fx = distribute_lateral_force(total_tire_fx, loads)

            slip_solution = solve_tire_slip_from_force_map(
                tire_map=tire_map,
                fz=state.Fz,
                fx=state.Fx,
                fy=state.Fy,
                slip_lookup=slip_lookup,
            )
            rolling_resistance = rolling_resistance_force_from_rrt(
                slip_solution.rrt,
                state.loaded_radius,
            )
            total_tire_fx = motion_fx + aero_drag + float(np.sum(rolling_resistance))

        if total_tire_fx > 0.05:
            state.Fx = power_brake_limit.distribute_drive_force(
                speed=float(speed),
                total_fx=total_tire_fx,
                loaded_radius=state.loaded_radius,
            )
        elif total_tire_fx < -0.05:
            state.Fx = power_brake_limit.distribute_brake_force(
                total_fx=total_tire_fx,
                loaded_radius=state.loaded_radius,
            )
        else:
            state.Fx = distribute_lateral_force(total_tire_fx, loads)

        slip_solution = solve_tire_slip_from_force_map(
            tire_map=tire_map,
            fz=state.Fz,
            fx=state.Fx,
            fy=state.Fy,
            slip_lookup=slip_lookup,
        )
        rolling_resistance = rolling_resistance_force_from_rrt(
            slip_solution.rrt,
            state.loaded_radius,
        )
        total_tire_fx = motion_fx + aero_drag + float(np.sum(rolling_resistance))
        state.F_cg = np.array([motion_fx, total_fy, -float(np.sum(loads))])
        gas_cmd = 0.0
        brake_cmd = 0.0
        if total_tire_fx > 0.05:
            gas_cmd = power_brake_limit.drive_command_from_force(
                speed=float(speed),
                total_fx=total_tire_fx,
                loaded_radius=state.loaded_radius,
            )
        elif total_tire_fx < -0.05:
            brake_cmd = power_brake_limit.brake_command_from_force(
                total_fx=total_tire_fx,
                loaded_radius=state.loaded_radius,
            )

        state.SA = slip_solution.sa_deg
        state.SL = slip_solution.sl
        state.Mx_w = slip_solution.mx
        state.Mz_w = slip_solution.mz
        state.RRT_w = slip_solution.rrt
        state.tire_map_Fx_error = slip_solution.fx_error
        state.tire_map_Fy_error = slip_solution.fy_error

        state.F_aero = np.array(
            [-float(speed_plan.drag[index]), 0.0, -float(speed_plan.downforce[index])],
            dtype=float,
        )
        state.COP = aero_cop.copy()
        state.aero_force = state.F_aero.copy()
        state.aero_F_down = float(speed_plan.downforce[index])
        state.aero_F_drag = float(speed_plan.drag[index])
        state.aero_CL = float(speed_plan.cl[index])
        state.aero_CD = float(speed_plan.cd[index])
        state.aero_COP = state.COP.copy()
        state.aero_qh = float(speed_plan.heave[index])
        state.aero_qp = float(theta[2])
        state.aero_qr = float((theta[0] + theta[1]) / 2.0)

        state.gas_cmd = gas_cmd
        state.brake_cmd = brake_cmd
        state.phase = str(speed_plan.phase[index])

        wheel_torque = state.Fx * state.loaded_radius
        state.T_drive = np.maximum(wheel_torque, 0.0)
        state.T_brake = np.minimum(wheel_torque, 0.0)
        state.T_motor = Motor_output_torque(state.T_drive, car)

        # Keep calculations in radians; export chassis attitude in degrees.
        state.theta = np.degrees(state.theta)
        logger.add_state(state)

    logger.set_metadata(
        source="theoretical_lap_simulator.py",
        description="Planned full-lap speed map exported as Vehicle_state samples.",
        track_csv=str(DEFAULT_SEGMENT_CSV),
        tire_map=str(DEFAULT_TIRE_MAP),
        samples=len(track.x),
        lap_time_s=float(time[-1]),
        vehicle_mass_kg=float(car.m),
        theta_unit="deg",
        omega_unit="rad/s",
        alpha_unit="rad/s^2",
        zu_unit="m",
        vzu_unit="m/s",
        azu_unit="m/s^2",
        unsprung_positive_direction="up",
        unsprung_reference="Static equilibrium at zero speed and acceleration on flat road.",
        static_loaded_radius_m=static_loaded_radius.tolist(),
        note=(
            "Generated from simplified steady-state speed planning. "
            "Front/rear roll are set equal for aero attitude; dynamic suspension "
            "fields are kinematic estimates, not full transient suspension output."
        ),
    )

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    log_step(f"exporting lap log: saving {output_path}")
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


def gradient_by_time(value: np.ndarray, time: np.ndarray) -> np.ndarray:
    value = np.asarray(value, dtype=float)
    time = np.asarray(time, dtype=float)

    if len(time) < 2 or np.ptp(time) <= 1e-12:
        return np.zeros_like(value, dtype=float)

    return np.gradient(value, time, axis=0, edge_order=1)


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
    """Expand track geometry using each section's original curvature."""

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
    curvature_raw = np.concatenate(curvature_parts)
    curvature = curvature_raw.copy()

    return SegmentSpeedPath(
        x=x,
        y=y,
        s=np.cumsum(ds),
        inner_x=np.concatenate(inner_x_parts),
        inner_y=np.concatenate(inner_y_parts),
        outer_x=np.concatenate(outer_x_parts),
        outer_y=np.concatenate(outer_y_parts),
        curvature_raw=curvature_raw,
        curvature=curvature,
        length=float(sum(section.length for section in segment_track.sections)),
    )


def plot_speed_map(
    track: SegmentSpeedPath,
    speed_plan: SpeedPlan,
    tire_limit_map: AeroTireLimitMap,
    lap_time_s: float,
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
    ax.set_title(f"Aero Fx/Fy Speed Map (lap time={lap_time_s:.3f} s)")
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
    log_step(f"saving speed map figure to {output_path}")
    fig.savefig(output_path, dpi=180)
    log_step("showing speed map figure; close the figure window to continue lap-data export")
    plt.show()
    log_step("speed map figure closed")
    return output_path


def main() -> None:
    log_step("loading track segments")
    segment_track = SegmentTrack(
        csv_file=DEFAULT_SEGMENT_CSV,
        track_width=3.0,
        ds=0.5,
    )
    log_step("building track speed path")
    track = build_segment_speed_path(segment_track)
    log_step("initializing car and power/brake limits")
    car = Car()
    power_brake_limit = PowerBrakeSystemLimit(car)
    log_step("loading tire force map")
    tire_map = TireForceMap.load(DEFAULT_TIRE_MAP)
    log_step("building aero tire limit map")
    tire_limit_map = AeroTireLimitMap.build(
        map_path=DEFAULT_TIRE_MAP,
        car=car,
        max_speed=DEFAULT_MAX_STRAIGHT_SPEED,
        aero_config=AeroConfig(enabled=True),
        tire_map=tire_map,
    )
    log_step("building friction-limited speed plan")
    speed_plan = build_friction_limited_speed_plan(
        track=track,
        tire_limit_map=tire_limit_map,
        power_brake_limit=power_brake_limit,
        max_straight_speed=DEFAULT_MAX_STRAIGHT_SPEED,
        safety_factor=0.98,
    )
    log_step("calculating planned lap time")
    planned_lap_time = lap_time_from_speed(track.s, speed_plan.speed)
    log_step("plotting speed map")
    planned_output_path = plot_speed_map(
        track,
        speed_plan,
        tire_limit_map,
        lap_time_s=planned_lap_time,
    )
    log_step("exporting Vehicle_state lap data")
    lap_data_output_path = export_lap_log(
        track=track,
        speed_plan=speed_plan,
        tire_limit_map=tire_limit_map,
        power_brake_limit=power_brake_limit,
        car=car,
    )
    log_step("all calculations complete")

    print("Tire-limited speed map complete")
    print(f"Track points: {len(track.x)}")
    print(f"Track length: {track.length:.2f} m")
    print(f"Aero Fz lookup range per tire: {tire_limit_map.fz_axis[0]:.1f} - {tire_limit_map.fz_axis[-1]:.1f} N")
    print(
        f"Lateral-only speed range: "
        f"{np.min(speed_plan.lateral_speed_limit):.2f} - "
        f"{np.max(speed_plan.lateral_speed_limit):.2f} m/s"
    )
    print(f"Planned speed range: {np.min(speed_plan.speed):.2f} - {np.max(speed_plan.speed):.2f} m/s")
    print(f"Acceleration range: {np.min(speed_plan.acceleration):.2f} - {np.max(speed_plan.acceleration):.2f} m/s^2")
    print(f"Planned steady-state wheel load range: {np.min(speed_plan.wheel_loads):.1f} - {np.max(speed_plan.wheel_loads):.1f} N")
    print(f"Planned downforce range: {np.min(speed_plan.downforce):.1f} - {np.max(speed_plan.downforce):.1f} N")
    print(f"Fx/Fy planned lap time: {planned_lap_time:.3f} s")
    print(f"Saved Fx/Fy planned map: {planned_output_path}")
    print(f"Saved planned lap data: {lap_data_output_path}")


if __name__ == "__main__":
    main()
