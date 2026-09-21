from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import sys

import numpy as np
from scipy.spatial import ConvexHull

from aero.Aero import ModalAeroModel
from car import Car_simplify
from Pacejka_tire_model.tire_mapping_tool import TireForceMap
from power_brake_sys.brake import brake as brake_torque
from power_brake_sys.motor import Throttle_command
from Suspension_sys.steady_state.sus_load import wheel_load_stiffness
from track.track_segments import SegmentTrack

Car = Car_simplify

BASE_DIR = Path(__file__).resolve().parent
DEFAULT_SEGMENT_CSV = BASE_DIR / "track" / "fsae_A_2025.csv"
DEFAULT_TIRE_MAP = BASE_DIR / "Pacejka_tire_model" / "tire_force_map.pkl"
DEFAULT_PLANNED_OUTPUT = BASE_DIR / "data" / "track_speed_map.png"
DEFAULT_LAP_DATA_OUTPUT = BASE_DIR / "data" / "data.pkl"
DEFAULT_MAX_STRAIGHT_SPEED = 30.0
GRAVITY = 9.81

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

ACCEL_CACHE: dict[tuple[int, str, float, float], float] = {}


def wheel_rate_from_kc(car: Car, shock_index: int, corner_count: float = 4.0) -> float:
    kc = car.shock_list[shock_index]
    return float(
        kc.spring.stiffness(0.0)
        * kc.motion_ratio.ratio(0.0) ** 2
        * corner_count
    )


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

@dataclass(frozen=True)
class WheelLoadState:
    loads: np.ndarray
    theta: np.ndarray
    heave: float
    aero_result: dict[str, float]



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
    ) -> "AeroTireLimitMap":
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
            TireFxLimitLookup.from_tire_map(map_path, fz)
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
                k_heave_front=wheel_rate_from_kc(self.car, 0),
                k_heave_rear=wheel_rate_from_kc(self.car, 1),
                k_roll_front=wheel_rate_from_kc(self.car, 2),
                k_roll_rear=wheel_rate_from_kc(self.car, 3),
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

    for _ in range(10):
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

