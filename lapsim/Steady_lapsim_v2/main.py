from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import sys

BASE_DIR = Path(__file__).resolve().parent
PROJECT_DIR = BASE_DIR.parent
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection

from Aero import ModalAeroModel
from car import Car_simplify
from data.Simulation_Logger import SimulationLogger
from TireModel import DEFAULT_JSON as DEFAULT_TIRE_MODEL, SimplifiedTireModel
from brake import brake as brake_torque
from motor import Motor_output_torque, Throttle_command
from suspension import wheel_load_stiffness
from track_segments import DEFAULT_SEGMENT_CSV, SegmentTrack
from Vehicle_status import Vehicle_state

Car = Car_simplify

DEFAULT_PLANNED_OUTPUT = BASE_DIR / "data" / "track_speed_map.png"
DEFAULT_LAP_DATA_OUTPUT = BASE_DIR / "data" / "data.pkl"
DEFAULT_MAX_STRAIGHT_SPEED = 30.0
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


def camber_from_roll(theta_rad: np.ndarray) -> np.ndarray:
    """Return wheel camber in FL, FR, RL, RR order with zero static camber and gain."""
    camber_deg = np.degrees(theta_rad[..., [0, 0, 1, 1]])
    # Positive chassis roll lifts the left side; positive camber tips outward.
    camber_deg[..., ::2] *= -1
    return camber_deg


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
class AeroTireLimits:
    fz_axis: np.ndarray
    max_fy_axis: np.ndarray
    tire_model: SimplifiedTireModel
    static_fz_per_tire: float
    car: Car
    aero: ModalAeroModel
    aero_config: AeroConfig
    gravity: float = GRAVITY
    attitude_iterations: int = 3

    @classmethod
    def build(
        cls,
        model_path: str | Path,
        car: Car,
        max_speed: float = DEFAULT_MAX_STRAIGHT_SPEED,
        gravity: float = GRAVITY,
        aero: ModalAeroModel | None = None,
        aero_config: AeroConfig = AeroConfig(),
        fz_samples: int = 25,
        tire_model: SimplifiedTireModel | None = None,
    ) -> "AeroTireLimits":
        if tire_model is None:
            tire_model = SimplifiedTireModel(model_path)
        if aero is None:
            aero = car.aero

        static_fz = car.m * gravity / 4.0
        mean_max_fz = tire_normal_load_per_tire(
            speed=max_speed,
            static_fz_per_tire=static_fz,
            aero=aero,
            aero_config=aero_config,
        )
        fz_min, fz_max = tire_model.fz_range
        fz_max = min(fz_max, max(static_fz, mean_max_fz) * 2.5)

        if abs(fz_max - fz_min) < 1e-6:
            fz_axis = np.array([fz_min], dtype=float)
        else:
            fz_axis = np.linspace(fz_min, fz_max, max(int(fz_samples), 2))

        max_fy_axis = np.array(
            [tire_model.peak_forces(float(fz))[1] for fz in fz_axis],
            dtype=float,
        )

        return cls(
            fz_axis=fz_axis,
            max_fy_axis=max_fy_axis,
            tire_model=tire_model,
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

    def max_fy_for_loads(
        self,
        loads: np.ndarray,
        camber_deg: float | np.ndarray = 0.0,
    ) -> np.ndarray:
        clipped_loads = np.clip(loads, *self.tire_model.fz_range)
        clipped_cambers = np.broadcast_to(
            np.clip(camber_deg, *self.tire_model.camber_range), clipped_loads.shape
        )
        return np.array(
            [
                self.tire_model.peak_forces(float(fz), float(camber))[1]
                for fz, camber in zip(clipped_loads, clipped_cambers)
            ],
            dtype=float,
        )

    def fx_limits_at_fz_and_fy(
        self,
        fz: float,
        fy: float,
        camber_deg: float = 0.0,
    ) -> tuple[float, float]:
        fz = float(np.clip(fz, *self.tire_model.fz_range))
        camber_deg = float(np.clip(camber_deg, *self.tire_model.camber_range))
        limits = self.tire_model.fx_limits(fy, fz=fz, camber_deg=camber_deg)
        if len(limits) < 2:
            return 0.0, 0.0
        return float(limits[0]), float(limits[1])

    def fx_limits_for_loads_and_fy(
        self,
        loads: np.ndarray,
        fy_per_wheel: np.ndarray,
        camber_deg: float | np.ndarray = 0.0,
    ) -> tuple[np.ndarray, np.ndarray]:
        brake = np.empty(4, dtype=float)
        drive = np.empty(4, dtype=float)
        cambers = np.broadcast_to(camber_deg, np.shape(loads))
        for index, (fz, fy, camber) in enumerate(zip(loads, fy_per_wheel, cambers)):
            brake[index], drive[index] = self.fx_limits_at_fz_and_fy(fz, fy, camber)
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
    model_path: str | Path = DEFAULT_TIRE_MODEL,
    car: Car | None = None,
    gravity: float = 9.81,
    normal_load_per_tire: float | None = None,
    tire_model: SimplifiedTireModel | None = None,
) -> TireLateralLimit:
    """Estimate the pure lateral limit at static load from the tire model."""
    if tire_model is None:
        tire_model = SimplifiedTireModel(model_path)

    if normal_load_per_tire is None:
        if car is None:
            raise ValueError("car is required when normal_load_per_tire is not given")
        normal_load_per_tire = car.m * gravity / 4.0

    normal_load_per_tire = float(np.clip(normal_load_per_tire, *tire_model.fz_range))
    max_lateral_force = tire_model.peak_forces(normal_load_per_tire)[1]
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
    tire_limit_map: AeroTireLimits,
    max_straight_speed: float = DEFAULT_MAX_STRAIGHT_SPEED,
    curvature_floor: float = 1e-5,
    safety_factor: float = 0.98,
) -> np.ndarray:
    curvature = np.asarray(track.curvature, dtype=float)
    speed = np.full_like(curvature, max_straight_speed, dtype=float)

    for index, kappa in enumerate(curvature):
        if abs(kappa) <= curvature_floor:
            continue

        low = 0.0
        high = max_straight_speed
        for _ in range(LATERAL_SPEED_MAX_ITERATIONS):
            if high - low <= LATERAL_SPEED_TOLERANCE:
                break
            mid = 0.5 * (low + high)
            ay = mid * mid * kappa
            load_state = tire_limit_map.wheel_load_state(speed=mid, ax=0.0, ay=ay)
            fy_need_total = tire_limit_map.car.m * abs(ay)
            fy_cap = tire_limit_map.max_fy_for_loads(
                load_state.loads, camber_from_roll(load_state.theta)
            )
            fy_cap_total = float(np.sum(fy_cap))
            if fy_need_total <= fy_cap_total * safety_factor:
                low = mid
            else:
                high = mid

        speed[index] = low

    return speed


def build_friction_limited_speed_plan(
    track: SegmentSpeedPath,
    tire_limit_map: AeroTireLimits,
    power_brake_limit: PowerBrakeSystemLimit,
    max_straight_speed: float = DEFAULT_MAX_STRAIGHT_SPEED,
    curvature_floor: float = 1e-5,
    safety_factor: float = 0.98,
    passes: int = 3,
) -> SpeedPlan:
    """
    Build a speed plan with lateral, acceleration, and braking tire limits.

    Lateral demand is converted to per-tire FY, then the tire model estimates
    remaining positive/negative FX. A forward pass handles exit acceleration,
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
    tire_limit_map: AeroTireLimits,
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
        _, drive_fx = tire_limit_map.fx_limits_for_loads_and_fy(
            loads, fy_wheels, camber_from_roll(load_state.theta)
        )
        loaded_radius = np.maximum(
            tire_limit_map.car.R_tire - loads / tire_limit_map.car.K_tire,
            1e-9,
        )
        aero_drag = float(load_state.aero_result["F_drag"])
        drive_force = power_brake_limit.scale_limited_drive_force(
            speed,
            drive_fx,
            loaded_radius=loaded_radius,
        )
        capacity_ax = max(
            (float(np.sum(drive_force)) - aero_drag)
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
    tire_limit_map: AeroTireLimits,
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
        brake_fx, _ = tire_limit_map.fx_limits_for_loads_and_fy(
            loads, fy_wheels, camber_from_roll(load_state.theta)
        )
        loaded_radius = np.maximum(
            tire_limit_map.car.R_tire - loads / tire_limit_map.car.K_tire,
            1e-9,
        )
        aero_drag = float(load_state.aero_result["F_drag"])
        brake_force = power_brake_limit.scale_limited_brake_force(
            brake_fx,
            loaded_radius=loaded_radius,
        )
        capacity_decel = max(
            (float(-np.sum(brake_force)) + aero_drag)
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


def max_entry_speed_for_braking(
    exit_speed: float,
    ds: float,
    curvature: float,
    upper_speed: float,
    tire_limit_map: AeroTireLimits,
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
    tire_limit_map: AeroTireLimits,
    power_brake_limit: PowerBrakeSystemLimit,
    car: Car,
    output_path: str | Path = DEFAULT_LAP_DATA_OUTPUT,
) -> Path:
    """Store one planned lap as Vehicle_state samples through SimulationLogger."""

    log_step("exporting lap log: using simplified tire model")
    logger = SimulationLogger()
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
    camber_history = camber_from_roll(theta_history)
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
    log_step(f"exporting lap log: preparing {total_samples} samples")
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
        state.camber_deg = camber_history[index].copy()

        state.Fy = distribute_lateral_force(total_fy, loads)
        total_tire_fx = motion_fx + aero_drag

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
        tire_model=str(DEFAULT_TIRE_MODEL),
        tire_model_type="simplified combined-force superellipse",
        tire_camber_range_deg=list(tire_limit_map.tire_model.camber_range),
        camber_definition=(
            "Road-relative wheel camber in FL, FR, RL, RR order, outward positive. "
            "Derived from chassis roll with zero static camber and camber gain. "
            "The tire force queries clip the angles to the fitted range."
        ),
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
            "fields are kinematic estimates, not full transient suspension output. "
            "The simplified tire model does not provide slip, tire moments, or "
            "rolling resistance; corresponding legacy output fields are zero."
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
    tire_limit_map: AeroTireLimits,
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
    log_step("loading simplified tire model")
    tire_model = SimplifiedTireModel(DEFAULT_TIRE_MODEL)
    log_step("building aero tire limits")
    tire_limit_map = AeroTireLimits.build(
        model_path=DEFAULT_TIRE_MODEL,
        car=car,
        max_speed=DEFAULT_MAX_STRAIGHT_SPEED,
        aero_config=AeroConfig(enabled=True),
        tire_model=tire_model,
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
    print(
        f"Tire-model Fz range per tire: "
        f"{tire_model.fz_range[0]:.1f} - {tire_model.fz_range[1]:.1f} N"
    )
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
