"""Steady-state combined Fx/Fy speed planning from new/lapsim/main.py."""
from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from math import ceil, log2

import numpy as np

from .brake import brake as brake_torque, hydraulic_brake
from .car import Car
from .motor import Throttle_command
from .suspension import wheel_load_stiffness
from .tire import FrictionEllipse, TabulatedTire, TireEnvelope
from .track import SectionGeometry


DEFAULT_MAX_STRAIGHT_SPEED = 30.0
LATERAL_SPEED_MAX_ITERATIONS = 12
LATERAL_SPEED_TOLERANCE = 0.02
ACCELERATION_MAX_ITERATIONS = 10
ACCELERATION_TOLERANCE = 0.1
ACCELERATION_MAX = 35.0
ACCELERATION_BINS = 2 ** min(
    ACCELERATION_MAX_ITERATIONS, ceil(log2(ACCELERATION_MAX / ACCELERATION_TOLERANCE)),
)
BRAKING_SPEED_MAX_ITERATIONS = 10
BRAKING_SPEED_TOLERANCE = 0.06
SPEED_PLAN_CONVERGENCE_TOLERANCE = 0.05
GRAVITY = 9.81

class PowerBrakeSystemLimit:
    """Full-throttle and full-brake force limits for the vehicle."""

    def __init__(self, car: Car):
        self.car = car
        self.accel_cache: dict[tuple[str, float, float], float] = {}
        brake_model = hydraulic_brake if car.settings.brake_model == "hydraulic" else brake_torque
        self.max_brake_torque = -np.asarray(
            brake_model(1.0, self.car),
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
    total_load: float


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
    curvature: np.ndarray
    length: float


@dataclass(frozen=True)
class AeroTireLimits:
    tire_model: TireEnvelope
    car: Car
    gravity: float = GRAVITY
    attitude_iterations: int = 3

    def wheel_load_state(
        self,
        speed: float,
        ax: float,
        ay: float,
        yaw: float = 0.0,
    ) -> WheelLoadState:
        aero_result = self.aero_result_from_attitude(speed, 0.0, 0.0, 0.0)
        if self.car.settings.load_model == "point_mass":
            total = max(self.car.m * self.gravity + float(aero_result["F_down"]), 0.0)
            return WheelLoadState(np.zeros(4), np.zeros(4), 0.0, aero_result, total)

        for _ in range(max(int(self.attitude_iterations), 1)):
            load_state = self.suspension_state(aero_result, ax, ay, yaw)
            aero_result = self.aero_result_from_attitude(
                speed,
                load_state.qh,
                load_state.qp,
                load_state.qr,
            )

        # Preserve the historical default load floor. Alternative tire models enforce contact
        # and conserve total vertical load when a wheel unloads.
        if self.car.settings.tire_model == "superellipse":
            loads = np.maximum(load_state.loads, 1.0)
        else:
            loads = np.maximum(load_state.loads, 0.0)
            loads *= max(float(np.sum(load_state.loads)), 0.0) / max(float(np.sum(loads)), 1e-9)
        return WheelLoadState(
            loads=loads,
            theta=load_state.theta,
            heave=float(load_state.heave),
            aero_result=aero_result,
            total_load=float(np.sum(loads)),
        )

    def suspension_state(self, aero: dict[str, float], ax: float, ay: float, yaw: float = 0.0):
        car = self.car
        return wheel_load_stiffness(
            mass=car.m,
            g=self.gravity,
            aero=float(aero["F_down"]),
            ax=ax,
            ay=ay,
            cg_height=car.h_cg,
            wheelbase=car.l,
            front_ratio=car.lr / car.l,
            track_front=car.tf,
            track_rear=car.tr,
            k_heave_front=car.k_heave_front,
            k_heave_rear=car.k_heave_rear,
            k_roll_front=car.k_roll_front,
            k_roll_rear=car.k_roll_rear,
            yaw=yaw,
            return_state=True,
            tire_stiffness=car.K_tire,
        )

    def aero_result_from_attitude(
        self,
        speed: float,
        qh: float,
        qp: float,
        qr: float,
    ) -> dict[str, float]:
        if self.car.aero is None:
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

        return self.car.aero.calculate_from_modal(speed, qh, qp, qr)

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
                self.tire_model.peak_forces(float(fz), float(camber))[1] if load > 0 else 0.0
                for load, fz, camber in zip(loads, clipped_loads, clipped_cambers)
            ],
            dtype=float,
        )

    def fx_limits_at_fz_and_fy(
        self,
        fz: float,
        fy: float,
        camber_deg: float = 0.0,
    ) -> tuple[float, float] | None:
        if fz <= 0:
            return (0.0, 0.0) if fy == 0 else None
        lower, upper = self.tire_model.fz_range
        fz = min(max(fz, lower), upper)
        lower, upper = self.tire_model.camber_range
        camber_deg = min(max(camber_deg, lower), upper)
        limits = self.tire_model.fx_limits(fy, fz=fz, camber_deg=camber_deg)
        if not limits:
            return None
        return float(limits[0]), float(limits[-1])

    def fx_limits_for_loads_and_fy(
        self,
        loads: np.ndarray,
        fy_per_wheel: np.ndarray,
        camber_deg: float | np.ndarray = 0.0,
    ) -> tuple[np.ndarray, np.ndarray] | None:
        """Return longitudinal bounds only when every wheel can supply its Fy."""
        brake = np.empty(4, dtype=float)
        drive = np.empty(4, dtype=float)
        cambers = np.broadcast_to(camber_deg, np.shape(loads))
        for index, (fz, fy, camber) in enumerate(zip(loads, fy_per_wheel, cambers)):
            bounds = self.fx_limits_at_fz_and_fy(fz, fy, camber)
            if bounds is None:
                return None
            brake[index], drive[index] = bounds
        return brake, drive

    def rolling_resistance(self, loads: np.ndarray, radii: np.ndarray) -> np.ndarray:
        model = self.car.settings.rolling_model
        if model == "constant":
            return self.car.settings.rolling_coefficient * loads
        if model == "table" and isinstance(self.tire_model, TabulatedTire):
            return np.where(loads > 0, self.tire_model.rolling_torque(loads) / radii, 0.0)
        return np.zeros_like(loads)

    def rolling_force(self, state: WheelLoadState, radii: np.ndarray) -> float:
        return float(self.rolling_resistance(state.loads, radii).sum())

    def point_mass_rolling(self, total_load):
        model = self.car.settings.rolling_model
        if model == "constant":
            return self.car.settings.rolling_coefficient * total_load
        if model == "table" and isinstance(self.tire_model, TabulatedTire):
            return np.where(total_load > 0, 4 * self.tire_model.rolling_torque(total_load / 4) / self.car.R_tire, 0.0)
        return total_load * 0.0

    def point_mass_fx_limits(self, total_load: float, total_fy: float):
        if isinstance(self.tire_model, FrictionEllipse):
            return self.tire_model.fx_limits(total_fy, fz=total_load)
        # Fitted and tabulated envelopes use single-tire normal loads.
        bounds = self.fx_limits_at_fz_and_fy(total_load / 4, total_fy / 4)
        return () if bounds is None else (4 * bounds[0], 4 * bounds[1])

    def lateral_forces(self, total_fy: float, loads: np.ndarray) -> np.ndarray:
        """Balance the front/rear lateral-force moments about the CG."""
        front_ratio = self.car.lr / self.car.l
        return np.r_[
            distribute_lateral_force(total_fy * front_ratio, loads[:2]),
            distribute_lateral_force(total_fy * (1 - front_ratio), loads[2:]),
        ]

    def balanced_lateral_limits(self, loads: np.ndarray, camber_deg: np.ndarray) -> np.ndarray:
        shares = self.lateral_forces(1.0, loads)
        capacity = self.max_fy_for_loads(loads, camber_deg)
        total_limit = np.min(np.divide(capacity, shares, out=np.full_like(capacity, np.inf), where=shares > 0))
        return shares * total_limit

    def available_lateral_force(self, state: WheelLoadState) -> float:
        if self.car.settings.load_model == "point_mass":
            if isinstance(self.tire_model, FrictionEllipse):
                return self.tire_model.peak_forces(state.total_load)[1]
            if state.total_load <= 0:
                return 0.0
            lower, upper = self.tire_model.fz_range
            load = min(max(state.total_load / 4, lower), upper)
            lower, upper = self.tire_model.camber_range
            camber = min(max(0.0, lower), upper)
            return 4 * self.tire_model.peak_forces(load, camber)[1]
        return float(np.sum(self.balanced_lateral_limits(state.loads, camber_from_roll(state.theta))))


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
    total_normal_load: np.ndarray
    rolling_resistance: np.ndarray
    total_rolling_resistance: np.ndarray
    passes_used: int
    converged: bool | None


def build_aero_lateral_speed_profile(
    track: SegmentSpeedPath,
    tire_limit_map: AeroTireLimits,
    max_straight_speed: float = DEFAULT_MAX_STRAIGHT_SPEED,
    curvature_floor: float = 1e-5,
    safety_factor: float = 0.98,
    *,
    check_deadline: Callable[[], None] | None = None,
) -> np.ndarray:
    curvature = np.asarray(track.curvature, dtype=float)
    speed = np.full_like(curvature, max_straight_speed, dtype=float)

    for index, kappa in enumerate(curvature):
        if check_deadline is not None:
            check_deadline()
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
            fy_cap_total = tire_limit_map.available_lateral_force(load_state)
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
    progress: Callable[[str], None] | None = None,
    *,
    check_deadline: Callable[[], None] | None = None,
) -> SpeedPlan:
    """
    Build a speed plan with lateral, acceleration, and braking tire limits.

    Lateral demand is converted to per-tire FY, then the tire model estimates
    remaining positive/negative FX. A forward pass handles exit acceleration,
    and a backward pass pulls braking zones back from lower-speed corners.
    """

    report = progress or (lambda message: None)
    report("計算側向抓地速度上限")
    lateral_speed_limit = build_aero_lateral_speed_profile(
        track=track,
        tire_limit_map=tire_limit_map,
        max_straight_speed=max_straight_speed,
        curvature_floor=curvature_floor,
        safety_factor=safety_factor,
        check_deadline=check_deadline,
    )
    settings = tire_limit_map.car.settings
    if settings.yaw_correction:
        lateral_speed_limit = apply_yaw_correction(
            track, lateral_speed_limit, tire_limit_map, check_deadline=check_deadline,
        )
    if settings.solver == "distance":
        report("沿距離向前積分")
        speed = integrate_forward(
            track, lateral_speed_limit, tire_limit_map, power_brake_limit, settings.initial_speed,
            check_deadline=check_deadline,
        )
        passes_used, converged = 0, None
    else:
        speed, passes_used, converged = iterate_lap(
            track, lateral_speed_limit, tire_limit_map, power_brake_limit, passes, report,
            check_deadline=check_deadline,
        )

    return summarize_plan(
        track, speed, lateral_speed_limit, tire_limit_map, report, passes_used, converged,
        check_deadline=check_deadline,
    )


def apply_yaw_correction(track, speed, limits, *, check_deadline=None):
    car = limits.car
    arms = np.array([car.lf, car.lf, car.lr, car.lr])
    ds = np.diff(track.s)
    rates = np.divide(np.diff(track.curvature), ds, out=np.zeros_like(ds), where=ds > 1e-9)
    gradient = np.r_[rates[0], (rates[:-1] + rates[1:]) / 2, rates[-1]]
    corrected = speed.copy()
    for index, (v, kappa, rate) in enumerate(zip(speed, track.curvature, gradient)):
        if check_deadline is not None:
            check_deadline()
        state = limits.wheel_load_state(v, 0.0, v * v * kappa)
        capacity = float(arms @ limits.max_fy_for_loads(state.loads, camber_from_roll(state.theta)))
        if capacity > 1e-9:
            ratio = car.settings.yaw_inertia * v * v * abs(rate) / capacity
            corrected[index] *= np.clip(1.0 - 0.25 * ratio, 0.05, 1.0)
    return corrected


def integrate_forward(track, lateral_limit, limits, power, initial_speed, *, check_deadline=None):
    """Advance once in s with either a braking envelope or a local distance estimate."""
    entry_limit = lateral_limit.copy()
    spacing = np.diff(track.s)
    lookahead = limits.car.settings.braking_lookahead
    if lookahead:
        for index in range(len(entry_limit) - 2, -1, -1):
            if check_deadline is not None:
                check_deadline()
            ds = float(spacing[index])
            if ds <= 1e-9:
                entry_limit[index] = min(entry_limit[index], entry_limit[index + 1])
            else:
                entry_limit[index] = min(entry_limit[index], max_entry_speed_for_braking(
                    entry_limit[index + 1], ds, track.curvature[index], entry_limit[index], limits, power,
                ))
    else:
        targets = np.flatnonzero(np.r_[
            False, (lateral_limit[1:-1] < lateral_limit[:-2]) & (lateral_limit[1:-1] <= lateral_limit[2:]), True,
        ])
    speed = np.empty_like(lateral_limit)
    speed[0] = initial_speed
    for index, ds in enumerate(spacing):
        if check_deadline is not None:
            check_deadline()
        if ds <= 1e-9:
            speed[index + 1] = speed[index]
            continue
        v = float(speed[index])
        curvature = float(track.curvature[index])
        drive = drive_accel_at_speed(v, curvature, limits, power)
        cap = entry_limit[index + 1]**2
        if not lookahead:
            brake = brake_decel_at_speed(v, curvature, limits, power)
            target = targets[np.searchsorted(targets, index, side="right")]
            distance_to_target = track.s[target] - track.s[index + 1]
            cap = min(cap, lateral_limit[target]**2 + 2 * brake * distance_to_target)
        # A reachable speed takes precedence over a missed target. The result
        # retains the lateral limit so any overspeed is visible to the caller.
        reachable_min = 0.0
        if cap < v * v:
            if lookahead:
                brake = brake_decel_at_speed(v, curvature, limits, power)
            reachable_min = max(v * v - 2 * brake * ds, 0.0)
        reachable_max = v * v + 2 * drive * ds
        speed[index + 1] = np.sqrt(np.clip(cap, reachable_min, reachable_max))
    return speed


def iterate_lap(track, lateral_speed_limit, tire_limit_map, power_brake_limit, passes, report, *, check_deadline=None):
    speed = lateral_speed_limit.copy()

    total_passes = max(int(passes), 1)
    closed = tire_limit_map.car.settings.closed_lap
    for pass_index in range(total_passes):
        speed_before_pass = speed.copy()
        if closed:
            speed[0] = speed[-1] = min(speed[0], speed[-1])
        report(f"第 {pass_index + 1}/{total_passes} 輪，向前掃描加速限制")
        for index in range(len(speed) - 1):
            if check_deadline is not None:
                check_deadline()
            ds = max(float(track.s[index + 1] - track.s[index]), 1e-6)
            accel = drive_accel_at_speed(
                speed[index],
                track.curvature[index],
                tire_limit_map,
                power_brake_limit,
            )
            reachable = np.sqrt(max(speed[index] ** 2 + 2.0 * accel * ds, 0.0))
            speed[index + 1] = min(speed[index + 1], reachable)

        report(f"第 {pass_index + 1}/{total_passes} 輪，向後掃描煞車限制")
        if closed:
            speed[0] = speed[-1] = min(speed[0], speed[-1])
        for index in range(len(speed) - 2, -1, -1):
            if check_deadline is not None:
                check_deadline()
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

        if closed:
            speed[0] = speed[-1] = min(speed[0], speed[-1])
        pass_speed_change = float(np.max(np.abs(speed_before_pass - speed)))
        if pass_speed_change <= SPEED_PLAN_CONVERGENCE_TOLERANCE:
            return speed, pass_index + 1, True
    return speed, total_passes, False


def summarize_plan(track, speed, lateral_speed_limit, tire_limit_map, report, passes_used, converged, *,
                   check_deadline=None):
    acceleration = speed_to_acceleration(track.s, speed)
    phase = np.full(len(speed), "hold", dtype=object)
    phase[acceleration > 0.05] = "accelerate"
    phase[acceleration < -0.05] = "brake"
    report("整理四輪輪荷與車身姿態")
    wheel_states = []
    for v, ax, kappa in zip(speed, acceleration, track.curvature):
        if check_deadline is not None:
            check_deadline()
        wheel_states.append(tire_limit_map.wheel_load_state(
            speed=float(v),
            ax=float(ax),
            ay=float(v * v * kappa),
        ))
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
                tire_limit_map.car.aero.cop_z if tire_limit_map.car.aero else 0.0,
            ]
            for state in wheel_states
        ],
        dtype=float,
    )
    cl = np.array([state.aero_result["CL"] for state in wheel_states], dtype=float)
    cd = np.array([state.aero_result["CD"] for state in wheel_states], dtype=float)
    car = tire_limit_map.car
    radii = np.maximum(car.R_tire - wheel_loads / car.K_tire, 1e-9)
    rolling = tire_limit_map.rolling_resistance(wheel_loads, radii)
    total_load = np.array([state.total_load for state in wheel_states])
    total_rolling = rolling.sum(axis=1)
    if car.settings.load_model == "point_mass":
        total_rolling = tire_limit_map.point_mass_rolling(total_load)

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
        total_normal_load=total_load,
        rolling_resistance=rolling,
        total_rolling_resistance=total_rolling,
        passes_used=passes_used,
        converged=converged,
    )


def drive_accel_at_speed(
    speed: float,
    curvature: float,
    tire_limit_map: AeroTireLimits,
    power_brake_limit: PowerBrakeSystemLimit,
) -> float:
    key = (
        "drive",
        round(float(speed), 2),
        round(float(curvature), 5),
    )
    cache = power_brake_limit.accel_cache
    if key not in cache:
        cache[key] = drive_accel_at_lateral_accel(
            speed, speed * speed * curvature, tire_limit_map, power_brake_limit,
        )
    return cache[key]


def drive_accel_at_lateral_accel(
    speed: float, ay: float, tire_limit_map: AeroTireLimits, power_brake_limit: PowerBrakeSystemLimit,
) -> float:
    if tire_limit_map.car.settings.load_model == "point_mass":
        return point_mass_accel(speed, ay, tire_limit_map, power_brake_limit, braking=False)
    vehicle_mass = tire_limit_map.car.m
    low = 0.0
    high = ACCELERATION_MAX

    for _ in range(ACCELERATION_MAX_ITERATIONS):
        if high - low <= ACCELERATION_TOLERANCE:
            break
        candidate_ax = 0.5 * (low + high)
        load_state = tire_limit_map.wheel_load_state(speed=speed, ax=candidate_ax, ay=ay)
        loads = load_state.loads
        loaded_radius = np.maximum(
            tire_limit_map.car.R_tire - loads / tire_limit_map.car.K_tire,
            1e-9,
        )
        aero_drag = float(load_state.aero_result["F_drag"])
        fy_wheels = tire_limit_map.lateral_forces(vehicle_mass * ay, loads)
        bounds = tire_limit_map.fx_limits_for_loads_and_fy(
            loads, fy_wheels, camber_from_roll(load_state.theta),
        )
        if bounds is None:
            high = candidate_ax
            continue
        _, drive_fx = bounds
        drive_force = float(power_brake_limit.scale_limited_drive_force(speed, drive_fx, loaded_radius).sum())
        rolling = tire_limit_map.rolling_force(load_state, loaded_radius)
        capacity_ax = max(
            (drive_force - aero_drag - rolling)
            / vehicle_mass,
            0.0,
        )

        if candidate_ax <= capacity_ax:
            low = candidate_ax
        else:
            high = candidate_ax

    return low


def brake_decel_at_speed(
    speed: float,
    curvature: float,
    tire_limit_map: AeroTireLimits,
    power_brake_limit: PowerBrakeSystemLimit,
) -> float:
    key = (
        "brake",
        round(float(speed), 2),
        round(float(curvature), 5),
    )
    cache = power_brake_limit.accel_cache
    if key not in cache:
        cache[key] = brake_decel_at_lateral_accel(
            speed, speed * speed * curvature, tire_limit_map, power_brake_limit,
        )
    return cache[key]


def brake_decel_at_lateral_accel(
    speed: float, ay: float, tire_limit_map: AeroTireLimits, power_brake_limit: PowerBrakeSystemLimit,
) -> float:
    if tire_limit_map.car.settings.load_model == "point_mass":
        return point_mass_accel(speed, ay, tire_limit_map, power_brake_limit, braking=True)
    vehicle_mass = tire_limit_map.car.m
    low = 0.0
    high = ACCELERATION_MAX

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
        loaded_radius = np.maximum(
            tire_limit_map.car.R_tire - loads / tire_limit_map.car.K_tire,
            1e-9,
        )
        aero_drag = float(load_state.aero_result["F_drag"])
        fy_wheels = tire_limit_map.lateral_forces(vehicle_mass * ay, loads)
        bounds = tire_limit_map.fx_limits_for_loads_and_fy(
            loads, fy_wheels, camber_from_roll(load_state.theta),
        )
        if bounds is None:
            high = candidate_decel
            continue
        brake_fx, _ = bounds
        brake_force = float(power_brake_limit.scale_limited_brake_force(brake_fx, loaded_radius).sum())
        rolling = tire_limit_map.rolling_force(load_state, loaded_radius)
        capacity_decel = max(
            (-brake_force + aero_drag + rolling)
            / vehicle_mass,
            0.0,
        )

        if candidate_decel <= capacity_decel:
            low = candidate_decel
        else:
            high = candidate_decel

    return low


def point_mass_accel(speed, ay, limits, power, *, braking):
    mass = limits.car.m
    state = limits.wheel_load_state(speed, 0.0, ay)
    lower, upper = limits.point_mass_fx_limits(state.total_load, mass * ay) or (0.0, 0.0)
    resistance = state.aero_result["F_drag"] + limits.point_mass_rolling(state.total_load)
    if braking:
        wheel_force = float(power.max_brake_force_distribution().sum())
        capacity = (-max(lower, wheel_force) + resistance) / mass
    else:
        wheel_force = float(power.max_drive_force_distribution(speed).sum())
        capacity = (min(upper, wheel_force) - resistance) / mass
    # Match the feasible lower endpoint of the regular acceleration bisection.
    step = ACCELERATION_MAX / ACCELERATION_BINS
    return min(max(capacity, 0.0), ACCELERATION_MAX - step) // step * step


def distribute_lateral_force(total_fy: float, loads: np.ndarray) -> np.ndarray:
    load_sum = float(np.sum(loads))
    if load_sum <= 1e-9:
        return np.full_like(loads, total_fy / len(loads), dtype=float)
    return total_fy * loads / load_sum


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
    if high <= low:
        return low

    # A feasible upper bound must remain unchanged. Repeatedly lowering it by
    # the bisection tolerance would reduce speeds on every full-lap pass.
    decel = brake_decel_at_speed(high, curvature, tire_limit_map, power_brake_limit)
    if high * high - 2.0 * decel * ds <= exit_speed * exit_speed:
        return high

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


def lap_time_vector(s: np.ndarray, speed: np.ndarray) -> np.ndarray:
    time = np.zeros_like(speed, dtype=float)
    ds = np.diff(s)
    segment_speed = 0.5 * (speed[:-1] + speed[1:])
    dt = np.zeros_like(ds, dtype=float)
    valid = (ds > 1e-9) & (segment_speed > 1e-9)
    dt[valid] = ds[valid] / segment_speed[valid]
    time[1:] = np.cumsum(dt)
    return time


def build_segment_speed_path(sections: list[SectionGeometry]) -> SegmentSpeedPath:
    """Expand track geometry using each section's original curvature."""

    x_parts = []
    y_parts = []
    inner_x_parts = []
    inner_y_parts = []
    outer_x_parts = []
    outer_y_parts = []
    curvature_parts = []

    for index, geometry in enumerate(sections):
        start = 0 if index == 0 else 1
        x_parts.append(geometry.x[start:])
        y_parts.append(geometry.y[start:])
        inner_x_parts.append(geometry.inner_x[start:])
        inner_y_parts.append(geometry.inner_y[start:])
        outer_x_parts.append(geometry.outer_x[start:])
        outer_y_parts.append(geometry.outer_y[start:])
        curvature_parts.append(
            np.full(len(geometry.x[start:]), geometry.curvature, dtype=float)
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
        length=float(sum(section.length for section in sections)),
    )
