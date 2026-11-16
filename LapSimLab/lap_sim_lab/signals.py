"""Per-sample telemetry derived from the current steady-state speed plan."""

from dataclasses import dataclass

import numpy as np

from .planner import BRAKING_SPEED_TOLERANCE, GRAVITY, camber_from_roll
from .tire import TabulatedTire


@dataclass(frozen=True)
class Signal:
    label: str
    unit: str
    values: np.ndarray


def time_gradient(values, time):
    # Repeated path points share a timestamp. Differentiate on the unique grid.
    unique, first, inverse = np.unique(time, return_index=True, return_inverse=True)
    if len(unique) < 2:
        return np.zeros_like(values)
    return np.gradient(values[first], unique, axis=0)[inverse]


def longitudinal_signals(plan, radius, load_share, power, *, check_deadline=None):
    """Allocate the estimated force demand with the planner's drivetrain and brake split."""
    radius = np.maximum(radius, 1e-9)
    total = power.car.m * plan.acceleration + plan.drag + plan.total_rolling_resistance
    force = total[:, None] * load_share
    drive_cmd = np.zeros_like(total)
    brake_cmd = np.zeros_like(total)
    for index in np.flatnonzero(total > 0.05):
        if check_deadline is not None:
            check_deadline()
        base = power.max_drive_force_distribution(plan.speed[index], radius[index])
        capacity = float(base.sum())
        ratio = total[index] / capacity if capacity > 1e-9 else 0.0
        force[index] = base * ratio
        drive_cmd[index] = min(ratio, 1.0)
    braking = total < -0.05
    base = power.max_brake_torque_distribution() / radius[braking]
    capacity = base.sum(axis=1)
    ratio = np.divide(-total[braking], capacity, out=np.zeros_like(capacity), where=capacity > 1e-9)
    force[braking] = -base * ratio[:, None]
    brake_cmd[braking] = np.minimum(ratio, 1.0)
    return force, drive_cmd, brake_cmd


def build_signals(track, plan, time, limits, power, *, check_deadline=None):
    car = limits.car
    signals = {}

    def add(key, label, unit, values):
        signals[key] = Signal(label, unit, values)

    def add_wheels(key, label, unit, suffix, values):
        for wheel, column in zip(("FL", "FR", "RL", "RR"), values.T):
            add(f"{key}_{wheel}_{suffix}", f"{label} {wheel}", unit, column)

    roll, pitch = plan.theta[:, 0], plan.theta[:, 2]
    point_mass = car.settings.load_model == "point_mass"
    yaw = np.unwrap(np.arctan2(np.gradient(track.y), np.gradient(track.x)))
    theta = np.column_stack((roll, pitch, yaw))
    omega = time_gradient(theta, time)
    alpha = time_gradient(omega, time)

    # Positive roll raises the left, positive pitch raises the nose.
    # The existing positive-down heave is converted to positive-up z.
    corner_y = np.array([car.tf, -car.tf, car.tr, -car.tr]) / 2.0
    corner_x = np.array([car.lf, car.lf, -car.lr, -car.lr])
    zs = -plan.heave[:, None] + roll[:, None] * corner_y + pitch[:, None] * corner_x
    vzs = time_gradient(zs, time)
    azs = time_gradient(vzs, time)

    loads = plan.wheel_loads
    radius = np.maximum(car.R_tire - loads / car.K_tire, 0.0)
    static_loads = limits.wheel_load_state(0.0, 0.0, 0.0).loads
    static_radius = np.maximum(car.R_tire - static_loads / car.K_tire, 0.0)
    zu = radius - static_radius
    vzu = time_gradient(zu, time)
    azu = time_gradient(vzu, time)
    wheel_omega = plan.speed[:, None] / np.maximum(radius, 1e-9)
    wheel_alpha = time_gradient(wheel_omega, time)

    denominator = 1.0 - car.lr * track.curvature
    denominator = np.where(np.abs(denominator) < 1e-9, np.copysign(1e-9, denominator), denominator)
    front_steer = np.arctan(car.l * track.curvature / denominator)
    zero = np.zeros_like(time)
    steer = np.degrees(np.column_stack((front_steer, front_steer, zero, zero)))
    ay = plan.speed**2 * track.curvature
    normal = loads.sum(axis=1, keepdims=True)
    load_share = np.divide(loads, normal, out=np.zeros_like(loads), where=normal > 0)
    fy = np.zeros_like(loads)
    if not point_mass:
        for index, (lateral, load) in enumerate(zip(ay, loads)):
            if check_deadline is not None:
                check_deadline()
            fy[index] = limits.lateral_forces(car.m * lateral, load)
    fx, drive_cmd, brake_cmd = longitudinal_signals(
        plan, radius, load_share, power, check_deadline=check_deadline,
    )
    wheel_torque = fx * radius
    drive_torque = np.maximum(wheel_torque, 0.0)
    brake_torque = np.minimum(wheel_torque, 0.0)
    total_fx = car.m * plan.acceleration + plan.drag + plan.total_rolling_resistance

    body_vz = time_gradient(-plan.heave, time)
    body_az = time_gradient(body_vz, time)
    c, s = np.cos(yaw), np.sin(yaw)
    add("time_s", "時間", "s", time)
    add("distance_m", "行駛距離", "m", track.s)
    add("speed_kmh", "車速", "km/h", plan.speed * 3.6)
    add("lateral_speed_limit_kmh", "側向速度上限", "km/h", plan.lateral_speed_limit * 3.6)
    add("lateral_overspeed", "超過側向速度上限", "代碼",
        (plan.speed > plan.lateral_speed_limit + BRAKING_SPEED_TOLERANCE).astype(int))
    add("x_m", "賽道 X", "m", track.x)
    add("y_m", "賽道 Y", "m", track.y)
    add("curvature_1_m", "賽道曲率", "1/m", track.curvature)
    add("phase_code", "行駛階段 −1 煞車、0 持速、1 加速", "代碼",
        (plan.phase == "accelerate").astype(int) - (plan.phase == "brake").astype(int))
    add("ax_g", "縱向加速度", "g", plan.acceleration / GRAVITY)
    add("ay_g", "橫向加速度", "g", ay / GRAVITY)
    add("heave_mm", "整車升沉（向下為正）", "mm", plan.heave * 1000)
    for index, (axis, label) in enumerate((("roll", "側傾"), ("pitch", "俯仰"), ("yaw", "航向"))):
        add(f"{axis}_deg", label, "deg", np.degrees(theta[:, index]))
        add(f"{axis}_rate_rad_s", f"{label}角速度", "rad/s", omega[:, index])
        add(f"{axis}_accel_rad_s2", f"{label}角加速度", "rad/s²", alpha[:, index])
    add("vx_m_s", "車身縱向速度", "m/s", plan.speed)
    add("vz_m_s", "車身垂直速度", "m/s", body_vz)
    add("ax_m_s2", "車身縱向加速度", "m/s²", plan.acceleration)
    add("ay_m_s2", "車身橫向加速度", "m/s²", ay)
    add("az_m_s2", "車身垂直加速度", "m/s²", body_az)
    add("world_vx_m_s", "地面座標 X 速度", "m/s", plan.speed * c)
    add("world_vy_m_s", "地面座標 Y 速度", "m/s", plan.speed * s)
    add("world_ax_m_s2", "地面座標 X 加速度", "m/s²", plan.acceleration * c - ay * s)
    add("world_ay_m_s2", "地面座標 Y 加速度", "m/s²", plan.acceleration * s + ay * c)
    for key, label, unit, suffix, values in (
        ("zs", "簧上位移", "mm", "mm", zs * 1000),
        ("vzs", "簧上速度", "m/s", "m_s", vzs),
        ("azs", "簧上加速度", "m/s²", "m_s2", azs),
        ("zu", "簧下位移", "mm", "mm", zu * 1000),
        ("vzu", "簧下速度", "m/s", "m_s", vzu),
        ("azu", "簧下加速度", "m/s²", "m_s2", azu),
        ("fx", "縱向輪胎力估算", "N", "N", fx),
        ("fy", "側向輪胎力估算", "N", "N", fy),
        ("fz", "垂直輪荷", "N", "N", loads),
        ("steer", "轉向角估算", "deg", "deg", steer),
        ("camber", "外傾角", "deg", "deg", camber_from_roll(plan.theta)),
        ("loaded_radius", "受載半徑", "mm", "mm", radius * 1000),
        ("wheel_omega", "車輪角速度", "rad/s", "rad_s", wheel_omega),
        ("wheel_alpha", "車輪角加速度", "rad/s²", "rad_s2", wheel_alpha),
        ("motor_speed", "馬達轉速估算", "RPM", "rpm", wheel_omega * car.train_rate * 60 / (2 * np.pi)),
        ("drive_torque", "輪端驅動扭矩估算", "N·m", "Nm", drive_torque),
        ("brake_torque", "輪端煞車扭矩估算", "N·m", "Nm", brake_torque),
        ("motor_torque", "馬達扭矩估算", "N·m", "Nm",
         drive_torque / (car.train_rate * car.gear_eta) if car.gear_eta else np.zeros_like(drive_torque)),
        ("rolling_resistance", "滾動阻力", "N", "N", plan.rolling_resistance),
    ):
        if point_mass:
            values = np.zeros_like(values)
        add_wheels(key, label, unit, suffix, values)
    if not point_mass and isinstance(limits.tire_model, TabulatedTire):
        diagnostics = limits.tire_model.estimate_slip(loads, fx, fy, check_deadline=check_deadline)
    else:
        diagnostics = np.zeros((len(TabulatedTire.diagnostic_names), *loads.shape))
    diagnostic_fields = (
        ("輪胎側滑角反推", "deg", "deg"),
        ("輪胎滑移率反推", "比例", "ratio"),
        ("輪胎傾覆力矩", "N·m", "Nm"),
        ("輪胎回正力矩", "N·m", "Nm"),
        ("輪胎滾阻力矩", "N·m", "Nm"),
        ("反推縱向力殘差", "N", "N"),
        ("反推側向力殘差", "N", "N"),
    )
    for key, (label, unit, suffix), values in zip(TabulatedTire.diagnostic_names, diagnostic_fields, diagnostics):
        add_wheels(key, label, unit, suffix, values)
    add("fx_total_N", "整車縱向輪胎力", "N", total_fx if point_mass else fx.sum(axis=1))
    add("fy_total_N", "整車側向輪胎力", "N", car.m * ay)
    add("fz_total_N", "整車垂直輪荷", "N", plan.total_normal_load)
    add("rolling_resistance_total_N", "整車滾動阻力", "N", plan.total_rolling_resistance)
    add("fx_demand_N", "縱向輪胎總需求（含阻力）", "N", total_fx)
    add("gas_cmd", "驅動踏板命令估算", "比例", drive_cmd)
    add("brake_cmd", "煞車踏板命令估算", "比例", brake_cmd)
    add("downforce_N", "空力下壓力", "N", plan.downforce)
    add("drag_N", "空力阻力", "N", plan.drag)
    add("CL", "升力係數 CL", "無因次", plan.cl)
    add("CD", "阻力係數 CD", "無因次", plan.cd)
    for axis, values in zip("xyz", plan.cop.T):
        add(f"cop_{axis}_m", f"空力中心 {axis.upper()}", "m", values)
    relative_travel = (zs - zu) * 1000
    if point_mass:
        relative_travel.fill(0.0)
    add_wheels("zs_minus_zu", "簧上－簧下相對位移", "mm", "mm", relative_travel)
    return signals
