"""Motor torque lookup and drivetrain conversion for the Excel motor map."""

import numpy as np


def wheel_outputforce_omega(omega, car):
    """Return maximum wheel torque (N·m) from wheel speed (rad/s)."""
    omega = np.asarray(omega, dtype=float)
    rpm = omega * car.train_rate * 60.0 / (2.0 * np.pi)
    motor_torque = np.asarray(car.get_motor_torque_data(rpm), dtype=float)
    return motor_torque * car.train_rate * car.gear_eta


def Throttle_command(omega, car, cmd=1):
    """Scale Excel-map wheel torque by throttle command(s) in the range 0–1."""
    omega = np.asarray(omega, dtype=float)
    cmd = np.asarray(cmd, dtype=float)
    cmd = np.clip(cmd, 0.0, 1.0)
    return wheel_outputforce_omega(omega, car) * cmd


def Motor_output_torque(T_out, car):
    """Convert wheel output torque (N·m) back to motor shaft torque."""
    return np.asarray(T_out, dtype=float) / car.train_rate / car.gear_eta
