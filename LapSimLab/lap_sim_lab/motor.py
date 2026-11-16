"""Linear interpolation and extrapolation of the original LapSim motor CSV."""

import csv

import numpy as np

from .localization import localize


class MotorCurve:
    def __init__(self, path):
        points = []
        with path.open(encoding="utf-8-sig", newline="") as stream:
            reader = csv.DictReader(stream)
            if not {"speed", "torque"}.issubset(reader.fieldnames or ()):
                raise ValueError(localize("{filename} 須包含 speed（RPM）和 torque（N·m）欄位。", filename=path.name))
            for row in reader:
                try:
                    points.append((float(row["speed"]), float(row["torque"])))
                except (TypeError, ValueError) as exc:
                    raise ValueError(localize(
                        "{filename} 第 {line_number} 列的馬達資料須為數字。",
                        filename=path.name, line_number=reader.line_num,
                    )) from exc
        if len(points) < 2:
            raise ValueError(localize("{filename} 至少需要兩筆馬達資料。", filename=path.name))
        self.rpm, self.torque = np.asarray(sorted(points), dtype=float).T
        if not np.isfinite(points).all() or self.rpm[0] < 0 or np.any(np.diff(self.rpm) <= 0):
            raise ValueError(localize("{filename} 的轉速須為不重複的非負數，且資料須為有限值。", filename=path.name))
        self.slope = np.diff(self.torque) / np.diff(self.rpm)

    def torque_at(self, speed_rpm):
        index = np.searchsorted(self.rpm, speed_rpm).clip(1, len(self.rpm) - 1) - 1
        return self.torque[index] + (speed_rpm - self.rpm[index]) * self.slope[index]


class VoltageMotorCurve:
    """Interpolate a fixed-voltage torque curve from the LP03 EV model's envelope."""

    @staticmethod
    def read_points(path):
        with path.open(encoding="utf-8-sig", newline="") as stream:
            reader = csv.DictReader(stream)
            keys = ("vdc_V", "rpm", "T_shaft_Nm")
            if not set(keys).issubset(reader.fieldnames or ()):
                raise ValueError(localize("{filename} 須包含 vdc_V、rpm 和 T_shaft_Nm。", filename=path.name))
            try:
                points = np.array([[float(row[key]) for key in keys] for row in reader])
            except (TypeError, ValueError) as error:
                raise ValueError(localize("{filename} 含有無效的電壓或扭矩資料。", filename=path.name)) from error
        if len(points) < 2 or not np.isfinite(points).all() or np.any(points[:, :2] < 0):
            raise ValueError(localize("{filename} 的電壓與轉速資料無效。", filename=path.name))
        return points

    def __init__(self, path, voltage):
        points = self.read_points(path)
        voltages = np.unique(points[:, 0])
        if not voltages[0] <= voltage <= voltages[-1]:
            raise ValueError(localize(
                "直流電壓須介於 {minimum:g} 與 {maximum:g} V。",
                minimum=voltages[0], maximum=voltages[-1],
            ))
        hi = int(np.searchsorted(voltages, voltage))
        lo = hi if voltages[hi] == voltage else hi - 1
        self.curves = []
        for index in (lo, hi):
            rows = points[points[:, 0] == voltages[index], 1:]
            rpm, torque = rows[np.argsort(rows[:, 0])].T
            if len(rpm) < 2 or np.any(np.diff(rpm) <= 0):
                raise ValueError(localize("{filename} 同一電壓須有至少兩筆不重複的轉速。", filename=path.name))
            self.curves.append((rpm, torque))
        self.weight = 0.0 if lo == hi else (voltage - voltages[lo]) / (voltages[hi] - voltages[lo])
        self.rpm = np.union1d(*(rpm for rpm, _ in self.curves))
        self.torque = self.torque_at(self.rpm)

    def torque_at(self, speed_rpm):
        low, high = (np.interp(speed_rpm, rpm, torque, right=0.0) for rpm, torque in self.curves)
        return (1 - self.weight) * low + self.weight * high


def wheel_outputforce_omega(omega, car):
    """Return maximum wheel torque (N·m) from wheel speed (rad/s)."""
    omega = np.asarray(omega, dtype=float)
    rpm = omega * car.train_rate * 60.0 / (2.0 * np.pi)
    motor_torque = np.asarray(car.get_motor_torque_data(rpm), dtype=float)
    return motor_torque * car.train_rate * car.gear_eta


def Throttle_command(omega, car, cmd=1):
    """Scale available wheel torque by throttle commands in the range 0–1."""
    omega = np.asarray(omega, dtype=float)
    cmd = np.asarray(cmd, dtype=float)
    cmd = np.clip(cmd, 0.0, 1.0)
    return wheel_outputforce_omega(omega, car) * cmd
