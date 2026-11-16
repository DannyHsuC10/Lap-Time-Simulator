"""Evaluate a fitted superellipse using only the Python standard library."""

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Protocol
from zipfile import BadZipFile

import numpy as np

from .localization import localize
from .settings import Settings


def read_tire_json(path: str | Path) -> dict:
    path = Path(path)
    try:
        data = json.loads(path.read_text(encoding="utf-8-sig"))
        if not isinstance(data, dict):
            raise ValueError(localize("JSON 須包含一個係數物件。"))
    except (OSError, ValueError) as error:
        raise ValueError(localize("無法讀取輪胎 JSON {filename}。{error}", filename=path.name, error=error)) from error
    return data


def has_fixed_ellipse_coefficients(data: dict) -> bool:
    return "mu_x" in data or "mu_y" in data


class TireEnvelope(Protocol):
    fz_range: tuple[float, float]
    camber_range: tuple[float, float]

    def peak_forces(self, fz: float, camber_deg: float = 0.0) -> tuple[float, float]: ...

    def fx_limits(self, fy: float, *, fz: float, camber_deg: float = 0.0) -> tuple[float, ...]: ...


@dataclass(frozen=True, slots=True)
class _ForceAxis:
    mu0: float
    b1: float
    b2: float
    c2: float
    c4: float

    def camber_factor(self, camber_sq):
        return 1.0 + camber_sq * (self.c2 + self.c4 * camber_sq)

    def peak_force(self, fz, load_offset, camber_sq):
        mu = self.mu0 * (1.0 + load_offset * (self.b1 + self.b2 * load_offset))
        return fz * mu * self.camber_factor(camber_sq)


class SimplifiedTireModel:
    """Load coefficients once and query signed force limits in N and degrees."""

    def __init__(self, json_path: str | Path, *, exponent: float | None = None,
                 utilization: float = 1.0, data: dict | None = None):
        path = Path(json_path)
        self.utilization = utilization
        if data is None:
            data = read_tire_json(path)
        try:
            if data["schema_version"] != 1:
                raise ValueError("unsupported schema version")
            self.fz0 = float(data["fz0_n"])
            self.n = float(data["n"]) if exponent is None else exponent
            self.fz_range = tuple(map(float, data["fz_range_n"]))
            self.camber_range = tuple(map(float, data["camber_range_deg"]))
            if not (len(self.fz_range) == len(self.camber_range) == 2
                    and 0 < self.fz_range[0] < self.fz_range[1]
                    and self.camber_range[0] < self.camber_range[1]
                    and self.fz0 > 0 and self.n >= 1
                    and all(map(math.isfinite, (self.fz0, self.n, *self.fz_range, *self.camber_range)))):
                raise ValueError("invalid Fz, camber, or exponent")
            self._fx = self._read_axis(data["fx"])
            self._fy = self._read_axis(data["fy"])
            self._inverse_n = 1.0 / self.n
        except (AttributeError, KeyError, TypeError, ValueError, OverflowError) as error:
            raise ValueError(f"Invalid simplified tire parameters in {path}: {error}") from error

    @staticmethod
    def _read_axis(data):
        coefficients = {key: float(value) for key, value in data.items()}
        axis = _ForceAxis(**coefficients)
        if axis.mu0 <= 0 or not all(map(math.isfinite, coefficients.values())):
            raise ValueError("axis coefficients must be finite and mu0 must be positive")
        return axis

    def peak_forces(self, fz: float, camber_deg: float = 0.0) -> tuple[float, float]:
        """Return (Fx_max, Fy_max) as positive magnitudes for this condition."""
        if not self.fz_range[0] <= fz <= self.fz_range[1]:
            raise ValueError(f"Fz must be within {self.fz_range} N")
        if not self.camber_range[0] <= camber_deg <= self.camber_range[1]:
            raise ValueError(f"Camber must be within {self.camber_range} deg")
        load_offset = (fz - self.fz0) / self.fz0
        camber_sq = camber_deg * camber_deg
        fx = self._fx.peak_force(fz, load_offset, camber_sq)
        fy = self._fy.peak_force(fz, load_offset, camber_sq)
        if not (fx > 0 and fy > 0 and math.isfinite(fx) and math.isfinite(fy)):
            raise ValueError("Fitted force limits must be finite and positive")
        return self.utilization * fx, self.utilization * fy

    def fx_limits(self, fy: float, *, fz: float, camber_deg: float = 0.0) -> tuple[float, ...]:
        """Return the signed Fx limits from braking to driving for a given Fy."""
        fx_max, fy_max = self.peak_forces(fz, camber_deg)
        return self._other_limits(fy, fy_max, fx_max)

    def fy_limits(self, fx: float, *, fz: float, camber_deg: float = 0.0) -> tuple[float, ...]:
        """Return the signed Fy limits in ascending order for a given Fx."""
        fx_max, fy_max = self.peak_forces(fz, camber_deg)
        return self._other_limits(fx, fx_max, fy_max)

    def _other_limits(self, force, given_max, other_max):
        if not math.isfinite(force):
            raise ValueError("The requested force must be finite")
        magnitude = abs(force)
        if magnitude > given_max:
            return ()
        if magnitude == given_max:
            return (0.0,)
        remaining = 1.0 - (magnitude / given_max) ** self.n
        limit = other_max * remaining ** self._inverse_n
        return -limit, limit


@dataclass(frozen=True)
class FrictionEllipse:
    mu_x: float
    mu_y: float
    utilization: float = 1.0
    fz_range = (0.0, math.inf)
    camber_range = (-math.inf, math.inf)

    @classmethod
    def from_coefficients(cls, data: dict, *, utilization: float = 1.0):
        try:
            mu_x, mu_y = (data.get(name) for name in ("mu_x", "mu_y"))
            if all(type(value) in (int, float) and math.isfinite(value) and value > 0
                   for value in (mu_x, mu_y)):
                return cls(mu_x, mu_y, utilization=utilization)
        except OverflowError:
            pass
        raise ValueError(localize("摩擦橢圓 JSON 的 mu_x 與 mu_y 均須為有限的正數。"))

    def peak_forces(self, fz: float, camber_deg: float = 0.0) -> tuple[float, float]:
        return self.utilization * (self.mu_x * fz), self.utilization * (self.mu_y * fz)

    def fx_limits(self, fy: float, *, fz: float, camber_deg: float = 0.0) -> tuple[float, ...]:
        fx_max, fy_max = self.peak_forces(fz)
        if abs(fy) > fy_max:
            return ()
        fx = fx_max * math.sqrt(max(0.0, 1.0 - (fy / fy_max) ** 2)) if fy_max > 0 else 0.0
        return -fx, fx


class TabulatedTire:
    """Interpolate a signed Fx/Fy envelope derived from the repository Pacejka map."""

    camber_range = (0.0, 0.0)
    diagnostic_names = ("slip_angle", "slip_ratio", "overturning_moment", "aligning_moment",
                        "rolling_torque", "fx_fit_error", "fy_fit_error")

    def __init__(self, path: Path, *, utilization: float = 1.0):
        self.utilization = utilization
        try:
            with np.load(path, allow_pickle=False) as data:
                self.fz, self.fy_fraction, self.fy_min, self.fy_max, self.fy_pure, self.brake, self.drive, self.rrt = (
                    np.asarray(data[key], dtype=float)
                    for key in ("fz", "fy_fraction", "fy_min", "fy_max", "fy_pure", "brake", "drive", "rrt")
                )
                self.slip_grids = None
                if "grid_FX" in data:
                    self.slip_angles = np.asarray(data["slip_angle"], dtype=float)
                    self.slip_ratios = np.asarray(data["slip_ratio"], dtype=float)
                    self.slip_grids = tuple(
                        np.asarray(data[f"grid_{key}"], dtype=np.float32)
                        for key in ("FX", "FY", "MX", "MZ", "RRT")
                    )
            axes = (self.fz, self.fy_fraction)
            if any(axis.ndim != 1 or len(axis) < 2 or not np.isfinite(axis).all()
                   or np.any(np.diff(axis) <= 0) for axis in axes):
                raise ValueError(localize("Fz 或 Fy 取樣軸無效"))
            if self.fz[0] <= 0 or self.fy_fraction[0] != 0 or self.fy_fraction[-1] != 1:
                raise ValueError(localize("Fz 必須為正，Fy 比例範圍須為 0–1"))
            columns = (self.fy_min, self.fy_max, self.fy_pure, self.rrt)
            if any(column.shape != self.fz.shape or not np.isfinite(column).all() for column in columns):
                raise ValueError(localize("逐載重資料欄位無效"))
            shape = (len(self.fz), len(self.fy_fraction))
            if any(table.shape != shape or not np.isfinite(table).all() for table in (self.brake, self.drive)):
                raise ValueError(localize("Fx 資料表尺寸或數值無效"))
            if np.any(self.fy_min >= 0) or np.any(self.fy_max <= 0) or np.any(self.fy_pure <= 0):
                raise ValueError(localize("Fy 極限無效"))
            if np.any(self.brake > self.drive) or np.any(self.rrt < 0):
                raise ValueError(localize("Fx 邊界或滾阻扭矩無效"))
            if self.slip_grids is not None:
                slip_axes = (self.slip_angles, self.slip_ratios)
                if any(axis.ndim != 1 or len(axis) < 2 or not np.isfinite(axis).all()
                       or np.any(np.diff(axis) <= 0) for axis in slip_axes):
                    raise ValueError(localize("滑移資料取樣軸無效"))
                shape = (len(self.fz), len(self.slip_angles), len(self.slip_ratios))
                if any(grid.shape != shape or not np.isfinite(grid).all() for grid in self.slip_grids):
                    raise ValueError(localize("滑移力或力矩資料無效"))
        except (KeyError, ValueError, TypeError, OSError, EOFError, BadZipFile) as error:
            raise ValueError(localize("無法讀取輪胎力資料表 {filename}。{error}", filename=path.name, error=error)) from error
        self.fz_range = (float(self.fz[0]), float(self.fz[-1]))

    def peak_forces(self, fz: float, camber_deg: float = 0.0) -> tuple[float, float]:
        brake, drive = self.fx_limits(0.0, fz=fz)
        return max(-brake, drive), self.utilization * float(np.interp(fz, self.fz, self.fy_pure))

    def fx_limits(self, fy: float, *, fz: float, camber_deg: float = 0.0) -> tuple[float, ...]:
        lower = self.utilization * float(np.interp(fz, self.fz, self.fy_min))
        upper = self.utilization * float(np.interp(fz, self.fz, self.fy_max))
        if not lower <= fy <= upper:
            return ()
        if lower == upper:
            return (0.0, 0.0)
        fraction = (fy - lower) / (upper - lower)
        hi = min(max(int(np.searchsorted(self.fz, fz)), 1), len(self.fz) - 1)
        lo = hi - 1
        weight = min(max(float((fz - self.fz[lo]) / (self.fz[hi] - self.fz[lo])), 0.0), 1.0)
        return tuple(self.utilization * float(
            (1 - weight) * np.interp(fraction, self.fy_fraction, table[lo])
            + weight * np.interp(fraction, self.fy_fraction, table[hi])
        ) for table in (self.brake, self.drive))

    def rolling_torque(self, loads: np.ndarray) -> np.ndarray:
        return np.interp(loads, self.fz, self.rrt)

    def estimate_slip(self, loads, fx, fy, *, check_deadline=None):
        """Find the nearest force pair at the interpolated load for each wheel."""
        diagnostics = np.zeros((len(self.diagnostic_names), *loads.shape))
        if self.slip_grids is None:
            return diagnostics
        output = diagnostics.reshape(len(self.diagnostic_names), -1)
        active = np.flatnonzero(loads.ravel() > 0)
        load_axis = np.clip(loads.ravel(), *self.fz_range)
        fx_grid, fy_grid, mx, mz, rrt = (
            grid.reshape(len(self.fz), -1) for grid in self.slip_grids
        )
        for indices in np.array_split(active, max(1, (len(active) + 63) // 64)):
            if check_deadline is not None:
                check_deadline()
            if not len(indices):
                continue
            fz = load_axis[indices]
            hi = np.searchsorted(self.fz, fz).clip(1, len(self.fz) - 1)
            lo = hi - 1
            weight = (fz - self.fz[lo]) / (self.fz[hi] - self.fz[lo])
            # Mirror left/right sides using the same conventions as Steady_lapsim.
            side = 1 - 2 * (indices % 2)
            target_fx = fx.ravel()[indices]
            target_fy = fy.ravel()[indices] * side
            surface_fx = self.utilization * (fx_grid[lo] * (1 - weight[:, None]) + fx_grid[hi] * weight[:, None])
            surface_fy = self.utilization * (fy_grid[lo] * (1 - weight[:, None]) + fy_grid[hi] * weight[:, None])
            error = ((surface_fx - target_fx[:, None]) / np.maximum(np.ptp(surface_fx, axis=1)[:, None], 1))**2
            error += ((surface_fy - target_fy[:, None]) / np.maximum(np.ptp(surface_fy, axis=1)[:, None], 1))**2
            best = np.argmin(error, axis=1)
            row = np.arange(len(indices))
            output[0, indices] = -side * self.slip_angles[best // len(self.slip_ratios)]
            output[1, indices] = self.slip_ratios[best % len(self.slip_ratios)]
            for col, grid, sign in ((2, mx, -side), (3, mz, side), (4, rrt, 1)):
                output[col, indices] = sign * (grid[lo, best] * (1 - weight) + grid[hi, best] * weight)
            output[5, indices] = surface_fx[row, best] - target_fx
            output[6, indices] = side * (surface_fy[row, best] - target_fy)
        return diagnostics


def build_tire_model(settings: Settings) -> TireEnvelope:
    utilization = settings.tire_utilization
    if settings.tire_model == "table":
        return TabulatedTire(settings.tire_table_path, utilization=utilization)
    if settings.tire_model == "ellipse":
        data = read_tire_json(settings.ellipse_path)
        if has_fixed_ellipse_coefficients(data):
            return FrictionEllipse.from_coefficients(data, utilization=utilization)
        return SimplifiedTireModel(settings.ellipse_path, exponent=2.0, utilization=utilization, data=data)
    return SimplifiedTireModel(settings.tire_path, utilization=utilization)
