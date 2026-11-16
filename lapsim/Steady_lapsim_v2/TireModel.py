"""Evaluate a fitted superellipse using only the Python standard library."""

from dataclasses import dataclass
import json
import math
from pathlib import Path


DEFAULT_JSON = Path(__file__).resolve().parents[0] / "input_data" / "hoosier_43075_r20_w8.json"


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

    def __init__(self, json_path: str | Path = DEFAULT_JSON):
        path = Path(json_path)
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
            if data["schema_version"] != 1:
                raise ValueError("unsupported schema version")
            self.fz0 = float(data["fz0_n"])
            self.n = float(data["n"])
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
        return fx, fy

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
