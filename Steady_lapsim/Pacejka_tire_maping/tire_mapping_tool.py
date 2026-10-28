"""
Pacejka tire force mapping and lookup utilities.

The generated pickle stores a regular 3D table over:
    Fz [N] x slip angle alpha [deg] x slip ratio kappa [-]

Outputs follow the single-tire Pacejka solver sign convention directly.
Use ``TireForceMap.evaluate(...)`` for interpolated lookup.
"""

from __future__ import annotations


import pickle
from pathlib import Path
from typing import Any

import numpy as np
from scipy.interpolate import RegularGridInterpolator


PACKAGE_DIR = Path(__file__).resolve().parent
DEFAULT_OUTPUT = PACKAGE_DIR / "tire_force_map.pkl"
DEFAULT_OUTPUTS = ("FX", "FY", "MX", "MZ", "RRT")


class TireForceMap:
    """Interpolated lookup interface for a generated tire map."""

    def __init__(self, payload: dict[str, Any]):
        self.payload = payload
        axes = payload["axes"]
        self.fz_axis = np.asarray(axes["FZ"], dtype=float)
        self.alpha_axis_deg = np.asarray(axes["SA_deg"], dtype=float)
        self.kappa_axis = np.asarray(axes["SL"], dtype=float)
        self._interpolators: dict[str, RegularGridInterpolator] = {}

    @classmethod
    def load(cls, path: str | Path = DEFAULT_OUTPUT) -> "TireForceMap":
        with Path(path).open("rb") as file:
            return cls(pickle.load(file))

    def evaluate(
        self,
        fz: float | np.ndarray,
        alpha_deg: float | np.ndarray,
        kappa: float | np.ndarray,
        *,
        outputs: tuple[str, ...] = DEFAULT_OUTPUTS,
        bounds_error: bool = False,
        fill_value: float | None = None,
    ) -> dict[str, np.ndarray]:
        """Return interpolated tire outputs at one or many operating points."""

        fz_arr, alpha_arr, kappa_arr = np.broadcast_arrays(fz, alpha_deg, kappa)
        points = np.column_stack(
            (fz_arr.ravel(), alpha_arr.ravel(), kappa_arr.ravel())
        )
        shape = fz_arr.shape

        result: dict[str, np.ndarray] = {}
        for output in outputs:
            interp = self._get_interpolator(
                output,
                bounds_error=bounds_error,
                fill_value=fill_value,
            )
            result[output] = interp(points).reshape(shape)

        return result

    def _get_interpolator(
        self,
        output: str,
        *,
        bounds_error: bool,
        fill_value: float | None,
    ) -> RegularGridInterpolator:
        key = f"{output}|{bounds_error}|{fill_value}"
        if key not in self._interpolators:
            self._interpolators[key] = RegularGridInterpolator(
                (self.fz_axis, self.alpha_axis_deg, self.kappa_axis),
                self.payload["tables"][output],
                bounds_error=bounds_error,
                fill_value=fill_value,
            )
        return self._interpolators[key]

