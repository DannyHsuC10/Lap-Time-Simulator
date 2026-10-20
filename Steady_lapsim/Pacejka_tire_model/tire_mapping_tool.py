"""
Pacejka tire force mapping and lookup utilities.

The generated pickle stores a regular 3D table over:
    Fz [N] x slip angle alpha [deg] x slip ratio kappa [-]

Outputs follow the single-tire Pacejka solver sign convention directly.
Use ``TireForceMap.evaluate(...)`` for interpolated lookup.
"""

from __future__ import annotations

import argparse
import pickle
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np
from scipy.interpolate import RegularGridInterpolator
from scipy.spatial import ConvexHull

try:
    from Pacejka_tire_model.model import api as tire_model
except ImportError:
    from model import api as tire_model


PACKAGE_DIR = Path(__file__).resolve().parent
DEFAULT_OUTPUT = PACKAGE_DIR / "tire_force_map.pkl"
DEFAULT_OUTPUTS = ("FX", "FY", "MX", "MZ", "RRT")


@dataclass(frozen=True)
class TireMapConfig:
    """Configuration used to generate a tire lookup table."""

    tir_file: str = "tire.tir"
    alpha_min_deg: float = -12.0
    alpha_max_deg: float = 12.0
    alpha_points: int = 121
    kappa_min: float = -1.0
    kappa_max: float = 1.0
    kappa_points: int = 121
    fz_min: float | None = None
    fz_max: float | None = None
    fz_points: int = 121
    inclination_angle_deg: float = 0.0
    pressure: float | None = None
    velocity: float | None = None
    rolling_radius: float | None = None


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


def build_tire_map(config: TireMapConfig = TireMapConfig()) -> dict[str, Any]:
    """Build a vectorized 3D table from the Pacejka model."""

    tir_params = tire_model.load_tir(config.tir_file)
    fz_min = _pick_fz_bound(config.fz_min, tir_params, "FZMIN", 0.0)
    fz_max = _pick_fz_bound(config.fz_max, tir_params, "FZMAX", 2000.0)

    axes = {
        "FZ": np.linspace(fz_min, fz_max, config.fz_points),
        "SA_deg": np.linspace(
            config.alpha_min_deg,
            config.alpha_max_deg,
            config.alpha_points,
        ),
        "SL": np.linspace(config.kappa_min, config.kappa_max, config.kappa_points),
    }

    fz_grid, alpha_grid, kappa_grid = np.meshgrid(
        axes["FZ"],
        axes["SA_deg"],
        axes["SL"],
        indexing="ij",
    )

    calc_input = tire_model.CalculationInput(
        FZ=fz_grid,
        SA=alpha_grid,
        SL=kappa_grid,
        IA=np.full_like(fz_grid, config.inclination_angle_deg, dtype=float),
        pressure=(
            np.full_like(fz_grid, config.pressure, dtype=float)
            if config.pressure is not None
            else None
        ),
        V=(
            np.full_like(fz_grid, config.velocity, dtype=float)
            if config.velocity is not None
            else None
        ),
        Ro=(
            np.full_like(fz_grid, config.rolling_radius, dtype=float)
            if config.rolling_radius is not None
            else None
        ),
        tir_params=tir_params,
    )

    tables = {
        output: np.asarray(tire_model.MF_Universal_solver(output, calc_input), dtype=float)
        for output in DEFAULT_OUTPUTS
    }
    tables["FZ"] = fz_grid
    tables["force_magnitude"] = np.hypot(tables["FX"], tables["FY"])
    tables["mu_x"] = np.divide(
        tables["FX"],
        fz_grid,
        out=np.zeros_like(tables["FX"]),
        where=fz_grid != 0.0,
    )
    tables["mu_y"] = np.divide(
        tables["FY"],
        fz_grid,
        out=np.zeros_like(tables["FY"]),
        where=fz_grid != 0.0,
    )
    tables["mu_total"] = np.divide(
        tables["force_magnitude"],
        fz_grid,
        out=np.zeros_like(tables["force_magnitude"]),
        where=fz_grid != 0.0,
    )

    payload = {
        "version": 1,
        "description": "Single-tire Pacejka force map over FZ, SA_deg, and SL.",
        "config": asdict(config),
        "tir_file": config.tir_file,
        "axes": axes,
        "axis_order": ("FZ", "SA_deg", "SL"),
        "tables": tables,
        "metadata": _metadata(config, tir_params),
        "limit_envelopes": _limit_envelopes(axes, tables),
    }
    return payload


def save_tire_map(
    path: str | Path = DEFAULT_OUTPUT,
    config: TireMapConfig = TireMapConfig(),
) -> dict[str, Any]:
    """Build and save a tire map pickle."""

    payload = build_tire_map(config)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as file:
        pickle.dump(payload, file, protocol=pickle.HIGHEST_PROTOCOL)
    return payload


def _limit_envelopes(
    axes: dict[str, np.ndarray],
    tables: dict[str, np.ndarray],
) -> dict[str, Any]:
    envelopes: dict[str, Any] = {}
    for fz_index, fz in enumerate(axes["FZ"]):
        points = np.column_stack(
            (
                tables["FX"][fz_index].ravel(),
                tables["FY"][fz_index].ravel(),
            )
        )
        unique_points = np.unique(points, axis=0)
        if len(unique_points) < 3:
            continue
        hull = ConvexHull(unique_points)
        hull_points = unique_points[hull.vertices]
        envelopes[float(fz)] = {
            "Fx": hull_points[:, 0],
            "Fy": hull_points[:, 1],
            "max_Fx": float(np.max(points[:, 0])),
            "min_Fx": float(np.min(points[:, 0])),
            "max_Fy": float(np.max(points[:, 1])),
            "min_Fy": float(np.min(points[:, 1])),
            "max_force_magnitude": float(
                np.max(tables["force_magnitude"][fz_index])
            ),
        }
    return envelopes


def _metadata(config: TireMapConfig, tir_params: dict[str, Any]) -> dict[str, Any]:
    return {
        "units": {
            "FZ": "N",
            "SA_deg": "deg",
            "SL": "ratio",
            "FX": "N",
            "FY": "N",
            "MX": "N*m",
            "MZ": "N*m",
            "RRT": "N*m",
        },
        "input_convention": {
            "SA_deg": "Pacejka solver input is slip angle in degrees.",
            "SL": "Longitudinal slip ratio. Transient_tire_port clips this to [-1, 1].",
        },
        "defaults_used": {
            "pressure": config.pressure if config.pressure is not None else tir_params["NOMPRES"],
            "velocity": config.velocity if config.velocity is not None else tir_params["LONGVL"],
            "rolling_radius": (
                config.rolling_radius
                if config.rolling_radius is not None
                else tir_params["UNLOADED_RADIUS"]
            ),
        },
        "tir_bounds": {
            "FZMIN": tir_params.get("FZMIN"),
            "FZMAX": tir_params.get("FZMAX"),
            "FNOMIN": tir_params.get("FNOMIN"),
        },
    }


def _pick_fz_bound(
    requested: float | None,
    tir_params: dict[str, Any],
    key: str,
    fallback: float,
) -> float:
    if requested is not None:
        return float(requested)
    return float(tir_params.get(key, fallback))


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build a pickle lookup table for the Pacejka tire model."
    )
    parser.add_argument("--tir-file", default="tire.tir")
    parser.add_argument("--output", default=str(DEFAULT_OUTPUT))
    parser.add_argument("--alpha-min", type=float, default=-12.0)
    parser.add_argument("--alpha-max", type=float, default=12.0)
    parser.add_argument("--alpha-points", type=int, default=49)
    parser.add_argument("--kappa-min", type=float, default=-1.0)
    parser.add_argument("--kappa-max", type=float, default=1.0)
    parser.add_argument("--kappa-points", type=int, default=81)
    parser.add_argument("--fz-min", type=float, default=None)
    parser.add_argument("--fz-max", type=float, default=None)
    parser.add_argument("--fz-points", type=int, default=49)
    parser.add_argument("--ia", type=float, default=0.0)
    parser.add_argument("--pressure", type=float, default=None)
    parser.add_argument("--velocity", type=float, default=None)
    parser.add_argument("--rolling-radius", type=float, default=None)
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    config = TireMapConfig(
        tir_file=args.tir_file,
        alpha_min_deg=args.alpha_min,
        alpha_max_deg=args.alpha_max,
        alpha_points=args.alpha_points,
        kappa_min=args.kappa_min,
        kappa_max=args.kappa_max,
        kappa_points=args.kappa_points,
        fz_min=args.fz_min,
        fz_max=args.fz_max,
        fz_points=args.fz_points,
        inclination_angle_deg=args.ia,
        pressure=args.pressure,
        velocity=args.velocity,
        rolling_radius=args.rolling_radius,
    )
    payload = save_tire_map(args.output, config)
    axes = payload["axes"]
    print(f"Saved tire map: {Path(args.output).resolve()}")
    print(
        "Grid shape: "
        f"FZ={len(axes['FZ'])}, "
        f"SA={len(axes['SA_deg'])}, "
        f"SL={len(axes['SL'])}"
    )


if __name__ == "__main__":
    main()
