"""
Example: read tire_force_map.pkl and query the Fx limit at a given Fy.

This script builds an interpolated Fz slice first, so the requested Fz does not
need to lie exactly on the saved table axis.
"""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import ConvexHull

try:
    from Pacejka_tire_model.tire_mapping_tool import TireForceMap
except ImportError:
    from tire_mapping_tool import TireForceMap


BASE_DIR = Path(__file__).resolve().parent
DEFAULT_MAP_PATH = BASE_DIR / "tire_force_map.pkl"


def get_fz_slice(
    map_path: str | Path = DEFAULT_MAP_PATH,
    fz: float = 800.0,
) -> dict[str, np.ndarray]:
    """Interpolate the tire map to one normal load and return 2D grids."""

    tire_map = TireForceMap.load(map_path)
    alpha_grid, kappa_grid = np.meshgrid(
        tire_map.alpha_axis_deg,
        tire_map.kappa_axis,
        indexing="ij",
    )

    result = tire_map.evaluate(
        fz=np.full_like(alpha_grid, fz, dtype=float),
        alpha_deg=alpha_grid,
        kappa=kappa_grid,
        outputs=("FX", "FY", "MZ", "mu_total"),
    )

    return {
        "FZ": np.full_like(alpha_grid, fz, dtype=float),
        "SA_deg": alpha_grid,
        "SL": kappa_grid,
        "FX": result["FX"],
        "FY": result["FY"],
        "MZ": result["MZ"],
        "mu_total": result["mu_total"],
    }


def get_fx_fy_boundary(fz_slice: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
    """Return the convex hull boundary of the Fx/Fy point cloud."""

    points = np.column_stack(
        (
            fz_slice["FX"].ravel(),
            fz_slice["FY"].ravel(),
        )
    )
    hull = ConvexHull(points)
    boundary = points[np.append(hull.vertices, hull.vertices[0])]

    return {
        "FX": boundary[:, 0],
        "FY": boundary[:, 1],
    }


def solve_fx_limits_at_fy(
    current_fy: float,
    fz: float = 800.0,
    map_path: str | Path = DEFAULT_MAP_PATH,
) -> dict[str, dict[str, float]]:
    """
    Given current Fy, solve braking and acceleration Fx limits.

    Returns:
        {
            "brake": {"FX": min Fx, "FY": current_fy, "SA_deg": ..., "SL": ...},
            "drive": {"FX": max Fx, "FY": current_fy, "SA_deg": ..., "SL": ...},
        }
    """

    fz_slice = get_fz_slice(map_path=map_path, fz=fz)
    candidates = _iso_fy_candidates(fz_slice, current_fy)

    if not candidates:
        fy_min = float(np.min(fz_slice["FY"]))
        fy_max = float(np.max(fz_slice["FY"]))
        raise ValueError(
            f"current_fy={current_fy:.3f} N is outside this Fz slice "
            f"range [{fy_min:.3f}, {fy_max:.3f}] N."
        )

    brake = min(candidates, key=lambda item: item["FX"])
    drive = max(candidates, key=lambda item: item["FX"])

    return {
        "brake": brake,
        "drive": drive,
    }


def _iso_fy_candidates(
    fz_slice: dict[str, np.ndarray],
    target_fy: float,
) -> list[dict[str, float]]:
    """Find linearly interpolated points where the grid crosses target_fy."""

    candidates: list[dict[str, float]] = []
    shape = fz_slice["FY"].shape

    # Scan alpha-direction and kappa-direction grid edges.
    for i in range(shape[0]):
        for j in range(shape[1] - 1):
            _append_edge_crossing(fz_slice, target_fy, (i, j), (i, j + 1), candidates)

    for i in range(shape[0] - 1):
        for j in range(shape[1]):
            _append_edge_crossing(fz_slice, target_fy, (i, j), (i + 1, j), candidates)

    return candidates


def _append_edge_crossing(
    fz_slice: dict[str, np.ndarray],
    target_fy: float,
    index_a: tuple[int, int],
    index_b: tuple[int, int],
    candidates: list[dict[str, float]],
) -> None:
    fy_a = float(fz_slice["FY"][index_a])
    fy_b = float(fz_slice["FY"][index_b])

    if fy_a == fy_b:
        return
    if not (min(fy_a, fy_b) <= target_fy <= max(fy_a, fy_b)):
        return

    ratio = (target_fy - fy_a) / (fy_b - fy_a)
    candidate = {"FY": float(target_fy)}

    for key in ("FX", "SA_deg", "SL", "MZ", "mu_total"):
        value_a = float(fz_slice[key][index_a])
        value_b = float(fz_slice[key][index_b])
        candidate[key] = value_a + ratio * (value_b - value_a)

    candidates.append(candidate)


def plot_friction_circle_at_fz(
    fz: float = 800.0,
    current_fy: float | None = None,
    map_path: str | Path = DEFAULT_MAP_PATH,
) -> None:
    """Plot the friction circle at the current normal load."""

    fz_slice = get_fz_slice(map_path=map_path, fz=fz)
    boundary = get_fx_fy_boundary(fz_slice)

    fig, ax = plt.subplots(figsize=(8, 7))
    plt.scatter(
        fz_slice["FX"],
        fz_slice["FY"],
        s=5,
        color="gray",
        alpha=0.18,
        label="map points",
    )

    alpha_count, kappa_count = fz_slice["FX"].shape
    alpha_step = max(1, alpha_count // 8)
    kappa_step = max(1, kappa_count // 8)

    for alpha_index in range(0, alpha_count, alpha_step):
        ax.plot(
            fz_slice["FX"][alpha_index, :],
            fz_slice["FY"][alpha_index, :],
            color="tab:blue",
            alpha=0.35,
            linewidth=0.8,
        )

    for kappa_index in range(0, kappa_count, kappa_step):
        ax.plot(
            fz_slice["FX"][:, kappa_index],
            fz_slice["FY"][:, kappa_index],
            color="tab:green",
            alpha=0.35,
            linewidth=0.8,
        )

    ax.plot(
        [],
        [],
        color="tab:blue",
        alpha=0.6,
        linewidth=1.0,
        label="constant alpha lines",
    )
    ax.plot(
        [],
        [],
        color="tab:green",
        alpha=0.6,
        linewidth=1.0,
        label="constant slip ratio lines",
    )
    ax.plot(
        boundary["FX"],
        boundary["FY"],
        "r-",
        linewidth=2.2,
        label="limit boundary",
    )

    if current_fy is not None:
        limits = solve_fx_limits_at_fy(
            current_fy=current_fy,
            fz=fz,
            map_path=map_path,
        )
        ax.axhline(current_fy, color="black", linestyle="--", linewidth=1)

        for name, point in limits.items():
            ax.scatter(point["FX"], point["FY"], s=75, label=f"{name} limit")
            ax.annotate(
                f"{name}\nFx={point['FX']:.1f} N\nSL={point['SL']:.3f}\nSA={point['SA_deg']:.2f} deg",
                xy=(point["FX"], point["FY"]),
                xytext=(8, 8),
                textcoords="offset points",
            )

    ax.set_xlabel("Fx [N]")
    ax.set_ylabel("Fy [N]")
    ax.set_title(f"Friction circle at Fz={fz:.0f} N")
    ax.axis("equal")
    ax.grid(True, linestyle=":")
    ax.legend()
    fig.tight_layout()
    plt.show()


def plot_boundary_example(
    fz: float = 800.0,
    current_fy: float = 500.0,
    map_path: str | Path = DEFAULT_MAP_PATH,
) -> None:
    """Backward-compatible wrapper for the friction circle example."""

    plot_friction_circle_at_fz(fz=fz, current_fy=current_fy, map_path=map_path)


if __name__ == "__main__":
    FZ_NOW = 800.0
    FY_NOW = 500.0

    fz_slice_800 = get_fz_slice(fz=FZ_NOW)
    boundary_800 = get_fx_fy_boundary(fz_slice_800)
    limits = solve_fx_limits_at_fy(current_fy=FY_NOW, fz=FZ_NOW)

    print(f"Fz={FZ_NOW:.1f} N boundary points: {len(boundary_800['FX'])}")
    print(f"Current Fy={FY_NOW:.1f} N")
    print("Brake limit:", limits["brake"])
    print("Drive limit:", limits["drive"])

    plot_friction_circle_at_fz(fz=FZ_NOW, current_fy=FY_NOW)
