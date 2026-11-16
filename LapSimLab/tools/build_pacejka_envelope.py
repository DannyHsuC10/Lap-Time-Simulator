"""Convert a Pacejka force map to a LapSimLab NumPy envelope.

Run this offline with SciPy installed. The desktop application reads only the
generated numeric NPZ and does not deserialize pickle files.
"""

import argparse
import pickle
from pathlib import Path

import numpy as np
from scipy.spatial import ConvexHull


AXIS_KEYS = ("FZ", "SA_deg", "SL")
TABLE_KEYS = ("FX", "FY", "MX", "MZ", "RRT")


def read_force_map(source: Path):
    if source.suffix.lower() == ".npz":
        with np.load(source, allow_pickle=False) as archive:
            axes = {key: archive[f"axis_{key}"] for key in AXIS_KEYS}
            tables = {key: archive[f"table_{key}"] for key in TABLE_KEYS}
        return axes, tables
    if source.suffix.lower() != ".pkl":
        raise ValueError(f"Expected a .npz or trusted .pkl force map: {source}")
    with source.open("rb") as stream:
        source_map = pickle.load(stream)
    return source_map["axes"], source_map["tables"]


def build_envelope(source: Path, destination: Path):
    axes, tables = read_force_map(source)
    fz = axes["FZ"]
    zero_alpha = int(np.argmin(np.abs(axes["SA_deg"])))
    zero_slip = int(np.argmin(np.abs(axes["SL"])))
    fy_pure = np.max(np.abs(tables["FY"][:, :, zero_slip]), axis=1)
    rrt = np.abs(tables["RRT"][:, zero_alpha, zero_slip])
    fraction = np.linspace(0, 1, 301)
    brake = np.empty((len(fz), len(fraction)))
    drive = np.empty_like(brake)
    fy_min = np.empty(len(fz))
    fy_max = np.empty(len(fz))

    for index, (fx, fy) in enumerate(zip(tables["FX"], tables["FY"])):
        points = np.column_stack((fx.ravel(), fy.ravel()))
        vertices = points[ConvexHull(points).vertices]
        start = vertices
        end = np.roll(vertices, -1, axis=0)
        fy_min[index], fy_max[index] = vertices[:, 1].min(), vertices[:, 1].max()
        targets = fy_min[index] + fraction * (fy_max[index] - fy_min[index])
        targets[-1] = fy_max[index]
        for column, target in enumerate(targets):
            crosses = ((start[:, 1] - target) * (end[:, 1] - target) <= 0) & (start[:, 1] != end[:, 1])
            a, b = start[crosses], end[crosses]
            crossings = a[:, 0] + (target - a[:, 1]) * (b[:, 0] - a[:, 0]) / (b[:, 1] - a[:, 1])
            brake[index, column], drive[index, column] = crossings.min(), crossings.max()

    np.savez_compressed(
        destination, fz=fz, fy_fraction=fraction, fy_min=fy_min, fy_max=fy_max,
        fy_pure=fy_pure, brake=brake, drive=drive, rrt=rrt,
        slip_angle=axes["SA_deg"], slip_ratio=axes["SL"],
        **{f"grid_{key}": np.asarray(tables[key], dtype=np.float32) for key in TABLE_KEYS},
    )


if __name__ == "__main__":
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source", "--trusted-source", type=Path,
        default=root / "Pacejka_tire_model" / "hoosier_43075_r20_w8.npz",
        help="Raw NPZ force map or a trusted legacy PKL force map.",
    )
    parser.add_argument("--output", type=Path, default=root / "input_data" / "pacejka_envelope.npz")
    args = parser.parse_args()
    build_envelope(args.source, args.output)
