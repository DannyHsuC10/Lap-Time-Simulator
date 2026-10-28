"""Extract suspension, attitude, steering, acceleration, speed and time to CSV.

Run with no arguments to read data.pkl next to this script. Examples:
    python extract_lap_signals.py
    python extract_lap_signals.py --input other.pkl --output other_signals.csv

Angles are exported in degrees, displacement in mm, and acceleration in m/s^2.
Speed is the horizontal velocity magnitude, exported in m/s and km/h.
Wheel order is FL, FR, RL, RR. Sprung displacement signs are preserved.

Recorded zu is preferred. If missing, reconstruct it on flat road as
loaded_radius - static_loaded_radius (upward positive). The reference can come
from --static-loaded-radius FL FR RL RR (meters), PKL metadata, or current car.py.
The car.py fallback checks recorded tire radii and mass against current settings;
its geometry and static weight distribution must also match the original run.
The JSON next to the CSV records the reference, units and reconstruction source.
"""

import argparse
import csv
import json
import pickle
from pathlib import Path
import sys

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
WHEELS = ("FL", "FR", "RL", "RR")


def load_recording(path: Path) -> tuple[dict, dict]:
    with path.open("rb") as stream:
        package = pickle.load(stream)
    if not isinstance(package, dict) or not isinstance(package.get("data"), dict):
        raise ValueError("Expected a SimulationLogger PKL with a 'data' dictionary.")
    metadata = package.get("metadata", {})
    if not isinstance(metadata, dict):
        raise ValueError("PKL metadata must be a dictionary.")
    return package["data"], metadata


def numeric_field(data: dict, name: str) -> np.ndarray:
    if name not in data:
        raise ValueError(f"Missing PKL field: {name}")
    values = np.asarray(data[name], dtype=float)
    if not np.all(np.isfinite(values)):
        raise ValueError(f"PKL field '{name}' contains non-finite values.")
    return values


def matrix_field(data: dict, name: str, samples: int, width: int) -> np.ndarray:
    values = numeric_field(data, name)
    if values.shape != (samples, width):
        raise ValueError(
            f"PKL field '{name}' must have shape ({samples}, {width}); got {values.shape}."
        )
    return values


def angles_in_degrees(values: np.ndarray, unit: str, name: str) -> np.ndarray:
    if unit == "deg":
        return values
    if unit == "rad":
        return np.degrees(values)
    raise ValueError(f"Unknown {name} unit {unit!r}; specify 'rad' or 'deg'.")


def static_reference_from_car(
    data: dict, metadata: dict, radii: np.ndarray
) -> tuple[np.ndarray, dict]:
    """Recover an older recording using explicitly reported current parameters."""
    project_dir = SCRIPT_DIR.parent
    if str(project_dir) not in sys.path:
        sys.path.insert(0, str(project_dir))
    from car import Car_simplify, g

    car = Car_simplify()
    recorded_mass = metadata.get("vehicle_mass_kg")
    if recorded_mass is not None and not np.isclose(float(recorded_mass), car.m):
        raise ValueError(
            "Recorded mass differs from car.py; provide the original four static "
            "loaded radii with --static-loaded-radius."
        )
    loads = matrix_field(data, "Fz", len(radii), 4)
    expected_radii = np.maximum(car.R_tire - loads / car.K_tire, 0.0)
    if not np.allclose(radii, expected_radii, rtol=0.0, atol=1e-9):
        raise ValueError(
            "Recorded tire radii do not match car.py; provide the original four "
            "static loaded radii with --static-loaded-radius."
        )
    front_ratio = car.lr / car.l
    static_loads = car.m * g * np.array(
        [front_ratio, front_ratio, 1.0 - front_ratio, 1.0 - front_ratio]
    ) / 2.0
    reference = np.maximum(car.R_tire - static_loads / car.K_tire, 0.0)
    return reference, {
        "reference_source": "current car.py (assumes original static weight distribution)",
        "vehicle_mass_kg": float(car.m),
        "gravity_m_s2": float(g),
        "front_weight_ratio": float(front_ratio),
        "tire_stiffness_N_m": float(car.K_tire),
        "free_tire_radius_m": float(car.R_tire),
    }


def unsprung_displacement(
    data: dict,
    metadata: dict,
    samples: int,
    static_loaded_radius: list[float] | None,
) -> tuple[np.ndarray, dict]:
    if "zu" in data:
        values = matrix_field(data, "zu", samples, 4)
        unit = metadata.get("zu_unit", "m")
        if unit not in ("m", "mm"):
            raise ValueError(f"Unknown zu unit: {unit!r}")
        return values * (1000.0 if unit == "m" else 1.0), {
            "source": "recorded zu",
            "reference": metadata.get("unsprung_reference", "as recorded"),
            "positive_direction": metadata.get("unsprung_positive_direction", "as recorded"),
            "static_loaded_radius_m": metadata.get("static_loaded_radius_m"),
        }

    radii = matrix_field(data, "loaded_radius", samples, 4)
    if static_loaded_radius is not None:
        reference = np.asarray(static_loaded_radius, dtype=float)
        details = {"reference_source": "--static-loaded-radius"}
    elif "static_loaded_radius_m" in metadata:
        reference = np.asarray(metadata["static_loaded_radius_m"], dtype=float)
        details = {"reference_source": "PKL metadata.static_loaded_radius_m"}
    else:
        reference, details = static_reference_from_car(data, metadata, radii)
    if reference.shape != (4,) or not np.all(np.isfinite(reference)) or np.any(reference < 0):
        raise ValueError("Static loaded radii must be four finite, nonnegative values in meters.")
    return (radii - reference) * 1000.0, {
        "source": "reconstructed from loaded_radius",
        "reference": "Static equilibrium at zero speed and acceleration on flat road.",
        "positive_direction": "up",
        "static_loaded_radius_m": reference.tolist(),
        **details,
    }


def extract_signals(
    data: dict,
    metadata: dict,
    *,
    theta_unit: str | None = None,
    steer_unit: str | None = None,
    static_loaded_radius: list[float] | None = None,
) -> tuple[dict[str, np.ndarray], dict]:
    time = numeric_field(data, "t")
    if time.ndim != 1 or not len(time):
        raise ValueError("PKL field 't' must be a nonempty one-dimensional series.")
    samples = len(time)
    sprung = matrix_field(data, "zs", samples, 4)
    theta = matrix_field(data, "theta", samples, 4)
    position = matrix_field(data, "s", samples, 3)
    steering = matrix_field(data, "steer", samples, 4)
    acceleration = matrix_field(data, "a", samples, 3)
    velocity = matrix_field(data, "v", samples, 3)
    theta_unit = theta_unit or metadata.get("theta_unit")
    if theta_unit is None:
        raise ValueError("Missing theta_unit metadata; specify --theta-unit rad or deg.")
    steer_unit = steer_unit or metadata.get("steer_unit", "rad")
    theta = angles_in_degrees(theta, theta_unit, "theta")
    steering = angles_in_degrees(steering, steer_unit, "steer")
    unsprung, unsprung_info = unsprung_displacement(data, metadata, samples, static_loaded_radius)
    speed = np.hypot(velocity[:, 0], velocity[:, 1])

    columns = {"time_s": time}
    for index, wheel in enumerate(WHEELS):
        columns[f"zs_{wheel}_mm"] = sprung[:, index] * 1000.0
    for index, wheel in enumerate(WHEELS):
        columns[f"zu_{wheel}_mm"] = unsprung[:, index]
    columns.update(
        roll_front_deg=theta[:, 0],
        roll_rear_deg=theta[:, 1],
        pitch_deg=theta[:, 2],
        heave_mm=position[:, 2] * 1000.0,
    )
    for index, wheel in enumerate(WHEELS):
        columns[f"steer_{wheel}_deg"] = steering[:, index]
    for index, axis in enumerate(("x", "y", "z")):
        columns[f"a{axis}_m_s2"] = acceleration[:, index]
    columns.update(speed_m_s=speed, speed_km_h=speed * 3.6)
    return columns, {
        "samples": samples,
        "wheel_order": list(WHEELS),
        "input_angle_units": {"theta": theta_unit, "steer": steer_unit},
        "output_units": {
            "time": "s", "zs_zu_heave": "mm", "roll_pitch_steer": "deg",
            "acceleration": "m/s^2", "speed": ["m/s", "km/h"],
        },
        "definitions": {
            "zs": "Recorded sprung corner displacement; sign and reference preserved.",
            "roll": "theta[0] = front roll; theta[1] = rear roll.",
            "pitch": "theta[2].",
            "heave": "Recorded chassis s[2]; not the mean of the four corner displacements.",
            "steering": "Recorded wheel steer angles; no additional steering geometry is inferred.",
            "acceleration": "Recorded body-frame a = [ax, ay, az]; no gravity adjustment.",
            "speed": "sqrt(vx^2 + vy^2); vertical heave velocity excluded.",
        },
        "unsprung": unsprung_info,
    }


def export_signals(
    input_path: Path, output_path: Path, **options
) -> tuple[Path, Path, dict]:
    if output_path.suffix.lower() != ".csv":
        raise ValueError("Output must have a .csv extension.")
    if input_path.resolve() == output_path.resolve():
        raise ValueError("Input and output paths must differ.")
    data, metadata = load_recording(input_path)
    columns, report = extract_signals(data, metadata, **options)
    report.update(input_pkl=str(input_path.resolve()), columns=list(columns), input_metadata=metadata)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.writer(stream)
        writer.writerow(columns)
        writer.writerows(zip(*columns.values()))
    report_path = output_path.with_suffix(".metadata.json")
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return output_path, report_path, report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--input", "--pkl", type=Path, default=SCRIPT_DIR / "data.pkl")
    parser.add_argument("--output", type=Path, help="Default: <input stem>_signals.csv next to the PKL.")
    parser.add_argument("--theta-unit", choices=("rad", "deg"), help="Override PKL theta_unit metadata.")
    parser.add_argument("--steer-unit", choices=("rad", "deg"), help="Default: metadata or rad.")
    parser.add_argument("--static-loaded-radius", type=float, nargs=4, metavar=("FL", "FR", "RL", "RR"),
                        help="Static loaded radii in meters for reconstructing missing zu.")
    args = parser.parse_args()
    output = args.output or args.input.with_name(f"{args.input.stem}_signals.csv")
    try:
        csv_path, report_path, report = export_signals(
            args.input, output, theta_unit=args.theta_unit, steer_unit=args.steer_unit,
            static_loaded_radius=args.static_loaded_radius,
        )
    except (OSError, ValueError, EOFError, pickle.UnpicklingError, ImportError) as error:
        parser.error(str(error))
    print(f"Exported {report['samples']} samples, {len(report['columns'])} columns: {csv_path.resolve()}")
    print(f"Units and source details: {report_path.resolve()}")
    print(f"zu source: {report['unsprung']['source']}")
    if report["unsprung"]["source"] != "recorded zu":
        print(f"zu reference: {report['unsprung']['reference_source']}; upward is positive.")


if __name__ == "__main__":
    main()
