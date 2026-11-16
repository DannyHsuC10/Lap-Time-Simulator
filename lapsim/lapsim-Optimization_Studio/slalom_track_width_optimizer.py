from __future__ import annotations

from contextlib import redirect_stdout
from dataclasses import dataclass
from io import StringIO
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from car import Car_simplify
from lap_simulation_runner import LapSimulationConfig, run_lap_simulation
from Slalom.Slalom import slalom_path_analysis


GENERATED_TRACK_DIR = Path("data") / "generated_slalom_tracks"
DEFAULT_OFFSET_LIST = [
    0.01,
    1.0 / 6.0,
    1.0 / 3.0,
]
FIXED_FRONT_TRACK = 1.28
FIXED_REAR_TRACK = 1.24
FREEZE_SLALOM_PATH = True
FIXED_SLALOM_PATH_TRACK = 1.28
SLALOM_INITIAL_SPEED_MPS = 8.0


@dataclass(frozen=True)
class OptimizationParameter:
    name: str
    initial: float
    lower: float
    upper: float


@dataclass(frozen=True)
class SlalomSample:
    iteration: int
    tf: float
    tr: float
    K_heave_rate: float
    K_roll_rate: float
    cost: float
    accepted: bool


@dataclass(frozen=True)
class SlalomGeometryConfig:
    track_width: float = 10.0
    offset_list: tuple[float, ...] = tuple(DEFAULT_OFFSET_LIST)
    r_cone: float = 0.16122034611053285
    driving_error: float = 0.2
    gap: float = 7.0
    wheelbase: float = 1.53
    num_circles: int = 10
    generated_track_dir: Path = GENERATED_TRACK_DIR
    fixed_front_track: float = FIXED_FRONT_TRACK
    fixed_rear_track: float = FIXED_REAR_TRACK
    freeze_slalom_path: bool = FREEZE_SLALOM_PATH
    fixed_slalom_path_track: float = FIXED_SLALOM_PATH_TRACK
    initial_speed_mps: float = SLALOM_INITIAL_SPEED_MPS


def build_track_optimization_parameters(car: Car_simplify) -> list[OptimizationParameter]:
    return [
        OptimizationParameter(
            name="K_heave_rate",
            initial=float(car.K_heave_rate),
            lower=float(car.K_heave_rate) * 0.5,
            upper=float(car.K_heave_rate) * 1.5,
        ),
        OptimizationParameter(
            name="K_roll_rate",
            initial=float(car.K_roll_rate),
            lower=float(car.K_roll_rate) * 0.5,
            upper=float(car.K_roll_rate) * 1.5,
        ),
    ]


def clip_to_bounds(values: np.ndarray, lower: np.ndarray, upper: np.ndarray) -> np.ndarray:
    return np.minimum(np.maximum(values, lower), upper)


def slalom_path_track_width(
    tf: float,
    tr: float,
    geometry: SlalomGeometryConfig,
) -> float:
    if geometry.freeze_slalom_path:
        return float(geometry.fixed_slalom_path_track)
    return max(float(tf), float(tr))


def slalom_radii_for_path_track(
    path_track: float,
    geometry: SlalomGeometryConfig,
) -> np.ndarray:
    sink = StringIO()
    with redirect_stdout(sink):
        radii = slalom_path_analysis(
            track_width=geometry.track_width,
            offset_list=list(geometry.offset_list),
            r_cone=geometry.r_cone,
            Driving_error=geometry.driving_error,
            gap=geometry.gap,
            L=geometry.wheelbase,
            best_wheel_track=float(path_track),
            num_circles=geometry.num_circles,
            show_plot=False,
        )
    return np.asarray(radii, dtype=float)


def slalom_clearance_radius(
    path_track: float,
    geometry: SlalomGeometryConfig,
) -> float:
    d_lim = 190.5 / 2.0 / 1000.0 + geometry.r_cone + geometry.driving_error
    return float(path_track) / 2.0 + d_lim


def slalom_arc_rows(
    curve_radius: float,
    offset: float,
    clearance_radius: float,
    geometry: SlalomGeometryConfig,
) -> list[dict[str, float | int | str]]:
    rows: list[dict[str, float | int | str]] = []
    radius = float(curve_radius)
    offset_theta = float(
        np.rad2deg(
            np.arctan(
                2.0 * (radius - float(offset) - clearance_radius)
                / geometry.gap
            )
        )
    )

    for index in range(int(geometry.num_circles)):
        cone_offset = float(offset) * ((-1.0) ** index)
        sign = 1.0 if cone_offset >= 0.0 else -1.0
        center_x = index * geometry.gap
        center_y = cone_offset + (clearance_radius - radius) * sign

        if cone_offset >= 0.0:
            direction = "Clockwise"
            start_angle = 180.0 - offset_theta
            end_angle = offset_theta
        else:
            direction = "Counterclockwise"
            start_angle = 180.0 + offset_theta
            end_angle = 360.0 - offset_theta

        start = point_on_circle(center_x, center_y, radius, start_angle)
        end = point_on_circle(center_x, center_y, radius, end_angle)
        rows.append(
            {
                "section": index + 1,
                "Type": "ARC",
                "Start_X": start[0],
                "Start_Y": start[1],
                "End_X": end[0],
                "End_Y": end[1],
                "Center_X": center_x,
                "Center_Y": center_y,
                "Radius": radius,
                "Start_Angle (deg)": start_angle,
                "End_Angle (deg)": end_angle,
                "length": radius * abs(np.deg2rad(end_angle - start_angle)),
                "Direction": direction,
            }
        )

    return rows


def point_on_circle(
    center_x: float,
    center_y: float,
    radius: float,
    angle_deg: float,
) -> tuple[float, float]:
    angle = np.deg2rad(angle_deg)
    return (
        float(center_x + radius * np.cos(angle)),
        float(center_y + radius * np.sin(angle)),
    )


def write_slalom_track_csv(
    path_track: float,
    radius_index: int,
    curve_radius: float,
    geometry: SlalomGeometryConfig,
) -> Path:
    offsets = geometry.track_width * np.asarray(geometry.offset_list, dtype=float)
    clearance_radius = slalom_clearance_radius(path_track, geometry)
    rows = slalom_arc_rows(
        curve_radius=float(curve_radius),
        offset=float(offsets[radius_index]),
        clearance_radius=clearance_radius,
        geometry=geometry,
    )

    output_dir = Path(geometry.generated_track_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / (
        f"slalom_track_{path_track:.6f}_"
        f"radius_{radius_index + 1}_"
        f"cones_{geometry.num_circles}.csv"
    )
    pd.DataFrame(rows).to_csv(output_path, index=False)
    return output_path


def evaluate_slalom_cost(
    values: np.ndarray,
    parameters: list[OptimizationParameter],
    config: LapSimulationConfig,
    geometry: SlalomGeometryConfig,
    iteration: int | None = None,
    label: str = "eval",
    best_cost: float | None = None,
) -> float:
    car_parameters = {
        "tf": float(geometry.fixed_front_track),
        "tr": float(geometry.fixed_rear_track),
    }
    car_parameters.update({
        parameter.name: float(value)
        for parameter, value in zip(parameters, values)
    })
    path_track = slalom_path_track_width(
        car_parameters["tf"],
        car_parameters["tr"],
        geometry,
    )
    radii = slalom_radii_for_path_track(path_track, geometry)

    total_lap_time = 0.0
    for radius_index, curve_radius in enumerate(radii):
        segment_csv = write_slalom_track_csv(
            path_track=path_track,
            radius_index=radius_index,
            curve_radius=float(curve_radius),
            geometry=geometry,
        )
        slalom_config = config.__class__(
            **{
                **config.__dict__,
                "segment_csv": segment_csv,
                "initial_speed_mps": geometry.initial_speed_mps,
            }
        )
        total_lap_time += run_lap_simulation(
            parameters=car_parameters,
            config=slalom_config,
        ).lap_time_s

    total_cost = total_lap_time
    best_text = "" if best_cost is None else f" | best={best_cost:.3f} s"
    iter_text = "--" if iteration is None else str(iteration)
    radius_text = ", ".join(f"{radius:.3f}" for radius in radii)
    print(
        f"iter {iter_text:>2} | {label:<8} | "
        f"tf={car_parameters['tf']:.3f} m | "
        f"tr={car_parameters['tr']:.3f} m | "
        f"K_heave_rate={car_parameters['K_heave_rate']:.3f} | "
        f"K_roll_rate={car_parameters['K_roll_rate']:.3f} | "
        f"radii=[{radius_text}] m | "
        f"initial_speed={geometry.initial_speed_mps:.2f} m/s | "
        f"slalom_time={total_lap_time:.3f} s | "
        f"cost={total_cost:.3f} s{best_text}",
        flush=True,
    )
    return float(total_cost)


def optimize_slalom_track_widths(
    tolerance_m: float = 0.01,
    max_iterations: int = 40,
    config: LapSimulationConfig = LapSimulationConfig(),
    geometry: SlalomGeometryConfig = SlalomGeometryConfig(),
) -> tuple[np.ndarray, float, list[SlalomSample], list[OptimizationParameter]]:
    base_car = Car_simplify()
    parameters = build_track_optimization_parameters(base_car)
    lower = np.array([parameter.lower for parameter in parameters], dtype=float)
    upper = np.array([parameter.upper for parameter in parameters], dtype=float)
    current = np.array([parameter.initial for parameter in parameters], dtype=float)
    step = (upper - lower) * 0.25
    best_cost = evaluate_slalom_cost(
        current,
        parameters,
        config,
        geometry,
        iteration=0,
        label="initial",
    )
    history = [
        SlalomSample(
            iteration=0,
            tf=float(geometry.fixed_front_track),
            tr=float(geometry.fixed_rear_track),
            K_heave_rate=float(current[0]),
            K_roll_rate=float(current[1]),
            cost=float(best_cost),
            accepted=True,
        )
    ]

    for iteration in range(1, max_iterations + 1):
        iteration_start = current.copy()
        accepted_any = False

        for axis in range(len(current)):
            axis_best = current.copy()
            axis_best_cost = best_cost

            for direction in (-1.0, 1.0):
                candidate = current.copy()
                candidate[axis] += direction * step[axis]
                candidate = clip_to_bounds(candidate, lower, upper)

                if np.allclose(candidate, current, atol=1e-12, rtol=0.0):
                    continue

                candidate_cost = evaluate_slalom_cost(
                    candidate,
                    parameters,
                    config,
                    geometry,
                    iteration=iteration,
                    label=f"{parameters[axis].name}{direction:+.0f}",
                    best_cost=best_cost,
                )
                history.append(
                    SlalomSample(
                        iteration=iteration,
                        tf=float(geometry.fixed_front_track),
                        tr=float(geometry.fixed_rear_track),
                        K_heave_rate=float(candidate[0]),
                        K_roll_rate=float(candidate[1]),
                        cost=float(candidate_cost),
                        accepted=False,
                    )
                )

                if candidate_cost < axis_best_cost:
                    axis_best = candidate
                    axis_best_cost = candidate_cost

            if axis_best_cost < best_cost:
                current = axis_best
                best_cost = axis_best_cost
                accepted_any = True
                print(
                    f"iter {iteration:>2} | accepted | "
                    f"tf={geometry.fixed_front_track:.3f} m | "
                    f"tr={geometry.fixed_rear_track:.3f} m | "
                    f"K_heave_rate={current[0]:.3f} | "
                    f"K_roll_rate={current[1]:.3f} | "
                    f"best_slalom_cost={best_cost:.3f} s",
                    flush=True,
                )
                history.append(
                    SlalomSample(
                        iteration=iteration,
                        tf=float(geometry.fixed_front_track),
                        tr=float(geometry.fixed_rear_track),
                        K_heave_rate=float(current[0]),
                        K_roll_rate=float(current[1]),
                        cost=float(best_cost),
                        accepted=True,
                    )
                )

        parameter_change = float(np.max(np.abs(current - iteration_start)))
        if parameter_change < tolerance_m:
            if not accepted_any:
                step *= 0.5
                print(
                    f"iter {iteration:>2} | shrink   | "
                    f"step_K_heave_rate={step[0]:.4f} | "
                    f"step_K_roll_rate={step[1]:.4f}",
                    flush=True,
                )
            if float(np.max(step)) < tolerance_m:
                print(
                    f"Converged: max step {float(np.max(step)):.4f} m "
                    f"< tolerance {tolerance_m:.4f} m",
                    flush=True,
                )
                break
        elif not accepted_any:
            step *= 0.5
            print(
                f"iter {iteration:>2} | shrink   | "
                f"step_K_heave_rate={step[0]:.4f} | "
                f"step_K_roll_rate={step[1]:.4f}",
                flush=True,
            )

    return current, float(best_cost), history, parameters


def plot_cost_history(
    history: list[SlalomSample],
    output_path: str | Path = Path("data") / "slalom_stiffness_ratio_optimization_cost.png",
) -> Path:
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    costs = np.array([sample.cost for sample in history], dtype=float)
    best_cost = np.minimum.accumulate(costs)
    evaluations = np.arange(len(history))

    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(evaluations, costs, marker="o", linewidth=1.0, label="evaluated cost")
    ax.plot(evaluations, best_cost, linewidth=2.0, label="best cost")
    ax.set_xlabel("Evaluation")
    ax.set_ylabel("Three-radius slalom time cost [s]")
    ax.set_title("Slalom Stiffness Ratio Optimization Cost")
    ax.grid(True, linestyle=":", linewidth=0.6)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    plt.show()
    return output_path


def main() -> None:
    best_values, best_cost, history, parameters = optimize_slalom_track_widths()
    cost_plot = plot_cost_history(history)
    best = {
        parameter.name: float(value)
        for parameter, value in zip(parameters, best_values)
    }

    print("Slalom stiffness ratio optimization complete")
    print(f"Fixed front track tf: {SlalomGeometryConfig().fixed_front_track:.3f} m")
    print(f"Fixed rear track tr: {SlalomGeometryConfig().fixed_rear_track:.3f} m")
    print(f"Best heave stiffness rear/front rate: {best['K_heave_rate']:.3f}")
    print(f"Best roll stiffness rear/front rate: {best['K_roll_rate']:.3f}")
    print(f"Best three-radius slalom cost: {best_cost:.3f} s")
    print(f"Cost plot saved: {cost_plot}")


if __name__ == "__main__":
    main()
