from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Mapping

import matplotlib.pyplot as plt
import numpy as np

from car import Car_simplify
from lap_simulation_runner import LapSimulationConfig, run_lap_simulation


TRACK_MISMATCH_PENALTY_COEFFICIENT = 70.0
TRACK_MISMATCH_PARAMETER_DEFAULTS = ("tf", "tr")
DEFAULT_PARAMETER_UNIT_SUFFIXES = {
    "tf": " m",
    "tr": " m",
}


@dataclass(frozen=True)
class OptimizationTarget:
    name: str
    lower_multiplier: float = 0.5
    upper_multiplier: float = 1.5
    lower: float | None = None
    upper: float | None = None
    unit_suffix: str = ""


# Change this list to choose optimization targets.
# Examples:
#   OptimizationTarget("tf", lower_multiplier=0.95, upper_multiplier=1.05, unit_suffix=" m")
#   OptimizationTarget("tr", lower_multiplier=0.97, upper_multiplier=1.05, unit_suffix=" m")
#   OptimizationTarget("K_heave_gain", lower=0.5, upper=1.5)
#   OptimizationTarget("K_roll_gain", lower=0.5, upper=1.5)
OPTIMIZATION_TARGETS: tuple[OptimizationTarget, ...] = (
    OptimizationTarget("K_heave_gain", lower=0.5, upper=1.5),
    OptimizationTarget("K_roll_gain", lower=0.5, upper=1.5),
#   OptimizationTarget("tf", lower_multiplier=0.95, upper_multiplier=1.05, unit_suffix=" m"),
#   OptimizationTarget("tr", lower_multiplier=0.97, upper_multiplier=1.05, unit_suffix=" m"),
)

# Use this when a parameter should be fixed during optimization but still passed
# into the lap simulator, for example {"tf": 1.28, "tr": 1.24}.
FIXED_CAR_PARAMETERS: dict[str, float] = {}


@dataclass(frozen=True)
class OptimizationParameter:
    name: str
    initial: float
    lower: float
    upper: float
    unit_suffix: str = ""


@dataclass(frozen=True)
class OptimizationSample:
    iteration: int
    values: dict[str, float]
    cost: float
    accepted: bool


def build_track_optimization_parameters(car: Car_simplify) -> list[OptimizationParameter]:
    parameters: list[OptimizationParameter] = []
    for target in OPTIMIZATION_TARGETS:
        if not hasattr(car, target.name):
            raise AttributeError(
                f"Car_simplify has no parameter named {target.name!r}. "
                "Please update OPTIMIZATION_TARGETS."
            )

        initial = float(getattr(car, target.name))
        lower = target.lower if target.lower is not None else initial * target.lower_multiplier
        upper = target.upper if target.upper is not None else initial * target.upper_multiplier
        if lower >= upper:
            raise ValueError(
                f"Invalid bounds for {target.name}: lower={lower}, upper={upper}"
            )

        parameters.append(
            OptimizationParameter(
                name=target.name,
                initial=initial,
                lower=float(lower),
                upper=float(upper),
                unit_suffix=target.unit_suffix,
            )
        )

    return parameters


def clip_to_bounds(values: np.ndarray, lower: np.ndarray, upper: np.ndarray) -> np.ndarray:
    return np.minimum(np.maximum(values, lower), upper)


def track_mismatch_fraction(tf: float, tr: float) -> float:
    larger = max(float(tf), float(tr))
    if larger <= 0.0:
        return 0.0
    return abs(float(tf) - float(tr)) / larger


def track_mismatch_penalty(tf: float, tr: float) -> float:
    mismatch = track_mismatch_fraction(tf, tr)
    return TRACK_MISMATCH_PENALTY_COEFFICIENT * mismatch * mismatch


def track_mismatch_penalty_from_parameters(parameters: Mapping[str, float]) -> float:
    if "tf" not in parameters or "tr" not in parameters:
        return 0.0
    return track_mismatch_penalty(parameters["tf"], parameters["tr"])


def parameter_values(
    values: np.ndarray,
    parameters: list[OptimizationParameter],
) -> dict[str, float]:
    return {
        parameter.name: float(value)
        for parameter, value in zip(parameters, values)
    }


def merged_car_parameters(
    values: np.ndarray,
    parameters: list[OptimizationParameter],
    default_parameters: Mapping[str, float] | None = None,
) -> dict[str, float]:
    car_parameters = dict(default_parameters or {})
    car_parameters.update(FIXED_CAR_PARAMETERS)
    car_parameters.update(parameter_values(values, parameters))
    return car_parameters


def parameter_unit_suffixes(
    parameters: list[OptimizationParameter],
) -> dict[str, str]:
    unit_suffixes = dict(DEFAULT_PARAMETER_UNIT_SUFFIXES)
    unit_suffixes.update(
        {
            parameter.name: parameter.unit_suffix
            for parameter in parameters
        }
    )
    return unit_suffixes


def format_parameter_values(
    values: Mapping[str, float],
    parameters: list[OptimizationParameter],
) -> str:
    unit_suffixes = parameter_unit_suffixes(parameters)
    return " | ".join(
        f"{name}={value:.3f}{unit_suffixes.get(name, '')}"
        for name, value in values.items()
    )


def format_step_values(
    step: np.ndarray,
    parameters: list[OptimizationParameter],
) -> str:
    return " | ".join(
        f"step_{parameter.name}={float(value):.4f}{parameter.unit_suffix}"
        for parameter, value in zip(parameters, step)
    )


def evaluate_track_cost(
    values: np.ndarray,
    parameters: list[OptimizationParameter],
    config: LapSimulationConfig,
    default_parameters: Mapping[str, float] | None = None,
    iteration: int | None = None,
    label: str = "eval",
    best_cost: float | None = None,
) -> float:
    car_parameters = merged_car_parameters(values, parameters, default_parameters)
    lap_time = run_lap_simulation(parameters=car_parameters, config=config).lap_time_s
    track_mismatch_cost = track_mismatch_penalty_from_parameters(car_parameters)
    cost = lap_time + track_mismatch_cost
    best_text = "" if best_cost is None else f" | best={best_cost:.3f} s"
    iter_text = "--" if iteration is None else str(iteration)
    print(
        f"iter {iter_text:>2} | {label:<8} | "
        f"{format_parameter_values(car_parameters, parameters)} | "
        f"lap_time={lap_time:.3f} s | "
        f"track_mismatch_penalty={track_mismatch_cost:.3f} s | "
        f"cost={cost:.3f} s{best_text}",
        flush=True,
    )
    return cost


def optimize_track_parameters(
    tolerance: float = 0.01,
    max_iterations: int = 40,
    config: LapSimulationConfig = LapSimulationConfig(),
) -> tuple[np.ndarray, float, list[OptimizationSample], list[OptimizationParameter]]:
    base_car = Car_simplify()
    default_parameters = {
        name: float(getattr(base_car, name))
        for name in TRACK_MISMATCH_PARAMETER_DEFAULTS
    }
    parameters = build_track_optimization_parameters(base_car)
    lower = np.array([parameter.lower for parameter in parameters], dtype=float)
    upper = np.array([parameter.upper for parameter in parameters], dtype=float)
    current = np.array([parameter.initial for parameter in parameters], dtype=float)
    step = (upper - lower) * 0.25
    best_cost = evaluate_track_cost(
        current,
        parameters,
        config,
        default_parameters=default_parameters,
        iteration=0,
        label="initial",
    )
    history = [
        OptimizationSample(
            iteration=0,
            values=parameter_values(current, parameters),
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

                candidate_cost = evaluate_track_cost(
                    candidate,
                    parameters,
                    config,
                    default_parameters=default_parameters,
                    iteration=iteration,
                    label=f"{parameters[axis].name}{direction:+.0f}",
                    best_cost=best_cost,
                )
                history.append(
                    OptimizationSample(
                        iteration=iteration,
                        values=parameter_values(candidate, parameters),
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
                    f"{format_parameter_values(parameter_values(current, parameters), parameters)} | "
                    f"best_cost={best_cost:.3f} s",
                    flush=True,
                )
                history.append(
                    OptimizationSample(
                        iteration=iteration,
                        values=parameter_values(current, parameters),
                        cost=float(best_cost),
                        accepted=True,
                    )
                )

        parameter_change = float(np.max(np.abs(current - iteration_start)))
        if parameter_change < tolerance:
            if not accepted_any:
                step *= 0.5
                print(
                    f"iter {iteration:>2} | shrink   | "
                    f"{format_step_values(step, parameters)}",
                    flush=True,
                )
            if float(np.max(step)) < tolerance:
                print(
                    f"Converged: max step {float(np.max(step)):.4f} "
                    f"< tolerance {tolerance:.4f}",
                    flush=True,
                )
                break
        elif not accepted_any:
            step *= 0.5
            print(
                f"iter {iteration:>2} | shrink   | "
                f"{format_step_values(step, parameters)}",
                flush=True,
            )

    return current, float(best_cost), history, parameters


def optimize_track_widths(
    tolerance: float = 0.01,
    max_iterations: int = 40,
    config: LapSimulationConfig = LapSimulationConfig(),
) -> tuple[np.ndarray, float, list[OptimizationSample], list[OptimizationParameter]]:
    return optimize_track_parameters(
        tolerance=tolerance,
        max_iterations=max_iterations,
        config=config,
    )


def plot_cost_history(
    history: list[OptimizationSample],
    output_path: str | Path = Path("data") / "track_parameter_optimization_cost.png",
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
    ax.set_ylabel("Lap time cost [s]")
    ax.set_title("Track Parameter Optimization Cost")
    ax.grid(True, linestyle=":", linewidth=0.6)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    plt.show()
    return output_path


def main() -> None:
    best_values, best_cost, history, parameters = optimize_track_parameters()
    cost_plot = plot_cost_history(history)
    best = parameter_values(best_values, parameters)

    print("Track parameter optimization complete")
    unit_suffixes = parameter_unit_suffixes(parameters)
    for name, value in best.items():
        print(f"Best {name}: {value:.3f}{unit_suffixes.get(name, '')}")
    print(f"Best lap time cost: {best_cost:.3f} s")
    print(f"Cost plot saved: {cost_plot}")


if __name__ == "__main__":
    main()
