from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from car import Car_simplify
from lap_simulation_runner import LapSimulationConfig, run_lap_simulation


@dataclass(frozen=True)
class OptimizationParameter:
    name: str
    initial: float
    lower: float
    upper: float


@dataclass(frozen=True)
class OptimizationSample:
    iteration: int
    tf: float
    tr: float
    cost: float
    accepted: bool


def build_track_optimization_parameters(car: Car_simplify) -> list[OptimizationParameter]:
    return [
        OptimizationParameter(
            name="tf",
            initial=float(car.tf),
            lower=float(car.tf) * 0.75,
            upper=float(car.tf) * 1.25,
        ),
        OptimizationParameter(
            name="tr",
            initial=float(car.tr),
            lower=float(car.tr) * 0.75,
            upper=float(car.tr) * 1.25,
        ),
    ]


def clip_to_bounds(values: np.ndarray, lower: np.ndarray, upper: np.ndarray) -> np.ndarray:
    return np.minimum(np.maximum(values, lower), upper)


def evaluate_track_cost(
    values: np.ndarray,
    parameters: list[OptimizationParameter],
    config: LapSimulationConfig,
    iteration: int | None = None,
    label: str = "eval",
    best_cost: float | None = None,
) -> float:
    car_parameters = {
        parameter.name: float(value)
        for parameter, value in zip(parameters, values)
    }
    cost = run_lap_simulation(parameters=car_parameters, config=config).lap_time_s
    best_text = "" if best_cost is None else f" | best={best_cost:.3f} s"
    iter_text = "--" if iteration is None else str(iteration)
    print(
        f"iter {iter_text:>2} | {label:<8} | "
        f"tf={car_parameters['tf']:.3f} m | "
        f"tr={car_parameters['tr']:.3f} m | "
        f"lap_time={cost:.3f} s{best_text}",
        flush=True,
    )
    return cost


def optimize_track_widths(
    tolerance_m: float = 0.01,
    max_iterations: int = 40,
    config: LapSimulationConfig = LapSimulationConfig(),
) -> tuple[np.ndarray, float, list[OptimizationSample], list[OptimizationParameter]]:
    base_car = Car_simplify()
    parameters = build_track_optimization_parameters(base_car)
    lower = np.array([parameter.lower for parameter in parameters], dtype=float)
    upper = np.array([parameter.upper for parameter in parameters], dtype=float)
    current = np.array([parameter.initial for parameter in parameters], dtype=float)
    step = (upper - lower) * 0.25
    best_cost = evaluate_track_cost(current, parameters, config, iteration=0, label="initial")
    history = [
        OptimizationSample(
            iteration=0,
            tf=float(current[0]),
            tr=float(current[1]),
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
                    iteration=iteration,
                    label=f"{parameters[axis].name}{direction:+.0f}",
                    best_cost=best_cost,
                )
                history.append(
                    OptimizationSample(
                        iteration=iteration,
                        tf=float(candidate[0]),
                        tr=float(candidate[1]),
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
                    f"tf={current[0]:.3f} m | "
                    f"tr={current[1]:.3f} m | "
                    f"best_lap_time={best_cost:.3f} s",
                    flush=True,
                )
                history.append(
                    OptimizationSample(
                        iteration=iteration,
                        tf=float(current[0]),
                        tr=float(current[1]),
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
                    f"step_tf={step[0]:.4f} m | "
                    f"step_tr={step[1]:.4f} m",
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
                f"step_tf={step[0]:.4f} m | "
                f"step_tr={step[1]:.4f} m",
                flush=True,
            )

    return current, float(best_cost), history, parameters


def plot_cost_history(
    history: list[OptimizationSample],
    output_path: str | Path = Path("data") / "track_width_optimization_cost.png",
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
    ax.set_title("Track Width Optimization Cost")
    ax.grid(True, linestyle=":", linewidth=0.6)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    plt.show()
    return output_path


def main() -> None:
    best_values, best_cost, history, parameters = optimize_track_widths()
    cost_plot = plot_cost_history(history)
    best = {
        parameter.name: float(value)
        for parameter, value in zip(parameters, best_values)
    }

    print("Track width optimization complete")
    print(f"Best front track tf: {best['tf']:.3f} m")
    print(f"Best rear track tr: {best['tr']:.3f} m")
    print(f"Best lap time cost: {best_cost:.3f} s")
    print(f"Cost plot saved: {cost_plot}")


if __name__ == "__main__":
    main()
