"""Minimize whole-lap time and measure each selected change in isolation."""

from dataclasses import dataclass, replace
import math

from .aero import QuadraticAeroModel
from .motor import VoltageMotorCurve
from .settings import Settings
from .simulation import InfeasibleLapError, Result, simulate


FIXED_MODELS = dict(load_model="point_mass", brake_model="leverage", aero_model="quadratic",
                    aero_enabled=True, solver="lap", yaw_correction=False)
POSITIVE_MIN = math.nextafter(0.0, 1.0)


@dataclass(frozen=True)
class Parameter:
    key: str
    label: str
    margin: float
    lower: float = -math.inf
    upper: float = math.inf


PARAMETERS = (
    Parameter("mass", "總質量 (kg)", 32, POSITIVE_MIN),
    Parameter("tire_utilization", "輪胎使用率", 0.1, 0, 1),
    Parameter("gear_ratio", "減速比", 1.12, POSITIVE_MIN),
    Parameter("gear_efficiency", "傳動效率", 0.095, 0, 1),
    Parameter("dc_voltage", "直流電壓 (V)", 53, POSITIVE_MIN, 600),
    Parameter("brake_leverage", "煞車機械利益", 1.26, POSITIVE_MIN),
    Parameter("aero_cl0", "CL", 0.24),
    Parameter("aero_cd0", "CD", 0.18, 0),
)


def active_parameters(motor_model):
    return tuple(p for p in PARAMETERS if p.key != "dc_voltage" or motor_model == "lp03")


@dataclass(frozen=True)
class SearchRange:
    parameter: Parameter
    baseline: float
    lower: float
    upper: float


@dataclass(frozen=True)
class OptimizationResult:
    ranges: tuple[SearchRange, ...]
    baseline_time: float
    settings: Settings
    simulation: Result
    evaluations: int
    single_parameter_times: dict[str, float]


def build_ranges(baseline, margins):
    voltage_lower, voltage_upper = 0.0, 600.0
    if baseline.motor_model == "lp03":
        voltages = VoltageMotorCurve.read_points(baseline.lp03_path)[:, 0]
        voltage_lower = float(voltages.min())
        voltage_upper = min(600.0, float(voltages.max()))
        if voltage_lower > voltage_upper:
            raise ValueError("LP03 電壓資料沒有 600 V 以下的有效範圍。")

    ranges = []
    for parameter in active_parameters(baseline.motor_model):
        margin = margins.get(parameter.key, parameter.margin)
        if not math.isfinite(margin) or margin < 0:
            raise ValueError(f"{parameter.label} 的 M 須為有限的非負數。")
        lower, upper = parameter.lower, parameter.upper
        if parameter.key == "dc_voltage":
            lower, upper = max(lower, voltage_lower), voltage_upper
        value = getattr(baseline, parameter.key)
        if not math.isfinite(value) or not lower <= value <= upper:
            raise ValueError(f"{parameter.label} 的基準須介於 {lower:g} 與 {upper:g}。")
        lower, upper = max(lower, value - margin), min(upper, value + margin)
        if not math.isfinite(lower) or not math.isfinite(upper):
            raise ValueError(f"{parameter.label} 的搜尋範圍須為有限值。")
        ranges.append(SearchRange(parameter, value, lower, upper))
    return tuple(ranges)


def optimize_lap(settings: Settings, *, margins=None, max_evaluations=64,
                 progress=None, check_cancelled=None) -> OptimizationResult:
    if type(max_evaluations) is not int or max_evaluations < 1:
        raise ValueError("評估上限須為正整數。")
    baseline = replace(settings, **FIXED_MODELS)
    baseline.validate()
    if baseline.aero_cl0 is None or baseline.aero_cd0 is None:
        aero = QuadraticAeroModel.from_json(
            baseline.aero_path, wheelbase=baseline.wheelbase, track=(baseline.front_track + baseline.rear_track) / 2,
            cl0=baseline.aero_cl0, cd0=baseline.aero_cd0,
        )
        baseline = replace(baseline, aero_cl0=aero.CL0, aero_cd0=aero.CD0)
    ranges = build_ranges(baseline, margins or {})
    start = tuple(bound.baseline for bound in ranges)
    trials = {}

    def candidate_settings(values):
        return replace(baseline, **{bound.parameter.key: value for bound, value in zip(ranges, values)})

    def evaluate(values, *, message=None):
        if check_cancelled is not None:
            check_cancelled()
        if values not in trials:
            if progress is not None:
                progress(message or f"評估 {len(trials) + 1}/{max_evaluations} 組參數")
            try:
                result = simulate(candidate_settings(values), include_signals=False, check_cancelled=check_cancelled)
                trials[values] = result.lap_time
            except InfeasibleLapError:
                trials[values] = math.inf
        return trials[values]

    best = start
    baseline_time = best_time = evaluate(start)
    # Explore both directions on each axis, then refine with progressively smaller steps.
    for fraction in (1.0, 0.5, 0.25, 0.125):
        for i, bound in enumerate(ranges):
            anchor = best
            step = max(bound.baseline - bound.lower, bound.upper - bound.baseline)
            for direction in (-1, 1):
                if len(trials) >= max_evaluations:
                    break
                trial = list(anchor)
                trial[i] = min(bound.upper, max(bound.lower, anchor[i] + direction * step * fraction))
                if trial[i] == anchor[i]:
                    continue
                values = tuple(trial)
                lap_time = evaluate(values)
                if lap_time < best_time:
                    best, best_time = values, lap_time

    if not math.isfinite(best_time):
        raise InfeasibleLapError("搜尋範圍內沒有找到可完成的單圈。請檢查基準與各參數的 M。")
    evaluations = len(trials)
    single_parameter_times = {}
    if math.isfinite(baseline_time):
        for i, bound in enumerate(ranges):
            values = start[:i] + (best[i],) + start[i + 1:]
            single_parameter_times[bound.parameter.key] = evaluate(values, message=f"計算「{bound.parameter.label}」單項效益")
    best_settings = candidate_settings(best)
    if progress is not None:
        progress(f"整理最佳結果（{best_time:.3f} 秒）")
    result = simulate(best_settings, check_cancelled=check_cancelled)
    return OptimizationResult(ranges, baseline_time, best_settings, result, evaluations, single_parameter_times)
