"""Minimize whole-lap time and measure each selected change in isolation."""

from dataclasses import dataclass, replace
import math

from .aero import AERO_COEFFICIENT_KEYS, read_aero_coefficients
from .motor import VoltageMotorCurve
from .settings import PERCENTAGE_FIELDS, Settings
from .simulation import InfeasibleLapError, Result, simulate


FIXED_MODELS = dict(load_model="point_mass", brake_model="leverage", aero_model="quadratic",
                    aero_enabled=True, solver="lap", yaw_correction=False)
POSITIVE_MIN = math.nextafter(0.0, 1.0)


@dataclass(frozen=True)
class Parameter:
    key: str
    label: str
    fixed_margin: float | None = None
    lower: float = -math.inf
    upper: float = math.inf

    def default_margin(self, baseline):
        return abs(baseline) * 0.1 if self.fixed_margin is None else self.fixed_margin


PARAMETERS = (
    Parameter("mass", "總質量 (kg)", lower=POSITIVE_MIN),
    Parameter("tire_utilization", "輪胎使用率", fixed_margin=0.01, lower=0, upper=1),
    Parameter("gear_ratio", "減速比", lower=POSITIVE_MIN),
    Parameter("gear_efficiency", "傳動效率", fixed_margin=0.01, lower=0, upper=1),
    Parameter("dc_voltage", "直流電壓 (V)", fixed_margin=53, lower=POSITIVE_MIN, upper=600),
    Parameter("brake_leverage", "煞車機械利益", lower=POSITIVE_MIN),
    Parameter("CL0", "CL"),
    Parameter("CD0", "CD", lower=0.0),
)
DEFAULT_EQUIVALENT_KEYS = frozenset(p.key for p in PARAMETERS if p.key != "gear_ratio")


def active_parameters(motor_model):
    return tuple(p for p in PARAMETERS if p.key != "dc_voltage" or motor_model == "lp03")


@dataclass(frozen=True)
class SearchRange:
    parameter: Parameter
    baseline: float
    lower: float
    upper: float


@dataclass(frozen=True)
class EquivalentChange:
    value: float
    lap_time: float


@dataclass(frozen=True)
class OptimizationResult:
    ranges: tuple[SearchRange, ...]
    baseline_time: float
    settings: Settings
    aero_coefficients: dict[str, float]
    simulation: Result
    evaluations: int
    single_parameter_times: dict[str, float]
    equivalent_keys: frozenset[str]
    equivalent_gain: float | None
    equivalent_changes: dict[str, EquivalentChange]

    def parameter_value(self, key):
        if key in AERO_COEFFICIENT_KEYS:
            return self.aero_coefficients[key]
        return getattr(self.settings, key)


def build_ranges(baseline, aero_coefficients, margins):
    voltage_lower, voltage_upper = 0.0, 600.0
    if baseline.motor_model == "lp03":
        voltages = VoltageMotorCurve.read_points(baseline.lp03_path)[:, 0]
        voltage_lower = float(voltages.min())
        voltage_upper = min(600.0, float(voltages.max()))
        if voltage_lower > voltage_upper:
            raise ValueError("LP03 電壓資料沒有 600 V 以下的有效範圍。")

    ranges = []
    for parameter in active_parameters(baseline.motor_model):
        lower, upper = parameter.lower, parameter.upper
        if parameter.key == "dc_voltage":
            lower, upper = max(lower, voltage_lower), voltage_upper
        if parameter.key in AERO_COEFFICIENT_KEYS:
            value = aero_coefficients[parameter.key]
        else:
            value = getattr(baseline, parameter.key)
        if not math.isfinite(value) or not lower <= value <= upper:
            if parameter.key in PERCENTAGE_FIELDS:
                raise ValueError(f"{parameter.label} 的基準須介於 {lower:.0%} 與 {upper:.0%}。")
            raise ValueError(f"{parameter.label} 的基準須介於 {lower:g} 與 {upper:g}。")
        margin = margins.get(parameter.key, parameter.default_margin(value))
        if not math.isfinite(margin) or margin < 0:
            raise ValueError(f"{parameter.label} 的 M 須為有限的非負數。")
        lower, upper = max(lower, value - margin), min(upper, value + margin)
        if not math.isfinite(lower) or not math.isfinite(upper):
            raise ValueError(f"{parameter.label} 的搜尋範圍須為有限值。")
        ranges.append(SearchRange(parameter, value, lower, upper))
    return tuple(ranges)


def find_equivalent_change(baseline, best, best_time, target_time, tolerance, evaluate):
    # Keep one value that misses the target and one that reaches it, in either parameter direction.
    misses_target = baseline
    reaches_target = EquivalentChange(best, best_time)
    for _ in range(20):
        if target_time - reaches_target.lap_time <= tolerance:
            break
        value = misses_target + (reaches_target.value - misses_target) / 2
        if value == misses_target or value == reaches_target.value:
            break
        lap_time = evaluate(value)
        if lap_time <= target_time:
            reaches_target = EquivalentChange(value, lap_time)
        else:
            misses_target = value
    return reaches_target


def optimize_lap(settings: Settings, *, margins=None, max_evaluations=64, equivalent_keys=None,
                 progress=None, check_cancelled=None) -> OptimizationResult:
    if type(max_evaluations) is not int or max_evaluations < 1:
        raise ValueError("評估上限須為正整數。")
    baseline = replace(settings, **FIXED_MODELS)
    baseline.validate()
    aero_baseline = read_aero_coefficients(baseline.aero_path)
    ranges = build_ranges(baseline, aero_baseline, margins or {})
    requested_keys = DEFAULT_EQUIVALENT_KEYS if equivalent_keys is None else frozenset(equivalent_keys)
    selected_keys = requested_keys.intersection(bound.parameter.key for bound in ranges)
    start = tuple(bound.baseline for bound in ranges)
    trials = {}

    def candidate_inputs(values):
        vehicle = {}
        coefficients = aero_baseline.copy()
        for bound, value in zip(ranges, values):
            key = bound.parameter.key
            if key in AERO_COEFFICIENT_KEYS:
                coefficients[key] = value
            else:
                vehicle[key] = value
        return replace(baseline, **vehicle), coefficients

    def evaluate(values, *, message=None):
        if check_cancelled is not None:
            check_cancelled()
        if values not in trials:
            if progress is not None:
                progress(message or f"評估 {len(trials) + 1}/{max_evaluations} 組參數")
            try:
                candidate, coefficients = candidate_inputs(values)
                result = simulate(candidate, include_signals=False, check_cancelled=check_cancelled,
                                  aero_coefficients=coefficients)
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

    def evaluate_single(i, value, message):
        return evaluate(start[:i] + (value,) + start[i + 1:], message=message)

    single_parameter_times = {}
    if math.isfinite(baseline_time):
        for i, bound in enumerate(ranges):
            single_parameter_times[bound.parameter.key] = evaluate_single(
                i, best[i], f"計算「{bound.parameter.label}」單項效益",
            )

    eligible_times = {
        key: lap_time for key, lap_time in single_parameter_times.items()
        if key in selected_keys and math.isfinite(lap_time) and lap_time < baseline_time
    }
    equivalent_gain = None
    equivalent_changes = {}
    if eligible_times:
        target_time = max(eligible_times.values())
        equivalent_gain = baseline_time - target_time
        tolerance = min(0.0001, 0.01 * equivalent_gain)
        for i, bound in enumerate(ranges):
            key = bound.parameter.key
            if key not in eligible_times:
                continue
            message = f"計算「{bound.parameter.label}」等效調整量"
            equivalent_changes[key] = find_equivalent_change(
                start[i], best[i], eligible_times[key], target_time, tolerance,
                lambda value: evaluate_single(i, value, message),
            )

    best_settings, best_coefficients = candidate_inputs(best)
    if progress is not None:
        progress(f"整理最佳結果（{best_time:.3f} 秒）")
    result = simulate(best_settings, check_cancelled=check_cancelled, aero_coefficients=best_coefficients)
    return OptimizationResult(ranges, baseline_time, best_settings, best_coefficients, result, evaluations,
                              single_parameter_times, selected_keys, equivalent_gain, equivalent_changes)
