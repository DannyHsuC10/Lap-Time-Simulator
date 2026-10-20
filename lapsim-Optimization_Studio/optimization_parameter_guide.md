# Optimization Parameter Guide

This note explains how to add, remove, or tune optimization variables in:

- `track_parameter_optimizer.py`
- `slalom_track_width_optimizer.py`

Both optimizers use the same basic pattern:

1. Build a list of `OptimizationParameter`.
2. Convert the current numeric vector into `car_parameters`.
3. Pass `car_parameters` into `run_lap_simulation(...)`.
4. Store and print the evaluated values.

## Current Optimization Variables

The main track optimizer (`track_parameter_optimizer.py`) currently optimizes:

```python
K_heave_gain
K_roll_gain
```

`K_heave_gain` multiplies both front and rear heave spring rates. `K_roll_gain`
multiplies both front and rear roll spring rates. The front/rear ratios remain
fixed by `K_heave_rate` and `K_roll_rate` on `Car_simplify`.

## Add A New Optimization Variable

### 1. Add The Variable To `car.py`

The variable must exist as a `Car_simplify` attribute before the optimizer can set it.

Example:

```python
self.new_parameter = 1.0
```

If the variable changes derived data, add or reuse a rebuild method.

Example:

```python
def rebuild_suspension_stiffness(self):
    ...
```

For suspension stiffness variables, `lap_simulation_runner.py` already calls:

```python
car.rebuild_suspension_stiffness()
```

after applying parameters.

### 2. Add It To `build_track_optimization_parameters(...)`

In each optimizer, add one `OptimizationParameter` entry:

```python
OptimizationParameter(
    name="new_parameter",
    initial=float(car.new_parameter),
    lower=float(car.new_parameter) * 0.5,
    upper=float(car.new_parameter) * 1.5,
),
```

Use absolute bounds if a multiplier could be unsafe:

```python
OptimizationParameter(
    name="new_parameter",
    initial=float(car.new_parameter),
    lower=0.0,
    upper=2.0,
),
```

### 3. Update The History Dataclass

Add the field to `OptimizationSample` or `SlalomSample`:

```python
new_parameter: float
```

Then add it anywhere history samples are created:

```python
new_parameter=float(current[4])
new_parameter=float(candidate[4])
```

The index must match the order in `build_track_optimization_parameters(...)`.

### 4. Update Logging

In `evaluate_track_cost(...)` or `evaluate_slalom_cost(...)`, add the value to the print output:

```python
f"new_parameter={car_parameters['new_parameter']:.3f} | "
```

Also update accepted and shrink logs if the new variable's step size should be visible:

```python
f"step_new_parameter={step[4]:.4f}"
```

### 5. Update Final Output

In `main()`, add:

```python
print(f"Best new parameter: {best['new_parameter']:.3f}")
```

## Remove An Optimization Variable

To remove a variable:

1. Delete its `OptimizationParameter(...)` entry.
2. Remove its field from `OptimizationSample` or `SlalomSample`.
3. Remove all `current[index]` and `candidate[index]` references for it.
4. Remove it from print/log strings.
5. Remove it from final `main()` output.

After deleting, renumber later vector indexes.

Example: if removing `K_heave_rate`, then `K_roll_rate` may move from:

```python
current[3]
```

to:

```python
current[2]
```

## Important: Vector Order

The optimizer relies on this order:

```python
parameters = build_track_optimization_parameters(base_car)
current = np.array([parameter.initial for parameter in parameters])
```

So every `current[index]`, `candidate[index]`, and `step[index]` must match the order of `parameters`.

The main track optimizer's current parameter order:

```python
0: K_heave_gain
1: K_roll_gain
```

## Track Width Mismatch Penalty

Both optimizers currently add a soft time penalty when `tf` and `tr` differ:

```python
TRACK_MISMATCH_PENALTY_COEFFICIENT = 70.0
```

Penalty formula:

```python
penalty = coefficient * (abs(tf - tr) / max(tf, tr)) ** 2
```

Increase the coefficient to make front/rear track widths stay closer.  
Decrease it to allow more difference.

## Slalom-Specific Path Mode

`slalom_track_width_optimizer.py` can freeze the Slalom driving line:

```python
FREEZE_SLALOM_PATH = True
FIXED_SLALOM_PATH_TRACK = 1.28
```

When frozen, Slalom radii are always generated from `FIXED_SLALOM_PATH_TRACK`, even while `tf` and `tr` are optimized.

When dynamic:

```python
FREEZE_SLALOM_PATH = False
```

Slalom radii are generated from:

```python
max(tf, tr)
```

## Quick Validation

After changing optimization variables, run:

```powershell
.\.venv\Scripts\python.exe -m py_compile .\car.py .\lap_simulation_runner.py .\track_parameter_optimizer.py .\slalom_track_width_optimizer.py
```

Then confirm parameter names:

```powershell
.\.venv\Scripts\python.exe -c "from car import Car_simplify; import track_parameter_optimizer as t; import slalom_track_width_optimizer as s; c=Car_simplify(); print([p.name for p in t.build_track_optimization_parameters(c)]); print([p.name for p in s.build_track_optimization_parameters(c)])"
```

Run the lightweight benchmark:

```powershell
.\.venv\Scripts\python.exe .\benchmark_lap_simulation_runner.py
```

Avoid running the full optimization just to check syntax.
