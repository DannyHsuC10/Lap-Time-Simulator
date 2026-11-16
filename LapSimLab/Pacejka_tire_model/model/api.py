"""Degree-based MF6.1.2 API. MY/RRT are the same rolling moment.

RRT_SAE explicitly selects the legacy empirical option.
"""
from pathlib import Path
import numpy as np
from ._common import CalculationInput
from .Fx_models import MF612_Fx0, MF612_Fx
from .Fy_models import MF612_Fy0, MF612_Fy
from .Mz_models import MF612_Mz0, MF612_Mz
from .Mx_models import MF612_Mx
from .RRT_models import MF612_My, MF612_RRT_SAE


def create_calculation_input_from_config(*, FZ, SA, SL=None, IA=None,
                                         pressure=None, tir_params, V=None,
                                         Ro=None, Fx=None, Fy=None):
    return CalculationInput(FZ=FZ, SA=SA, SL=SL, IA=IA, pressure=pressure,
                            tir_params=tir_params, V=V, Ro=Ro, Fx=Fx, Fy=Fy)


_MODELS = {"FX0": MF612_Fx0, "FX": MF612_Fx, "FY0": MF612_Fy0,
           "FY": MF612_Fy, "MZ0": MF612_Mz0, "MZ": MF612_Mz,
           "MX": MF612_Mx, "MY": MF612_My, "RRT": MF612_My,
           "RRT_SAE": MF612_RRT_SAE}


def MF_Universal_solver(out, MF):
    try:
        implementation = _MODELS[str(out).upper()]
    except KeyError as exc:
        raise ValueError(f"Unknown output {out!r}; choose {', '.join(_MODELS)}") from exc
    result = np.asarray(implementation.calculate(MF), dtype=float)
    return result.item() if result.size == 1 else result


def calculate_fx(calc_input):
    return MF612_Fx.calculate(calc_input)


def calculate_fx0(calc_input):
    return MF612_Fx0.calculate(calc_input)


def calculate_fy(calc_input):
    return MF612_Fy.calculate(calc_input)


def calculate_fy0(calc_input):
    return MF612_Fy0.calculate(calc_input)


def calculate_mz(calc_input):
    return MF612_Mz.calculate(calc_input)


def calculate_mz0(calc_input):
    return MF612_Mz0.calculate(calc_input)


def calculate_mx(calc_input):
    return MF612_Mx.calculate(calc_input)


def calculate_my(calc_input):
    return MF612_My.calculate(calc_input)


def calculate_rrt(calc_input):
    return MF612_My.calculate(calc_input)


def load_tir(filename):
    """Read section-aware TIR data, preserving historical relative searches."""
    path = Path(filename)
    root = Path(__file__).resolve().parent.parent
    candidates = [path, root / "tir" / path, root / "data" / "tir" / path]
    resolved = next((p for p in candidates if p.is_file()), None)
    if resolved is None:
        raise FileNotFoundError(filename)
    try:
        from tir_io import load_tir as reader
    except ImportError:
        # Standalone export package carries its own tir_io.
        from ..tir_io import load_tir as reader
    return reader(resolved)
