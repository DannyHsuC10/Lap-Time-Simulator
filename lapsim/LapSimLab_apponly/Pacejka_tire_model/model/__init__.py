"""MF6.1.2 force/moment package; SA and IA inputs remain in degrees."""
from ._common import CalculationInput, MF612
from .Fx_models import MF612_Fx0, MF612_Fx
from .Fy_models import MF612_Fy0, MF612_Fy
from .Mz_models import MF612_Mz0, MF612_Mzt, MF612_Mz
from .Mx_models import MF612_Mx
from .RRT_models import MF612_My, MF612_RRT_MF, MF612_RRT_SAE
from .api import (MF_Universal_solver, create_calculation_input_from_config,
                  calculate_fx0, calculate_fx, calculate_fy0, calculate_fy,
                  calculate_mz0, calculate_mz, calculate_mx, calculate_my,
                  calculate_rrt, load_tir)

__version__ = "6.1.2"
__all__ = [name for name in globals() if name.startswith(("MF", "calculate_"))]
__all__ += ["CalculationInput", "create_calculation_input_from_config", "load_tir"]
