"""MF6.1.2 shared inputs; public SA/IA remain in degrees, SI elsewhere.

Reference: teasit/magic-formula-tyre-library v61, commit
e9f0354b9bad910bfdb1f2b4342edc65fef8c2f8. Modified 2026-10-29.
See third_party/magic-formula-tyre-library for provenance.
"""
from dataclasses import dataclass, field, replace
from typing import ClassVar
import numpy as np

SCALING_NAMES = (
    "LFZO LCX LMUX LEX LKX LHX LVX LCY LMUY LEY LKY LKYC LKZC LHY LVY "
    "LTR LRES LXAL LYKA LVYKA LS LMX LVMX LMY LMP"
).split()


def matlab_eps(value):
    """MATLAB eps(x): positive floating-point spacing at abs(x)."""
    return np.spacing(np.abs(np.asarray(value, dtype=float)))


def signed_epsilon(value):
    """(4.E39)/(4.E26): retain sign; sign(0) is taken as +1."""
    value = np.asarray(value, dtype=float)
    return matlab_eps(value) * np.where(value < 0, -1.0, 1.0)


@dataclass
class CalculationInput:
    """FZ [N], SA/IA [deg], SL [-], pressure [Pa], V [m/s].

    Ro defaults to UNLOADED_RADIUS [m]; an explicit legacy override is
    supported, but measured loaded radius RL is not equivalent to Ro.
    Missing SL/IA mean zero. TIR operating limits are not silently clamped.
    """
    FZ: object
    SA: object
    SL: object = None
    IA: object = None
    pressure: object = None
    V: object = None
    Ro: object = None
    Fx: object = None
    Fy: object = None
    tir_params: dict = field(default_factory=dict)

    # Legacy names retained. Formula denominators use scale-aware eps(x).
    EPSILON_K: ClassVar[float] = 1e-6
    EPSILON_Y: ClassVar[float] = 1e-6
    EPSILON_X: ClassVar[float] = 1e-6
    epsilonK: ClassVar[float] = 1e-6
    EPSILON: ClassVar[float] = 1e-6

    def __post_init__(self):
        reserved = {"FZ", "SA", "SL", "IA", "pressure", "V", "Ro", "Fx", "Fy", "tir_params"}
        for name in SCALING_NAMES:
            setattr(self, name, 1.0)
        for name, value in self.tir_params.items():
            if name in reserved or name.startswith("_"):
                raise ValueError(f"TIR parameter conflicts with input: {name}")
            setattr(self, name, value)
        for name in ("FNOMIN", "NOMPRES", "LONGVL", "UNLOADED_RADIUS"):
            if not hasattr(self, name):
                raise ValueError(f"Missing required tire parameter: {name}")
            if not np.isfinite(float(getattr(self, name))) or float(getattr(self, name)) <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if not np.isfinite(self.LFZO) or self.LFZO <= 0:
            raise ValueError("LFZO must be finite and positive")
        self.PIO = self.NOMPRES
        values = [self.FZ, self.SA, 0.0 if self.SL is None else self.SL,
                  0.0 if self.IA is None else self.IA,
                  self.NOMPRES if self.pressure is None else self.pressure,
                  self.LONGVL if self.V is None else self.V,
                  self.UNLOADED_RADIUS if self.Ro is None else self.Ro]
        try:
            arrays = np.broadcast_arrays(*[np.atleast_1d(np.asarray(v, dtype=float)) for v in values])
        except ValueError as exc:
            raise ValueError("FZ/SA/SL/IA/pressure/V/Ro must have broadcastable shapes") from exc
        for name, value in zip(("FZ", "SA", "SL", "IA", "pressure", "V", "Ro"), arrays):
            if not np.all(np.isfinite(value)):
                raise ValueError(f"{name} contains non-finite values")
            setattr(self, name, value)
        if np.any(self.FZ < 0) or np.any(self.pressure <= 0) or np.any(self.Ro <= 0):
            raise ValueError("FZ must be nonnegative; pressure and Ro must be positive")
        for name in ("Fx", "Fy"):
            if getattr(self, name) is not None:
                value = np.broadcast_to(np.asarray(getattr(self, name), dtype=float), self.FZ.shape)
                if not np.all(np.isfinite(value)):
                    raise ValueError(f"{name} contains non-finite values")
                setattr(self, name, value)

    def with_inputs(self, **changes):
        """Clone a condition without mutating this input or tire parameters."""
        return replace(self, **changes)


class MF612:
    """MF6.1.2 steady-state formulas; turn-slip zeta factors are fixed to 1."""

    @staticmethod
    def _normalize_load(MF):
        # (4.E1), (4.E2a)
        Fz0 = MF.LFZO * MF.FNOMIN
        return Fz0, (MF.FZ - Fz0) / Fz0

    @staticmethod
    def _normalize_pressure(MF):
        # (4.E2b)
        dpi = (MF.pressure - MF.NOMPRES) / MF.NOMPRES
        return dpi, dpi**2

    @staticmethod
    def _normalize_angles(SA, IA):
        # (4.E3), (4.E4); direction is supplied by _alpha_star.
        gamma = np.sin(np.deg2rad(IA))
        return np.tan(np.deg2rad(SA)), gamma, gamma**2

    @staticmethod
    def _alpha_star(MF, *, standstill_forward=False):
        # (4.E3). Reference Fx treats sign(Vcx=0) as +1.
        direction = np.sign(MF.V)
        if standstill_forward:
            direction = np.where(direction == 0, 1.0, direction)
        return np.tan(np.deg2rad(MF.SA)) * direction

    @staticmethod
    def _alpha_cos(MF):
        # (4.E6a), (4.E6)
        Vcy = -MF.V * np.tan(np.deg2rad(MF.SA))
        Vc = np.hypot(MF.V, Vcy)
        return MF.V / (Vc + matlab_eps(Vc))
