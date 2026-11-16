# SPDX-License-Identifier: GPL-3.0-only
"""MF6.1.2 rolling moment My; RRT is its backwards-compatible alias.

Reference: teasit/magic-formula-tyre-library v61, commit
e9f0354b9bad910bfdb1f2b4342edc65fef8c2f8. Modified 2026-10-29.
"""
import numpy as np
from ._common import MF612
from .Fx_models import MF612_Fx


class MF612_My(MF612):
    @staticmethod
    def calculate(MF):
        Fx = MF612_Fx.calculate(MF) if MF.Fx is None else MF.Fx
        gamma = np.deg2rad(MF.IA)
        ratioFz = MF.FZ / MF.FNOMIN
        ratioP = MF.pressure / MF.NOMPRES
        ratioV = MF.V / MF.LONGVL
        # (4.E70). Reference sign convention: negative forward rolling moment.
        # Element-wise powers also support vector inputs (upstream uses ^).
        bracket = (MF.QSY1 + MF.QSY2 * Fx / MF.FNOMIN
                   + MF.QSY3 * np.abs(ratioV) + MF.QSY4 * ratioV**4
                   + (MF.QSY5 + MF.QSY6 * ratioFz) * gamma**2)
        # At no contact return zero, avoiding 0 * 0**negative at QSY7 < 0.
        # Positive-load calculations remain exactly (4.E70).
        load_factor = np.power(ratioFz, MF.QSY7, out=np.ones_like(ratioFz), where=ratioFz > 0)
        return -MF.FZ * MF.Ro * bracket * load_factor * ratioP**MF.QSY8 * MF.LMY


MF612_RRT_MF = MF612_My


class MF612_RRT_SAE(MF612):
    """Explicit legacy SAE empirical option, separate from MF My/RRT."""

    @staticmethod
    def calculate(MF):
        Crr = 0.7 / 1000 * (5.1 + (5.5e5 + 90 * MF.FZ) / MF.pressure
                            + (1100 + 0.0388 * MF.FZ) * MF.V**2 / MF.pressure)
        return -MF.Ro * MF.FZ * Crr
