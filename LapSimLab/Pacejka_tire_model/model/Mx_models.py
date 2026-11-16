# SPDX-License-Identifier: GPL-3.0-only
"""MF6.1.2 overturning moment (4.E69).

Reference: teasit/magic-formula-tyre-library v61, commit
e9f0354b9bad910bfdb1f2b4342edc65fef8c2f8. Modified 2026-10-29.
"""
import numpy as np
from ._common import MF612
from .Fy_models import MF612_Fy


class MF612_Mx(MF612):
    @staticmethod
    def calculate(MF):
        Fy = MF612_Fy.calculate(MF) if MF.Fy is None else MF.Fy
        gamma = np.deg2rad(MF.IA)
        dpi, _ = MF612._normalize_pressure(MF)  # (4.E2b)
        # (4.E69) uses FNOMIN rather than LFZO-scaled nominal load.
        ratioFy, ratioFz = Fy / MF.FNOMIN, MF.FZ / MF.FNOMIN
        return MF.Ro * MF.FZ * (
            MF.QSX1 * MF.LVMX - MF.QSX2 * gamma * (1 + MF.PPMX1 * dpi)
            + MF.QSX3 * ratioFy
            + MF.QSX4 * np.cos(MF.QSX5 * np.arctan(MF.QSX6 * ratioFz)**2)
            * np.sin(MF.QSX7 * gamma + MF.QSX8 * np.arctan(MF.QSX9 * ratioFy))
            + MF.QSX10 * np.arctan(MF.QSX11 * ratioFz) * gamma
        ) * MF.LMX  # (4.E69)
