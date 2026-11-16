# SPDX-License-Identifier: GPL-3.0-only
"""MF6.1.2 longitudinal forces; public angles are degrees.

Reference: teasit/magic-formula-tyre-library v61, commit
e9f0354b9bad910bfdb1f2b4342edc65fef8c2f8. Modified 2026-10-29.
Equation numbers retained from reference; turn-slip zeta=1.
"""
import numpy as np
from ._common import MF612, matlab_eps


def fx0_terms(MF):
    """Pure longitudinal force and intermediate terms reused by Mz."""
    Fz0, dfz = MF612._normalize_load(MF)  # (4.E1), (4.E2a)
    dpi, dpi2 = MF612._normalize_pressure(MF)  # (4.E2b)
    gamma = np.deg2rad(MF.IA)
    Cx = MF.PCX1 * MF.LCX  # (4.E11)
    # (4.E13): direct LMUX, no asterisk variant in this MF6.1.2 set.
    mux = MF.LMUX * (MF.PDX1 + MF.PDX2 * dfz) * (1 + MF.PPX3 * dpi + MF.PPX4 * dpi2) * (1 - MF.PDX3 * gamma**2)
    Dx = mux * MF.FZ  # (4.E12), ZETA1=1
    SHx = MF.LHX * (MF.PHX1 + MF.PHX2 * dfz)  # (4.E17)
    LMUX_prime = 10 * MF.LMUX / (1 + 9 * MF.LMUX)  # (4.E8), AMU=10
    SVx = LMUX_prime * MF.LVX * MF.FZ * (MF.PVX1 + MF.PVX2 * dfz)  # (4.E18)
    kappax = MF.SL + SHx  # (4.E10)
    Ex = MF.LEX * (MF.PEX1 + MF.PEX2 * dfz + MF.PEX3 * dfz**2) * (1 - MF.PEX4 * np.sign(kappax))  # (4.E14)
    Kxk = MF.LKX * MF.FZ * (MF.PKX1 + MF.PKX2 * dfz) * np.exp(MF.PKX3 * dfz) * (1 + MF.PPX1 * dpi + MF.PPX2 * dpi2)  # (4.E15)
    Bx = Kxk / (Cx * Dx + matlab_eps(Kxk))  # (4.E16)
    arg = Bx * kappax
    Fx0 = Dx * np.sin(Cx * np.arctan(arg - Ex * (arg - np.arctan(arg)))) + SVx  # (4.E9)
    return dict(Fx0=Fx0, mux=mux, Cx=Cx, Dx=Dx, Ex=Ex, Bx=Bx, Kxk=Kxk, dfz=dfz, Fz0=Fz0)


def fx_terms(MF):
    terms = fx0_terms(MF)
    alphaAst = MF612._alpha_star(MF, standstill_forward=True)  # (4.E3)
    gammaAst = np.sin(np.deg2rad(MF.IA))  # (4.E4)
    Cxa = MF.RCX1  # (4.E55)
    Exa = MF.REX1 + MF.REX2 * terms["dfz"]  # (4.E56)
    SHxa = MF.RHX1  # (4.E57)
    alphaS = alphaAst + SHxa  # (4.E53)
    Bxa = MF.LXAL * (MF.RBX1 + MF.RBX3 * gammaAst**2) * np.cos(np.arctan(MF.RBX2 * MF.SL))  # (4.E54)

    def gain(angle):
        arg = Bxa * angle
        return np.cos(Cxa * np.arctan(arg - Exa * (arg - np.arctan(arg))))

    Gxa0 = gain(SHxa)  # (4.E52)
    Gxa = gain(alphaS) / Gxa0  # (4.E51): exact normalization.
    Fx = Gxa * terms["Fx0"]  # (4.E50)
    return terms | dict(Fx=Fx, Gxa=Gxa, Bxa=Bxa, Exa=Exa)


class MF612_Fx0(MF612):
    @staticmethod
    def calculate(MF):
        return fx0_terms(MF)["Fx0"]


class MF612_Fx(MF612):
    @staticmethod
    def calculate(MF):
        return fx_terms(MF)["Fx"]
