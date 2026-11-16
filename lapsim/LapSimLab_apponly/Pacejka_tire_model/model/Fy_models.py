# SPDX-License-Identifier: GPL-3.0-only
"""MF6.1.2 lateral forces, retaining reference equation numbers.

Reference: teasit/magic-formula-tyre-library, commit
e9f0354b9bad910bfdb1f2b4342edc65fef8c2f8. Modified 2026-10-29.
Public angles are degrees. Pure Fy follows reference radian alpha and
sign(alpha). Turn-slip ZETA0..ZETA4 are fixed to 1.
"""
import numpy as np
from ._common import MF612, signed_epsilon


def fy0_terms(MF):
    gammaAst = np.sin(np.deg2rad(MF.IA))  # (4.E4)
    gamma2 = gammaAst**2
    Fz0, dfz = MF612._normalize_load(MF)  # (4.E1), (4.E2a)
    dpi, dpi2 = MF612._normalize_pressure(MF)  # (4.E2b)
    Cy = MF.LCY * MF.PCY1  # (4.E21)
    # (4.E23): LMUY directly, without an asterisk variant.
    muy = (MF.PDY1 + MF.PDY2 * dfz) * (1 + MF.PPY3 * dpi + MF.PPY4 * dpi2) * (1 - MF.PDY3 * gamma2) * MF.LMUY
    Dy = muy * MF.FZ  # (4.E22)
    load_ratio = MF.FZ / Fz0 / ((MF.PKY2 + MF.PKY5 * gamma2) * (1 + MF.PPY2 * dpi))
    Kya = MF.PKY1 * Fz0 * (1 + MF.PPY1 * dpi) * (1 - MF.PKY3 * np.abs(gammaAst)) * np.sin(MF.PKY4 * np.arctan(load_ratio)) * MF.LKY  # (4.E25)
    Kya_prime = Kya + signed_epsilon(Kya)  # (4.E39)
    SVyg = MF.LKYC * MF.LMUY * MF.FZ * (MF.PVY3 + MF.PVY4 * dfz) * gammaAst  # (4.E28)
    Kyg0 = MF.FZ * (MF.PKY6 + MF.PKY7 * dfz) * (1 + MF.PPY5 * dpi) * MF.LKYC  # (4.E30)
    SVy = MF.LMUY * MF.LVY * MF.FZ * (MF.PVY1 + MF.PVY2 * dfz) + SVyg  # (4.E29)
    SHy = MF.LHY * (MF.PHY1 + MF.PHY2 * dfz) + (Kyg0 * gammaAst - SVyg) / Kya_prime  # (4.E27)
    alpha = np.deg2rad(MF.SA)
    alphay = alpha + SHy  # (4.E20): radian alpha as in reference Fy0.m.
    Ey = (MF.PEY1 + MF.PEY2 * dfz) * (1 + MF.PEY5 * gamma2 - (MF.PEY3 + MF.PEY4 * gammaAst) * np.sign(alpha)) * MF.LEY  # (4.E24)
    By = Kya / (Cy * Dy + signed_epsilon(Cy))  # (4.E26)
    arg = By * alphay
    Fy0 = Dy * np.sin(Cy * np.arctan(arg - Ey * (arg - np.arctan(arg)))) + SVy  # (4.E19)
    return dict(Fy0=Fy0, muy=muy, dfz=dfz, Fz0=Fz0, dpi=dpi, By=By, Cy=Cy,
                Ey=Ey, Dy=Dy, Kya=Kya, Kya_prime=Kya_prime, SVy=SVy, SHy=SHy,
                SVyg=SVyg, Kyg0=Kyg0, gammaAst=gammaAst)


def fy_terms(MF):
    terms = fy0_terms(MF)
    gammaAst = terms["gammaAst"]  # (4.E4)
    alphaAst = MF612._alpha_star(MF)  # (4.E3)
    Byk = MF.LYKA * (MF.RBY1 + MF.RBY4 * gammaAst**2) * np.cos(np.arctan(MF.RBY2 * (alphaAst - MF.RBY3)))  # (4.E62)
    Cyk = MF.RCY1  # (4.E63)
    DVyk = terms["muy"] * MF.FZ * (MF.RVY1 + MF.RVY2 * terms["dfz"] + MF.RVY3 * gammaAst) * np.cos(np.arctan(MF.RVY4 * alphaAst))  # (4.E67)
    Eyk = MF.REY1 + MF.REY2 * terms["dfz"]  # (4.E64)
    SHyk = MF.RHY1 + MF.RHY2 * terms["dfz"]  # (4.E65)
    SVyk = MF.LVYKA * DVyk * np.sin(MF.RVY5 * np.arctan(MF.RVY6 * MF.SL))  # (4.E66)
    kappaS = MF.SL + SHyk  # (4.E61)

    def gain(kappa):
        arg = Byk * kappa
        return np.cos(Cyk * np.arctan(arg - Eyk * (arg - np.arctan(arg))))

    Gyk0 = gain(SHyk)  # (4.E60)
    Gyk = gain(kappaS) / Gyk0  # (4.E59)
    Fy = Gyk * terms["Fy0"] + SVyk  # (4.E58)
    return terms | dict(Fy=Fy, Gyk=Gyk, Byk=Byk, Eyk=Eyk, SVyk=SVyk)


class MF612_Fy0(MF612):
    @staticmethod
    def calculate(MF):
        return fy0_terms(MF)["Fy0"]


class MF612_Fy(MF612):
    @staticmethod
    def calculate(MF):
        return fy_terms(MF)["Fy"]
