# SPDX-License-Identifier: GPL-3.0-only
"""MF6.1.2 aligning moments; reference equation numbers retained.

Reference: teasit/magic-formula-tyre-library, commit
e9f0354b9bad910bfdb1f2b4342edc65fef8c2f8. Modified 2026-10-29.
Documented correction: (4.E76) uses actual sin(IA); upstream Mz.m
accidentally reuses gammaAst returned by its zero-camber Fy calculation.
All turn-slip zeta factors are 1; this is not a parking/turn-slip model.
"""
import numpy as np
from ._common import MF612
from .Fx_models import fx0_terms, MF612_Fx
from .Fy_models import fy0_terms, fy_terms, MF612_Fy


def mz0_terms(MF):
    alphaAst = MF612._alpha_star(MF)  # (4.E3)
    alphaCos = MF612._alpha_cos(MF)  # (4.E6a), (4.E6)
    direction = np.sign(MF.V)
    gamma = np.sin(np.deg2rad(MF.IA))  # (4.E4)
    # Reference Mz0 uses zero-camber Fy0 for the trail/residual base.
    lateral = fy0_terms(MF.with_inputs(IA=0.0))
    Fz0, dfz, dpi = lateral["Fz0"], lateral["dfz"], lateral["dpi"]
    SHf = lateral["SHy"] + lateral["SVy"] / lateral["Kya_prime"]  # (4.E38), (4.E39)
    SHt = MF.QHZ1 + MF.QHZ2 * dfz + (MF.QHZ3 + MF.QHZ4 * dfz) * gamma  # (4.E35)
    alphat = alphaAst + SHt  # (4.E34)
    alphar = alphaAst + SHf  # (4.E37)
    Dt0 = MF.FZ * (MF.Ro / Fz0) * (MF.QDZ1 + MF.QDZ2 * dfz) * (1 - MF.PPZ1 * dpi) * MF.LTR * direction  # (4.E42)
    # (4.E40): QBZ4 retained; QBZ6 excluded per MF6.1.2 reference.
    Bt = (MF.QBZ1 + MF.QBZ2 * dfz + MF.QBZ3 * dfz**2) * (1 + MF.QBZ4 * gamma + MF.QBZ5 * np.abs(gamma)) * MF.LKY / MF.LMUY
    Ct = MF.QCZ1  # (4.E41)
    Dt = Dt0 * (1 + MF.QDZ3 * np.abs(gamma) + MF.QDZ4 * gamma**2)  # (4.E43)
    Et = (MF.QEZ1 + MF.QEZ2 * dfz + MF.QEZ3 * dfz**2) * (1 + (MF.QEZ4 + MF.QEZ5 * gamma) * (2 / np.pi) * np.arctan(Bt * Ct * alphat))  # (4.E44)
    Br = MF.QBZ9 * MF.LKY / MF.LMUY + MF.QBZ10 * lateral["By"] * lateral["Cy"]  # (4.E45)
    Cr = 1.0  # (4.E46), ZETA7
    Dr = MF.FZ * MF.Ro * ((MF.QDZ6 + MF.QDZ7 * dfz) * MF.LRES + ((MF.QDZ8 + MF.QDZ9 * dfz) * (1 + MF.PPZ2 * dpi) + (MF.QDZ10 + MF.QDZ11 * dfz) * np.abs(gamma)) * gamma * MF.LKZC) * MF.LMUY * direction * alphaCos  # (4.E47)
    arg = Bt * alphat
    t0 = Dt * np.cos(Ct * np.arctan(arg - Et * (arg - np.arctan(arg)))) * alphaCos  # (4.E33)
    Mz0_trail = -t0 * lateral["Fy0"]  # (4.E32)
    Mzr0 = Dr * np.cos(Cr * np.arctan(Br * alphar)) * alphaCos  # (4.E36)
    Mz0 = Mz0_trail + Mzr0  # (4.E31)
    return dict(Mz0=Mz0, Bt=Bt, Ct=Ct, Dt=Dt, Et=Et, Br=Br, Cr=Cr, Dr=Dr,
                alphaCos=alphaCos, alphar=alphar, alphat=alphat, dfz=dfz, Fz0=Fz0,
                gammaAst=gamma)


def mz_terms(MF):
    terms = mz0_terms(MF)
    Kxk = fx0_terms(MF)["Kxk"]
    Kya_prime = fy0_terms(MF)["Kya_prime"]
    lateral_zero = fy_terms(MF.with_inputs(IA=0.0))
    stiffness_slip = (Kxk / Kya_prime)**2 * MF.SL**2
    ar, at = terms["alphar"], terms["alphat"]
    alpharEq = np.sqrt(ar**2 + stiffness_slip) * np.sign(ar)  # (4.E78); alternate (A54) not used.
    alphatEq = np.sqrt(at**2 + stiffness_slip) * np.sign(at)  # (4.E77); alternate (A55) not used.
    Fx = MF612_Fx.calculate(MF) if MF.Fx is None else MF.Fx
    Fy = MF612_Fy.calculate(MF) if MF.Fy is None else MF.Fy
    # (4.E76): actual gammaAst, not the zero-camber helper's gammaAst.
    arm = MF.Ro * (MF.SSZ1 + MF.SSZ2 * Fy / terms["Fz0"] + (MF.SSZ3 + MF.SSZ4 * terms["dfz"]) * terms["gammaAst"]) * MF.LS
    Mzr = terms["Dr"] * np.cos(terms["Cr"] * np.arctan(terms["Br"] * alpharEq)) * terms["alphaCos"]  # (4.E75)
    Fy_prime = lateral_zero["Gyk"] * lateral_zero["Fy0"]  # (4.E74), excluding SVyk.
    arg = terms["Bt"] * alphatEq
    trail = terms["Dt"] * np.cos(terms["Ct"] * np.arctan(arg - terms["Et"] * (arg - np.arctan(arg)))) * terms["alphaCos"]  # (4.E73)
    Mzt = -trail * Fy_prime  # (4.E72)
    Mz = Mzt + Mzr + arm * Fx  # (4.E71)
    return terms | dict(Mz=Mz, Mzt=Mzt, Mzr=Mzr, arm=arm)


class MF612_Mz0(MF612):
    @staticmethod
    def calculate(MF):
        return mz0_terms(MF)["Mz0"]


class MF612_Mzt(MF612):
    @staticmethod
    def calculate(MF):
        return mz_terms(MF)["Mzt"]


class MF612_Mz(MF612):
    @staticmethod
    def calculate(MF):
        return mz_terms(MF)["Mz"]


# Retain historic import name while using one tested combined formulation.
MF612_Mz_simplify = MF612_Mz
