import numpy as np


def piston_area(diameter):
    radius = diameter / 2

    return np.pi * radius**2


def brake(v,car):#煞車減速狀態(4車輪)#========================================================(for import)
    #設定參數
    bb_f,bb_r = car.bb#設定balance bar大小
    D_mc_f,D_mc_r = car.mc#設定mc直徑
    mu_Disc,D_Disc = car.Disc#設定碟盤參數
    D_caliper_f,n_caliper_f,D_caliper_r,n_caliper_r = car.caliper #設定卡鉗參數
    #計算力量
    mu_Disc = mu_Disc*np.sign(v)
    F_driver =  car.F_diver
    r_Disc = D_Disc/2#碟盤半徑
    F_mc = F_driver*car.PR#計算在mc上面的受力
    P_mc_f = F_mc*bb_f/piston_area(D_mc_f)#加上bb和mc壓力
    P_mc_r = F_mc*bb_r/piston_area(D_mc_r)#加上bb和mc壓力
    A_caliper_f = n_caliper_f*piston_area(D_caliper_f)
    A_caliper_r = n_caliper_r*piston_area(D_caliper_r)
    F_caliper_f = P_mc_f*mu_Disc*A_caliper_f#煞車夾持力量
    F_caliper_r = P_mc_r*mu_Disc*A_caliper_r
    tau_wheel_f = F_caliper_f*r_Disc#計算出前後輪上力
    tau_wheel_r = F_caliper_r*r_Disc
    return -tau_wheel_f,-tau_wheel_f,-tau_wheel_r,-tau_wheel_r#前後分別兩個輪子的煞車力
