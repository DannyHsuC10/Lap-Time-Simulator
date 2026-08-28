classdef Vehicle_data
    properties
        mass
        balance_bar
        r_disc_o
        d_gap
        D_mc_f
        D_mc_r
        N_caliper_f
        N_caliper_r
        D_caliper_f
        D_caliper_r
        PR
        mu_pad
        gear_rate
        Braking_rate
        P_tir
        camber
        h
        L_front
        L_rear
        K_roll
        K_roll_f
        K_roll_r
        K_pitch
        K_heave
        K_heave_f
        K_heave_r
        t_front
        t_rear
        h_rc_f
        h_rc_r
        h_rc
        L
        r_w
        F_driver
        crr
        Cl_aero
        mu_w
    end
    
    methods
        % 建構子：初始化車輛參數
        function obj = Vehicle_data()
            obj.mass = 360;% 質量(kg)

            obj.balance_bar = 0.7;% BB
            obj.r_disc_o = 0.21;% 碟盤半徑(m)
            obj.d_gap = 0.02;% 偏移(m)
            obj.D_mc_f = 0.012;% 前MC直徑(m)
            obj.D_mc_r = 0.014;% 後MC直徑(m)
            obj.N_caliper_f = 2;% 對N前
            obj.N_caliper_r = 2;% 對N後
            obj.D_caliper_f = 0.034;% 前卡前活塞直徑(m)
            obj.D_caliper_r = 0.034;% 後卡前活塞直徑(m)
            obj.PR = 2;% 踏板比
            obj.mu_pad = 0.55;% 來令片摩擦係數
            obj.Braking_rate = 0.6908;%直接建立比例 腳力>>煞車扭矩

            obj.gear_rate = 15; % 齒輪比

            obj.P_tir = 82319;% 胎壓
            obj.camber = 0;% camber角度(直接預設0)

            obj.h = 0.286;% 重心高度(m)
            
            obj.L_front = 0.806;% 重心到前軸距離(m)
            obj.L_rear  = 0.744;% 重心到後軸距離(m)
            obj.L = obj.L_front+obj.L_rear;
            
            obj.t_front = 1.30;% 前輪輪距(m)
            obj.t_rear  = 1.25;% 後輪輪距(m)
            
            obj.K_heave_f = 64500;% heave 剛性
            obj.K_heave_r = 56500;% heave 剛性

            obj.K_roll_f  = 5000;% roll 剛性
            obj.K_roll_r  = 5000;% roll 剛性
            obj.h_rc_f = 0.055;% roll centor(m)
            obj.h_rc_r = 0.044;% roll centor(m)
            
            obj.K_pitch = 8000;% pitch 剛性
            
            obj.r_w = 0.259;% 輪胎工作半徑(m假設相同)
            obj.F_driver = 500;%車手出力踏板(N)
            obj.crr = 0.01;% 滾動阻力

            obj.mu_w = 1.5;
            obj.Cl_aero = -0.86289;

        end
    end
end