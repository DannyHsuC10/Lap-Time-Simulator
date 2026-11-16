classdef Braking_system_v2 < handle
    properties
        Braking_rate
    end
    
    methods
        % ===== 建構：讀 Excel =====
        function obj = Braking_system_v2(car)
            % balance_bar


            bb_r = 1-car.balance_bar;
            bb_f = car.balance_bar;
            
            % 有效半徑
            r_disc = car.r_disc_o / 2 - car.d_gap;
            
            % 面積計算
            A_mc_f = pi * (car.D_mc_f / 2)^2;
            A_mc_r = pi * (car.D_mc_r / 2)^2;
            A_caliper_f = car.N_caliper_f * pi * (car.D_caliper_f / 2)^2;
            A_caliper_r = car.N_caliper_r * pi * (car.D_caliper_r / 2)^2;
            
            % 轉換成一個固定的單位筆預先計算
            R_f = r_disc*car.PR*bb_f*A_caliper_f*car.mu_pad/A_mc_f;
            R_r = r_disc*car.PR*bb_r*A_caliper_r*car.mu_pad/A_mc_r;
            obj.Braking_rate = [R_f,R_f,R_r,R_r];%Braking_rate: [1.0508 1.0508 0.3309 0.3309]
             disp("Brake loading complete")
        end
        
        % ===== 算扭矩 =====
        function T = getTorque(obj,F)
            T = F*obj.Braking_rate;
        end

        
    end
end

