classdef Vehicle_data
    properties
        mass
        gear_rate
        Braking_rate
        r_w
        F_driver
        Cl_aero
        mu_w
    end
    
    methods
        % 建構子：初始化車輛參數
        function obj = Vehicle_data()
            obj.mass = 320;% 質量(kg)

            obj.gear_rate = 11.2; % 齒輪比
 
            obj.r_w = 0.203;% 輪胎工作半徑(m假設相同)
            obj.mu_w = 1.77;         

            obj.Braking_rate = 1;%直接建立比例 腳力>>煞車扭矩            
            obj.F_driver = 500;%車手出力踏板(N)

            obj.Cl_aero = 2.4;

        end
    end
end