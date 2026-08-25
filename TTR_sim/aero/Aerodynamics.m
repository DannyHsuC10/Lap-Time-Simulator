% aero
classdef Aerodynamics < handle
    properties
        Cl_aero
        Cd_aero
        COP
        data
        rho
        A
        Cl
        
    end
    
    methods
        % ===== 建構：讀 Excel =====
        function obj = Aerodynamics(path)
            aero = load_data(path);
            obj.data = aero;
            obj.rho = aero.rho;
            obj.A = aero.A;
            obj.Cl = aero.Cl;
            obj.Cl_aero = 1/2*aero.rho*aero.Cl*aero.A;
            obj.Cd_aero = 1/2*aero.rho*aero.Cd*aero.A;
        end
        
        % ===== 算扭矩 =====
        function [F,COP] = getF_air(obj,v,attitude)
            Fx_air = v.^2*obj.Cd_aero;
            Fz_air = v.^2*obj.Cl_aero;
            Fy_air = 0;

            COP = [0,0.155,0.289];% 空力重心先設定 這個可以變動
            
            F = [Fx_air,Fy_air,Fz_air];
        end
    end
end

