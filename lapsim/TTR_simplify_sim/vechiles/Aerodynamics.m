% aero
classdef Aerodynamics < handle
    properties
        Cl_aero
    end
    
    methods
        % ===== 建構：讀 Excel =====
        function obj = Aerodynamics(car)
            obj.Cl_aero = car.Cl_aero;
        end
        
        % ===== 算扭矩 =====
        function Fz_air = getF_air(obj,v)
            Fz_air = v.^2*obj.Cl_aero;

        end
    end
end

