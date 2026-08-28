classdef Braking_system < handle
    properties
        Braking_rate
    end
    
    methods
        function obj = Braking_system(car)
            obj.Braking_rate = car.Braking_rate;%直接建立比例
            disp("Brake loading complete")
        end
        
        % ===== 算扭矩 =====
        function T = getTorque(obj,F)
            T = F*obj.Braking_rate;
        end

        
    end
end

