classdef tir_model
    properties
        mu
    end
    
    methods
        function obj = tir_model(car)
            obj.mu = car.mu_w;
            disp("tir model loading complete")
        end
        function Fx = lim_output(obj,Fz,Fy,direction)
           
            % 5. 實作摩擦橢圓公式
            F_ratio = sqrt(max(0, 1 - (Fy / Fz*obj.mu).^2));
            if direction =="A"
            Fx = Fz*obj.mu * F_ratio;
            elseif direction == "B"
                Fx = -Fz*obj.mu * F_ratio;
            else
                Fx = 0; % Default case for invalid direction
            end
        end

    end
end
