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
        
            if any(Fz <= 0)
                Fx = zeros(size(Fz));
                return;
            end
        
            Fx_limit = sqrt(max(0,(obj.mu.*Fz).^2 - Fy.^2));
        
            switch upper(direction)
        
                case 'A'      % Acceleration
                    Fx = Fx_limit;
        
                case 'B'      % Braking
                    Fx = -Fx_limit;
        
                otherwise
                    Fx = zeros(size(Fz));
            end
        
        end
    end
end
