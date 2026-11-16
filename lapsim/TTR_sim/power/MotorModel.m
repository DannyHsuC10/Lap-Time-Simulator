classdef MotorModel < handle
    properties
        rpm
        torque
        e
        interp_method = 'linear'
    end
    
    methods
        % ===== 建構：讀 Excel =====
        function obj = MotorModel(car)
            data = readtable('motor_table.xlsx');
            obj.e = car.gear_rate;
            obj.rpm = data{:,1};
            obj.torque = data{:,2};
            disp("motor loading complete")
        end
        
        % ===== 查扭矩 =====
        function T = getTorque(obj, omega,power)
            rpm_query = omega*60/2/pi*obj.e;
            T_motor = interp1( ...
                obj.rpm, ...
                obj.torque, ...
                rpm_query, ...
                obj.interp_method, ...
                'extrap');
            T = T_motor*obj.e;
        end
    end
end