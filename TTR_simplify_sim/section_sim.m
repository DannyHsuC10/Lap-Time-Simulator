classdef section_sim < handle
    properties
        aero
        car
        tire
        motor
        brake
        ds
        t_list
        s_list
        v_list
        a_list
        g
        brake_flag
    end

    methods

        function obj = section_sim(aero,car,tire,motor,brake)
            addpath("power")
            addpath("Vehicle_Dynamics")
            addpath("braking")
            addpath("MF62_tir_model")
            addpath("vechiles")
            addpath("aero")
            obj.aero = aero;
            obj.car = car;
            obj.tire = tire;
            obj.motor = motor;
            obj.brake = brake;
            obj.ds = 0.01;
            obj.t_list = [];
            obj.s_list = [];
            obj.v_list = [];
            obj.a_list = []; % Log acceleration
            obj.g = 9.81;
            obj.brake_flag = "A";

        end

        function [t,s,v,a] = section_run(obj,t,s,v,r,L,vf)

            %% logs            
            ay = v^2/r;
            ab = vehicle_model(v, ay, obj.car,obj.aero,obj.motor,"B",obj.tire,obj.brake);
            bs = (v^2-vf^2)/2/abs(ab);

            if bs>L && v>vf 
                obj.brake_flag = "B";
            end
            

            if obj.brake_flag == "B" % 如果需要煞車就煞車
                ay = v^2/r;
                if v>vf% 如果太快
                    a = vehicle_model(v, ay, obj.car,obj.aero,obj.motor,"B",obj.tire,obj.brake);
                else
                    a = 0;
                end
                dv = (a / v) * obj.ds;
                v = v + dv;

                dt = obj.ds / v;
                t = t + dt;
            
                s = s + obj.ds;
                if v<0
                    v = 0;
                end
                obj.s_list(end+1) = s;
                obj.v_list(end+1) = v;
                obj.a_list(end+1) = a;
                obj.t_list(end+1) = t;

                %obj.state_list = [obj.state_list;state];

                fprintf("BRK | t:%f s:%f v:%f km/h a:%f g\n", ...
                    obj.t_list(end), s, v*3.6, a/obj.g);
            
            else% 否則繼續開
                % force
                ay = v^2/r;
                a = vehicle_model(v, ay, obj.car,obj.aero,obj.motor,"A",obj.tire,obj.brake);
            
                if v < 0.1
                    v = 0.1;
                end
                
                % spatial integration
                dv = (a / v) * obj.ds;
                v = v + dv;
            
                % time increment
                dt = obj.ds / v;
                t = t + dt;
            
                % advance position
                s = s + obj.ds;
            
                % log
                obj.s_list(end+1) = s;
                obj.v_list(end+1) = v;
                obj.a_list(end+1) = a;
                obj.t_list(end+1) = t;
                
                %obj.state_list = [obj.state_list;state];


                fprintf("ACC | t:%f s:%f v:%f km/h a:%f g\n", ...
                    obj.t_list(end), s, v*3.6, a/obj.g);            
            end            
        end
     end
end