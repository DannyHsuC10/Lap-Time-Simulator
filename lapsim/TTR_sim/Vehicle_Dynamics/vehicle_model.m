function [state,ax_out] = vehicle_model(ax, v, ay,state, car,aero,motor,direction,tire,brake)

    power_i = state.power;
    attitude_i = state.attitude;

    omega_list = repmat(v / car.r_w, 1, 4);


    Fyc = car.mass*ay;
    [F_aero,COP] = aero.getF_air(v,attitude_i);% 空力
    [Fz_4w,attitude_f,dz,~] = Suspension_3dof(ax,ay,F_aero,COP,car);% 呼叫懸吊
    % Calculate the resultant forces and return the output
    [Fx_tire,Fy_4w] = Power_distribution(Fyc,Fz_4w,car,direction,tire);% 最大力量輸出

    if direction == "A"
        T_list = motor.getTorque(omega_list,power_i);
        Fx_motor = T_list / car.r_w;
        Fx_4w = min(Fx_motor,Fx_tire);
        Fx_4w = max(Fx_4w,30);
    elseif direction =="B" 
        T_list = brake.getTorque(-car.F_driver);
        Fx_motor = T_list / car.r_w;
        Fx_4w = max(Fx_motor,Fx_tire);
    else
        disp("方向必須是A or B")
    end


    Fz = sum(Fz_4w);
    Fy = sum(Fy_4w);
    Fx = sum(Fx_4w)-sum(Fz_4w)*car.crr+F_aero(1);
    %disp([sum(Fx_4w),-sum(Fz_4w)*car.crr,F_aero(1)]) % 驗證

    P = Fx_4w.*omega_list;
    ax_out = Fx/car.mass;

    %% 參數封裝
    state.Fx = Fx;
    state.Fy = Fy;
    state.Fz = Fz;
    state.attitude = attitude_f;
    state.dz = dz;
    state.omega_list = omega_list;
    state.F_aero = F_aero;
    state.Fz_4w = Fz_4w;
    state.Fy_4w = Fy_4w;
    state.Fx_4w = Fx_4w;
    state.Fx_tire_4w = Fx_tire;
    state.Fx_tire = sum(Fx_tire);
    state.Fx_motor_4w = Fx_motor;
    state.Fx_motor = sum(Fx_motor); 
    state.ayc = ay;
    state.ax = ax_out;
    state.Fyc = Fyc;
    state.power = P;


 
end

function [Fx,Fy]= Power_distribution(Fyc,Fz,car,direction,tire)% 力量分配
    % 力矩進行力量分配
    Fyf = Fyc * car.L_rear / car.L;
    Fyr = Fyc * car.L_front / car.L;

    % 正向力進行力量分配
    Fzfl = Fz(1);
    Fzfr = Fz(2);
    Fzrl = Fz(3);
    Fzrr = Fz(4);

    Fzf = Fzfl+Fzfr;% 前
    Fzr = Fzrl+Fzrr;% 後

    Fyfl = Fyf*Fzfl/Fzf;
    Fyfr = Fyf*Fzfr/Fzf;
    Fyrl = Fyr*Fzrl/Fzr;
    Fyrr = Fyr*Fzrr/Fzr;
    
    Fy = [Fyfl,Fyfr,Fyrl,Fyrr];%全部

    Fx = tire.lim_output_ellipse(Fz,Fy,direction);% 極限力量
    Fxfl = Fx(1);
    Fxfr = Fx(2);
    Fxrl = Fx(3);
    Fxrr = Fx(4);

    % x方向平衡(最小值防止超出極限)
    Fxf = min(Fxfl,Fxfr);
    Fxr = min(Fxrl,Fxrr);
    Fx = [Fxf,Fxf,Fxr,Fxr];%前後力量x方向可以不同但左右平衡
end