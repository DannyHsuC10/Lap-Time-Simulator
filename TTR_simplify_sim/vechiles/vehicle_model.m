function ax_out = vehicle_model(v, ay, car,aero,motor,direction,tire,brake)


    omega = v / car.r_w;
    Fyc = car.mass*ay;
    F_aero = aero.getF_air(v);% 空力
    Fz = Suspension(F_aero,car);% 呼叫懸吊
    Fx_tire = tire.lim_output(Fz,Fyc,direction);% 極限力量% 最大力量輸出
    
    if direction == "A"
        T = motor.getTorque(omega);
        Fx_motor = T / car.r_w;
        Fx = min(Fx_motor,Fx_tire);
        Fx = max(Fx,5);% 除擾動
    elseif direction =="B" 
        T = brake.getTorque(-car.F_driver);
       
        Fx_motor = T / car.r_w;
        Fx = max(Fx_motor,Fx_tire);
    else
        disp("方向必須是A or B")
    end

    Fx = Fx*4-Fz*car.crr;
    
    ax_out = Fx/car.mass;
 
end
