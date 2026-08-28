%% sim (Spatial Domain Lap / Straight Simulation)
clc; clear; close all;
addpath("vechiles")


car = Vehicle_data();
aero = Aerodynamics(car);
tire = tir_model(car);
motor = MotorModel(car);
brake = Braking_system(car);

%% parameters
s = 0;
a = 0;
ds = 0.01;   % 空間步長 (m)
g = 9.81;

vf = inf;
L = 75;
r = inf;
v = 0;         % vi設定

%% logs
s_list = [];
v_list = [];
a_list = [];
t_list = [];

t = 0;

% =========================
% 1. ACCELERATION (Forward in space)
% =========================
while s <= L

    % force
    ay = v^2/r;
    a = vehicle_model(v, ay,car,aero,motor,"A",tire,brake);

    if v < 0.1
        v = 0.1;
    end
    
    % spatial integration
    dv = (a / v) * ds;
    v = v + dv;

    % time increment
    dt = ds / v;
    t = t + dt;

    % advance position
    s = s + ds;

    % log
    s_list(end+1) = s;
    v_list(end+1) = v;
    a_list(end+1) = a;
    t_list(end+1) = t;

    fprintf("ACC | t:%f s:%f v:%f km/h a:%f g\n", ...
        t, s, v*3.6, a/g);

end

% =========================
% 2. BRAKING (Backward dynamics in space)
% =========================
while v > vf
    ay = v^2/r;
    a = vehicle_model(a, v, ay, car,aero,motor,"B",tire,brake);

    dv = (a / v) * ds;
    v = v + dv;
    if v<0
        break
    end
    dt = ds / v;
    t = t + dt;

    s = s + ds;

    s_list(end+1) = s;
    v_list(end+1) = v;
    a_list(end+1) = a;
    t_list(end+1) = t;

    fprintf("BRK | t:%f s:%f v:%f km/h a:%f g\n", ...
        t, s, v*3.6, a/g);


end

%% =========================
% plots
% =========================
figure;
plot(t_list,a_list);
xlabel('t');
ylabel("a");
title('a vs t');
grid on;

figure;
plot(t_list,v_list);
xlabel('t');
ylabel("v");
title('v vs t');
grid on;

figure;
plot(t_list,s_list);
xlabel('t');
ylabel("s");
title('s vs t');
grid on;
