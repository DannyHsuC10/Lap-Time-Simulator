clc;
clear;

close all;
addpath("power")
addpath("Vehicle_Dynamics")
addpath("braking")
addpath("MF62_tir_model")
addpath("vechiles")
addpath("aero")

aero = Aerodynamics('aero_data.csv');
car = Vehicle_data();
tire = tir_model(car);
motor = MotorModel(car);
brake = Braking_system_v1(car);

sim = section_sim(aero,car,tire,motor,brake);

%% =====================================================
% Initial condition
%% =====================================================

t = 0;
s = 0;
a = 0;

state.attitude = [0,0,0];
state.power = [0,0,0,0];

%% =====================================================
% set
%% =====================================================
v = 7.8;         % vi設定
r  = 8.5;     % straight
L  = r*2*pi;     % 100 m
vf = inf;       % stop at end


%% =====================================================
% Simulation
%% =====================================================
sim.brake_flag = "A";
while s < L

    [t,s,v,a,state] = sim.section_run( ...
        t,...
        s,...
        v,...
        a,...
        state,...
        r,...
        L-s,...
        vf);
        %disp(["ay : ",state.ayc])
end

disp("Simulation Finished")

%% =====================================================
% Plot
%% =====================================================

figure;
plot(sim.s_list,sim.v_list*3.6,'LineWidth',2);
xlabel('Distance (m)');
ylabel('Speed (km/h)');
title('s vs v');
grid on;

figure;
plot(sim.t_list,sim.s_list*3.6,'LineWidth',2);
xlabel('Time (s)');
ylabel('Distance (m)');
title('s vs t');
grid on;

figure;
plot(sim.t_list,sim.v_list*3.6,'LineWidth',2);
xlabel('Time (s)');
ylabel('Speed (km/h)');
title('v vs t');
grid on;

figure;
plot(sim.t_list,sim.a_list/9.81,'LineWidth',2);
xlabel('Time (s)');
ylabel('Acceleration (g)');
title('a vs t');
grid on;
