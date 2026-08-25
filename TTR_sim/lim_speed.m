function v_max = find_max_speed_by_curvature(kappa, car, aero, tire, motor, brake)
% =====================================================
% Find maximum stable speed under given curvature
%
% This uses FULL vehicle_model and detects limit
% by increasing speed until failure or instability
%
% INPUT:
%   kappa  - curvature (1/m)
%
% OUTPUT:
%   v_max  - maximum stable speed (m/s)
% =====================================================

    % initial conditions
    v = 0.1;           % start speed (avoid zero)
    dv = 0.5;          % speed sweep step (m/s)

    state.power = 0;
    state.attitude = [0,0,0];
    a_prev = 0;

    v_last_valid = v;

    max_iter = 500;

    for i = 1:max_iter

        ay = v^2 * kappa;

        % call full vehicle model
        [state,a] = vehicle_model(a_prev, v, ay,state, car,aero,motor,"A",tire,brake);

        % ===== FAILURE CHECK =====
        if ~isfinite(a) || any(isnan(a)) || any(isinf(a))
            break;
        end

        if isfield(state, "Fx_tire_4w")
            if any(isnan(state.Fx_tire_4w))
                break;
            end
        end

        if isfield(state, "Fx_motor_4w")
            if any(isnan(state.Fx_motor_4w))
                break;
            end
        end

        % ===== SAFE → store state =====
        v_last_valid = v;

        a_prev = a;

        % next speed
        v = v + dv;
    end

    v_max = v_last_valid;
end


%%
clc;
clear;
close all;

addpath("track")
addpath("power")
addpath("Vehicle_Dynamics")
addpath("braking")
addpath("MF62_tir_model")
addpath("aero")

aero = Aerodynamics('aero_data.csv');
car = Vehicle_data();
tire = tir_model(car);
motor = MotorModel(car);
brake = Braking_system_v1(car);

segments = load_segments("input_map.xlsx");

ds = 0.01;
track = build_track(segments, ds);
feature = find_track_features(track);
v_limit = zeros(length(feature.idx),1);

for i = 1:length(feature.idx)
    v_limit(i) = find_max_speed_by_curvature( ...
        feature.kappa(i), car, aero, tire, motor, brake);
    disp(v_limit(i))
end

figure;
plot(feature.idx, v_limit*3.6, 'LineWidth', 1.5);
grid on;
