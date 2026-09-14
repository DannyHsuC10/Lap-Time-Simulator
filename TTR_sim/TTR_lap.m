%%
clc;
clear;
close all;

% 路徑
addpath("track")
addpath("power")
addpath("Vehicle_Dynamics")
addpath("braking")
addpath("MF62_tir_model")
addpath("vechiles")
addpath("aero")

result_dir = fullfile(fileparts(mfilename('fullpath')), 'result');
if ~exist(result_dir, 'dir')
    mkdir(result_dir);
end

% 車輛
aero = Aerodynamics('aero_data.csv');
car = Vehicle_data();
tire = tir_model(car);
motor = MotorModel(car);
brake = Braking_system_v1(car);
sim = section_sim(aero,car,tire,motor,brake);
segments = load_segments("input_map.xlsx");

% 圖資
ds = 0.01;
track = build_track(segments, ds);
feature = find_track_features(track);
fprintf("Track length = %.2f m\n", track.s(end));
fprintf("Track points = %d\n", track.n);

% Initial condition
t = 0;
s = 0;
a = 0;
v = 0;
state.attitude = [0,0,0];
state.power = [0,0,0,0];

%% =====================================================
% Track geometry
%% =====================================================
fig_track_geometry = figure;
hold on
grid on
axis equal
axis tight

plot(track.x, track.y, "w-", "LineWidth", 1.5);

scatter(feature.x,...
        feature.y,...
        90,...
        'r',...
        'filled');

% Label
for i = 1:length(feature.idx)

    % text(feature.x(i),...% 位置
    %      feature.y(i),...
    %      sprintf('%.0fm',feature.idx(i)*ds),...
    %      'Color','r',...
    %      'FontWeight','bold',...
    %      'VerticalAlignment','bottom');

    text(feature.x(i),...% 彎道編號
         feature.y(i),...
         sprintf('%.0fm',i),...
         'Color','y',...
         'FontWeight','bold',...
         'VerticalAlignment','bottom');


end

axis equal;
grid on;
xlabel("X (m)");
ylabel("Y (m)");
title("Track Geometry");

%% =====================================================
% sim
%% =====================================================

i = 1;

for section = 1:length(feature.idx)
    L = feature.idx(section)*ds;

    if section < length(feature.idx)% 預測
        r_section = 1/abs(feature.kappa(section+1));
        vf = (17./abs(feature.kappa(section+1))).^0.5;
    else
        r_section = 1/abs(feature.kappa(1));
        vf = (17./abs(feature.kappa(1))).^0.5;
    end
    
    fprintf("L: %fm, vf: %fkm/h, r: %fm\n",L,vf*3.6,r_section)
    sim.brake_flag = "A";
    while s < L
        r = abs(1./track.kappa(i));
        [t,s,v,a,state] = sim.section_run( ...
            t,...
            s,...
            v,...
            a,...
            state,...
            r,...
            L-s,...
            vf);
    
    i = i+1;
    end

end
%% =====================================================
% plot
%% =====================================================
fig_s_vs_v = figure;
plot(sim.s_list,sim.v_list*3.6,'LineWidth',2);
xlabel('Distance (m)');
ylabel('Speed (km/h)');
title('s vs v');
grid on;

fig_s_vs_t = figure;
plot(sim.t_list,sim.s_list*3.6,'LineWidth',2);
xlabel('Time (s)');
ylabel('Distance (m)');
title('s vs t');
grid on;

fig_v_vs_t = figure;
plot(sim.t_list,sim.v_list*3.6,'LineWidth',2);
xlabel('Time (s)');
ylabel('Speed (km/h)');
title('v vs t');
grid on;

fig_a_vs_t = figure;
plot(sim.t_list,sim.a_list/9.81,'LineWidth',2);
xlabel('Time (s)');
ylabel('Acceleration (g)');
title('a vs t');
grid on;


%% =====================================================
% Speed Map
%% =====================================================

% 將 section_sim 的結果插值回整條賽道
v_map = interp1( ...
    sim.s_list,...
    sim.v_list,...
    track.s,...
    'linear',...
    'extrap');

a_map = interp1( ...
    sim.s_list,...
    sim.a_list,...
    track.s,...
    'linear',...
    'extrap');

t_map = interp1( ...
    sim.s_list,...
    sim.t_list,...
    track.s,...
    'linear',...
    'extrap');

fig_vehicle_speed_map = figure;
hold on;
grid on;
axis equal;
axis tight;

scatter( ...
    track.x,...
    track.y,...
    20,...
    v_map*3.6,...
    'filled');

colormap(turbo);

cb = colorbar;
ylabel(cb,'Speed (km/h)');

xlabel('X (m)');
ylabel('Y (m)');
title('Vehicle Speed Map');

%% =====================================================
% Simulation Result
%% =====================================================

lap = struct();

% Track

lap.track = track;
lap.feature = feature;

% Vehicle

lap.car = car;
lap.aero = aero;

% Simulation History

lap.sim.t = sim.t_list;
lap.sim.s = sim.s_list;
lap.sim.v = sim.v_list;
lap.sim.a = sim.a_list;

% Track-based Results

lap.result.time  = t_map;
lap.result.speed = v_map;
lap.result.accel = a_map;

% Summary

lap.summary.lap_time       = sim.t_list(end);
lap.summary.track_length   = track.s(end);
lap.summary.max_speed      = max(sim.v_list);
lap.summary.average_speed  = track.s(end)/sim.t_list(end);

lap.summary.date = datetime("now");

summary_text = sprintf([ ...
    '\n', ...
    '=============================================\n', ...
    ' Lap Simulation Finished\n', ...
    '=============================================\n', ...
    'Lap Time      : %.3f s\n', ...
    'Track Length  : %.2f m\n', ...
    'Average Speed : %.2f km/h\n', ...
    'Maximum Speed : %.2f km/h\n', ...
    '=============================================\n'], ...
    lap.summary.lap_time, ...
    lap.summary.track_length, ...
    lap.summary.average_speed*3.6, ...
    lap.summary.max_speed*3.6);

fprintf("%s", summary_text);

%% =====================================================
% Save
%% =====================================================

save(fullfile(result_dir, 'lap_result.mat'), 'lap');

summary_file = fullfile(result_dir, 'lap_summary.txt');
fid = fopen(summary_file, 'w');
fprintf(fid, "%s", summary_text);
fclose(fid);

saveas(fig_track_geometry, fullfile(result_dir, 'track_geometry.png'));
saveas(fig_s_vs_v, fullfile(result_dir, 's_vs_v.png'));
saveas(fig_s_vs_t, fullfile(result_dir, 's_vs_t.png'));
saveas(fig_v_vs_t, fullfile(result_dir, 'v_vs_t.png'));
saveas(fig_a_vs_t, fullfile(result_dir, 'a_vs_t.png'));
saveas(fig_vehicle_speed_map, fullfile(result_dir, 'vehicle_speed_map.png'));

fprintf("\nResult saved to %s\n", result_dir);
