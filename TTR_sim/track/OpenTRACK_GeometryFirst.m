%% OpenLAP Laptime Simulation Project
clear;clc;close all;

mesh_size = 0.01 ; % [m]0


table_shape = load_segments('input_map.xlsx') ;% 最後一步驟

%% build

track = build_track(table_shape, mesh_size);% 建立圖資


%% output
plot_track(track);% 出圖

