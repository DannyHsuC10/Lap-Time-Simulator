%% 1. 初始化環境與模型設定
clear; clc;close all;

addpath("vechiles")


car = Vehicle_data();
aero = Aerodynamics(car);
tire = tir_model(car);
motor = MotorModel(car);
brake = Braking_system(car);

% 初始化狀態
state.attitude = [0,0,0];
state.power = [0,0,0,0];

%% 2. 定義掃描範圍 (根據你的車輛性能調整)
v_range  = linspace(5, 30, 20);    % 車速範圍 (m/s)，避免從 0 開始以防分母為 0
ay_range = linspace(-25, 25, 30);  % 側向加速度範圍 (m/s^2)

% 建立網格格點
[V_mesh, Ay_mesh] = meshgrid(v_range, ay_range);
Ax_max_mesh = zeros(size(V_mesh));
Ax_min_mesh = zeros(size(V_mesh));

%% 3. 雙重迴圈掃描極限 ax
fprintf('開始掃描 GGV 曲面...\n');
for i = 1:size(V_mesh, 1)
    for j = 1:size(V_mesh, 2)
        v = V_mesh(i, j);
        ay = Ay_mesh(i, j);
        
        % --- 尋找最大加速 ax (a > 0) ---
        % 這裡假設 a 的範圍是 0 到 1 (油門踏板開度)
        % 如果你的 a 代表總驅動力或踏板，請調整最佳化範圍
        opt_fun_max = @(a) -eval_ax(v, ay, car, aero, motor, "A", tire, brake);
        [a_opt_max, fval_max] = fminbnd(opt_fun_max, 0, 1); 
        Ax_max_mesh(i, j) = -fval_max;
        
        % --- 尋找最大煞車 ax (a < 0) ---
        % 這裡假設 a 的範圍是 -1 到 0 (煞車踏板開度)
        opt_fun_min = @(a) eval_ax(v, ay, car, aero, motor, "B", tire, brake);
        [a_opt_min, fval_min] = fminbnd(opt_fun_min, -1, 0);
        Ax_min_mesh(i, j) = fval_min;
        
    end
end
fprintf('掃描完成！準備畫圖...\n');

%% 4. 繪製 3D GGV 曲面
figure();
% 畫出加速上半球
surf(Ay_mesh, V_mesh, Ax_max_mesh, 'FaceColor', 'interp', 'EdgeColor', 'none', 'FaceAlpha', 0.8);
hold on;
% 畫出煞車下半球
surf(Ay_mesh, V_mesh, Ax_min_mesh, 'FaceColor', 'interp', 'EdgeColor', 'none', 'FaceAlpha', 0.8);

% 介面優化
grid on;
colormap(jet);
colorbar;
view(45, 30); % 調整視角
xlabel('Lateral Acceleration a_y (m/s^2)', 'FontSize', 12);
ylabel('Velocity v (m/s)', 'FontSize', 12);
zlabel('Longitudinal Acceleration a_x (m/s^2)', 'FontSize', 12);
title('Vehicle GGV Plot (3D Surface)', 'FontSize', 14);


%% 5. 輔助函式：呼叫車輛模型並提取 ax
function ax = eval_ax(v, ay, car, aero, motor, d, tire, brake)
    % 呼叫你的車輛模型
    ax = vehicle_model(v, ay, car, aero, motor, d, tire, brake);
    
end