%% 1. 初始化環境與模型設定
clear; clc;close all;

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
brake = Braking_system_v2(car);

% 初始化狀態
state.attitude = [0,0,0];
state.power = [0,0,0,0];

%% 2. 定義掃描範圍 (根據你的車輛性能調整)
v_range  = linspace(5, 30, 20);    % 車速範圍 (m/s)，避免從 0 開始以防分母為 0
ay_range = linspace(-22, 22, 81);  % 側向加速度範圍 (m/s^2)，奇數點確保包含 ay = 0

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
        
        % --- 最大加速 ax ---
        % vehicle_model 的第一個輸入是用於載荷轉移的 ax，不是油門開度。
        % 因此這裡解 ax_in = ax_out 的自洽狀態，而不是用 fminbnd 掃假的油門。
        Ax_max_mesh(i, j) = solve_consistent_ax( ...
            v, ay, state, car, aero, motor, "A", tire, brake);
        
        % --- 最大煞車 ax ---
        Ax_min_mesh(i, j) = solve_consistent_ax( ...
            v, ay, state, car, aero, motor, "B", tire, brake);
        
    end
end
fprintf('掃描完成！準備畫圖...\n');

% 左右對稱性檢查：前後不需要對稱，但 ay 正負應該接近鏡像。
ax_acc_sym_error = max(abs(Ax_max_mesh-flipud(Ax_max_mesh)),[],'all','omitnan');
ax_brk_sym_error = max(abs(Ax_min_mesh-flipud(Ax_min_mesh)),[],'all','omitnan');
fprintf('Acceleration left/right max error: %.6f m/s^2\n', ax_acc_sym_error);
fprintf('Braking left/right max error     : %.6f m/s^2\n', ax_brk_sym_error);

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


%% 5. 輔助函式：解出自洽的 ax
function ax = solve_consistent_ax(v, ay, state, car, aero, motor, d, tire, brake)

    if d == "A"
        ax_guess = 0;
    else
        ax_guess = -5;
    end

    relaxation = 0.5;
    tolerance = 1e-5;
    max_iter = 80;

    for k = 1:max_iter
        [~, ax_out] = vehicle_model( ...
            ax_guess, v, ay, state, car, aero, motor, d, tire, brake);

        if ~isfinite(ax_out)
            ax = NaN;
            return
        end

        if abs(ax_out-ax_guess) < tolerance
            ax = ax_out;
            return
        end

        ax_guess = (1-relaxation)*ax_guess + relaxation*ax_out;
    end

    ax = ax_guess;
end
