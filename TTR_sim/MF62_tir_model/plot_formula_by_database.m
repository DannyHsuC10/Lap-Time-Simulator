%% =====================================================================
% Tire Model Surface Plot (兩曲面合併單一張圖)
% =====================================================================
clc; clear; close all;

% 1. 載入模型與路徑
addpath("power")
addpath("braking")
addpath("MF62_tir_model")
addpath("aero")

car = Vehicle_data();
tire = tir_model(car);

%% 2. 設定您指定的固定網格範圍
Fz_grid_vec = 100:50:1500;      % 垂直力軸
Fy_grid_vec = -3000:25:3000;    % 側向力軸

% 建立 2D 網格矩陣
[FY, FZ] = meshgrid(Fy_grid_vec, Fz_grid_vec);
[Rows, Cols] = size(FY);

% 初始化 3D 曲面所需的 Fx 矩陣
Fx_acc_surf = zeros(Rows, Cols);
Fx_brk_surf = zeros(Rows, Cols);

%% 3. 雙重迴圈掃描
fprintf('正在根據指定網格計算 Fx 曲面...\n');

for r = 1:Rows
    for c = 1:Cols
        Fz_test = FZ(r, c);
        Fy_test = FY(r, c);
        
        % 呼叫您的輪胎限制函式並加上 abs() 確保安全
        Fx_acc_surf(r, c) = tire.lim_output(Fz_test, abs(Fy_test), "A");
        Fx_brk_surf(r, c) = tire.lim_output(Fz_test, abs(Fy_test), "B");
    end
end

fprintf('計算完成，正在產生合併的 3D 曲面圖...\n');

%% 4. 繪製合併的 3D 曲面
figure();
hold on;

% 繪製加速曲面 (使用 FaceColor = 'interp' 表現色彩漸層)
surf_acc = surf(FY, FZ, Fx_acc_surf, 'FaceAlpha', 0.7, 'EdgeColor', 'none');

% 繪製煞車曲面 (通常為負值，放在下方)
surf_brk = surf(FY, FZ, Fx_brk_surf, 'FaceAlpha', 0.7, 'EdgeColor', 'none');

% 圖表視覺美化
cb = colorbar;
ylabel(cb, 'Longitudinal Force F_x (N)', 'FontSize', 10, 'FontWeight', 'bold');
colormap(jet); % 顏色代表 F_x 的大小 (上方加速為紅/橘，下方煞車為藍)

% 設定標籤
xlabel('Lateral Force F_y (N)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Vertical Load F_z (N)', 'FontSize', 11, 'FontWeight', 'bold');
zlabel('Longitudinal Force F_x (N)', 'FontSize', 11, 'FontWeight', 'bold');
title('Combined Tire Traction Envelope (F_x vs F_y vs F_z)', 'FontSize', 14, 'FontWeight', 'bold');

% 視覺優化
grid on;
view(-60, 20); % 設定一個能同時看到上下曲面與 Fz 軸的立體視角
camlight('headlight');
lighting gouraud;

% 加上圖例說明
% 建立虛擬物件給 legend 使用，避免色彩過於複雜
plot3(NaN, NaN, NaN, 'r', 'LineWidth', 5, 'DisplayName', 'Max Acceleration Limit (+Fx)');
plot3(NaN, NaN, NaN, 'b', 'LineWidth', 5, 'DisplayName', 'Max Braking Limit (-Fx)');
legend('show', 'Location', 'best');

hold off;