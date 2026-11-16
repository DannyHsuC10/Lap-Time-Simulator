%% 使用範例

%{

1. 請將本檔案與(load_tir)複製在(model_matlab上層)
2. 同時確認model_matlab上層有tir檔案，沒有請複製DEMO裡面的tir
3. 按下run執行就會有MF62模型計算結果

%}


clc; clear;
addpath('MF62_tir_model'); % 導入資料夾
tir = load_tir("D2704_mf612.tir"); % 載入tir

Fx = MF_api("FX", 800, 0.0, 0.1, 0.0, 80000, tir); % 呼叫api接口


disp(Fx)
