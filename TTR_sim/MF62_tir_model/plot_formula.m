clc;clear;close all;

addpath('MF62_tir_model/Pacejka_matlab_model'); % 導入資料夾
tir = load_tir("D2704_mf612.tir"); % 載入tir

FZ = 600;
P = 80000;
IA = 0;


Friction_Ellipse(FZ,IA,P,tir)


function Friction_Ellipse(FZ,IA,P,tir)% 繪製摩擦圓
    
    sa_vec = linspace(-10, 10, 100); % Slip 範圍 -1.0 到 1.0
    sl_vec = linspace(-1, 1, 100); % Fz 範圍 0 到 2000
    
    [SA, SL] = meshgrid(sa_vec, sl_vec);
    FX = zeros(size(SA));
    FY = zeros(size(SA));

    for i = 1:numel(SA)
        FY(i) = MF_api("FY", FZ, SA(i), SL(i) , IA, P, tir);%可以換成 FX FY MX MZ
        FX(i) = MF_api("FX", FZ, SA(i), SL(i) , IA, P, tir);%可以換成 FX FY MX MZ
    end
    
    figure% 3D
    surf(FX, FY, SL)
    xlabel('Slip Ratio')
    ylabel('Slip Angle')
    zlabel('Fx')
    
    figure% 極限狀態
    plot(FX(:),FY(:),'.')
    hold on
    
    K = convhull(FX(:),FY(:));
    
    plot(FX(K),FY(K),'r','LineWidth',2)
    
    axis equal
    grid on
    
    xlabel('Fx (N)')
    ylabel('Fy (N)')
    title('Friction Ellipse Envelope')

end

