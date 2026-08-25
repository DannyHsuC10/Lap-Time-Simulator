function out = MF_api(cmd, FZ, SA, SL, IA, P, tir) % API接口
    
    MF = CalculationInput(FZ, SA, SL, IA, P, tir);
    
    switch cmd % 切換開關，套用不同模型
        case 'FX'
            out = MF612_Fx(MF);
        case 'FY'
            out = MF612_Fy(MF);
        case 'MZ'
            out = MF612_Mz(MF);
        case 'MX'
            out = MF612_Mx(MF);
        otherwise
            error("Unknown tire API command");
    end

end

function MF = CalculationInput(FZ,SA,SL,IA,P,tir) % 創建輸入(修改模型降低tir讀取頻率)
    MF = tir;
    MF.FZ = FZ;
    MF.SA = SA;
    MF.SL = SL;
    MF.IA = IA;
    MF.pressure = P;
    % 調整變數命名
    MF.PIO = tir.NOMPRES;
    MF.FZO = tir.FNOMIN;
    MF.Ro = tir.UNLOADED_RADIUS;
    % 預設速度
    MF.V = tir.LONGVL;  
    % 固定參數
    MF.lambdaFzo=1;        %λFzo   (nominal (rated) load) 
    MF.lambdaMuyMi=0.6;      %λμy*   (peak friction coefficient) 這之後實際使用要改0.6
    MF.lambdyMuyPrime=1;   %λμy'   (peak friction coefficient)
    MF.lambdaKya=1;        %λKya   (cornering stiffness)
    MF.lambdaCy=1;         %λCy    (shape factor)
    MF.lambdaEy=1;         %λEy    (curvature factor)
    MF.lambdaHy=1;         %λHy    (horizontal shift)
    MF.lambdaVy=1;         %λVy    (vertical shift)
    MF.lambdaKyia=1;       %λKyγ   (camber force stiffness)
    MF.lambdat=1;          %λt   
    MF.lambdaKzia=1;
    MF.lambdaMr=1;
    MF.lambdaS=1;
    MF.lambdaYk=1;
    MF.lambdaVyk=1;
    MF.lambdaHx=1;
    MF.lambdaVx=1;
    MF.lambdaMuxPrime=1;
    MF.lambdaKxk=1;
    MF.lambdaEx=1;
    MF.lambdaMuxMi=0.6; % 這之後實際使用要改0.6
    MF.lambdaCx=1;
    MF.lambdaXa=1;
    MF.lambdaVMX = 1;
    MF.lambdaMX = 1;
    MF.zeta1=1;
    % 調整參數
    MF.zeta0=1;
    MF.zeta2=1;
    MF.zeta3=1;
    MF.zeta4=1;
    MF.zeta5=1;
    MF.zeta6=1;
    MF.zeta7=1;
    MF.zeta8=1;
    MF.epsilonK=0.1;
    % 穩定參數
    MF.epsilonY=0.1;
    MF.epsilonX=0.1;
end




