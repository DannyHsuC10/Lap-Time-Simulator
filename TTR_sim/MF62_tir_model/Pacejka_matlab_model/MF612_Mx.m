function Mx = MF612_Mx(MF)
    % 將導入數據賦值給FZ,IA,A
    Fz=MF.FZ;
    Fy=MF612_Fy(MF);
    ia=MF.IA;
    pressure=MF.pressure;
    Ro=MF.Ro;
    % 處理FZ數據
    Fzo=MF.FZO; % 額定負載，以廠商提供為主，沒有的話就全部fz加起來平均
    % 處理P數據
    pi0=MF.PIO;
    dpi=(pressure-pi0)./pi0; % (4.E2b)dfz
    % 處理IA數據
    iaR = ia.*(pi/180);
    % Full formula set(lateral force pure side slip)
    ratioFy = Fy./Fzo;
    ratioFz = Fz./Fzo;
    
    Mx = Ro.*Fz.*(...
        MF.QSX1*MF.lambdaVMX - MF.QSX2*iaR.*(1+MF.PPMX1*dpi) + MF.QSX3*ratioFy ...
        +MF.QSX4*cos(MF.QSX5*atan(MF.QSX6*ratioFz).^2).*sin(MF.QSX7*iaR + MF.QSX8*atan(...
        MF.QSX9*ratioFy)) + MF.QSX10*atan(MF.QSX11*ratioFz).*iaR)*MF.lambdaMX;% (4.E69)
end