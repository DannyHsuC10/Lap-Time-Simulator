function Fx = MF612_Fx(MF)
    % 將導入數據賦值給FZ,IA,A
    Fz=MF.FZ;
    ia=MF.IA;
    kappa=MF.SL;
    pressure=MF.pressure;
    % 處理FZ數據
    Fzo=MF.FZO;                %額定負載，以廠商提供為主，沒有的話就全部fz加起來平均
    FzoPrime=MF.lambdaFzo.*Fzo;        %(4.E1)Fzo'
    dfz=(Fz-FzoPrime)./FzoPrime;    %(4.E2a)dfz
    % 處理P數據
    pi0=MF.PIO;
    dpi=(pressure-pi0)./pi0;        %(4.E2b)dfz
    dpi2 = dpi.^2;
    % 處理SA數據
    a=MF.SA;
    ami=tan(a.*(pi/180));           %(4.E3)α*(要*sgn(vx)，測試流程中均為正向速度所以這裡省略)
    % 處理IA數據
    iaMi=sin(ia.*(pi/180));         %(4.E4)γ*
    iaR = ia.*(pi/180);
    % Full formula set(lateral force pure side slip)
    Shx=(MF.PHX1+MF.PHX2.*dfz).*MF.lambdaHx;%(4.E17)
    
    kappax=kappa+Shx;%(4.E10)
    
    Svx=Fz.*(MF.PVX1+MF.PVX2.*dfz).*MF.lambdaVx.*MF.lambdaMuxPrime.*MF.zeta1;%(4.E18)
    
    Kxk=Fz.*(MF.PKX1+MF.PKX2.*dfz).*exp(MF.PKX3.*dfz).*(1+MF.PPX1.*dpi+MF.PPX2.*dpi2).*MF.lambdaKxk;%(4.E15)
    
    Ex=(MF.PEX1+MF.PEX2.*dfz+MF.PEX3.*(dfz.^2)).*(1-MF.PEX4.*sign(kappax)).*MF.lambdaEx;%(4.E14)
    
    mux=(MF.PDX1+MF.PDX2.*dfz).*(1+MF.PPX3.*dpi+MF.PPX4.*dpi2).*(1-MF.PDX3.*(iaR.^2)).*MF.lambdaMuxMi;%(4.E13)
    
    Dx=mux.*Fz.*MF.zeta1;%(4.E12)
    
    Cx=MF.PCX1.*MF.lambdaCx;%(4.E11)
    
    Bx=Kxk./(Cx.*Dx+MF.epsilonX);%(4.E16)
    
    Fx0=Dx.*sin(Cx.*atan(Bx.*kappax-Ex.*(Bx.*kappax-atan(Bx.*kappax))))+Svx;%(4.E9)
    %% Full formula set(lateral force pure side slip)
    
    Cxa = MF.RCX1;% (4.E55)
    
    Exa = MF.REX1 + MF.REX2.*dfz;% (4.E56)
    
    SHxa = MF.RHX1;% (4.E57)
    
    aS = ami + SHxa;% (4.E53)
    
    Bxa = (MF.RBX1+MF.RBX3*iaMi.^2).*cos(atan(MF.RBX2.*kappa)).*MF.lambdaXa;% (4.E54)
    
    Gxao = cos(Cxa.*atan(Bxa.*SHxa-Exa.*(Bxa.*SHxa-atan(Bxa.*SHxa))));% (4.E52)
    
    Gxa = cos(Cxa.*atan(Bxa.*aS-Exa.*(Bxa.*aS-atan(Bxa.*aS))))./Gxao;% (4.E51)

    Fx = Gxa.*Fx0;% (4.E50)
end