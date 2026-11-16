function Fy = MF612_Fy(MF)
    %　處理輸入
    Fz=MF.FZ;
    ia=MF.IA;
    a=MF.SA;
    pressure=MF.pressure;
    kappa=MF.SL;
    % 處理FZ數據
    Fzo=MF.FZO;%額定負載，以廠商提供為主，沒有的話就全部fz加起來平均
    FzoPrime=MF.lambdaFzo.*Fzo;%(4.E1)Fzo'
    dfz=(Fz-FzoPrime)./FzoPrime;%(4.E2a)dfz
    % 處理P數據
    pi0=MF.PIO;
    dpi=(pressure-pi0)./pi0;%(4.E2b)dfz
    dpi2 = dpi.^2;
    % 處理SA數據
    ami=tan(a.*(pi/180));%(4.E3)α*(要*sgn(vx)，測試流程中均為正向速度所以這裡省略)
    % 處理IA數據
    iaMi=sin(ia.*(pi/180));%(4.E4)γ*
    iaMi2=iaMi.^2;
    % Full formula set(lateral force pure side slip)
    muy=(MF.PDY1+MF.PDY2.*dfz).*(...
        1+MF.PPY3.*dpi+MF.PPY4.*dpi2).*(...
        1-MF.PDY3.*iaMi2).*MF.lambdaMuyMi;%(4.E23)
    
    Kya=MF.PKY1.*FzoPrime.*(1+MF.PPY1.*dpi).*(1-MF.PKY3.*abs(iaMi)).*sin(MF.PKY4.*atan(((Fz./FzoPrime)./ ...
        ((MF.PKY2+MF.PKY5.*(iaMi2)).*(1+MF.PPY2.*dpi))))).*MF.zeta3.*MF.lambdaKya;%(4.E25)
    
    Svyia=Fz.*(MF.PVY3+MF.PVY4.*dfz).*iaMi.*MF.lambdaKyia.*MF.lambdyMuyPrime.*MF.zeta2;%(4.E28)
    
    Kyiao=Fz.*(MF.PKY6+MF.PKY7.*dfz).*(1+MF.PPY5.*dpi).*MF.lambdaKyia;%(4.E30)
    
    Shy=(MF.PHY1+MF.PHY2.*dfz).*MF.lambdaHy+((Kyiao.*iaMi-Svyia).*MF.zeta0./(Kya+MF.epsilonK))+MF.zeta4-1;%(4.E27)
    
    ay=ami+Shy;%(4.E20)
    
    Cy=MF.PCY1.*MF.lambdaCy;%(4.E21)>0
    
    Dy=muy.*Fz.*MF.zeta2;%(4.E22)
    
    Ey=(MF.PEY1+MF.PEY2.*dfz).*(1+MF.PEY5.*iaMi2-(MF.PEY3+MF.PEY4.*iaMi).*sign(ay)).*MF.lambdaEy;%(4.E24)<=1
    
    By=Kya./(Cy.*Dy+MF.epsilonY);%(4.E26)
    
    Svy=Fz.*(MF.PVY1+MF.PVY2.*dfz).*MF.lambdaVy.*MF.lambdyMuyPrime.*MF.zeta2+Svyia;%(4.E29)
    
    Fy0=Dy.*sin(Cy.*atan(By.*ay-Ey.*(By.*ay-atan(By.*ay))))+Svy;%(4.E19)final result=Fy0(pure side slip)

    %% Full formula set(lateral force combine slip)
    Byk = (MF.RBY1+MF.RBY4*iaMi.^2).*cos(atan(MF.RBY2.*(ami-MF.RBY3))).*MF.lambdaYk;% (4.E62)
    
    Cyk = MF.RCY1;% (4.E63)
    
    DVyk = muy.*Fz.*(MF.RVY1+MF.RVY2.*dfz+MF.RVY3*iaMi).*cos(atan(MF.RVY4.*ami)).*MF.zeta2;% (4.E67)
    
    Eyk = MF.REY1+MF.REY2.*dfz;% (4.E64)
    
    SHyk = MF.RHY1+MF.RHY2.*dfz;% (4.E65)
    
    SVyk = DVyk.*sin(MF.RVY5.*atan(MF.RVY6.*kappa)).*MF.lambdaVyk;% (4.E66)
    
    kappaS = kappa + SHyk;% (4.E61)
    
    Gyk0 = cos(Cyk.*atan(Byk.*SHyk-Eyk.*(Byk.*SHyk-atan(Byk.*SHyk))));% (4.E60)
    
    Gyk = cos(Cyk.*atan(Byk.*kappaS-Eyk.*(Byk.*kappaS-atan(Byk.*kappaS))))./Gyk0;% (4.E59)
    
    Fy = Gyk.*Fy0 + SVyk;% (4.E58)
end