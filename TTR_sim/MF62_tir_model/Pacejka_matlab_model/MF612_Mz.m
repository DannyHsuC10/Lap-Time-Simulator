function Mz = MF612_Mz(MF)
    % 輸入處理
    Fz=MF.FZ;
    ia=MF.IA.*(pi/180);
    a=MF.SA.*(pi/180);
    sl=MF.SL;
    pressure=MF.pressure;
    Ro=MF.Ro;
    Fx=MF612_Fx(MF);
    Fy=MF612_Fy(MF);
    % 處理FZ數據
    Fzo=MF.FZO;                %額定負載，以廠商提供為主，沒有的話就全部fz加起來平均
    FzoPrime=MF.lambdaFzo.*Fzo;        %(4.E1)Fzo'
    dfz=(Fz-FzoPrime)./FzoPrime;    %(4.E2a)dfz
    % 處理P數據
    pi0=MF.PIO;
    dpi=(pressure-pi0)./pi0;        %(4.E2b)dfz
    dpi2 = dpi.^2;
    % 處理SA數據
    ami=tan(a);           %(4.E3)α*(要*sgn(vx)，測試流程中均為正向速度所以這裡省略)
    % 處理IA數據
    iaMi=sin(ia);         %(4.E4)γ*
    iaMi2=iaMi.^2;
    % 處理v數據
    Vcx=MF.V;
    Vcy = -Vcx.*tan(a);%.*(pi/180));% (4.E3)lambdyMuyPrime
    sgnVcx = sign(Vcx);% (4.E6a)
    Vc = sqrt(Vcx.^2 + Vcy.^2);
    Vc = Vc + eps(Vc);% (4.E6)
    cosPrimea = Vcx./Vc;
    %% Full formula set(lateral force pure side slip)
    
    muy=(MF.PDY1+MF.PDY2.*dfz).*(1+MF.PPY3.*dpi+MF.PPY4.*dpi2).*(1-MF.PDY3.*iaMi2).*MF.lambdaMuyMi;%(4.E23)
    
    Kya=MF.PKY1.*FzoPrime.*(1+MF.PPY1.*dpi).*(1-MF.PKY3.*abs(iaMi)).*sin(MF.PKY4.*atan(((Fz./FzoPrime)./ ...
        ((MF.PKY2+MF.PKY5.*(iaMi2)).*(1+MF.PPY2.*dpi))))).*MF.zeta3.*MF.lambdaKya;%(4.E25)
    
    Svyia=Fz.*(MF.PVY3+MF.PVY4.*dfz).*iaMi.*MF.lambdaKyia.*MF.lambdyMuyPrime.*MF.zeta2;%(4.E28)
    
    Kyiao=Fz.*(MF.PKY6+MF.PKY7.*dfz).*(1+MF.PPY5.*dpi).*MF.lambdaKyia;%(4.E30)
    
    Shy=(MF.PHY1+MF.PHY2.*dfz).*MF.lambdaHy+((Kyiao.*iaMi-Svyia).*MF.zeta0./(Kya+MF.epsilonK))+MF.zeta4-1;%(4.E27)
    
    
    Cy=MF.PCY1.*MF.lambdaCy;%(4.E21) >0
    
    
    Dy=muy.*Fz.*MF.zeta2;%(4.E22)
    
    
    By=Kya./(Cy.*Dy+MF.epsilonY);%(4.E26)
    
    
    Svy=Fz.*(MF.PVY1+MF.PVY2.*dfz).*MF.lambdaVy.*MF.lambdyMuyPrime.*MF.zeta2+Svyia;%(4.E29)

    %% Full formula set(Aligning Torque pure side slip)
    Br=(MF.QBZ9.*MF.lambdaKya./MF.lambdaMuyMi+MF.QBZ10.*By.*Cy).*MF.zeta6;%(4.E45)
    
    
    Cr=MF.zeta7;%(4.E46)
    
    Dr=Fz.*Ro.*((MF.QDZ6+MF.QDZ7.*dfz).*MF.lambdaMr.*MF.zeta2+( ...
        (MF.QDZ8+MF.QDZ9.*dfz).*(1+MF.PPZ2.*dpi)+(MF.QDZ10+MF.QDZ11.*dfz).*abs(iaMi)) ...
        .*iaMi.*MF.lambdaKzia.*MF.zeta0).*MF.lambdaMuyMi.*(1).*cosPrimea+MF.zeta8-1;%(4.E47)
    
    KyaPrime=Kya+MF.epsilonK;%(4.E39)
    
    Shf=Shy+Svy./KyaPrime;%(4.E38)
    
    ar=ami+Shf;%(4.E37)
    
    
    Sht=MF.QHZ1+MF.QHZ2.*dfz+(MF.QHZ3+MF.QHZ4.*dfz).*iaMi;%(4.E35)
    
    at=ami+Sht;%(4.E34)
    
    Bt=(MF.QBZ1+MF.QBZ2.*dfz+MF.QBZ3.*(dfz.^2)).*(1+MF.QBZ5.*abs(iaMi)+MF.QBZ4.*iaMi).*MF.lambdaKya./MF.lambdaMuyMi;%(4.E40)
    
    Ct=MF.QCZ1;%(4.E41)
    
    Et=(MF.QEZ1+MF.QEZ2.*dfz+MF.QEZ3*(dfz.^2)).*(1+(MF.QEZ4+MF.QEZ5.*iaMi).*(2/pi).*atan(Bt.*Ct.*at));%(4.E44)
    
    Dto=Fz.*(Ro./FzoPrime).*(MF.QDZ1+MF.QDZ2.*dfz).*(1-MF.PPZ1.*dpi).*MF.lambdat.*sgnVcx;%(4.E42)
    
    %這裡的(1)是sgn(Vx)，同樣沒有往後轉的測試所以用1代替
    Dt=Dto.*(1+MF.QDZ3.*abs(iaMi)+MF.QDZ4.*iaMi2).*MF.zeta5;%(4.E43)
       
    %% Full formula set(lateral force pure side slip)
    
    Kxk=Fz.*(MF.PKX1+MF.PKX2.*dfz).*exp(MF.PKX3.*dfz).*(1+MF.PPX1.*dpi+MF.PPX2.*dpi2).*MF.lambdaKxk;%(4.E15)
    
    kappa2 = sl.^2;
    KxkKya2 = (Kxk./KyaPrime).^2;
    
    alphar2 = ar.^2;
    alpharSgn = sign(ar);
    alpharEq = sqrt(alphar2 + kappa2.*KxkKya2).*alpharSgn;% (4.E78)
    % alpharEq = atan(sqrt(tan(alphar).^2+(Kxk./Kya_).^2.*kappa.^2)).*alpharSgn; % (A54)
    
    alphat2 = a.^2;
    alphatSgn = sign(a);
    alphatEq = sqrt(alphat2 + kappa2.*KxkKya2).*alphatSgn;% (4.E77)
    % alphatEq = atan(sqrt(tan(alphat).^2+(Kxk./Kya_prime).^2.*kappa.^2)).*sign(alphat); % (A55)
    
    s = Ro.*(MF.SSZ1+MF.SSZ2.*(Fy/FzoPrime)+(MF.SSZ3+MF.SSZ4.*dfz).*iaMi).*MF.lambdaS;% (4.E76)
    
    Mzr = Dr.*cos(Cr.*atan(Br.*alpharEq)).*sgnVcx;% (4.E75)
    
    
    t = Dt.*cos(Ct.*atan(Bt.*alphatEq-Et.*(Bt.*alphatEq-atan(Bt.*alphatEq)))).*sgnVcx;% (4.E73) Pneumatic trail
    
    Mz_ = -t.*Fy;% (4.E72)
    
    Mz = Mz_ + Mzr + s.*Fx;% (4.E71)
    
end