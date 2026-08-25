function Mz0 = MF612_Mz0(MF)
%% 將導入數據賦值給FZ,IA,A
Fz=MF.FZ;
ia=MF.IA;
a=MF.SA;
pressure=MF.pressure;
Ro=MF.Ro;
%% 處理FZ數據
Fzo=MF.FZO;                %額定負載，以廠商提供為主，沒有的話就全部fz加起來平均
FzoPrime=MF.lambdaFzo.*Fzo;        %(4.E1)Fzo'
dfz=(Fz-FzoPrime)./FzoPrime;    %(4.E2a)dfz
%% 處理P數據
pi0=MF.PIO;
dpi=(pressure-pi0)./pi0;        %(4.E2b)dfz
dpi2 = dpi.^2;
%% 處理SA數據
ami=tan(a.*(pi/180));           %(4.E3)α*(要*sgn(vx)，測試流程中均為正向速度所以這裡省略)
%% 處理IA數據
iaMi=sin(ia.*(pi/180));         %(4.E4)γ*
iaMi2=iaMi.^2;
%% 處理v數據
% (4.E3)
Vcx=MF.V;
Vcy = -Vcx.*tan(a.*(pi/180));
sgnVcx = sign(Vcx);
% (4.E6a)
Vc = sqrt(Vcx.^2 + Vcy.^2);
Vc = Vc + eps(Vc);

% (4.E6)
cosPrimea = Vcx./Vc;
%% Full formula set(lateral force pure side slip)
%(4.E23)
muy=(MF.PDY1+MF.PDY2.*dfz).*(1+MF.PPY3.*dpi+MF.PPY4.*dpi2).*(1-MF.PDY3.*iaMi2).*MF.lambdaMuyMi;

%(4.E25)
Kya=MF.PKY1.*FzoPrime.*(1+MF.PPY1.*dpi).*(1-MF.PKY3.*abs(iaMi)).*sin(MF.PKY4.*atan(((Fz./FzoPrime)./((MF.PKY2+MF.PKY5.*(iaMi2)).*(1+MF.PPY2.*dpi))))).*MF.zeta3.*MF.lambdaKya;

%(4.E28)
Svyia=Fz.*(MF.PVY3+MF.PVY4.*dfz).*iaMi.*MF.lambdaKyia.*MF.lambdyMuyPrime.*MF.zeta2;

%(4.E30)
Kyiao=Fz.*(MF.PKY6+MF.PKY7.*dfz).*(1+MF.PPY5.*dpi).*MF.lambdaKyia;

%(4.E27)
Shy=(MF.PHY1+MF.PHY2.*dfz).*MF.lambdaHy+((Kyiao.*iaMi-Svyia).*MF.zeta0./(Kya+MF.epsilonK))+MF.zeta4-1;

%(4.E20)
ay=ami+Shy;

%(4.E21)
Cy=MF.PCY1.*MF.lambdaCy;%>0

%(4.E22)
Dy=muy.*Fz.*MF.zeta2;

%(4.E24)
Ey=(MF.PEY1+MF.PEY2.*dfz).*(1+MF.PEY5.*iaMi2-(MF.PEY3+MF.PEY4.*iaMi).*sign(ay)).*MF.lambdaEy;%<=1

%(4.E26)
By=Kya./(Cy.*Dy+MF.epsilonY);

%(4.E29)
Svy=Fz.*(MF.PVY1+MF.PVY2.*dfz).*MF.lambdaVy.*MF.lambdyMuyPrime.*MF.zeta2+Svyia;

%(4.E19)final result=Fy0(pure side slip)
Fy0=Dy.*sin(Cy.*atan(By.*ay-Ey.*(By.*ay-atan(By.*ay))))+Svy;

%% Full formula set(Aligning Torque pure side slip)
%(4.E45)
Br=(MF.QBZ9.*MF.lambdaKya./MF.lambdaMuyMi+MF.QBZ10.*By.*Cy).*MF.zeta6;

%(4.E46)
Cr=MF.zeta7;%(4.E47)

%(4.E47)
Dr=Fz.*Ro.*((MF.QDZ6+MF.QDZ7.*dfz).*MF.lambdaMr.*MF.zeta2+((MF.QDZ8+MF.QDZ9.*dfz).*(1+MF.PPZ2.*dpi)+(MF.QDZ10+MF.QDZ11.*dfz).*abs(iaMi)).*iaMi.*MF.lambdaKzia.*MF.zeta0).*MF.lambdaMuyMi.*(1).*cosPrimea+MF.zeta8-1;

%(4.E39)
KyaPrime=Kya+MF.epsilonK;

%(4.E38)
Shf=Shy+Svy./KyaPrime;

%(4.E37)
ar=ami+Shf;

%(4.E36)
Mzro=Dr.*cos(Cr.*atan(Br.*ar)).*cosPrimea;

%(4.E35)
Sht=MF.QHZ1+MF.QHZ2.*dfz+(MF.QHZ3+MF.QHZ4.*dfz).*iaMi;

%(4.E34)
at=ami+Sht;

%(4.E40)
Bt=(MF.QBZ1+MF.QBZ2.*dfz+MF.QBZ3.*(dfz.^2)).*(1+MF.QBZ5.*abs(iaMi)+MF.QBZ4.*iaMi).*MF.lambdaKya./MF.lambdaMuyMi;

%(4.E41)
Ct=MF.QCZ1;

%(4.E44)
Et=(MF.QEZ1+MF.QEZ2.*dfz+MF.QEZ3*(dfz.^2)).*(1+(MF.QEZ4+MF.QEZ5.*iaMi).*(2/pi).*atan(Bt.*Ct.*at));

%(4.E42)
Dto=Fz.*(Ro./FzoPrime).*(MF.QDZ1+MF.QDZ2.*dfz).*(1-MF.PPZ1.*dpi).*MF.lambdat.*sgnVcx;
%這裡的(1)是sgn(Vx)，同樣沒有往後轉的測試所以用1代替

%(4.E43)
Dt=Dto.*(1+MF.QDZ3.*abs(iaMi)+MF.QDZ4.*iaMi2).*MF.zeta5;

%(4.E49)
Kziao=Fz.*Ro.*(MF.QDZ8+MF.QDZ9.*dfz).*(1+MF.PPZ2.*dpi).*MF.lambdaKzia.*MF.lambdaMuyMi-Dto.*Kyiao;

%(4.E48)
Kzao=Dto.*Kya;

%(4.E33)
to=Dt.*cos(Ct.*atan(Bt.*at-Et.*(Bt.*at-atan(Bt.*at)))).*cosPrimea;
%(1)=cos'(a)，為了因應往後轉的狀況所以導入輪心速度和前進速度比值(類似滑移率)，理論上測試流程不會有往後轉的狀況所以省略

%(4.E32)
MzoPrime=-to.*Fy0;

%(4.E31)
Mz0=MzoPrime+Mzro;
end