function Fx0 = MF612_Fx0(MF)
%% 將導入數據賦值給FZ,IA,A
Fz=MF.FZ;
ia=MF.IA;
kappa=MF.SL;
pressure=MF.pressure;
%% 處理FZ數據
Fzo=MF.FZO;                %額定負載，以廠商提供為主，沒有的話就全部fz加起來平均
FzoPrime=MF.lambdaFzo.*Fzo;        %(4.E1)Fzo'
dfz=(Fz-FzoPrime)./FzoPrime;    %(4.E2a)dfz
%% 處理P數據
pi0=MF.PIO;
dpi=(pressure-pi0)./pi0;        %(4.E2b)dfz
dpi2 = dpi.^2;

iaR = ia.*(pi/180);
%% Full formula set(lateral force pure side slip)
%(4.E17)
Shx=(MF.PHX1+MF.PHX2.*dfz).*MF.lambdaHx;

%(4.E10)
kappax=kappa+Shx;

%(4.E18)
Svx=Fz.*(MF.PVX1+MF.PVX2.*dfz).*MF.lambdaVx.*MF.lambdaMuxPrime.*MF.zeta1;


%(4.E15)
Kxk=Fz.*(MF.PKX1+MF.PKX2.*dfz).*exp(MF.PKX3.*dfz).*(1+MF.PPX1.*dpi+MF.PPX2.*dpi2).*MF.lambdaKxk;

%(4.E14)
Ex=(MF.PEX1+MF.PEX2.*dfz+MF.PEX3.*(dfz.^2)).*(1-MF.PEX4.*sign(kappax)).*MF.lambdaEx;

%(4.E13)
mux=(MF.PDX1+MF.PDX2.*dfz).*(1+MF.PPX3.*dpi+MF.PPX4.*dpi2).*(1-MF.PDX3.*(iaR.^2)).*MF.lambdaMuxMi;

%(4.E12)
Dx=mux.*Fz.*MF.zeta1;

%(4.E11)
Cx=MF.PCX1.*MF.lambdaCx;

%(4.E16)
Bx=Kxk./(Cx.*Dx+MF.epsilonX);

%(4.E9)
Fx0=Dx.*sin(Cx.*atan(Bx.*kappax-Ex.*(Bx.*kappax-atan(Bx.*kappax))))+Svx;

end