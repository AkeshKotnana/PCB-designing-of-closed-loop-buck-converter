 clc; close all;
 s=tf('s');


%% Buck Calculations
Vin=300;Vo=120;fs=25e3;
PercentagedelIL=10;PercentagedelVo=5;
Po=300;
R=Vo^2/Po;Io=Po/Vo;Iin=Po/Vin;IL=Io;
delIL=(PercentagedelIL/100)*IL;delVo=(PercentagedelVo/100)*Vo;
D=Vo/Vin;  % Duty Ratio;
L= ((1-D)*Vo)/(fs*delIL);
C= ((1-D)*Vo)/(8*fs^2*L*delVo);


%% Buck Converter Modelling Equations
 % Kp=2.38e-3;Ki=4.258;
PM=120;Wgc= 2e3;

rC=0.02;rL=0.002;Rs=0.02;RD=0.02;
RE=[rL+D*Rs+(1-D)*RD]*[1+(delIL^2/(3*IL^2))];
W0=sqrt((R+RE)/((R+rC)*L*C));
Wz1=(1/(rC*C));

Q=sqrt((R+RE)*(R+rC)*L*C)/(R*RE*C+ R*rC*C+RE*rC*C+L);
Gp=(((Vin*R)/(R+RE))*(1+s/Wz1))/(1+(s/(Q*W0))+(s/(Q*W0))^2);

[mag,angl]=bode(Gp,Wgc);

Ki=sqrt(((Wgc/mag)^2)/(1+(tand(PM-90-angl))^2))
Kp=tand(PM-90-angl)*Ki/Wgc


Gc=Kp+Ki/s;
G=Gc*Gp;
bode(Gp);
hold on;grid on;
bode(G);
margin(G)
[Gm,Pm,Wpc,Wgcf] = margin(G);

%% Integrator Design (R1 & C1 (series) are in feedback )
C1=10e-6;
R2=(C1*Ki)^-1
R1=Kp*R2