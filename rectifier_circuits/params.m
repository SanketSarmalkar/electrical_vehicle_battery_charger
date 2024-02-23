% buck
f = 10000;
dI = 5/100;
dV = 1/100;
Vout = 5;
Vin = 530;
d = Vout*100/Vin;
R = 1;

L = Vout*(1-d/100)/(f*dI);
C = Vout*(1-d/100)/(8*dV*f*f*L);

load('BatteryModel.mat')
load('SOC-OCV.mat')
F_R0    = scatteredInterpolant(param.T,param.SOC,param.R0);
F_R1    = scatteredInterpolant(param.T,param.SOC,param.R1);
F_R2    = scatteredInterpolant(param.T,param.SOC,param.R2);
F_C1    = scatteredInterpolant(param.T,param.SOC,param.C1);
F_C2    = scatteredInterpolant(param.T,param.SOC,param.C2);
K1  = F_C1(25,0.25);
SOCOCV  = polyfit(SOC_OCV.SOC,SOC_OCV.OCV,11);

dSOCOCV = polyder(SOCOCV);
X           = [1; 0; 0];
n_x = size(X);
R_x = 2.5e-5;
P_x = [0.025 0 0;
0 0.01 0;
0 0 0.01];

Q_x = [1.0e-6 0 0; 0 1.0e-5 0;
0 0 1.0e-5];
SOC_Estimated = [0];
Vt_Estimated = [0];
Vt_Error = [0];
DeltaT = 1;
Qn_rated    = 4.81 * 3600;