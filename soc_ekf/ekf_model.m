function [SOC_Estimated, Vt_Estimated, Vt_Error] = ekf_model(Current, Vt_Actual, Temperature)

load 'BatteryModel.mat';
load 'SOC-OCV.mat';

SOC_Init    = 1;
X           = [SOC_Init; 0; 0];
DeltaT      = 1; 
Qn_rated    = 4.81 * 3600; 

% interpolation
F_R0    = scatteredInterpolant(param.T,param.SOC,param.R0);
F_R1    = scatteredInterpolant(param.T,param.SOC,param.R1);
F_R2    = scatteredInterpolant(param.T,param.SOC,param.R2);
F_C1    = scatteredInterpolant(param.T,param.SOC,param.C1);
F_C2    = scatteredInterpolant(param.T,param.SOC,param.C2);

SOCOCV  = polyfit(SOC_OCV.SOC,SOC_OCV.OCV,11); 
dSOCOCV = polyder(SOCOCV); 

n_x   = size(X,1);
R_x   = 2.5e-5;
P_x   = [0.025 0 0;
         0 0.01 0;
         0 0 0.01];
Q_x   = [1.0e-6 0 0;
         0 1.0e-5 0;
         0 0 1.0e-5];

SOC_Estimated   = [];
Vt_Estimated    = [];
Vt_Error        = [];
ik = length(Current);

for k=1:1:ik
    T           = Temperature(k); % C
    U           = Current(k); % A
    SOC         = X(1);
    V1          = X(2);
    V2          = X(3);
    
    R0     = F_R0(T,SOC);
    R1     = F_R1(T,SOC);
    R2     = F_R2(T,SOC);
    C1     = F_C1(T,SOC);
    C2     = F_C2(T,SOC);
    OCV = polyval(SOCOCV,SOC);
    
    Tau_1       = C1 * R1;
    Tau_2       = C2 * R2;
    
    a1 = exp(-DeltaT/Tau_1);
    a2 = exp(-DeltaT/Tau_2);
    
    b1 = R1 * (1 - exp(-DeltaT/Tau_1));
    b2 = R2 * (1 - exp(-DeltaT/Tau_2)); 
    TerminalVoltage = OCV - R0*U - V1 - V2;
    if U > 0
        eta = 1; %discharging
    elseif U <= 0 
        eta = 1; % charging
    end
    dOCV = polyval(dSOCOCV, SOC);
    C_x    = [dOCV -1 -1];
    Error_x   = Vt_Actual(k) - TerminalVoltage;
    Vt_Estimated    = [Vt_Estimated;TerminalVoltage];
    SOC_Estimated   = [SOC_Estimated;X(1)];
    Vt_Error        = [Vt_Error;Error_x];
    A   = [1 0  0;
           0 a1 0;
           0 0  a2];
    B   = [-(eta * DeltaT/Qn_rated); b1; b2];
    X   = (A * X) + (B * U);
    P_x = (A * P_x * A') + Q_x;
    KalmanGain_x = (P_x) * (C_x') * (inv((C_x * P_x * C_x') + (R_x)));
    X            = X + (KalmanGain_x * Error_x);
    P_x          = (eye(n_x,n_x) - (KalmanGain_x * C_x)) * P_x;
end