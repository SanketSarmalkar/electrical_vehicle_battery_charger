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