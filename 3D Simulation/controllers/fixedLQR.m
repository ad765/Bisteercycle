function K = fixedLQR(bike_p,sys_p,Xlin)
% LQR controller

% Bicycle parameters
I11 = bike_p.I11;
I22 = bike_p.I22;
I33 = bike_p.I33;
I13 = bike_p.I13;
g = bike_p.g;
h = bike_p.h;
lF = bike_p.lF;
lR = bike_p.lR;
m = bike_p.m;

% LQR parameters
Q = sys_p.Q;
R = sys_p.R;

% Linearization points
p      = Xlin(1);
V      = Xlin(2);
pd     = Xlin(3);
dlf    = Xlin(4);
dlr    = Xlin(5);

% Linearization
Amat = A_lin(I11,I13,I22,I33,0,0,V,dlf,0,dlr,0,g,h,lF,lR,m,p,pd);
Bmat = B_lin(I11,I13,I22,I33,V,dlf,dlr,h,lF,lR,m,p);

% LQR calculation
[K,~,~] = lqr( Amat, Bmat, Q, R);

end