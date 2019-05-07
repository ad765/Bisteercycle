function K = fixedLQR(param,Xlin)
% LQR controller

% Unpack parameters
I11 = param.I11;
I22 = param.I22;
I33 = param.I33;
I13 = param.I13;
g = param.g;
h = param.h;
lF = param.lF;
lR = param.lR;
m = param.m;
Q = param.Q;
R = param.R;

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