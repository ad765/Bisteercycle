function dX = bike_eom( t, X, param )
% Full state:       X   = [x, y, p, s, V, pd, dlf, dlr]
% Cyclic state:     Zc  = [x, y, s]
% Non-cyclic state: Znc = [p, V, pd, dlf, dlr]
% Control:          U   = [TF, TR, dlfd, dlrd]

% Unpack parameters
I11     = param.I11;
I22     = param.I22;
I33     = param.I33;
I13     = param.I13;
g       = param.g;
h       = param.h;
lF      = param.lF;
lR      = param.lR;
m       = param.m;

% Unpack state
% x       = X(1);
% y       = X(2);
p       = X(3);
s       = X(4);
V       = X(5);
pd      = X(6);
dlf     = X(7);
dlr     = X(8);

% Reference inputs
pr      = param.pr(t);
Vr      = param.Vr(t);
pdr     = param.pdr(t);
dlfr    = param.dlfr(t);
dlrr    = param.dlrr(t);

% Calculate derivative variables
xd      = -V*(sin(dlr)*sin(s) - cos(dlr)*cos(p)*cos(s));
yd      =  V*(cos(s)*sin(dlr) + cos(dlr)*cos(p)*sin(s));
sd      =  V*sin(dlf - dlr)/(cos(dlf)*(lF + lR));

% PD controller
%{
TR      = param.TR(p,pd,s,sd);
TF      = param.TF(p,pd,s,sd);
dlfd    = param.dlfd(p,pd);
dlrd    = param.dlrd(p,pd);
%}

% LQR controller
%{
u       = - param.K *[p-pr, V-Vr, pd-pdr, dlf-dlfr, dlr-dlrr]';
TF      = u(1);
TR      = u(2);
dlfd    = u(3);
dlrd    = u(4);
%}

% Perfect gain-scheduled LQR controller
%{
K       = bikeLQR(param,[p,V,pd,dlf,dlr]);
u       = -K * [p-pr; V-Vr; pd-pdr; dlf-dlfr; dlr-dlrr];
TF      = u(1);
TR      = u(2);
dlfd    = u(3);
dlrd    = u(4);
%}

% Gain-scheduled LQR controller
%
K = zeros(size(param.Kmat));
for i = 1:size(param.Kmat,1)
    for j = 1:size(param.Kmat,2)
        K(i,j) = param.Kmat{i,j}(dlf,dlr,V);
    end
end
u       = -K * [p-pr; V-Vr; pd-pdr; dlf-dlfr; dlr-dlrr];
TF      = u(1);
TR      = u(2);
dlfd    = u(3);
dlrd    = u(4);
%}

% Nonlinear EOM
Vd      =    Vdot(I11,I13,I22,I33,TF,TR,V,dlf,dlfd,dlr,dlrd,g,h,lF,lR,m,p,pd);
pdd     = PHIddot(I11,I13,I22,I33,TF,TR,V,dlf,dlfd,dlr,dlrd,g,h,lF,lR,m,p,pd);

% State derivative
dX      = [xd; yd; pd; sd; Vd; pdd; dlfd; dlrd];

disp(t)

end