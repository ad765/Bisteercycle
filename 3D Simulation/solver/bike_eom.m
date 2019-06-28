function dX = bike_eom( t, X, bike_p, ctr_p, sys_p )
% Full state:       X   = [x, y, p, s, V, pd, dlf, dlr]
% Cyclic state:     Zc  = [x, y, s]
% Non-cyclic state: Znc = [p, V, pd, dlf, dlr]
% Control:          U   = [TF, TR, dlfd, dlrd]

% Unpack parameters
I11     = bike_p.I11;
I22     = bike_p.I22;
I33     = bike_p.I33;
I13     = bike_p.I13;
g       = bike_p.g;
h       = bike_p.h;
lF      = bike_p.lF;
lR      = bike_p.lR;
m       = bike_p.m;

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
pr      = sys_p.pr(t);
Vr      = sys_p.Vr(t);
pdr     = sys_p.pdr(t);
dlfr    = sys_p.dlfr(t);
dlrr    = sys_p.dlrr(t);

% Reference inputs
%{
Xlin    = findLinPts(bike_p,[0,sys_p.Vr(t),0,sys_p.dlfr(t),sys_p.dlrr(t)]);
pr      = Xlin(1);
Vr      = Xlin(2);
pdr     = Xlin(3);
dlfr    = Xlin(4);
dlrr    = Xlin(5);
%}

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
K = zeros(size(ctr_p.Kmat));
for i = 1:size(ctr_p.Kmat,1)
    for j = 1:size(ctr_p.Kmat,2)
        K(i,j) = ctr_p.Kmat{i,j}(dlf,dlr,V);
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