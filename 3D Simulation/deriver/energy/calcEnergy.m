function [E, LKE, RKE, PE] = calcEnergy( time, ans_struct, param)
% Calculate the energy taken throughout the entire passage of the bicycle's
% trajectory
%
% E = (1/2)*m*dot(v_OG,v_OG) + (1/2)*dot(w_BI,H_G)

% Unpack parameters
m = param.m;
g = param.g;
I11 = param.I11;
I22 = param.I22;
I33 = param.I33;
I13 = param.I13;
lF = param.lF;
lR = param.lR;
h = param.h;

% Evaluate at time
soln = deval(ans_struct,time);

% Unpack state
p   = soln(1,:);
s   = soln(2,:);
V   = soln(3,:);
pd  = soln(4,:);
dlr = soln(5,:);
dlf = soln(6,:);

sd = PSIdot(V,dlf,dlr,lF,lR);

% Energy
LKE = AMB_LKE(V,dlr,h,lR,m,p,pd,s,sd);
RKE = AMB_RKE(I11,I13,I22,I33,p,pd,s,sd);
PE  = AMB_PE(g,h,m,p);

E = LKE + RKE + PE;

end