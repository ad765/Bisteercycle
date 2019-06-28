    function sys_p = system_param(bike_p)

% Solution
sys_p.eom = 'n';

% Falling detection
sys_p.fall  = pi/180 * 2; % radians

%%%%%%%% INITIAL CONDITIONS %%%%%%%%%
sys_p.X0    = 0;
sys_p.Y0    = 0;
sys_p.P0    = pi/6;
sys_p.S0    = 0;
sys_p.V0    = 5;
sys_p.Pd0   = 0;
sys_p.dlf0  = 0;
sys_p.dlr0  = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%% REFERENCE INPUTS %%%%%%%%%%%
sys_p.dlfr  = @(t) pi/3;
sys_p.dlrr  = @(t) pi/4;
sys_p.Vr    = @(t) 2;
Xlin        = findLinPts(bike_p,[0,sys_p.Vr(0),0,sys_p.dlfr(0),sys_p.dlrr(0)]);
sys_p.pr    = @(t) 0;
sys_p.Vr    = @(t) Xlin(2);
sys_p.pdr   = @(t) 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%% LQR WEIGHTING %%%%%%%%%%%%
sys_p.Q = diag([1e3,1e1,1e3,1e2,1e2]);
sys_p.R = diag([1e0, 1e0, 1e0, 1e0]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end