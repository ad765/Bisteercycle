function p = param()

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% INITIAL CONDITIONS %%%%%%%%%
X0      = 0;
Y0      = 0;
P0      = pi/6;
S0      = 0;
V0      = 5;
Pd0     = 0;
dlr0    = 0;
dlf0    = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% REFERENCE INPUTS %%%%%%%%%%%
p.pr  	= @(t) pi/6;
p.Vr    = @(t) 2;
p.pdr   = @(t) 0;
p.dlfr  = @(t) pi/3*sin(t);
p.dlrr  = @(t) 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% LQR WEIGHTING %%%%%%%%%%%%
p.Q = diag([1e3,1e1,1e3,1e2,1e2]);
p.R = diag([1e0,1e0,1e0,1e0]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Solution
p.eom = 'n';


%% Parameters
p.m     = 1;
p.g     = 10;
p.I11   = 0.1;
p.I22   = 0.2;
p.I33   = 0.1;
p.I13   = 0;

% Distance to wheels
p.lR = 1;
p.lF = 1;

% Height of bicycle
p.h = 1;

% Handle-bar width
p.hw = 0.5;

% Radius of wheels
p.rR = 0.3;
p.rF = 0.3;

% Friction
p.cR = 0;
p.cF = 0;


%% Initial condition
p.X0    = X0;
p.Y0    = Y0;
p.P0    = P0;
p.S0    = S0;
p.V0    = V0;
p.Pd0   = Pd0;
p.dlf0  = dlf0;
p.dlr0  = dlr0;

% Falling detection
p.fall  = pi/180 * 2; % radians


%% Control input

% PD Controller
%{
Kp_p_T = 30;
Kd_p_T = 80;

Kp_s_T = 0;
Kd_s_T = 0;

K_T = [Kp_p_T, Kd_p_T, ...
    Kp_s_T, Kd_s_T];

p.TR    = @(pp,pd,sp,sd,xp,xd,yp,yd) -K_T*[pp,pd,sp,sd]';
p.TF    = @(pp,pd,sp,sd,xp,xd,yp,yd) 0; %-K_T*[pp,pd,sp,sd,xp,xd,yp,yd]';


% Closed-loop steering
Kp_p_dlf  = 0;
Kd_p_dlf  = 0;
p.dlfd 	= @(p,pd) -Kp_p_dlf*p - Kd_p_dlf*pd;
p.dlrd 	= @(p,pd) -Kp_p_dlf*p - Kd_p_dlf*pd;
%}

% Gain-scheduled LQR controller
%
sp          = 10;
p.method    = 'cubic';
p.dlflim    = linspace( -pi/3, pi/3, sp );
p.dlrlim    = linspace( -pi/3, pi/3, sp );
p.Vlim      = linspace( 0.1, 10, sp );
p.Kmat      = gainSched(p);
%}


end