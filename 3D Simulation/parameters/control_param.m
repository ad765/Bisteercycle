function ctr_p = control_param( sys_p, bike_p, ctr_p )

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
sp              = 11;
ctr_p.method    = 'cubic';
ctr_p.dlflim    = linspace( -pi/3, pi/3, sp );
ctr_p.dlrlim    = linspace( -pi/3, pi/3, sp );
ctr_p.Vlim      = linspace( 0.1, 100, sp );
ctr_p.Kmat      = gainSched( sys_p, bike_p, ctr_p );
%}


end