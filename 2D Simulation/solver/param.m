function p = param(method)

% solution
p.eom = 'n';
p.SOL = method;

p.m = 10;
p.I = 10;

% Distance to wheels
p.lF = 1;
p.lR = 1;

% Radius of wheels
p.rF = 0.4;
p.rR = 0.4;

% Friction
p.cR = 0;
p.cF = 0;

% Control inputs
% Drive
p.TF	= @(t) 0;
p.TR	= @(t) 0;

% Fixed angle steering
% DEFAULT
p.dlFd 	= @(t) 0;
p.dlRd 	= @(t) 0;
%}

% Exponential decay front wheel steering
%{
p.a = pi/3;
p.b = 2;
p.dlFd 	= @(t) -(2*a*b*t)./(b*t.^2 + 1).^2;
p.dlRd  = @(t) 0;
%}

% Sinusoidal both wheels steering
%
p.w1        = 1;
p.w2        = 2;
p.dlR_max   = pi/4;
p.dlF_max   = pi/6;
p.dlFd      = @(t) p.dlF_max*p.w2*cos(p.w2*t);
p.dlRd      = @(t) p.dlR_max*p.w1*cos(p.w1*t);
%}
 
% Initial conditions
p.deltaR0 = pi/3;   % Initial rear wheel steer angle
p.deltaF0 = pi/4;   % Initial front wheel steer angle
p.X0      = 1;      % Initial COM x-position
p.Y0      = 0;      % Initial COM y-position
p.THETA0  = pi/2;   % Initial body orientation
p.VR0     = 10;     % Initial rear wheel velocity


end