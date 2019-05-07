function S = bike_solver(p,tf)

% Derive equations of motion
if strcmp(p.eom,'y')
    if strcmp(p.SOL,'DAE')
        DAE_deriver
        fprintf('Done solving DAE equations.\n')
    elseif strcmp(p.SOL,'AMB')
        AMB_deriver2
        fprintf('Done solving AMB equations.\n')
    end
end

% Solution method
SOL = p.SOL;

% initial conditions
deltaR0 = p.deltaR0;  	% Initial rear wheel steer angle
deltaF0 = p.deltaF0; 	% Initial front wheel steer angle
X0      = p.X0;         % Initial COM x-position
Y0      = p.Y0;         % Initial COM y-position
THETA0  = p.THETA0; 	% Initial body orientation
VR0     = p.VR0;        % Initial rear wheel velocity

% parameters
lR = p.lR;

if strcmp(SOL,'DAE')
    Z0 = initConds(X0,Y0,THETA0,VR0,deltaR0,deltaF0,p);
elseif strcmp(SOL,'AMB')
    Z0	= [X0-lR*cos(THETA0) Y0-lR*sin(THETA0)...
        THETA0 VR0 deltaR0 deltaF0];  % initial conditions
end

% options
tol     = 1e-9;
opts    = odeset( 'RelTol', tol, 'AbsTol', tol);

% solve
ode = @( t, y ) bike_ode2( t, y, p );
S = ode45( ode, [0 tf], Z0, opts);

end