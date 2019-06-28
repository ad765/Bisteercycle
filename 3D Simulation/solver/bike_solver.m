function S = bike_solver(sys_p,bike_p,ctr_p,tf,tol)

% Derive equations of motion
if strcmp(sys_p.eom,'y')
    AMB
    fprintf('Done solving AMB equations.\n')
else
    fprintf('Equations already solved.\n')
end


% initial conditions
X0 = [sys_p.X0;
    sys_p.Y0;
    sys_p.P0;
    sys_p.S0;
    sys_p.V0;
    sys_p.Pd0;
    sys_p.dlf0;
    sys_p.dlr0];

% options
evn     = @( t, X ) bike_events( t, X, sys_p );
opts    = odeset( 'RelTol', tol, 'AbsTol', tol, 'Events', evn);

% solve
ode = @( t, X ) bike_eom( t, X, bike_p, ctr_p, sys_p );
S = ode45( ode, [0 tf], X0, opts);

fprintf('Done solving ODE.\n')

end