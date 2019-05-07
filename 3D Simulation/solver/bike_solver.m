function S = bike_solver(param,tf,tol)

% Derive equations of motion
if strcmp(param.eom,'y')
    AMB
    fprintf('Done solving AMB equations.\n')
else
    fprintf('Equations already solved.\n')
end


% initial conditions
X0 = [param.X0;
    param.Y0;
    param.P0;
    param.S0;
    param.V0;
    param.Pd0;
    param.dlf0;
    param.dlr0];

% options
evn     = @( t, X ) bike_events( t, X, param );
opts    = odeset( 'RelTol', tol, 'AbsTol', tol, 'Events', evn);

% solve
ode = @( t, X ) bike_eom( t, X, param );
S = ode45( ode, [0 tf], X0, opts);

fprintf('Done solving ODE.\n')

end