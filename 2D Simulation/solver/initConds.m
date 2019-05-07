function Z0 = initConds(X0,Y0,THETA0,VR,deltaR0,deltaF0,p)

% Unpack parameters
dlF0 = deltaF0;
dlR0 = deltaR0;
lF = p.lF;
lR = p.lR;

% If the wheels aren't parallel
if mod(abs(dlF0-dlR0),pi) >= 1e-10
    
    tau = cos(dlF0)*(lF + lR)/sin(dlF0 - dlR0);
    THETAdot0 = VR/tau(1);
    Xdot0 = VR*cos(THETA0+dlR0) - lR*THETAdot0*sin(THETA0);
    Ydot0 = VR*sin(THETA0+dlR0) + lR*THETAdot0*cos(THETA0);
    Z0 = [X0 Y0 THETA0 Xdot0 Ydot0 THETAdot0 dlR0 dlF0];
    
else
    
    % If the wheels are parallel
    Xdot0 = VR*cos(THETA0+dlR0);
    Ydot0 = VR*sin(THETA0+dlR0);
    Z0 = [X0 Y0 THETA0 Xdot0 Ydot0 0 dlR0 dlF0].';
    
end
