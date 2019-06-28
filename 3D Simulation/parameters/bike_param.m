function bike_p = bike_param()

% Bike properties
bike_p.m     = 1;
bike_p.g     = 10;
bike_p.I11   = 0.1;
bike_p.I22   = 0.2;
bike_p.I33   = 0.1;
bike_p.I13   = 0;

% Distance to wheels
bike_p.lR = 1;
bike_p.lF = 1;

% Height of bicycle
bike_p.h = 1;

% Handle-bar width
bike_p.hw = 0.5;

% Radius of wheels
bike_p.rR = 0.3;
bike_p.rF = 0.3;

% Friction
bike_p.cR = 0;
bike_p.cF = 0;

end