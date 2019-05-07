function K = bikeLQR(param,X)
% Gain-scheduled LQR controller

% Find closest equilibrium point to state
Xlin    = findLinPts(param,X);

% Calculate K matrix based on 
K       = fixedLQR(param,Xlin);

end