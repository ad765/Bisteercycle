function Xlin = outerLoop(param,X,t)

% Unpack parameters
h       = param.h;
lF      = param.lF;
lR      = param.lR;

% Path
xdr   	= param.xdr(t);
ydr  	= param.ydr(t);
zdr   	= param.zdr(t);
sr  	= param.sr(t);


% Function to optimize
    function F = zeroCalc(X)
        
        % Solver variables
        p       = X(1);
        V       = X(2);
        
        % Equations
        F(1)    =  h*pd*cos(p)*sin(sr) - V*(sin(dlr)*sin(sr) - cos(dlr)*cos(p)*cos(sr))...
            - (V*sin(dlf - dlr)*(lR*sin(sr) - h*cos(sr)*sin(p)))/(cos(dlf)*(lF + lR))...
            - xdr;
        F(2)    = -h*pd*cos(p)*cos(sr) + V*(cos(sr)*sin(dlr) + cos(dlr)*cos(p)*sin(sr))...
            + (V*sin(dlf - dlr)*(lR*cos(sr) + h*sin(p)*sin(sr)))/(cos(dlf)*(lF + lR))...
            - ydr;
        
    end

% Options
opts    = optimoptions('fsolve','Display','none');

% Solve optimizer
soln    = fsolve(@zeroCalc,X(1:3),opts);


end

