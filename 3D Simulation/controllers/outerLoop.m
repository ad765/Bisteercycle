function Xlin = outerLoop(param,X)

% Unpack parameters
h       = param.h;
lF      = param.lF;
lR      = param.lR;

% Path
xd      = param.xdr;
yd      = param.ydr;
zd      = param.zdr;
s       = param.sr;

% Function to optimize
    function F = zeroCalc(X)
        
        % Solver variables
        p       = X(1);
        V       = X(2);
        pd      = X(3);
        dlf     = X(4);
        dlr     = X(5);
        
        % Equations
        F(1)    =  h*pd*cos(p)*sin(s) - V*(sin(dlr)*sin(s) - cos(dlr)*cos(p)*cos(s))...
            - (V*sin(dlf - dlr)*(lR*sin(s) - h*cos(s)*sin(p)))/(cos(dlf)*(lF + lR))...
            - xd;
        F(2)    = -h*pd*cos(p)*cos(s) + V*(cos(s)*sin(dlr) + cos(dlr)*cos(p)*sin(s))...
            + (V*sin(dlf - dlr)*(lR*cos(s) + h*sin(p)*sin(s)))/(cos(dlf)*(lF + lR))...
            - yd;
        F(3)    = -h*pd*sin(p) - zd;
        F(4)    = V*sin(dlf - dlr)/(cos(dlf)*(lF + lR)) - psid;
        
    end

% Options
opts    = optimoptions('fsolve','Display','none');

% Solve optimizer
soln    = fsolve(@zeroCalc,X(1:3),opts);


end

