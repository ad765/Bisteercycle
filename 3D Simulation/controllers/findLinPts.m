function Xlin = findLinPts(param,X)

% Unpack parameters
I11     = param.I11;
I22     = param.I22;
I33     = param.I33;
I13     = param.I13;
g       = param.g;
h       = param.h;
lF      = param.lF;
lR      = param.lR;
m       = param.m;

% Unpack angles
dlf     = X(4);
dlr     = X(5);

% Function to optimize
    function F = zeroCalc(X)
        
        % Solver variables
        p       = X(1);
        V       = X(2);
        pd      = X(3);
        
        % Equations
        F(1)    = pd;
        F(2)    = Vdot(I11,I13,I22,I33,0,0,V,dlf,0,dlr,0,g,h,lF,lR,m,p,pd);
        F(3)    = PHIddot(I11,I13,I22,I33,0,0,V,dlf,0,dlr,0,g,h,lF,lR,m,p,pd);
        
    end

% Options
opts    = optimoptions('fsolve','Display','none');

% Solve optimizer
soln    = fsolve(@zeroCalc,X(1:3),opts);

% Solution
Xlin    = [soln, dlf, dlr];

end

