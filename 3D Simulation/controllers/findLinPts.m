function Xlin = findLinPts(bike_p,X)

% Unpack parameters
I11     = bike_p.I11;
I22     = bike_p.I22;
I33     = bike_p.I33;
I13     = bike_p.I13;
g       = bike_p.g;
h       = bike_p.h;
lF      = bike_p.lF;
lR      = bike_p.lR;
m       = bike_p.m;

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
opts    = optimoptions('fsolve','Algorithm','levenberg-marquardt','Display','none');

% Solve optimizer
soln    = fsolve(@zeroCalc,X(1:3),opts);

% Solution
Xlin    = [soln, dlf, dlr];

end

