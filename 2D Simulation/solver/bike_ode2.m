function dZ = bike_ode2(t,Z,p)

% Unpack parameters
SOL = p.SOL;
I = p.I;
TF = p.TF(t);
TR = p.TR(t);
cF = p.cF;
cR = p.cR;
dlFd = p.dlFd(t);
dlRd = p.dlRd(t);
lF = p.lF;
lR = p.lR;
m = p.m;


if strcmp(SOL,'DAE')
    
    % Unpack state
    % xR = Z(1);
    % yR = Z(2);
    th = Z(3);
    xRd = Z(4);
    yRd = Z(5);
    thd = Z(6);
    dlR = Z(7);
    dlF = Z(8);
    
    A = A_DAE(I,dlF,dlR,lF,lR,m,th);
    b = b_DAE(TF,TR,cF,cR,dlF,dlFd,dlR,dlRd,lF,lR,th,thd,xRd,yRd);
    
    soln = A\b;
    
    dZ = [Z(4:6);
          soln(1:3);
          dlRd;
          dlFd];
      
    % fprintf( 't = %d,\t deltaR = %d,\t deltaF = %d,\t thd = %d\n', t, dlR, dlF, thd )
    
elseif strcmp(SOL,'AMB')
    
    xR = Z(1);
    yR = Z(2);
    th = Z(3);
    vR = Z(4);
    dlR = Z(5);
    dlF = Z(6);
    
    thd = vR*sin(dlF - dlR)/(cos(dlF)*(lF + lR));
    acc	= vRd(I,TF,TR,cF,cR,dlF,dlFd,dlR,dlRd,lF,lR,m,th,thd,vR);
    
    xRd = vR*cos(th+dlR);
    yRd = vR*sin(th+dlR);
    
    dZ = [xRd;
          yRd;
          thd;
          acc;
          dlRd;
          dlFd];
    
    % fprintf( 't = %d,\t deltaR = %d,\t deltaF = %d,\t thd = %d\n', t, dlR, dlF, thd )
    
end

end