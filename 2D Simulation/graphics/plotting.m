function plotting( S, p, POINTS, a, b, c, d  )

% Unpack parameters
SOL = p.SOL;
lR  = p.lR;
lF  = p.lF;

T = linspace( S.x(1), S.x(end), POINTS );
Z = deval(S,T);

% Unpack state
if strcmp(SOL,'DAE')
    THETA   = Z(3,:);
    XG      = Z(1,:);
    YG      = Z(2,:);
    THETAdot= Z(6,:);
    XGdot 	= Z(4,:);
    YGdot 	= Z(5,:);
    dlR     = Z(7,:);
    dlF     = Z(8,:);
    E = (1/2)*p.m*(XGdot.^2 + YGdot.^2) + (1/2)*p.I*THETAdot.^2;
elseif strcmp(SOL,'AMB')
    THETA   = Z(3,:);
    XG      = Z(1,:) + lR*cos(THETA);
    YG      = Z(2,:) + lR*sin(THETA);
    VR      = Z(4,:);
    dlR     = Z(5,:);
    dlF     = Z(6,:);
    tauR = (cos(dlF)*(lF + lR))./sin(dlF - dlR);
    THETAdot = VR./tauR;
    XGdot = VR.*cos(THETA+dlR) - lR*THETAdot.*sin(THETA);
    YGdot = VR.*sin(THETA+dlR) + lR*THETAdot.*cos(THETA);
    E = (1/2)*p.m*(XGdot.^2 + YGdot.^2) + (1/2)*p.I*THETAdot.^2;
end

% Trajectory
figure(a)
hold on
grid on
axis equal
plot(XG(1),YG(1),'ro');
plot(XG(end),YG(end),'rx');
plot(XG,YG,'k-');
title(['Trajectory ', SOL])
legend({'start','end'})
xlabel('x')
ylabel('y')
hold off

% Time histories
figure(b)
subplot(2,1,1)
hold on
grid on
plot(T,XG,'b-');
plot(T,YG,'r-');
xlabel('t (s)')
ylabel('r (m)')
title(['Position ($x$ and $y$) vs. $t$ ', SOL],'Interpreter','latex')
xlabel('Time (t)')
legend({'x','y'},'Interpreter','latex')
hold off
subplot(2,1,2)
hold on
grid on
plot(T,XGdot,'b-');
plot(T,YGdot,'r-');
xlabel('t (s)')
ylabel('v (m/s)')
title(['Velocity ($x$ and $y$) vs. $t$ ', SOL],'Interpreter','latex')
xlabel('Time (t)')
legend({'$\dot{x}$','$\dot{y}$'},'Interpreter','latex')
hold off

% Energy
%
figure(c)
hold on
grid on
plot(T,E,'k-','LineWidth',2);
title(['Kinetic Energy vs. Time ', SOL])
xlabel('Time')
ylabel('Energy')
hold off
%}

% Control inputs
figure(d)
hold on
grid on
plot( T, dlF, 'b-' )
plot( T, dlR, 'r-' )
plot( T, mod(THETA,2*pi), 'k-' )
title(['Angles vs. Time ', SOL])
legend({'$\delta_{F}$','$\delta_{R}$','$\theta$'}, 'Interpreter', 'latex' )
hold off
%}

end