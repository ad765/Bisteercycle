function comparison( S1, S2, p, POINTS, b, c )

% Unpack parameters
lR  = p.lR;
lF  = p.lF;

T = linspace( S1.x(1), S1.x(end), POINTS );
Z1 = deval(S1,T);
Z2 = deval(S2,T);

% Unpack state
% DAE solution
THETA1   = Z1(3,:);
XG1      = Z1(1,:);
YG1      = Z1(2,:);
THETAdot1= Z1(6,:);
XGdot1 	= Z1(4,:);
YGdot1 	= Z1(5,:);
% dlR1     = Z1(7,:);
% dlF1     = Z1(8,:);

% AMB solution
THETA2   = Z2(3,:);
XG2      = Z2(1,:) + lR*cos(THETA2);
YG2      = Z2(2,:) + lR*sin(THETA2);
VR2      = Z2(4,:);
dlR2     = Z2(5,:);
dlF2     = Z2(6,:);

% Time histories
figure(b)
hold on
grid on
plot(T,XG1-XG2,'b-');
plot(T,YG1-YG2,'r-');
plot(T,mod(THETA1,2*pi)-mod(THETA2,2*pi), 'k-' )
xlabel('t (s)')
ylabel('\Delta s (m)')
title('Position differences ($x$ and $y$) vs. $t$','Interpreter','latex')
legend({'\Delta X','\Delta Y','\Delta \theta'})
hold off

% Energy
tauR2 = (cos(dlF2)*(lF + lR))./sin(dlF2 - dlR2);
THETAdot2 = VR2./tauR2;
XGdot2 = VR2.*cos(THETA2+dlR2) - lR*THETAdot2.*sin(THETA2);
YGdot2 = VR2.*sin(THETA2+dlR2) + lR*THETAdot2.*cos(THETA2);
E2 = (1/2)*p.m*(XGdot2.^2 + YGdot2.^2) + (1/2)*p.I*THETAdot2.^2;   
E1 = (1/2)*p.m*(XGdot1.^2 + YGdot1.^2) + (1/2)*p.I*THETAdot1.^2;
figure(c)
hold on
grid on
plot(T,E1-E2,'k-');
title('Kinetic Energy Difference vs. Time')
xlabel('t (s)')
ylabel('\Delta E (J)')
hold off
%}

end