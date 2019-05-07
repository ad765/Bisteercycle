function plotting( ans_struct, param, POINTS, a, b, c, d, e  )

% Unpack parameters
h = param.h;
lR = param.lR;
lF = param.lF;

time = linspace( ans_struct.x(1), ans_struct.x(end), POINTS );
soln = deval(ans_struct,time);

% Unpack state
X   = soln(1,:);
Y   = soln(2,:);
P   = soln(3,:);
S   = soln(4,:);
V   = soln(5,:);
Pd  = soln(6,:);
dlf = soln(7,:);
dlr = soln(8,:);

% Extract data
XG      = X + h*sin(P).*sin(S) + lR*cos(S);
YG      = Y - h*sin(P).*cos(S) + lR*sin(S);
ZG      = h*cos(P);
Sd      = PSIdot(V,dlf,dlr,lF,lR);
XGdot   =  h*Pd.*cos(P).*sin(S) - Sd.*(lR*sin(S) - h*cos(S).*sin(P)) - V.*(sin(dlr).*sin(S) - cos(dlr).*cos(P).*cos(S));
YGdot   = -h*Pd.*cos(P).*cos(S) + Sd.*(lR*cos(S) + h*sin(S).*sin(P)) + V.*(sin(dlr).*cos(S) + cos(dlr).*cos(P).*sin(S));
ZGdot   = -h*Pd.*sin(P);

% Control
TF = zeros(size(time));
TR = zeros(size(time));
dlfd = zeros(size(time));
dlrd = zeros(size(time));
%
for k = 1:length(time)
    K = zeros(size(param.Kmat));
    for i = 1:size(param.Kmat,1)
        for j = 1:size(param.Kmat,2)
            K(i,j) = param.Kmat{i,j}(dlf(k),dlr(k),V(k));
        end
    end
    u           = -K * [P(k)-param.pr(time(k));
                        V(k)-param.Vr(time(k));
                        Pd(k)-param.pdr(time(k));
                        dlf(k)-param.dlfr(time(k));
                        dlr(k)-param.dlrr(time(k))];
    TF(k)       = u(1);
    TR(k)       = u(2);
    dlfd(k)     = u(3);
    dlrd(k)     = u(4);
end
%}

% Trajectory
figure(a)
hold on
grid on
axis equal
plot3(XG(1),YG(1),ZG(1),'ro');
plot3(XG(end),YG(end),ZG(end),'rx');
plot3(XG,YG,ZG,'k-');
title('Trajectory')
legend({'start','end'})
xlabel('x')
ylabel('y')
hold off

%
% Time histories
figure(b)
subplot(2,1,1)
hold on
grid on
plot(time,XG,'b-');
plot(time,YG,'r-');
plot(time,ZG,'k-');
xlabel('t (s)')
ylabel('r (m)')
title('Position vs. $t$ ','Interpreter','latex')
xlabel('Time (t)')
legend({'x','y','z'},'Interpreter','latex')
hold off
subplot(2,1,2)
hold on
grid on
plot(time,XGdot,'b-');
plot(time,YGdot,'r-');
plot(time,ZGdot,'k-');
xlabel('t (s)')
ylabel('v (m/s)')
title('Velocity vs. $t$ ','Interpreter','latex')
xlabel('Time (t)')
legend({'$\dot{x}$','$\dot{y}$','$\dot{z}$'},'Interpreter','latex')
hold off

% Time histories (angles)
figure(c)
subplot(2,1,1)
subplot(2,1,1)
hold on
grid on
plot(time,P,'b-');
plot(time,S,'r-');
xlabel('t (s)')
ylabel('r (m)')
title('Angular Position vs. $t$ ','Interpreter','latex')
xlabel('Time (t)')
legend({'$\phi$','$\psi$'},'Interpreter','latex')
hold off
subplot(2,1,2)
hold on
grid on
plot(time,Pd,'b-');
plot(time,Sd,'r-');
xlabel('t (s)')
ylabel('v (m/s)')
title('Angular Rates vs. $t$ ','Interpreter','latex')
xlabel('Time (t)')
legend({'$\dot{\phi}$','$\dot{\psi}$'},'Interpreter','latex')
hold off



% Energy
[E, LKE, RKE, PE] = calcEnergy( time, ans_struct, param );
figure(d)
hold on
grid on
plot(time,E,'k-','LineWidth',2);
plot(time,PE,'b-');
plot(time,LKE+RKE,'r-')
title('Energy vs. Time ')
xlabel('Time')
ylabel('Energy')
legend({'E','PE','KE'},'Interpreter','latex')
hold off

% Control inputs
figure(e)
subplot(3,1,1)
hold on
grid on
plot( time, dlr, 'r-' )
plot( time, dlf, 'b-' )
title('Steering Angle vs. Time ')
legend({'$\delta_{R}$','$\delta_{F}$'}, 'Interpreter', 'latex' )
hold off
%
subplot(3,1,2)
hold on
grid on
plot( time, dlrd, 'r-' )
plot( time, dlfd, 'b-' )
title('Control Input vs. Time ')
legend({'$\dot{\delta}_{R}$','$\dot{\delta}_{F}$'}, 'Interpreter', 'latex' )
hold off
subplot(3,1,3)
hold on
grid on
plot( time, TR, 'r-' )
plot( time, TF, 'b-' )
title('Thrust Input vs. Time')
legend({'TR','TF'})
hold off
%}

end