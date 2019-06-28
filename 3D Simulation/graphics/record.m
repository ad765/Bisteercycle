function record( ans_struct, bike_p, name)
% Real-time animation of the bicycle

% Unpack parameters
rF  = bike_p.rF;
rR  = bike_p.rR;
lF  = bike_p.lF;
lR  = bike_p.lR;
h   = bike_p.h;
hw  = bike_p.hw;

% Maximum distance
dist = 2*max([lF,lR]);

% Animate simulation
anim = figure;%('units','normalized','outerposition',[0 0 1 1]);
hold on

% Global view
subplot(2,2,1)
hold on
grid on
axis equal
title('Global view of bicycle')
ax1 = gca;
set( ax1, 'View', [45,20] )
global_axes = [min(ans_struct.y(1,:))-dist-h, max(ans_struct.y(1,:))+dist+h, ...
    min(ans_struct.y(2,:))-dist-h, max(ans_struct.y(2,:))+dist+h, 0, h+dist];
axis( ax1, global_axes )
b1  = plot3(ax1,0,0,0,'k-','LineWidth',2);
hr1 = plot3(ax1,0,0,0,'r-','LineWidth',2);
hf1 = plot3(ax1,0,0,0,'b-','LineWidth',2);
wr1 = plot3(ax1,0,0,0,'r-','LineWidth',2);
wf1 = plot3(ax1,0,0,0,'b-','LineWidth',2);

% Local view
subplot(2,2,2)
hold on
grid on
axis equal
title('Local view of bicycle')
ax2 = gca;
set( ax2, 'View', [45,20] );
b2  = plot3(ax2,0,0,0,'k-','LineWidth',2);
hr2 = plot3(ax2,0,0,0,'r-','LineWidth',2);
hf2 = plot3(ax2,0,0,0,'b-','LineWidth',2);
wr2 = plot3(ax2,0,0,0,'r-','LineWidth',2);
wf2 = plot3(ax2,0,0,0,'b-','LineWidth',2);

% Top-down view
subplot(2,2,3)
hold on
grid on
axis equal
xlabel('X-axis')
ylabel('Y-axis')
title('Top-down view of bicycle')
ax3 = gca;
yaw = plot(ax3,0,0,'k--','LineWidth',2);
b3  = plot(ax3,0,0,'k-','LineWidth',2);
wr3 = plot(ax3,0,0,'r-','LineWidth',2);
wf3 = plot(ax3,0,0,'b-','LineWidth',2);

% Lean view
subplot(2,2,4)
hold on
grid on
axis equal
xlabel('Body Y-axis')
ylabel('Body Z-axis')
title('Bicycle Lean')
plot([0,0],[0,h*1.2],'k--','LineWidth',2);
lean = plot(0,0,'k-','LineWidth',2);
mass = plot(0,0,'ko','MarkerSize',15,'MarkerFaceColor',[0,0,0]);

hold off

% Set graphics properties
set( anim, 'DoubleBuffer', 'on' )

% Create wheel parameterization
tau = linspace(0, 2*pi, 16);

% Update graphics
for i = 1:length(ans_struct.x)
    
    soln = deval(ans_struct,ans_struct.x(i));
    
    % Unpack state
    X   = soln(1,:);
    Y   = soln(2,:);
    P   = soln(3,:);
    S   = soln(4,:);
    dlf = soln(7,:);
    dlr = soln(8,:);
    
    % Calculation of coordinates of points of interest
    rear_wheel_X    = X;
    rear_wheel_Y    = Y;
    rear_wheel_Z    = 0;
    bike_tail_X     = rear_wheel_X + h*sin(P)*sin(S);
    bike_tail_Y     = rear_wheel_Y - h*cos(S)*sin(P);
    bike_tail_Z     = rear_wheel_Z + h*cos(P);
    bike_front_X    = bike_tail_X + cos(S)*(lF + lR);
    bike_front_Y    = bike_tail_Y + sin(S)*(lF + lR);
    bike_front_Z    = bike_tail_Z;
    front_wheel_X   = bike_front_X - h*sin(P)*sin(S);
    front_wheel_Y   = bike_front_Y + h*cos(S)*sin(P);
    front_wheel_Z   = bike_front_Z - h*cos(P);
    
    % Calculation of rear handle bar
    hand_R_center_X = bike_tail_X + 0.2*h*sin(P)*sin(S);
    hand_R_center_Y = bike_tail_Y - 0.2*h*cos(S)*sin(P);
    hand_R_center_Z = bike_tail_Z + 0.2*h*cos(P);
    hand_R_left_X   = hand_R_center_X - 0.5*hw*(cos(S)*sin(dlr) + cos(dlr)*cos(P)*sin(S));
    hand_R_left_Y   = hand_R_center_Y - 0.5*hw*(sin(dlr)*sin(S) - cos(dlr)*cos(P)*cos(S));
    hand_R_left_Z   = hand_R_center_Z + 0.5*hw*cos(dlr)*sin(P);
    hand_R_right_X  = hand_R_left_X + hw*(cos(S)*sin(dlr) + cos(dlr)*cos(P)*sin(S));
    hand_R_right_Y  = hand_R_left_Y + hw*(sin(dlr)*sin(S) - cos(dlr)*cos(P)*cos(S));
    hand_R_right_Z  = hand_R_left_Z - hw*cos(dlr)*sin(P);
    
    % Calculation of front handle bar
    hand_F_center_X = bike_front_X + 0.2*h*sin(P)*sin(S);
    hand_F_center_Y = bike_front_Y - 0.2*h*cos(S)*sin(P);
    hand_F_center_Z = bike_front_Z + 0.2*h*cos(P);
    hand_F_left_X   = hand_F_center_X - 0.5*hw*(cos(S)*sin(dlf) + cos(dlf)*cos(P)*sin(S));
    hand_F_left_Y   = hand_F_center_Y - 0.5*hw*(sin(dlf)*sin(S) - cos(dlf)*cos(P)*cos(S));
    hand_F_left_Z   = hand_F_center_Z + 0.5*hw*cos(dlf)*sin(P);
    hand_F_right_X  = hand_F_left_X + hw*(cos(S)*sin(dlf) + cos(dlf)*cos(P)*sin(S));
    hand_F_right_Y  = hand_F_left_Y + hw*(sin(dlf)*sin(S) - cos(dlf)*cos(P)*cos(S));
    hand_F_right_Z  = hand_F_left_Z - hw*cos(dlf)*sin(P);
    
    % Calculation of wheels
    wheel_RX = rear_wheel_X + rR*sin(P)*sin(S) + rR*cos(tau)*(cos(dlr)*cos(S) - cos(P)*sin(dlr)*sin(S)) + rR*sin(P)*sin(tau)*sin(S);
    wheel_RY = rear_wheel_Y - rR*cos(S)*sin(P) + rR*cos(tau)*(cos(dlr)*sin(S) + cos(P)*cos(S)*sin(dlr)) - rR*cos(S)*sin(P)*sin(tau);
    wheel_RZ = rear_wheel_Z + rR*(cos(P) + cos(P)*sin(tau) + cos(tau)*sin(dlr)*sin(P));
    wheel_FX = front_wheel_X + rF*sin(P)*sin(S) + rF*cos(tau)*(cos(dlf)*cos(S) - cos(P)*sin(dlf)*sin(S)) + rR*sin(P)*sin(S)*sin(tau);
    wheel_FY = front_wheel_Y + rF*cos(tau)*(cos(dlf)*sin(S) + cos(P)*cos(S)*sin(dlf)) - rF*cos(S)*sin(P) - rR*cos(S)*sin(P)*sin(tau);
    wheel_FZ = front_wheel_Z + rF*cos(P) + rR*cos(P)*sin(tau) + rF*cos(tau)*sin(dlf)*sin(P);
    
    % Global view
    set( b1, 'Xdata', [rear_wheel_X, bike_tail_X, bike_front_X, front_wheel_X] );
    set( b1, 'Ydata', [rear_wheel_Y, bike_tail_Y, bike_front_Y, front_wheel_Y] );
    set( b1, 'Zdata', [rear_wheel_Z, bike_tail_Z, bike_front_Z, front_wheel_Z] );
    set( hr1, 'Xdata', [bike_tail_X, hand_R_center_X, hand_R_left_X, hand_R_right_X] );
    set( hr1, 'Ydata', [bike_tail_Y, hand_R_center_Y, hand_R_left_Y, hand_R_right_Y] );
    set( hr1, 'Zdata', [bike_tail_Z, hand_R_center_Z, hand_R_left_Z, hand_R_right_Z] );
    set( hf1, 'Xdata', [bike_front_X, hand_F_center_X, hand_F_left_X, hand_F_right_X] );
    set( hf1, 'Ydata', [bike_front_Y, hand_F_center_Y, hand_F_left_Y, hand_F_right_Y] );
    set( hf1, 'Zdata', [bike_front_Z, hand_F_center_Z, hand_F_left_Z, hand_F_right_Z] );
    set( wr1, 'Xdata', wheel_RX );
    set( wr1, 'Ydata', wheel_RY );
    set( wr1, 'Zdata', wheel_RZ );
    set( wf1, 'Xdata', wheel_FX );
    set( wf1, 'Ydata', wheel_FY );
    set( wf1, 'Zdata', wheel_FZ );
    %plot3( ax1, rear_wheel_X,	rear_wheel_Y,   0,          'r.', 'MarkerSize', 0.5 )
    %plot3( ax1, front_wheel_X,	front_wheel_Y,  0,          'b.', 'MarkerSize', 0.5 )
    
    % Local view
    % local_axes = [X-dist-h, X+dist+h, Y-dist-h, Y+dist+h, 0, h+dist];
    % axis( ax2, local_axes )
    set( ax2, 'XLim', [X-dist-h, X+dist+h] );
    set( ax2, 'YLim', [Y-dist-h, Y+dist+h] );
    set( ax2, 'ZLim', [0, h+dist] );
    set( b2, 'Xdata', [rear_wheel_X, bike_tail_X, bike_front_X, front_wheel_X] );
    set( b2, 'Ydata', [rear_wheel_Y, bike_tail_Y, bike_front_Y, front_wheel_Y] );
    set( b2, 'Zdata', [rear_wheel_Z, bike_tail_Z, bike_front_Z, front_wheel_Z] );
    set( hr2, 'Xdata', [bike_tail_X, hand_R_center_X, hand_R_left_X, hand_R_right_X] );
    set( hr2, 'Ydata', [bike_tail_Y, hand_R_center_Y, hand_R_left_Y, hand_R_right_Y] );
    set( hr2, 'Zdata', [bike_tail_Z, hand_R_center_Z, hand_R_left_Z, hand_R_right_Z] );
    set( hf2, 'Xdata', [bike_front_X, hand_F_center_X, hand_F_left_X, hand_F_right_X] );
    set( hf2, 'Ydata', [bike_front_Y, hand_F_center_Y, hand_F_left_Y, hand_F_right_Y] );
    set( hf2, 'Zdata', [bike_front_Z, hand_F_center_Z, hand_F_left_Z, hand_F_right_Z] );
    set( wr2, 'Xdata', wheel_RX );
    set( wr2, 'Ydata', wheel_RY );
    set( wr2, 'Zdata', wheel_RZ );
    set( wf2, 'Xdata', wheel_FX );
    set( wf2, 'Ydata', wheel_FY );
    set( wf2, 'Zdata', wheel_FZ );
    %plot3( ax2, rear_wheel_X,	rear_wheel_Y,   0,          'r.', 'MarkerSize', 0.5 )
    %plot3( ax2, front_wheel_X,	front_wheel_Y,  0,          'b.', 'MarkerSize', 0.5 )
    
    % Top down view
    set( ax3, 'XLim', [X-dist-h, X+dist+h] );
    set( ax3, 'YLim', [Y-dist-h, Y+dist+h] );
    set( yaw,'Xdata', [rear_wheel_X, rear_wheel_X+1.2*dist] );
    set( yaw,'Ydata', [rear_wheel_Y, rear_wheel_Y] );
    set( b3, 'Xdata', [rear_wheel_X, front_wheel_X] );
    set( b3, 'Ydata', [rear_wheel_Y, front_wheel_Y] );
    set( wr3, 'Xdata',[X + rR*cos(S+dlr+pi), X + rR*cos(S+dlr)] );
    set( wr3, 'Ydata',[Y + rR*sin(S+dlr+pi), Y + rR*sin(S+dlr)] );
    set( wf3, 'Xdata',[X + (lF+lR)*cos(S) + rF*cos(S+dlf+pi), X + (lF+lR)*cos(S) + rF*cos(S+dlf)] );
    set( wf3, 'Ydata',[Y + (lF+lR)*sin(S) + rF*sin(S+dlf+pi), Y + (lF+lR)*sin(S) + rF*sin(S+dlf)] );
    %plot3( ax3, rear_wheel_X,	rear_wheel_Y,   0,          'r.', 'MarkerSize', 0.5 )
    %plot3( ax3, front_wheel_X,	front_wheel_Y,  0,          'b.', 'MarkerSize', 0.5 )
    
    % Lean of bicycle
    set(lean,'Xdata',[0,h*sin(P)])
    set(lean,'Ydata',[0,h*cos(P)])
    set(mass,'Xdata',h*sin(P))
    set(mass,'Ydata',h*cos(P))
    
    % Draw figure at current step
    drawnow
    
    % Movie
    M(i) = getframe(anim);
    
    fprintf('t = %d\n',ans_struct.x(i))
    
end

% movie(M,1)
myVideo = VideoWriter([name,'.avi']);
open(myVideo);
writeVideo(myVideo, M);
close(myVideo);

end