function animate_v2( ans_struct, param, scale)
% Real-time animation of the bicycle

% Unpack parameters
rF  = param.rF;
rR  = param.rR;
lF  = param.lF;
lR  = param.lR;
h   = param.h;
hw  = param.hw;

% Animate simulation
figure;%('units','normalized','outerposition',[0 0 1 1]);

% Maximum distance
dist = 2*max([lF,lR]);

% Set graphics properties
% set( anim, 'DoubleBuffer', 'on' )

% Create wheel parameterization
tau = linspace(0, 2*pi, 1e3);

% Initialize real-time
t_temp = 0;
tic;

while (t_temp < ans_struct.x(end))
    
    soln = deval(ans_struct,t_temp);
    
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
    
    bikex = [rear_wheel_X, bike_tail_X, bike_front_X, front_wheel_X];
    bikey = [rear_wheel_Y, bike_tail_Y, bike_front_Y, front_wheel_Y];
    bikez = [rear_wheel_Z, bike_tail_Z, bike_front_Z, front_wheel_Z];
    handRx= [bike_tail_X, hand_R_center_X, hand_R_left_X, hand_R_right_X];
    handRy= [bike_tail_Y, hand_R_center_Y, hand_R_left_Y, hand_R_right_Y];
    handRz= [bike_tail_Z, hand_R_center_Z, hand_R_left_Z, hand_R_right_Z];
    handFx= [bike_front_X, hand_F_center_X, hand_F_left_X, hand_F_right_X];
    handFy= [bike_front_Y, hand_F_center_Y, hand_F_left_Y, hand_F_right_Y];
    handFz= [bike_front_Z, hand_F_center_Z, hand_F_left_Z, hand_F_right_Z];
    
    % Plot the bodies
    subplot(2,2,1);
    hold on
    grid on
    axis equal
    animax1 = gca;
    % Global view
    set( animax1, 'View', [45,20] );
    global_axes = [min(ans_struct.y(1,:))-dist-h, max(ans_struct.y(1,:))+dist+h, ...
        min(ans_struct.y(2,:))-dist-h, max(ans_struct.y(2,:))+dist+h, 0, h+dist];
    axis( animax1, global_axes )
    % Plot bike
    plot3( bikex,           bikey,          bikez,      'k-','LineWidth',2)
    plot3( handRx,          handRy,         handRz,     'r-','LineWidth',2)
    plot3( handFx,          handFy,         handFz,     'b-','LineWidth',2)
    plot3( wheel_RX,        wheel_RY,       wheel_RZ,   'r-','LineWidth',2)
    plot3( wheel_FX,        wheel_FY,       wheel_FZ,   'b-','LineWidth',2)
    plot3( rear_wheel_X,	rear_wheel_Y,   0,          'r-', 'MarkerSize', 0.5 )
    plot3( front_wheel_X,	front_wheel_Y,  0,          'b-', 'MarkerSize', 0.5 )
    title('Global view of bicycle')
    hold off
    
    subplot(2,2,2);
    hold on
    grid on
    axis equal
    animax2 = gca;
    % Local view
    set( animax2, 'View', [45,20] );
    local_axes = [X-dist-h, X+dist+h, Y-dist-h, Y+dist+h, 0, h+dist];
    axis( animax2, local_axes )
    plot3( bikex,           bikey,          bikez,      'k-','LineWidth',2)
    plot3( handRx,          handRy,         handRz,     'r-','LineWidth',2)
    plot3( handFx,          handFy,         handFz,     'b-','LineWidth',2)
    plot3( wheel_RX,        wheel_RY,       wheel_RZ,   'r-','LineWidth',2)
    plot3( wheel_FX,        wheel_FY,       wheel_FZ,   'b-','LineWidth',2)
    plot3( rear_wheel_X,	rear_wheel_Y,   0,          'r.', 'MarkerSize', 0.5 )
    plot3( front_wheel_X,	front_wheel_Y,  0,          'b.', 'MarkerSize', 0.5 )
    title('Local view of bicycle')
    hold off
    
    subplot(2,2,3);
    hold on
    grid on
    axis equal
    animax3 = gca;
    % Top down view
    axis( animax3, local_axes(1:4) )
    plot( bikex,            bikey,          'k-','LineWidth',2)
    % plot( handRx,           handRy,         'r-','LineWidth',2)
    % plot( handFx,           handFy,         'b-','LineWidth',2)
    plot( wheel_RX,         wheel_RY,       'r-','LineWidth',2)
    plot( wheel_FX,         wheel_FY,       'b-','LineWidth',2)
    plot( rear_wheel_X,     rear_wheel_Y,  	'r.', 'MarkerSize', 0.5 )
    plot( front_wheel_X,	front_wheel_Y,  'b.', 'MarkerSize', 0.5 )
    title('Top-down view of bicycle')
    hold off
    
    subplot(2,2,4);
    hold on
    grid on
    axis equal
    plot([0,0],[0,h],'k--','LineWidth',2)
    plot([h*sin(P)],[h*cos(P)],'ko','MarkerSize',10,'MarkerFaceColor',[0,0,0])
    plot([0,h*sin(P)],[0,h*cos(P)],'k-','LineWidth',2)
    title('Bicycle Lean')
    hold off
    
    % Draw figure at current step
    drawnow
    
    t_temp  = scale*toc;
    fprintf('t = %d\n',t_temp)
    
end

end