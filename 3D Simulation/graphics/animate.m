function animate( ans_struct, bike_p, angle, tarray, name )
% Real-time animation of the bicycle

% Re-evaluate solution at provided time array
soln = deval(ans_struct,tarray);

% Unpack state
Xtemp   = soln(1,:);
Ytemp   = soln(2,:);
Ptemp   = soln(3,:);
Stemp   = soln(4,:);
dlftemp = soln(7,:);
dlrtemp = soln(8,:);

clear soln

% Unpack parameters
rF  = bike_p.rF;
rR  = bike_p.rR;
lF  = bike_p.lF;
lR  = bike_p.lR;
h   = bike_p.h;
hw  = bike_p.hw;

% Animate simulation
anim    = figure;%('units','normalized','outerposition',[0 0 1 1]);
animax  = gca;
hold on
grid on
axis equal
title('Animation of bicycle trajectory')

dist = 2*max([lF,lR]);

% Set graphics properties
set( anim, 'DoubleBuffer', 'on' )

% Create bike object
bike    = plot3( 0, 0, 0, 'k', 'LineWidth', 3 );

% Create handle bars
hand_R  = plot3( 0, 0, 0, 'r', 'LineWidth', 3 );
hand_F  = plot3( 0, 0, 0, 'b', 'LineWidth', 3 );

% Create wheels
tau = linspace(0, 2*pi, 1e3);
rear_wheel  = plot3(0, 0, 0, 'r', 'LineWidth', 3);
front_wheel = plot3(0, 0, 0, 'b', 'LineWidth', 3);
%}

% Update graphics
parfor i = 1:length(tarray)
    
    % Unpack state
    X   = Xtemp(i);
    Y   = Ytemp(i);
    P   = Ptemp(i);
    S   = Stemp(i);
    dlf = dlftemp(i);
    dlr = dlrtemp(i);
    
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
    
    % View of the bike
    if strcmp(angle,'lean')
        set( animax, 'PlotBoxAspectRatio', [2 2 1] )
        set( animax, 'CameraPosition',[front_wheel_X, front_wheel_Y, 0])
        set( animax, 'CameraTarget', [rear_wheel_X, rear_wheel_Y, 0])
    elseif strcmp(angle,'global')
        view(45,20);
        axis([min(ans_struct.y(1,:))-dist-h, max(ans_struct.y(1,:))+dist+h, ...
            min(ans_struct.y(2,:))-dist-h, max(ans_struct.y(2,:))+dist+h, 0, h+dist]);
    elseif strcmp(angle,'local')
        view(45,20);
        axis([X-dist-h, X+dist+h, Y-dist-h, Y+dist+h, 0, h+dist]);
    end
    
    
    % Plot bike body
    set( bike, 'XData', [rear_wheel_X, bike_tail_X, bike_front_X, front_wheel_X] )
    set( bike, 'YData', [rear_wheel_Y, bike_tail_Y, bike_front_Y, front_wheel_Y] )
    set( bike, 'ZData', [rear_wheel_Z, bike_tail_Z, bike_front_Z, front_wheel_Z] )
    
    % Plot rear bike handle bars
    set( hand_R, 'XData', [bike_tail_X, hand_R_center_X, hand_R_left_X, hand_R_right_X] )
    set( hand_R, 'YData', [bike_tail_Y, hand_R_center_Y, hand_R_left_Y, hand_R_right_Y] )
    set( hand_R, 'ZData', [bike_tail_Z, hand_R_center_Z, hand_R_left_Z, hand_R_right_Z] )
    
    % Plot front bike handle bars
    set( hand_F, 'XData', [bike_front_X, hand_F_center_X, hand_F_left_X, hand_F_right_X] )
    set( hand_F, 'YData', [bike_front_Y, hand_F_center_Y, hand_F_left_Y, hand_F_right_Y] )
    set( hand_F, 'ZData', [bike_front_Z, hand_F_center_Z, hand_F_left_Z, hand_F_right_Z] )
    
    % Plot rear and wheel
    set( rear_wheel, 'XData', wheel_RX )
    set( rear_wheel, 'YData', wheel_RY )
    set( rear_wheel, 'ZData', wheel_RZ )
    set( front_wheel, 'XData', wheel_FX )
    set( front_wheel, 'YData', wheel_FY )
    set( front_wheel, 'ZData', wheel_FZ )
    
    % Plot the wheel traces
    % plot3( rear_wheel_X,	rear_wheel_Y,  0,	'r.', 'MarkerSize', 0.5 )
    % plot3( front_wheel_X,	front_wheel_Y, 0,   'b.', 'MarkerSize', 0.5 )
    
    % Draw figure at current step
    drawnow
    
    % Movie
    M(i) = getframe(anim);
    
    disp(tarray(i))
        
end

hold off

myVideo = VideoWriter([name,'.avi']);
open(myVideo);
writeVideo(myVideo, M);
close(myVideo);

end