function animate2( S, p, cam, scale)
% Real-time animation of the bicycle

% Unpack parameters
SOL = p.SOL;
rF = p.rF;
rR = p.rR;
lF = p.lF;
lR = p.lR;

% Animate simulation
anim = figure;
set( anim, 'DoubleBuffer', 'on' )
hold on
grid on
axis equal
xlabel('X')
ylabel('Y')
title('Animation of bicycle trajectory')

% ext = 2;
X = S.y(1,:);
Y = S.y(2,:);
% ext = 2;
if strcmp(cam, 'fixed')
    ax_lim = [min(X)-lF, max(X)+lF, min(Y)-lF, max(Y)+lF];
    axis(ax_lim)
end

% Create bike object
bike = plot( 0, 0, 'k', 'LineWidth', 3 );
% COM  = plot( 0, 0, 'o', 'MarkerSize', 15, 'MarkerFaceColor', 'k');

% Create wheel objects
wR  = plot(0, 0, 'r', 'LineWidth', 5);
wF  = plot(0, 0, 'c', 'LineWidth', 5);

t_temp = 0;

tic;

while (t_temp < S.x(end))
    
    Z = deval(S,t_temp);
    
    % Unpack state
    if strcmp(SOL,'DAE')
        THETA   = Z(3);
        X       = Z(1);
        Y       = Z(2);
        dlR     = Z(7);
        dlF     = Z(8);
    elseif strcmp(SOL,'AMB')
        THETA   = Z(3);
        X       = Z(1) + lR*cos(THETA);
        Y       = Z(2) + lR*sin(THETA);
        dlR     = Z(5);
        dlF     = Z(6);
    end
    
    % Plot traces of wheels
    plot( X + lF*cos(THETA),    Y + lF*sin(THETA),      'k.', 'MarkerSize', 0.5 )
    plot( X + lR*cos(THETA+pi), Y + lR*sin(THETA+pi),   'k.', 'MarkerSize', 0.5 )
    
    % Plot rigid bodies
    %set( COM, 'Xdata', X )
    %set( COM, 'Ydata', Y )
    %
    set( bike, 'Xdata', [X + lR*cos(THETA+pi), X + lF*cos(THETA)] )
    
    set( bike, 'Ydata', [Y + lR*sin(THETA+pi), Y + lF*sin(THETA)] )
    
    set( wR, 'Xdata',   [X + lR*cos(THETA+pi) + rR*cos(THETA+dlR+pi),...
        X + lR*cos(THETA+pi) + rR*cos(THETA+dlR)])
    
    set( wR, 'Ydata',   [Y + lR*sin(THETA+pi) + rR*sin(THETA+dlR+pi),...
        Y + lR*sin(THETA+pi) + rR*sin(THETA+dlR)])
    
    set( wF, 'Xdata',   [X + lF*cos(THETA) + rF*cos(THETA+dlF+pi),...
        X + lF*cos(THETA) + rF*cos(THETA+dlF)])
    
    set( wF, 'Ydata',   [Y + lF*sin(THETA) + rF*sin(THETA+dlF+pi),...
        Y + lF*sin(THETA) + rF*sin(THETA+dlF)])
    %}
    drawnow
    
    t_temp = scale*toc;
    fprintf('t = %d\n',t_temp)
    
end

hold off

end