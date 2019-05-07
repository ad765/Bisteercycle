function animate( T, S, p)

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
% ax_lim = [min(X)-ext, max(X)+ext, min(Y)-ext, max(Y)+ext];
% axis(ax_lim)

% Create bike object
bike = plot( 0, 0, 'k', 'LineWidth', 3 );
% COM  = plot( 0, 0, 'o', 'MarkerSize', 15, 'MarkerFaceColor', 'k');

% Create wheel objects
wR  = plot(0, 0, 'r', 'LineWidth', 5);
wF  = plot(0, 0, 'c', 'LineWidth', 5);

t_temp = 0;

tic
for i = 1:length(T)
    if (t_temp > T(end))
        break;
    else
        Z = deval(S,t_temp);
    end
    
    % Unpack state
    if strcmp(SOL,'DAE')
        THETA   = Z(3,:);
        X       = Z(1,:);
        Y       = Z(2,:);
        dlR     = Z(7,:);
        dlF     = Z(8,:);
    elseif strcmp(SOL,'AMB')
        THETA   = Z(3,:);
        X       = Z(1,:) + lR*cos(THETA);
        Y       = Z(2,:) + lR*sin(THETA);
        dlR     = Z(5,:);
        dlF     = Z(6,:);
    end
    % Plot traces of wheels
    plot( X(i) + lF*cos(THETA(i)),    Y(i) + lF*sin(THETA(i)),      'k.', 'MarkerSize', 0.5 )
    plot( X(i) + lR*cos(THETA(i)+pi), Y(i) + lR*sin(THETA(i)+pi),   'k.', 'MarkerSize', 0.5 )
    
    % Plot rigid bodies
    %set( COM, 'Xdata', X(i) )
    %set( COM, 'Ydata', Y(i) )
    
    set( bike, 'Xdata', [X(i) + lR*cos(THETA(i)+pi), ...
        X(i) + lF*cos(THETA(i))] )
    
    set( bike, 'Ydata', [Y(i) + lR*sin(THETA(i)+pi), ...
        Y(i) + lF*sin(THETA(i))] )
    
    set( wR, 'Xdata',   [X(i) + lR*cos(THETA(i)+pi) + rR*cos(THETA(i)+dlR(i)+pi),...
        X(i) + lR*cos(THETA(i)+pi) + rR*cos(THETA(i)+dlR(i))])
    
    set( wR, 'Ydata',   [Y(i) + lR*sin(THETA(i)+pi) + rR*sin(THETA(i)+dlR(i)+pi),...
        Y(i) + lR*sin(THETA(i)+pi) + rR*sin(THETA(i)+dlR(i))])
    
    set( wF, 'Xdata',   [X(i) + lF*cos(THETA(i)) + rF*cos(THETA(i)+dlF(i)+pi),...
        X(i) + lF*cos(THETA(i)) + rF*cos(THETA(i)+dlF(i))])
    
    set( wF, 'Ydata',   [Y(i) + lF*sin(THETA(i)) + rF*sin(THETA(i)+dlF(i)+pi),...
        Y(i) + lF*sin(THETA(i)) + rF*sin(THETA(i)+dlF(i))])
    
    drawnow
    t_temp = toc;
end

hold off

end