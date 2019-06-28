function [value, isterminal, direction] = bike_events( ~, Z, sys_p )

% Check when the bike is about to hit the ground
value = abs(Z(3)) - (pi/2 - sys_p.fall);
isterminal = 1;
direction = 0;

end