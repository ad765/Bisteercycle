function [value, isterminal, direction] = bike_events( t, Z, p )

KE = (1/2)*p.m*(Z(4).^2 + Z(5).^2) + (1/2)*p.I*Z(6).^2;
cost = (Z(1)-1).^2 + (Z(2)).^2;

value = [KE - 1e-10, cost ];
isterminal = [1, 1];
direction = [0, 0];

end