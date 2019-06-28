function K = gainSched( sys_p, bike_p, ctr_p )

% Create a grid of all bike steer angles
dlf = ctr_p.dlflim;
dlr = ctr_p.dlrlim;
V   = ctr_p.Vlim; 
[dlfgrid, dlrgrid, Vgrid] = ndgrid( dlf, dlr, V );

% Organize each gridpoint as a row
Kmat = zeros( 4, 5, length(dlf), length(dlr), length(V) );
for i = 1:length(dlf)
    for j = 1:length(dlr)
        for k = 1:length(V)
            Xlin = findLinPts(bike_p,[0,V(k),0,dlf(i),dlr(j)]);
            Kmat(:,:,i,j,k) = fixedLQR(bike_p,sys_p,Xlin);
            %if abs(Xlin(1))<=1e-6 && abs(Xlin(4))<=1e-6 && abs(Xlin(5))<=1e-6
            %    Kmat(3:4,:,i,j,k) = zeros(2,5);
            %end
        end
    end
end

% Interpolate the K matrix through grid
K = cell( 4, 5 );
for i = 1:size(Kmat,1)
    for j = 1:size(Kmat,2)
        K{i,j} = griddedInterpolant( dlfgrid, dlrgrid, Vgrid, squeeze(Kmat(i,j,:,:,:)), ctr_p.method);
    end
end


