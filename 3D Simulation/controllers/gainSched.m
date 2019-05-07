function K = gainSched(param)

% Create a grid of all bike steer angles
dlf = param.dlflim;
dlr = param.dlrlim;
V   = param.Vlim; 
[dlfgrid, dlrgrid, Vgrid] = ndgrid( dlf, dlr, V );

% Organize each gridpoint as a row
Kmat = zeros( 4, 5, length(dlf), length(dlr), length(V) );
for i = 1:length(dlf)
    for j = 1:length(dlr)
        for k = 1:length(V)
            Xlin = findLinPts(param,[0,V(k),0,dlf(i),dlr(j)]);
            Kmat(:,:,i,j,k) = fixedLQR(param,Xlin);
        end
    end
end

% Interpolate the K matrix through grid
K = cell( 4, 5 );
for i = 1:size(Kmat,1)
    for j = 1:size(Kmat,2)
        K{i,j} = griddedInterpolant( dlfgrid, dlrgrid, Vgrid, squeeze(Kmat(i,j,:,:,:)), param.method);
    end
end


