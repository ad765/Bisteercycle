% Differential equations for dual-steering bicycle
% Aerial-view 2D dynamics

clear,clc
%% create variables

% state vars
syms t td tdd real      % theta derivatives

syms vR vRd real    	% velocity derivatives 

% force vars
syms TF TR real      	% wheel thrust forces
syms fF fR real      	% wheel friction forces

% control vars
syms dlR dlRd real      % rear wheel steering control
syms dlF dlFd real      % front wheel steering control
syms delta deltad real  % difference in wheel steering and derivative

% temp vars
syms tR tF real

% import parameters
syms m I lF lR rF rR cR cF real

%% frames
% Inertial I-frame
i = [1;
     0;
     0];
 
j = [0;
     1;
     0];
 
k = [0;
     0;
     1];

% B-frame
b1 = cos(t)*i + sin(t)*j;
b2 = -sin(t)*i + cos(t)*j;

% R-frame (rear wheel)
r1 = cos(dlR)*b1 + sin(dlR)*b2;
r2 = -sin(dlR)*b1 + cos(dlR)*b2;

% F-frame (front wheel)
f1 = cos(dlF)*b1 + sin(dlF)*b2;
f2 = -sin(dlF)*b1 + cos(dlF)*b2;


%% calculate intersection point of normals (C)
calcC = (lR+lF)*b1 == tR*r2 - tF*f2;        % triangle calculation
[ tauR, tauF ] = solve( calcC, [tR; tF], 'IgnoreAnalyticConstraints', true );
tauR = simplify( tauR );
tauF = simplify( tauF );

% Position of R relative to C
r_RC = -tauR*r2;

% Position of F relative to C
r_FC = -tauF*f2;


%% kinematics
r_GC    = r_RC + lR*b1;

v_RO = vR*r1;                   % rear wheel velocity
v_FO = v_RO + (lR+lF)*td*b2; 	% front wheel velocity
% v_GO = v_RO + lR*td*b2;     	% COM velocity


%% constraints
front_cons  = v_FO.'*f2 == 0;
front_consd = simplify( jacobian( front_cons, [ dlF, dlR, t, td, vR ] )*...
    [ dlFd, dlRd, td, tdd, vRd ]' );

% td = solve(front_cons, td);
thdd = solve(front_consd, tdd);


%% acceleration kinematics
a_RO = jacobian(v_RO',[t, dlR, vR])*[td, dlRd, vRd]';
a_GR = -lR*td^2*b1 + lR*thdd*b2;
a_GO = a_RO + a_GR;


%% fbd
% Friction
fR = -cR * v_RO.' * r1 * r1;
fF = -cF * v_FO.' * f1 * f1;

FR = TR*r1 + fR;
FF = TF*f1 + fF;


%% angular momentum balance about C
% H_C     = cross( r_GC, m*v_GO ) + I*diff( TH )*k;
Hdot_C  = cross( r_GC, m*a_GO ) + I*thdd*k;
M_C     = cross( r_RC, FR ) + cross( r_FC, FF );

AMB     = formula( simplify( M_C == Hdot_C ) );

%% checks for singularities
% Hdot_Ctemp = subs(Hdot_C, [dlF-dlR, dlFd-dlRd], [delta deltad]);
% pretty(limit(Hdot_Ctemp, delta, 0))


%% EOM
vRd = simplify( solve( AMB(3), vRd ), 'IgnoreAnalyticConstraints', true );

                
%% matlabfunction
% solve equations
% matlabFunction( tdd, 'File', '/Volumes/GoogleDrive/My Drive/Cornell Engineering/Course Work/Projects/Dual-Steer Bicycle/2D Simulations/Aerial View/Code/deriver/thdd' );
matlabFunction( vRd, 'File', '/Volumes/GoogleDrive/My Drive/Cornell Engineering/Course Work/Projects/Dual-Steer Bicycle/2D Simulations/Aerial View/Code/deriver/vRd' );
