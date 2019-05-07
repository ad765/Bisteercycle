% Differential equations for dual-steering bicycle
% Aerial-view 2D dynamics

%% create variables

% state vars
syms TH(t)       	% state as fcn of time
syms th thd thdd  	% z derivatives from O to R

syms VR(t)          % velocity of rear wheel as fcn of time
syms vR vRd     	% velocity derivatives 

% force vars
syms TF TR       	% wheel thrust forces
syms fF fR       	% wheel friction forces

% control vars
syms DLR(t) DLF(t) 	% steering control as fcn of time
syms dlR dlRd       % rear wheel steering control
syms dlF dlFd       % front wheel steering control

% temp vars
syms tR tF

% import parameters
syms m I lF lR rF rR cR cF

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
b1 = cos(TH)*i + sin(TH)*j;
b2 = -sin(TH)*i + cos(TH)*j;

% R-frame (rear wheel)
r1 = cos(DLR)*b1 + sin(DLR)*b2;
r2 = -sin(DLR)*b1 + cos(DLR)*b2;

% F-frame (front wheel)
f1 = cos(DLF)*b1 + sin(DLF)*b2;
f2 = -sin(DLF)*b1 + cos(DLF)*b2;


%% kinematics
v_RO = VR*r1;                    	% rear wheel velocity
v_FO = v_RO + (lR+lF)*diff(TH)*b2; 	% front wheel velocity
v_GO = v_RO + lR*diff(TH)*b2;     	% COM velocity


%% calculate intersection point of normals (C)
calcC = (lR+lF)*b1 == tR*r2 - tF*f2;        % triangle calculation
[ tauR, tauF ] = solve( calcC, [tR; tF], 'IgnoreAnalyticConstraints', true );
tauR = simplify( tauR );
tauF = simplify( tauF );

% Position of R relative to C
r_RC = -tauR*r2;

% Position of F relative to C
r_FC = -tauF*f2;


%% fbd
% Friction
fR = -cR * v_RO.' * r1 * r1;
fF = -cF * v_FO.' * f1 * f1;

FR = TR*r1 + fR;
FF = TF*f1 + fF;


%% constraints
front_cons   = simplify( diff( v_FO.'*f2 ) == 0 );

%% angular momentum balance about C

r_GC    = r_RC + lR*b1;

H_C     = cross( r_GC, m*v_GO ) + I*diff( TH )*k;
Hdot_C  = diff( H_C );
M_C     = cross( r_RC, FR ) + cross( r_FC, FF );

AMB     = formula( simplify( M_C == Hdot_C ) );

EOM     = [AMB(3); front_cons]; 

%% substitutions

EOM     = subs(EOM,[VR,diff(VR,1),TH,diff(TH,1),diff(TH,2),DLR,diff(DLR,1),DLF,diff(DLF,1)], ...
                   [vR,vRd,th,thd,thdd,dlR,dlRd,dlF,dlFd]);
                
                
%% matlabfunction
% solve equations
[thetaddot, velddot] = solve( EOM, [thdd, vRd], 'IgnoreAnalyticConstraints', true );
matlabFunction( simplify( thetaddot ), 'File', '/Volumes/GoogleDrive/My Drive/Cornell Engineering/Course Work/Projects/Dual-Steer Bicycle/2D Simulations/Aerial View/Code/deriver/thdd' );
matlabFunction( simplify( velddot ), 'File', '/Volumes/GoogleDrive/My Drive/Cornell Engineering/Course Work/Projects/Dual-Steer Bicycle/2D Simulations/Aerial View/Code/deriver/vRd' );

% [A,b] = equationsToMatrix( EOM, [thdd, vRd] );
% A = simplify(A);
% b = simplify(b);
% matlabFunction(A, 'File', '/Volumes/GoogleDrive/My Drive/Cornell Engineering/Course Work/Projects/Dual-Steer Bicycle/2D Simulations/Aerial View/Code/deriver/A_AMB' );
% matlabFunction(b, 'File', '/Volumes/GoogleDrive/My Drive/Cornell Engineering/Course Work/Projects/Dual-Steer Bicycle/2D Simulations/Aerial View/Code/deriver/b_AMB' );