%% Parameters
syms g m lR lF h real
syms I11 I22 I33 I13 real


%% Coordinates
% Uncontrolled coordinates
syms x y                real        % Rear wheel position
syms        V       Vd  real        % Rear wheel velocity
syms p      pd      pdd real        % Lean angle of bike
syms s      sd      sdd real        % Heading angle of bike

% Controlled coordinates
syms dlf    dlfd        real        % Front steering angle
syms dlr    dlrd        real        % Rear steering angle


%% Reference frames (3-1) rotation
%
i = [1; 0; 0];
j = [0; 1; 0];
k = [0; 0; 1];

% Intermediate (heading) frame
a3 = k;
a1 = cos(s)*i + sin(s)*j;
a2 = -sin(s)*i + cos(s)*j;

% Body (lean) frame
b1 = a1;
b2 = cos(p)*a2 + sin(p)*a3;
b3 = -sin(p)*a2 + cos(p)*a3;

% Rear steering frame
r3 = b3;
r1 = cos(dlr)*b1 + sin(dlr)*b2;
r2 = -sin(dlr)*b1 + cos(dlr)*b2;

% Front steering frame
f3 = b3;
f1 = cos(dlf)*b1 + sin(dlf)*b2;
f2 = -sin(dlf)*b1 + cos(dlf)*b2;

% Tread unit vectors
r1_ij = cross(r2, k);
r2_ij = cross(k, r1_ij);

f1_ij = cross(f2, k);
f2_ij = cross(k, f1_ij);


%% calculate intersection point of normals (C)

syms tR1 tF1 real

% Triangle calculation
calcC = (lR+lF)*b1 == tR1*r2_ij - tF1*f2_ij;
[ tauR, tauF ] = solve( calcC(1:2), [tR1; tF1], 'IgnoreAnalyticConstraints', true );
tauR = simplify( tauR );
tauF = simplify( tauF );

% Position vector from C to R
r_CR    = -tauR*r2_ij;

% Position vector from C to F
r_CF    = -tauF*f2_ij;


%% Position kinematics
r_RG = lR*b1 + h*b3;
r_RF = (lR+lF)*b1;


%% Velocity kinematics
w_AI = sd*a3;       % Angular velocity of A frame relative to I
w_BA = pd*b1;       % Angular velocity of B frame relative to A
w_BI = simplify( w_AI + w_BA ); 

v_OR = V*r1_ij;         % rear wheel velocity
v_RF = simplify( jacobian( r_RF, s) * sd );

v_OF = v_OR + v_RF;     % front wheel velocity
v_RG = simplify( jacobian( r_RG, [p, s]' ) * [pd, sd]' );


%% Constraints
front_cons  = simplify( v_OF.'*f2_ij == 0 );
front_consd = simplify( jacobian( front_cons, [ V, dlf, dlr, p, s, sd]') * [ Vd, dlfd, dlrd, pd, sd, sdd]' );

PSId    = simplify( solve(front_cons, sd) );
sdd     = simplify( solve(front_consd, sdd) );


%% Acceleration kinematics
a_OR = simplify( jacobian( v_OR, [ V, dlr, p, s]') * [ Vd, dlrd, pd, sd]' );
a_RG = simplify( jacobian( v_RG, [ p, pd, s, sd]' ) * [ pd, pdd, sd, sdd]' );
a_OG = simplify( a_OR + a_RG );


%% Forces
Fg = -m*g*k;

syms Nr Nf fr ff Trear Tfront real
% NR = Nr*k;
% NF = Nf*k;
% fR = fr*r2_ij; % projection of fR onto (i,j)-plane
% fF = ff*f2_ij; % projection of fF onto (i,j)-plane
TR = Trear*r1_ij;
TF = Tfront*f1_ij;


%% Angular momentum
H_G = simplify( I11*b1*dot(b1,w_BI) + I22*b2*dot(b2,w_BI) + I33*b3*dot(b3,w_BI) + I13*b1*dot(b3,w_BI) + I13*b3*dot(b1,w_BI) );
Hdot_G = simplify( jacobian( H_G, [p, pd, s, sd]' ) * [pd, pdd, sd, sdd]' );


%% AMB about r_RF
r_RE = lR*b1;
r_EG = r_RG - r_RE;

% Moment about point E
M_E = simplify( cross(r_EG, Fg) );

% AMB about r_RF
AMB_RF = simplify( dot(M_E, b1) == dot(m*cross(r_EG, a_OG) + Hdot_G, b1 ) );


%% AMB about intersection of normals (C)
r_CG = r_CR + r_RG;

% Moment about point C
M_C = simplify( cross(r_CR, TR) + cross(r_CF, TF) );

% AMB about k going through C
AMB_Ck = simplify( expand( dot(M_C, k) == dot(m*cross(r_CG, a_OG) + Hdot_G, k )));


%% EOM
eom = [AMB_RF; AMB_Ck];


%% Solve equations
[veldot, phiddot] = solve( eom, [Vd, pdd] );
veldot = simplify( subs(veldot,sd,PSId) );
phiddot = simplify( subs(phiddot,sd,PSId) );
%{
matlabFunction( veldot, 'File', '/Volumes/GoogleDrive/My Drive/Cornell Engineering/Course Work/Projects/Dual-Steer Bicycle/3D Simulations/deriver/Vdot' );
matlabFunction( phiddot, 'File', '/Volumes/GoogleDrive/My Drive/Cornell Engineering/Course Work/Projects/Dual-Steer Bicycle/3D Simulations/deriver/PHIddot' );
matlabFunction( PSId, 'File', '/Volumes/GoogleDrive/My Drive/Cornell Engineering/Course Work/Projects/Dual-Steer Bicycle/3D Simulations/deriver/PSIdot' );
%}

%% Linearize EOM
%
%{
eom1 = 0 == -V*(sin(dlr)*sin(s) - cos(dlr)*cos(p)*cos(s));
eom2 = 0 ==  V*(cos(s)*sin(dlr) + cos(dlr)*cos(p)*sin(s));
eom3 = 0 == pd;
eom4 = 0 == V*sin(dlf - dlr)/(cos(dlf)*(lF + lR));
eom5 = 0 == subs(sdd,Vd,veldot);
eom6 = 0 == dlfd;
eom7 = 0 == dlrd;
%}

% xd = -V*(sin(dlr)*sin(s) - cos(dlr)*cos(p)*cos(s));
% yd =  V*(cos(s)*sin(dlr) + cos(dlr)*cos(p)*sin(s));
phidot  = pd;
% psidot  = V*sin(dlf - dlr)/(cos(dlf)*(lF + lR));
dlfdot  = dlfd;
dlrdot  = dlrd;

lin_eom = [phidot; veldot; phiddot; dlfdot; dlrdot];
state   = [p, V, pd, dlf, dlr]';
control = [Tfront, Trear, dlfd, dlrd]';

lin_A = jacobian( lin_eom, state );
lin_b = jacobian( lin_eom, control );
%
matlabFunction( lin_A, 'File', '/Volumes/GoogleDrive/My Drive/Cornell Engineering/Course Work/Projects/Dual-Steer Bicycle/bike_sims/3D Simulations/deriver/linearized/A_lin','Optimize',false );
matlabFunction( lin_b, 'File', '/Volumes/GoogleDrive/My Drive/Cornell Engineering/Course Work/Projects/Dual-Steer Bicycle/bike_sims/3D Simulations/deriver/linearized/B_lin','Optimize',false );
%}

%% Energy
%{
LKE = (1/2)*m*dot(v_OR + v_RG,v_OR + v_RG);
RKE = (1/2)*dot(w_BI,H_G);
PE  = m*g*dot(k,r_RG);
matlabFunction( LKE, 'File', '/Volumes/GoogleDrive/My Drive/Cornell Engineering/Course Work/Projects/Dual-Steer Bicycle/3D Simulations/deriver/energy/LKE' );
matlabFunction( RKE, 'File', '/Volumes/GoogleDrive/My Drive/Cornell Engineering/Course Work/Projects/Dual-Steer Bicycle/3D Simulations/deriver/energy/RKE' );
matlabFunction( PE, 'File', '/Volumes/GoogleDrive/My Drive/Cornell Engineering/Course Work/Projects/Dual-Steer Bicycle/3D Simulations/deriver/energy/PE' );
%}