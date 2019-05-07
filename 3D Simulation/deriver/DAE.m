%% Parameters
syms g m lR lF h real
syms I11 I22 I33 I13 real


%% Coordinates
% Uncontrolled coordinates
syms x      xd      xdd real        % X-coordinate of COM
syms y      yd      ydd real        % Y-coordinate of COM
syms z      zd      zdd real        % Z-coordinate of COM
syms p      pd      pdd real        % Lean angle of bike (phi)
syms t      td      tdd real        % Pitch angle of bike (theta)
syms s      sd      sdd real        % Heading angle of bike (psi)

% Controlled coordinates
syms dlf    dlfd        real        % Front steering angle
syms dlr    dlrd        real        % Rear steering angle


%% Reference frames (3-1-3) rotation
% Inertial frame
e1 = [1; 0; 0];
e2 = [0; 1; 0];
e3 = [0; 0; 1];

% Intermediate frame 1
a3 = e3;
a1 =  cos(s)*e1 + sin(s)*e2;
a2 = -sin(s)*e1 + cos(s)*e2;

% Intermediate frame 2
b2 = a2;
b1 = cos(t)*a1 - sin(t)*a3;
b3 = sin(t)*a1 + cos(t)*a3;

% Body frame
c1 = b1;
c2 =  cos(p)*b2 + sin(p)*b3;
c3 = -sin(p)*b2 + cos(p)*b3;

% Rear wheel frame
r3 = c3;
r1 =  cos(dlr)*c1 + sin(dlr)*c2;
r2 = -sin(dlr)*c1 + cos(dlr)*c2;

% Front wheel frame
f3 = c3;
f1 =  cos(dlf)*c1 + sin(dlf)*c2;
f2 = -sin(dlf)*c1 + cos(dlf)*c2;
%}

r1_ij = cross(r2, e3);
f1_ij = cross(f2, e3);

r2_ij = cross(e3, r1_ij);
f2_ij = cross(e3, f1_ij);


%% Kinematics
w_AI = sd*a3;	% Angular velocity of A relative to I
w_BA = td*b2;	% Angular velocity of B relative to A
w_CB = pd*c1;	% Angular velocity of C relative to B
w_RC = dlrd*r3; % Angular velocity of R relative to C
w_FC = dlfd*f3; % Angular velocity of F relative to C

r_GR = -lR*c1 - h*c3;
r_GF =  lF*c1 - h*c3;

r_OG = x*e1 + y*e2 + z*e3;
v_OG = simplify( jacobian( r_OG, [x,y,z]' ) * [xd,yd,zd]' );
a_OG = simplify( jacobian( v_OG, [xd,yd,zd]' ) * [xdd,ydd,zdd]' );


%% Forces
Fg = -m*g*e3;

syms Nr Nf fr ff Trear Tfront real
NR = Nr*e3;
NF = Nf*e3;
fR = fr*r2_ij;
fF = ff*f2_ij;
TR = Trear*r1_ij;
TF = Tfront*f1_ij;


F = Fg + NR + NF + fR + fF + TR + TF;


%% LMB
LMB = simplify( F == m*a_OG );


%% AMB about G
% Angular acceleration of C relative to I frame
w_CI = w_AI + w_BA + w_CB;

% Angular momentum about G
H_G = simplify( I11*c1*dot(c1,w_CI) + I22*c2*dot(c2,w_CI) + I33*c3*dot(c3,w_CI) + I13*c1*dot(c3,w_CI) + I13*c3*dot(c1,w_CI) );
Hdot_G = simplify( jacobian( H_G, [ p, pd, s, sd, t, td]' ) * [ pd, pdd, sd, sdd, td, tdd]' );

% Sum of moments about G
M_G = cross( r_GR, (fR+NR+TR) ) + cross( r_GF, (fF+NF+TF) );

% AMB about G
AMB = simplify( M_G == Hdot_G);


%% Constraints
v_OR = simplify( jacobian( r_OG + r_GR, [ p, s, t, x, y, z]' ) * [ pd, sd, td, xd, yd, zd]' );
v_OF = simplify( jacobian( r_OG + r_GF, [ p, s, t, x, y, z]' ) * [ pd, sd, td, xd, yd, zd]' );

cons1 = simplify( jacobian( dot( v_OR, e3), [ p, pd, t, td, zd]' ) * [ pd, pdd, td, tdd, zdd]' == 0 );
cons2 = simplify( jacobian( dot( v_OF, e3), [ p, pd, t, td, zd]' ) * [ pd, pdd, td, tdd, zdd]' == 0 );
cons3 = simplify( jacobian( dot( v_OR, r2_ij), [ dlr, p, pd, s, sd, t, td, xd, yd]' ) * [ dlrd, pd, pdd, sd, sdd, td, tdd, xdd, ydd]' == 0 );
cons4 = simplify( jacobian( dot( v_OF, f2_ij), [ dlf, p, pd, s, sd, t, td, xd, yd]' ) * [ dlfd, pd, pdd, sd, sdd, td, tdd, xdd, ydd]' == 0 );


%% EOM
eom = [LMB; AMB; cons1; cons2; cons3; cons4];


%% Solve equations
%
[A,b] = equationsToMatrix( eom, [xdd, ydd, zdd, pdd, tdd, sdd, Nf, Nr, ff, fr] );
A = simplify(A);
b = simplify(b);
matlabFunction( A, 'File', '/Volumes/GoogleDrive/My Drive/Cornell Engineering/Course Work/Projects/Dual-Steer Bicycle/3D Simulations/deriver/DAE_A' );
matlabFunction( b, 'File', '/Volumes/GoogleDrive/My Drive/Cornell Engineering/Course Work/Projects/Dual-Steer Bicycle/3D Simulations/deriver/DAE_b' );

LKE = (1/2)*m*dot(v_OG,v_OG);
RKE = (1/2)*dot(w_CI,H_G);
PE  = m*g*dot(e3,r_OG);
matlabFunction( LKE, 'File', '/Volumes/GoogleDrive/My Drive/Cornell Engineering/Course Work/Projects/Dual-Steer Bicycle/3D Simulations/deriver/energy/DAE_LKE' );
matlabFunction( RKE, 'File', '/Volumes/GoogleDrive/My Drive/Cornell Engineering/Course Work/Projects/Dual-Steer Bicycle/3D Simulations/deriver/energy/DAE_RKE' );
matlabFunction( PE, 'File', '/Volumes/GoogleDrive/My Drive/Cornell Engineering/Course Work/Projects/Dual-Steer Bicycle/3D Simulations/deriver/energy/DAE_PE' );
%}