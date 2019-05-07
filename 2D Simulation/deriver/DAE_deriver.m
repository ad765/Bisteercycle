%% create variables
% state vars
syms X(t) Y(t) TH(t)    % state as fcn of time
syms x xd xdd           % x derivatives
syms y yd ydd           % y derivatives
syms th thd thdd        % z derivatives

% force vars
syms NF NR              % wheel normal forces
syms TF TR              % wheel thrust forces
syms fF fR              % wheel friction forces

% control vars
syms DLR(t) DLF(t)      % steering control as fcn of time
syms dlR dlRd dlRdd     % rear wheel steering control
syms dlF dlFd dlFdd     % front wheel steering control

% temp vars
syms tR tF

% import parameters
syms m I lF lR rF rR cR cF

%% frames

% inertial frame
i = [1; 0; 0];
j = [0; 1; 0];
k = [0; 0; 1];

% b frame
b1 = cos(TH)*i + sin(TH)*j;
b2 = -sin(TH)*i + cos(TH)*j;

% r frame
r1 = cos(DLR)*b1 + sin(DLR)*b2;
r2 = -sin(DLR)*b1 + cos(DLR)*b2;

% f frame
f1 = cos(DLF)*b1 + sin(DLF)*b2;
f2 = -sin(DLF)*b1 + cos(DLF)*b2;


%% kinematics

r_GO = X*i + Y*j;
v_GO = diff( r_GO, 1 );
a_GO = diff( v_GO, 1 );

%% fbd

r_RO = r_GO - lR*b1;
v_RO = diff( r_RO, 1 );

r_FO = r_GO + lF*b1;
v_FO = diff( r_FO, 1 );

% Friction
fR = -cR * v_RO.' * r1 * r1;
fF = -cF * v_FO.' * f1 * f1;

FR = NR*r2 + TR*r1 + fR;
FF = NF*f2 + TF*f1 + fF;
Ft = FR + FF;


%% constraints
rear_cons   = formula( expand( diff( v_RO.' * r2, 1 ) == 0 ) );
front_cons  = formula( expand( diff( v_FO.' * f2, 1 ) == 0 ) );


%% equations

% write equations
LMB         = formula( expand( Ft - m*a_GO == 0 ) );
AMB         = formula( expand( cross( (r_RO-r_GO), FR ) + cross( (r_FO-r_GO), FF ) - I*diff(TH,2)*k == 0 ) );

EOM = [LMB(1); LMB(2); AMB(3); rear_cons; front_cons];


%% substitutions

EOM = subs( EOM, [X, diff(X,1), diff(X,2), ...
                  Y, diff(Y,1), diff(Y,2), ...
                  TH, diff(TH,1), diff(TH,2), ...
                  DLR, diff(DLR,1), diff(DLR,2), ...
                  DLF, diff(DLF,1), diff(DLF,2)], ...
                 [x, xd, xdd, ...
                  y, yd, ydd, ...
                  th, thd, thdd, ...
                  dlR, dlRd, dlRdd, ...
                  dlF, dlFd, dlFdd] );
                
                
%% matlabfunction
% solve equations
[A,b] = equationsToMatrix( EOM, [xdd, ydd, thdd, NF, NR] );
A = simplify(A);
b = simplify(b);
matlabFunction(A, 'File', '/Volumes/GoogleDrive/My Drive/Cornell Engineering/Course Work/Projects/Dual-Steer Bicycle/2D Simulations/Aerial View/Code/deriver/A_DAE');
matlabFunction(b, 'File', '/Volumes/GoogleDrive/My Drive/Cornell Engineering/Course Work/Projects/Dual-Steer Bicycle/2D Simulations/Aerial View/Code/deriver/b_DAE');