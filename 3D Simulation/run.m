% Simulation of the 3D dynamics of dual-steering bicycle.
clc
clear
close all

% Insert path name
my_path = pwd;
addpath( genpath( my_path ) )

%% Parameters

% Uncomment this if you would like to store a new set of variables. Name
% the file something OTHER THAN 'parameters'
%
par     = struct;
bike_p	= bike_param();
sys_p 	= system_param(bike_p);
ctr_p   = control_param(sys_p,bike_p,par);
% save([pwd,'\parameters\ctr\','ctr1.mat'],'-struct','ctr_p')
% save([pwd,'\parameters\bike\','bike1.mat'],'-struct','bike_p')
%}

%{
bike_p  = load('bike1.mat');
ctr_p   = load('ctr1.mat');
%}


%% Solution
t_final = 20;       % final time
n       = 1e4;      % number of points in time interval
tol     = 1e-3;     % ODE solver tolerance
S       = bike_solver( sys_p, bike_p, ctr_p, t_final, tol );


%% Visualization
animate_v3( S, bike_p, 2)
% plotting( S, sys_p, bike_p, ctr_p, n, 2, 3, 4, 5, 6 )

% animate( S, bike_p, 'local', 1)
% animate( S, bike_p, 'lean', 1)
