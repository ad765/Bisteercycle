% Simulation of the 3D dynamics of dual-steering bicycle.
clc
clear
close all

% Insert path name
my_path = pwd;
addpath( genpath( my_path ) )

%% parameters

% Uncomment this if you would like to store a new set of variables. Name
% the file something OTHER THAN 'parameters'
%
par = param;
% save([pwd,'/parameters/','bike1.mat'],'-struct','par')
%}

% par = load('bike1.mat');

%% solution
t_final = 20;       % final time
n       = 1e4;      % number of points in time interval
tol     = 1e-3;     % ODE solver tolerance
S       = bike_solver( par, t_final, tol );

%% comparing methods
animate_v2( S, par, 0.1)
% animate( S, par, 'local', 1)
% animate( S, par, 'lean', 1)
% plotting( S, par, n, 2, 3, 4, 5, 6 )

