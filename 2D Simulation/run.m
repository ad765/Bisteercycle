% Run file

% Simulation of the top-down dynamics. Includes nonholonomic constraints
% due to the wheels imposing a rolling constraint.

clc
clear
close all

% Insert path name
my_path = '/Volumes/GoogleDrive/My Drive/Cornell Engineering/Course Work/Projects/Dual-Steer Bicycle/2D Simulations/Code';
addpath( genpath( my_path ) )


%% parameters
par = [param('DAE'), param('AMB')];


%% solutions
tf = 200;    % final time
n = 1e4;    % number of points in time interval
% S_DAE = bike_solver( par(1), tf );
S_AMB = bike_solver( par(2), tf );


%% comparing methods
scale = 3;
% animate2( S_DAE, par(1), 'unfixed', scale )
animate2( S_AMB, par(2), 'unfixed', scale )
% plotting( S_DAE, par(1), n, 1, 2, 3, 4 )
% plotting( S_AMB, par(2), n, 5, 6, 7, 8 )


%% plotting differences
comparison( S_DAE, S_AMB, par(1), n, 9, 10 )

