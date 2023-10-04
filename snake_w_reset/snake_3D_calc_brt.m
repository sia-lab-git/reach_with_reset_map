clc
clear
%% Grid
grid_min = [-1; -1;-pi]; % Lower corner of computation domain
grid_max = [11; 11;+pi];    % Upper corner of computation domain
N = [31;31;31];         % Number of grid points per dimension
pdDims = 3;               % -- dimension is periodic
g = createGrid(grid_min, grid_max, N, pdDims);

%% target set
ignoredims = 3;
params.center =[7,7,0];
params.R=1; 
data0 = shapeCylinder(g, ignoredims, params.center, params.R);

%% time vector
t0 = 0;
tMax = 3.0;
dt = 0.1;
tau = t0:dt:tMax;

%% problem parameters
x0=[0;0;0];
v = 2;
uRange = 1;
% control trying to min or max value function?
uMode = 'min';
dMode = 'max';
%% Pack problem parameters
% Define dynamic system
sys = snake3D(x0,v,uRange,params); 

% Put grid and dynamic systems into schemeData
schemeData.grid = g;
schemeData.dynSys = sys;
schemeData.accuracy = 'high'; %set accuracy
schemeData.uMode = uMode;
schemeData.dMode = dMode;

%% Compute value function

%HJIextraArgs.visualize = true; %show plot
HJIextraArgs.visualize.valueSet = 1;
HJIextraArgs.visualize.initialValueSet = 1;
HJIextraArgs.visualize.figNum = 1; %set figure number
HJIextraArgs.visualize.deleteLastPlot = true; %delete previous plot as you update

% uncomment if you want to see a 2D slice
HJIextraArgs.visualize.plotData.plotDims = [1 1 1]; %plot xy
HJIextraArgs.visualize.plotData.projpt = []; %project at 
%HJIextraArgs.visualize.viewAngle = [0,90]; % view 2D

%% get reset map
[schemeData.reset_map, sys.params] = snake3D_get_reset_map(g, sys.params);

%% calc brt
[data, tau2, extraOuts] = ...
        HJIPDE_solve_with_reset_map(data0, tau, schemeData, 'minVWithV0', HJIextraArgs);

%% store the brt

snake3D.data=data;
snake3D.g=g;
snake3D.sys=sys;
snake3D.tau2=tau2;
%snake3D.deriv=derivatives;
