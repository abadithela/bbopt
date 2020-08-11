%% This file uses Breach to evaluate the robust satisfaction value of a STL formula
% Apurva Badithela // 6/9/2020

%% Load Models and Data
% Load data:
load('cont.mat')
load_system('CM')

%% Initialize Model
M = BreachSimulinkSystem('CM');
M.PrintAll

%% Read in STL specifications:
STL_ReadFile('MountainCar_STL_spec.stl');
M.PlotRobustSat(reach_goal)


%%
phi = set_params(phi, {'delta', 'threshold'}, [10, 0.6])

% The specification is <>_[0, delta] (x(t) > threshold), where threshold =
% 0.6 and delta = 10s
phi = STL_Formula('phi',  'ev_[0, delta] (x[t] > threshold)')
[val, tau] = STL_Eval(Sys, phi, P, trajs[, taus][, method]

vars = {''}
Sys = CreateSystem(vars, params, vals)
%  P = CreateParamSet(Sys, 'F', [10, 20]);
%  P = SetParam(P, 'theta', 2);
%  P = ComputeTraj(Sys, P, 0:.01:10);
%  phi = STL_Formula('phi','ev_[0,1] (x0[t]>theta)');
%  [val,tau] = STL_Eval(Sys, phi, P, P.traj{P.traj_ref(1}), 0)
%  [val,tau] = STL_Eval(Sys, phi, P, P.traj{P.traj_ref(1}), [3,7])