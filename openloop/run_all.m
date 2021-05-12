%% Run all openloop experiments in one file
clc;clear;close all;
% Here we define variables shared across all the scripts to assure all experiments are equivalent 
n_sim = 2000; % number of simulations
i_lb = 0; % lower bound current
v_lb = 0; % lower bound voltage
i_ub = 0.2; % upper bound current
v_ub = 0.2; % upper bound voltage
% nSteps = [AverageHardware;AverageOrig;AverageHW2;hybridHW]
rSteps = [1000;500;500;523]; % Number of reachability steps (different for each method)
tfinal = [0.3;2.5e-4;0.05;0.011667]; % Final time of simulation (different for each model)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Run all reachability scripts AM
cd 'AM';
disp('AM orig');
run reach_orig.m;

disp('MFAM hardware');
run reach_hw.m
cd ..

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Run all reachability scripts AM (2)
cd 'AM2';
disp('AM orig');
run reach_orig.m;

disp('MFAM hardware');
run reach_hw.m
cd ..

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Run all reachability scripts MFAM
cd 'MFAM';
disp('MFAM orig');
run reach_orig.m;

disp('MFAM hardware');
run reach_hw.m
cd ..

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Run all reachability scripts MFAM (2)
cd 'MFAM2';
disp('MFAM orig (2)');
run reach_orig.m;

disp('MFAM hardware (2)');
run reach_hw.m
cd ..

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Run all reachability scripts Hybrid model
cd 'HM';
disp('HM orig');
run reach_orig.m;

disp('HM hardware');
run reach_hw_time.m
cd ..

% %% Create results tables
% % TODO