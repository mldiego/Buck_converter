% Reachability analysis of the average model
% Diego Manzanas, October 12th 2020
clc;clear;close all

%% Load Controller and Plant

% Controller
load('final_model.mat'); % Load weights
layer1 = LayerS(W{1},b{1}','purelin'); % hidden layer #1
layer2 = LayerS(W{2},b{2}','purelin'); % hidden layer #2
layer3 = LayerS(W{3},b{3}','satlins'); % output layer (satlins)
% layer4 = LayerS(0.5,1,'purelin'); % Normalization
layer4 = LayerS(0.5,0.5,'purelin'); % Normalization
Layers = [layer1 layer2 layer3 layer4];
% Layers = [layer1 layer2 layer3];
Controller = FFNNS(Layers); % neural network controller

% Load hybrid automata
% buck = HybridA(5,1,buck_v2,2);
% This model does not really allow for any inputs, and it is not the same
% as the one that we have in Simulink (????)
% Use this better, need to attempt to model it: http://www.taylortjohnson.com/research/bak2017sttt.pdf

% Load hybrid Automata
HA = HybridA(4,1,Buck_Converter,3);
HA.options.enclose = {'pca'};
Vs = 10;% input (source) voltage (Average model)
Vref = 6;% reference voltage
% Or (simulink model)
% Vs = 30;
% Vref = 10;

%% Set reachability
T = 0.00001;% sample time
controlPeriod = T;
nSteps = 5;
reachStep = controlPeriod/nSteps;
out_mat = [0 1 0 0; 1 0 0 0];
% N = 20; % Number of steps to simulate the system
% N = 5 % (5,10 steps works);
N = 40;

lb = [0;0;1;0];
ub = [0.05;0.05;1;0];
init_set = Star(lb,ub);

% Example (one set for the plant)
HA.set_tFinal(T); % set control period
HA.set_timeStep(reachStep); % reachability step
inp_set = Star(1,1); % same mode as initial state
S = HA.stepReachStar(init_set,inp_set);

%% Reachability analysis
disp(' ');
disp('---------------------------------------------------');
disp('Method 1 - Hybrid Automata')
init_set = Star(lb,ub);
try
    reachSet_1 = [init_set];
    all_out = [];
    for i=1:N
        inNN = input_to_Controller(Vref,init_set,out_mat);
        outC = Controller.reach(inNN,'approx-star');
        init_set = HA.stepReachStar(init_set,outC);
        reachSet_1 = [reachSet_1 init_set];
        all_out = [all_out outC];
    end
catch e
    disp(' ');
    warning('Hybrid Automata method failed'); pause(0.01);
    fprintf(2,'THERE WAS AN ERROR. THE MESSAGE WAS:\n\n%s',getReport(e));
end


%% Visualize results
figure;
hold on;
Star.plotBoxes_2D_noFill(reachSet_1,1,2,'b');
xlabel('i')
ylabel('V');
title('HA reachability');

figure;
hold on;
Star.plotBoxes_2D_noFill(reachSet_1,4,3,'b');
xlabel('time (s)')
ylabel('x_3');
title('HA reachability');

figure;
hold on;
Star.plotBoxes_2D_noFill(all_out(1),1,1,'b');
xlabel('x_1')
ylabel('x_1');
title('Controller reachability');

% Compute ranges of controller output
m = zeros(40,1);
M = zeros(40,1);
for i=1:40
    [m(i), M(i)] = all_out(i).getRanges;
end

%% Helper Functions

function inNN = input_to_Controller(Vref,init_set,out_mat)
    l = length(init_set);
    inNN = [];
    for i = 1:l
        out1 = init_set(i).affineMap([0 1 0 0],-Vref); % input 1 (Vref - Vout)
        out1 = out1.affineMap(-eye(1),[]);
        out12 = out1.concatenate_with_vector(Vref); % input 1 + input 2
        out = out12.concatenate(init_set(i).affineMap(out_mat,[])); % add inputs 3 and 4 (Vout,Iout)
        inNN = [inNN out];
    end
end


%% Notes & Problems & Errors

% CORA does not allow for the system to have outputs... Weird, right?
% Need to figure out how to encode this in our class... My initial guess
% would be to store the output matrix for each location, and used that with
% affineMap in NNV... 

% For now, set C empty and just evaluate the states as we usually do

% Only got the guard to work with conHyperplane... Need to learn which one
% is better in general. Invariants cannot be empty, so just write something