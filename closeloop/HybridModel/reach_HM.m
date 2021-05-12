% Reachability analysis of the average model
% Diego Manzanas, October 12th 2020
clc;clear;close all

%% Load Controller and Plant

% Controller
load('final_model.mat'); % Load weights
layer1 = LayerS(W{1},b{1}','poslin'); % hidden layer #1
layer2 = LayerS(W{2},b{2}','poslin'); % hidden layer #2
layer3 = LayerS(W{3},b{3}','satlins'); % output layer (satlins)
layer4 = LayerS(0.5,0.5,'purelin'); % Normalization
Layers = [layer1 layer2 layer3 layer4];
% Layers = [layer1 layer2 layer3];
Controller = FFNNS(Layers); % neural network controller

% Load hybrid Automata
HA = HybridA(3,1,HM_hardware,3);
HA.options.enclose = {'pca'};
T = 1/60000; % sample time
Vs = 100; % source voltage
Vref = 48; % reference voltage

%% Set reachability
controlPeriod = T;
nSteps = 5;
reachStep = controlPeriod/nSteps;
out_mat = [1 0 0; 0 1 0];
tfinal = T*1800;
% tfinal = 0.03; % Ideally, which corresponds to T*1800
% N = 20; % Number of steps to simulate the system
% N = 10; % (5,10 steps works);
% N = 40;

lb = [0;0;0];
ub = [0.05;0.05;0];
init_set = Star(lb,ub);

% Example (one set for the plant)
HA.set_tFinal(tfinal); % set control period
HA.set_timeStep(reachStep); % reachability step
inp_set = Star(1,1); % same mode as initial state
S = HA.stepReachStar(init_set,inp_set);
Sall = [init_set HA.intermediate_reachSet];
% %% Reachability analysis
% disp(' ');
% disp('---------------------------------------------------');
% disp('Method 1 - Hybrid Automata')
% init_set = Star(lb,ub);
% try
%     reachSet_1 = [init_set];
%     all_out = [];
%     for i=1:N
%         inNN = input_to_Controller(Vref,init_set,out_mat);
%         outC = Controller.reach(inNN,'approx-star');
%         init_set = HA.stepReachStar(init_set,outC);
%         reachSet_1 = [reachSet_1 init_set];
%         all_out = [all_out outC];
%     end
% catch e
%     disp(' ');
%     warning('Hybrid Automata method failed'); pause(0.01);
%     fprintf(2,'THERE WAS AN ERROR. THE MESSAGE WAS:\n\n%s',getReport(e));
% end


%% Visualize results
timeV = 0:reachStep:tfinal;
f = figure;
hold on;
Star.plotBoxes_2D_noFill(Sall,1,2,'b');
xlabel('i')
ylabel('V');
title('Open Loop - hybrid (hw)');
saveas(f,'OpenLoop_hybrid_reach_hw.png');

% Plot reach sets vs time (Current)
f = figure;
hold on;
Star.plotRanges_2D(Sall,1,timeV,'b');
xlabel('Time (seconds)')
ylabel('Current');
title('Open Loop - hybrid (hw)');
saveas(f,'OpenLoop_hybrid_reachI_hw.png');

% Plot reach sets vs time (Voltage)
f = figure;
hold on;
Star.plotRanges_2D(Sall,2,timeV,'b');
xlabel('Time (seconds)')
ylabel('Voltage');
title('Open Loop - Hybrid (hw)');
saveas(f,'OpenLoop_hybrid_reachV_hw.png');

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