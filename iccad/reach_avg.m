% Reachability analysis of the average model
% Diego Manzanas, September 30th 2020
% clc;clear;close all

%% Load Controller

load('best_avg_nnc.mat'); % Load weights
layer1 = LayerS(W{1},b{1}','poslin'); % hidden layer #1
layer2 = LayerS(W{2},b{2}','poslin'); % hidden layer #2
% layer3 = LayerS(W{3},b{3}','satlins'); % output layer (satlins)
layer3 = LayerS(W{3},b{3}','tansig'); % output layer (tanh)
layer4 = LayerS(0.5,0.5,'purelin'); % Normalization
Layers = [layer1 layer2 layer3 layer4];
Controller = FFNNS(Layers); % neural network controller

%% Load plant model

% Parameters
C = 2.2e-3; % capacitor value
L = 2.65e-3; % inductor value
R = 10; % resistor value
% T = 0.00001;% sample time
fs = 1000;
fws = 10000;
Vs = 100;% input (source) voltage
Vref = 48;% reference voltage
% Tmax = 1000*(1/R*C);% max simulation time
Du = Vref / Vs;% duty cycle        

% Define ss matrices
% x = [voltage,current];
dim = 2;
% A_avg = [0, -(1/L); (1/C), -(1/(R*C))];% switch closed
A_avg = [-(1/(R*C)), (1/C); -(1/L), 0];% switch closed
% B_avg = [Vs/L; 0];
B_avg = [0; Vs/L];
C = eye(2);
D = [0;0];

% Create plant
controlPeriod = 1/fs;
nSteps = 30;
reachStep = controlPeriod/nSteps;
Plant = LinearODE(A_avg, B_avg, C, D, controlPeriod,nSteps); % Linear ODE plant
plant_cora = LinearODE_cora(A_avg, B_avg, C, D, reachStep, controlPeriod); % LinearODE cora plant
% nlPlant = NonLinearODE(2,1,@dynamicsAM,reachStep,controlPeriod,C); % Nonlinear ODE plant 


%% Define reachability parameters

N = 50; % Number of control steps to simulate the system

lb = [0.1;0.1];
ub = [0.2;0.2];

%% Simulation 

% Sim (CORA)
% n_sim = 100; % Number of simulations
n_sim = 1;
step_sim = N; % Number of simulation steps
X0s = lb'+rand(n_sim,2).*(ub'-lb'); % Set of random initial points to simulate
t = 0;
T = 1/fs;
dT = T/100;
sim2 = zeros(n_sim,2,step_sim+1);
sim1 = zeros(n_sim,2,step_sim*nSteps);
control_outputs = [];
control_inputs = [];
for j=1:n_sim
    x0 = X0s(j,:)';
    sim2(j,:,1) = x0;
    for i=1:step_sim
        Vout = x0(1);
        inCont = [Vref - Vout;Vref;x0(1:2)];
        control_inputs = [control_inputs inCont];
        yC = Controller.evaluate(inCont);
        control_outputs = [control_outputs yC];
        [tV,y] = plant_cora.evaluate(x0,yC);
        x0 = y(end,:)';
        x0a = y';
        t = t+dT;
        sim2(j,:,i+1) = x0;
        sim1(j,:,((i-1)*(nSteps)+1):i*nSteps+1) = x0a;
    end
end

%% Reachability analysis 1 (NNV, direct and approx-star NN)
disp(' ');
disp('---------------------------------------------------');
disp('Method 1 - NNV')
init_set = Star(lb,ub);
plant1 = Plant;

reachSet = [init_set];
reachAll = [];
cont_reach = [];
cont_inps = [];
for i=1:N
    inNN = input_to_Controller(Vref,init_set);
    cont_inps = [cont_inps inNN]; % Store controller inputs
    outC = Controller.reach(inNN,'approx-star');
    cont_reach = [cont_reach outC]; % Store controller outputs
    init_set = plant1.simReach('direct', init_set, outC, reachStep, nSteps); % reduce the order (basic vectors) in order for the code to finish
%     init_set = plant_cora.stepReachStar(init_set,outC);
    reachAll = [reachAll init_set(1:end-1)]; % store all reachable sets
%     reahAll = [reachAll init_set];
    init_set = init_set(end);
    reachSet = [reachSet init_set]; % store reach sets at each control period
    if init_set.nVar > 1000
        init_set = init_set.getBox;
        init_set = init_set.toStar;
    end
end
reachAll = [reachAll init_set]; % Add last reach set

%% Visualize results (1)

timeV = 0:reachStep:(controlPeriod*N);
% Plot Reach Sets
f = figure;
hold on;
for p=1:n_sim
    simP = sim1(p,:,:);
    nl = size(simP,3);
    simP = reshape(simP,[dim, nl]);
    plot(simP(2,:),simP(1,:),'r');
end
Star.plotBoxes_2D_noFill(reachAll,2,1,'b');
xlabel('current')
ylabel('voltage');
title('Close Loop - AM');
saveas(f,'CloseLoop_AM_reach.png');

% Plot reach sets vs time (Current)
f = figure;
hold on;
for p=1:n_sim
    simP = sim1(p,:,:);
    nl = size(simP,3);
    simP = reshape(simP,[dim, nl]);
    plot(timeV,simP(1,:),'r');
end
Star.plotRanges_2D(reachAll,1,timeV,'b');
xlabel('Time (seconds)')
ylabel('Voltage');
title('Close Loop - AM');
saveas(f,'CloseLoop_AM_reachV.png');

% Plot reach sets vs time (Voltage)
f = figure;
hold on;
for p=1:n_sim
    simP = sim1(p,:,:);
    nl = size(simP,3);
    simP = reshape(simP,[dim, nl]);
    plot(timeV,simP(2,:),'r');
end
Star.plotRanges_2D(reachAll,2,timeV,'b');
xlabel('Time (seconds)')
ylabel('Current');
title('Close Loop - AM');
saveas(f,'CloseLoop_AM_reachI.png');

figure;
Star.plotRanges_2D(cont_reach,1,1:1:N,'b');
hold on;
plot(control_outputs,'r');
title('Control Outputs');
saveas(f,'AM_controlActions.png');

figure;
Star.plotBoxes_2D(cont_inps,3,4,'b');
hold on;
plot(control_inputs(3,:),control_inputs(4,:),'r');
title('Control Inputs 3-4');
saveas(f,'AM_controlInpus.png');

figure;
Star.plotBoxes_2D(cont_inps,1,2,'b');
hold on;
plot(control_inputs(1,:),control_inputs(2,:),'--r');
title('Control Inputs 1 - 2');
saveas(f,'AM_controlInpus12.png');


%% Helper Functions

function inNN = input_to_Controller(Vref,init_set)
    l = length(init_set);
    inNN = [];
    for i = 1:l
        out1 = init_set(i).affineMap([1 0],-Vref); % input 1 (Vref - Vout)
        out1 = out1.affineMap(-eye(1),[]);
        out12 = out1.concatenate(Star(Vref,Vref)); % input 1 and input 2
        out = out12.concatenate(init_set(i));
%         out = out12.concatenate(init_set(i).affineMap([0 1;1 0],[])); % add inputs 3 and 4 (Vout,Iout)
        inNN = [inNN out];
    end
end