% Reachability analysis of the average model
% Diego Manzanas, September 30th 2020

%% Load Controller

load('final_model.mat'); % Load weights
layer1 = LayerS(W{1},b{1}','purelin'); % hidden layer #1
layer2 = LayerS(W{2},b{2}','purelin'); % hidden layer #2
layer3 = LayerS(W{3},b{3}','satlins'); % output layer (satlins)
layer4 = LayerS(0.05,1,'purelin'); % Normalization
Layers = [layer1 layer2 layer3 layer4];
Controller = FFNNS(Layers); % neural network controller

%% Load plant model

% Parameters
C = 4.4e-006; % capacitor value
L = 5e-5; % inductor value
R = 4; % resistor value
T = 0.00001;% sample time
Vs = 10;% input (source) voltage
Vref = 6;% reference voltage
% Tmax = 1000*(1/R*C);% max simulation time
D = Vref / Vs;% duty cycle        

% Define ss matrices
A_avg = [0, -(1/L); (1/C), -(1/(R*C))];% switch closed
B_avg = [(D/L); 0];
C = eye(2);
D = [0;0];

% Create plant
controlPeriod = T;
numReachSteps = 100;
Plant = LinearODE(A_avg, B_avg, C, D, controlPeriod,numReachSteps); % Linear ODE plant
nlPlant = NonLinearODE(2,1,@dynamicsAM,T/numReachSteps,T,eye(2)); % Nonlinear ODE plant 
% Create NNCS
feedbackMap = [0];
ncs = LinearNNCS(Controller,Plant);
ncs3 = NNCS(Controller,nlPlant);

%% Define reachability parameters

N = 20; % Number of steps to simulate the system

lb = [0;0];
ub = [0.1;0.1];
reachPRM.init_set = Star(lb,ub); % Initial set of the converter
reachPRM.ref_input = Star([Vref;Vs],[Vref;Vs]); % Input reference to controller
reachPRM.reachMethod = 'approx-star'; % Reachability method for NNCS
reachPRM.numCores = 4; % Number of cores to be used for exact methods
reachPRM.numSteps = N; % Number of steps to simulate the system
% [RL,rTL] = ncs.reach(reachPRM); % Continuous-time Linear
[RnL,rTnL] = ncs3.reach(reachPRM); % Continuous-time nonlinear

%% Simulate NNCS
[simTL,conTL,sTL] = ncs.simulate([0;0],[Vref;Vs],N); % Simulate NNNCS 1
[simTnL,conTnL] = ncs3.evaluate(controlPeriod,N,[0;0],[Vref;Vs]); % Simulate NNCS 2

%% Visualize results

figure;
hold on;
% Star.plotBoxes_2D_noFill(RL,1,2,'m');
plot(simTL(1,:),simTL(2,:),'b');
xlabel('x1')
ylabel('y2');
title('Linear');


% figure;
% hold on;
Star.plotBoxes_2D_noFill(RnL,1,2,'m');
plot(simTnL(1,:),simTnL(2,:),'k');
xlabel('x1');
ylabel('y2');
title('Nonlinear');