% Reachability analysis of the average model
% Diego Manzanas, September 30th 2020
clc;clear;close all

%% Load plant model

% Parameters
C = 2.2e-3; % capacitor value
L = 2.65e-3; % inductor value
R = 10; % resistor value
T = 1.6667e-05; % sample time
Vs = 100;% input (source) voltage
Vref = 48;% reference voltage
% Tmax = 1000*(1/R*C);% max simulation time
D = Vref / Vs;% duty cycle        

% Define ss matrices
dim = 2;
A_avg = [0, -(1/L); (1/C), -(1/(R*C))];% switch closed
B_avg = [Vs*(D/L); 0];
C = eye(dim);
D = [0;0];

% Create plant
controlPeriod = 0.3; % controlPeriod = tfinal
nSteps = 1000;
reachStep = controlPeriod/nSteps;
Plant = LinearODE(A_avg, B_avg, C, D, controlPeriod,nSteps); % Linear ODE plant
plant_cora = LinearODE_cora(A_avg, B_avg, C, D, reachStep, controlPeriod); % LinearODE cora plant
% nlPlant = NonLinearODE(2,1,@dynamicsAM,reachStep,controlPeriod,C); % Nonlinear ODE plant 


%% Define reachability parameters

N = 25; % Number of control steps to simulate the system

lb = [0;0];
ub = [0.5;0.5];
init_set = Star(lb,ub);

%% Simulation (MATLAB & CORA)

% Sim (CORA)
n_sim = 25;
X0s = lb'+rand(n_sim,dim).*(ub'-lb'); % Set of random initial points to simulate
sim1 = zeros(n_sim,dim,nSteps+1); % Store all simulation data
yC = 1; % Initial input (If u = 0, the reach plots don't look appealing)
for j=1:n_sim
    x0 = X0s(j,:)';
    [tV,y] = plant_cora.evaluate(x0,yC);
    sim1(j,:,:) = y';
end

%% Reachability analysis 1 (NNV, direct and approx-star NN)

outC = Star(yC,yC); % Initial input set
reachAll = Plant.simReach('direct', init_set, outC, reachStep, nSteps); % reduce the order (basic vectors) in order for the code to finish

%% Visualize results
timeV = 0:reachStep:controlPeriod;
% Plot Reach Sets
f = figure;
hold on;
for p=1:n_sim
    simP = sim1(p,:,:);
    nl = size(simP,3);
    simP = reshape(simP,[dim, nl]);
    plot(simP(1,:),simP(2,:),'r');
end
Star.plotBoxes_2D_noFill(reachAll,1,2,'b');
xlabel('x_1')
ylabel('x_2');
title('Open Loop - Average Model (hw)');
saveas(f,'OpenLoop_AM_reach_hw.png');

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
ylabel('Current');
title('Open Loop - Average Model (hw)');
saveas(f,'OpenLoop_AM_reachI_hw.png');

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
ylabel('Voltage');
title('Open Loop - Average Model (hw)');
saveas(f,'OpenLoop_AM_reachV_hw.png');