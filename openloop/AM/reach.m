% Reachability analysis of the average model
% Diego Manzanas, September 30th 2020
clc;clear;close all

%% Load plant model

% Parameters
C = 4.4e-6; % capacitor value
L = 5e-5; % inductor value
R = 4; % resistor value
% T = 0.00001;% sample time
T = 1e-5;
Vs = 10;% input (source) voltage
Vref = 6;% reference voltage
% Tmax = 1000*(1/R*C);% max simulation time
D = Vref / Vs;% duty cycle        

% Define ss matrices
dim = 2;
A_avg = [0, -(1/L); (1/C), -(1/(R*C))];% switch closed
B_avg = [Vs*(D/L); 0];
C = eye(dim);
D = [0;0];

% Create plant
controlPeriod = 2.5000e-04; % controlPeriod = tfinal
nSteps = 300;
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
yC = 0.5; % Initial input (If u = 0, the reach plots don't look appealing)
for j=1:n_sim
    x0 = X0s(j,:)';
    [tV,y] = plant_cora.evaluate(x0,yC);
    sim1(j,:,:) = y';
end

%% Reachability analysis 1 (NNV, direct and approx-star NN)

outC = Star(yC,yC); % Initial input set
reachAll = Plant.simReach('direct', init_set, outC, reachStep, nSteps); % reduce the order (basic vectors) in order for the code to finish

%% Visualize results

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
title('Open Loop - Average Model');
saveas(f,'OpenLoop_AM_reach.png');