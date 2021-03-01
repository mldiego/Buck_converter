% Open loop Reachability analysis of the average model
% Diego Manzanas, September 30th 2020
clc;clear;close all

%% Load plant model

% set up parameters
C = 4.4e-006; % capacitor value
L = 50e-006; % inductor
R = 4;% resistor 
T = 0.00001;% sample time
Vs = 10;% input (source) voltage 10
Vref = 6;% desired output voltage 6
Tmax = 1000*(1/R*C);% max simulation time
Du = Vref / Vs;% duty cycle

% index 1 avg of duty cycle
D_R1=1/(2*pi)*sin(2*pi*Du); %  Real part 
D_I1=1/(2*pi)*(cos(2*pi*Du)-1); % Imaginary part
fs=1/T;
ws=2*pi*fs;

% A matrices
dim = 6;
A1=[0 -1/L;1/C -1/(R*C)];
A2=[0 0 0 0;0 0 0 0];
A3=[0 0;0 0;0 0;0 0];
A4=[0 ws -1/L 0;-ws 0 0 -1/L;1/C 0 -1/(R*C) ws;0 1/C -ws -1/(R*C)];

% State-space matrices
A=[A1 A2;A3 A4];
B=[Du/L;0;D_R1/L;D_I1/L;0;0];
B = Vs*B;
C=eye(size(A));
D=[0 0 0 0 0 0]';

% Initial state
% x0=[0 0 0 0 0 0]'; % [i_L0 v_C0 i_L1Real i_L1Imag v_C1Real v_C1Imag\begin]


% Create plant
% controlPeriod = T;
controlPeriod = 2.5000e-04; % controlPeriod = tfinal
% nSteps = 5;
nSteps = 300;
reachStep = controlPeriod/nSteps;
Plant = LinearODE(A, B, C, D, controlPeriod,nSteps); % Linear ODE plant
plant_cora = LinearODE_cora(A, B, C, D, reachStep, controlPeriod); % LinearODE cora plant
% nlPlant = NonLinearODE(6,1,@dynamicsMFAM,reachStep,controlPeriod,C); % Nonlinear ODE plant 


%% Define reachability parameters

lb = [0;0;0;0;0;0];
ub = [0.3;0.3;0.1;0.1;0.1;0.1];
init_set = Star(lb,ub);

%% Simulation

% Sim (CORA)
n_sim = 25;
X0s = lb'+rand(n_sim,dim).*(ub'-lb'); % Set of random initial points to simulate
sim1 = zeros(n_sim,6,nSteps+1); % Store all simulation data
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
title('Open Loop - MFAM');
saveas(f,'OpenLoop_MFAM_reach.png');
