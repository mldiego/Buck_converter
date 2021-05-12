% Reachability analysis of the average model
% Diego Manzanas, September 30th 2020
clc;clear;close all

%% Load plant model

C = 2.2e-3;
L = 2.65e-3;
R = 8;
rs=100e-3;% switching loss 
rL=520e-3;% inductor loss

% Define Buck converter parameters
T = 1/10000; % switching time period
Vs = 30;%30
Vref = 10; % Could change this value if needed
Du = Vref / Vs;% duty cycle (not important if open loop?)        

% Define ss matrices
dim = 2;
A_avg = [0, -(1/L); (1/C), -(1/(R*C))];% switch closed
B_avg = [Vs*(Du/L); 0];
oC = eye(dim);
D = [0;0];
x0 = [0;0]; % Initial state to compute epsilon

% Create plant
controlPeriod = 0.3;
nSteps = 1000;
reachStep = controlPeriod/nSteps;
Plant = LinearODE(A_avg, B_avg, oC, D, controlPeriod,nSteps); % Linear ODE plant
plant_cora = LinearODE_cora(A_avg, B_avg, oC, D, reachStep, controlPeriod); % LinearODE cora plant
% nlPlant = NonLinearODE(2,1,@dynamicsAM,reachStep,controlPeriod,C); % Nonlinear ODE plant 

% How do we incorporate the bounded region?
alpha = max([1/L, 1/C, 1/(R*C), Vref/L]);
epsilon = alpha*T;
term1 = (norm(x0)+norm(B_avg)/norm(A_avg));
term2 = exp(epsilon*norm(A_avg)*(0.3-0))-1; % t0 = 0
diff_eq = term1*term2;
% Let's plot the norm bounds between average and hyrbid as a function of time
t = 0:T:0.01;
norm_func = (norm(x0)+norm(B_avg)/norm(A_avg)) * (exp(epsilon*norm(A_avg)*(t-0))-1);
figure;
plot(t,norm_func);
xlabel('Time (s)');
ylabel('Bound');
title('Bound Average Model');

% What is the Beta function in the Appendix?
% These computed bounds seem unusable...
% Goal is to make epsilon small, so alpha needs to be small


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
t = tic;
reachAll = Plant.simReach('direct', init_set, outC, reachStep, nSteps); % reduce the order (basic vectors) in order for the code to finish
toc(t);

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
Star.plotBoxes_2D_noFill(reachAll,1,2,'b',0.5);
xlabel('x_1')
ylabel('x_2');
title('Average Model');
saveas(f,'reachAM_sim.png');

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
title('Average Model');
saveas(f,'reachAM_sim_I.png');

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
title('Average Model');
saveas(f,'reachAM_sim_V.png');