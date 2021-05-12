% Reachability analysis of the average model
% Diego Manzanas, September 30th 2020
close all

%% Load plant model

% Parameters
T = 1/60000;
Vs = 100;
Vref = 48;
C = 2.2e-3; % measured value 2.19e-3; %data sheet value 2.2e-3
L = 2.65e-3; % measured value 3.12e-3 ; %data sheet value 2.65e-3
R = 10;
rs = 3.5;
rL = 520e-3;
Du = Vref / Vs;% duty cycle     

% Define ss matrices
dim = 2;
A_avg = [-1*(rs+rL)/L, -(1/L); (1/C), -(1/(R*C))];% switch closed
B_avg = [Vs*(Du/L); 0];
outC = eye(dim);
D = [0;0];

% Create plant
%controlPeriod =0.05;
%nSteps = 500;
controlPeriod = tfinal(3);
nSteps = rSteps(3);
reachStep = controlPeriod/nSteps;
Plant = LinearODE(A_avg, B_avg, outC, D, controlPeriod,nSteps); % Linear ODE plant
plant_cora = LinearODE_cora(A_avg, B_avg, outC, D, reachStep, controlPeriod); % LinearODE cora plant
% nlPlant = NonLinearODE(2,1,@dynamicsAM,reachStep,controlPeriod,C); % Nonlinear ODE plant 


%% Define reachability parameters

lb = [i_lb;v_lb];
ub = [i_ub;v_ub];
init_set = Star(lb,ub);

%% Simulation (MATLAB & CORA)

% Sim (CORA)
%n_sim = 25;
X0s = lb'+rand(n_sim,dim).*(ub'-lb'); % Set of random initial points to simulate
sim1 = zeros(n_sim,dim,nSteps+1); % Store all simulation data
yC = 1; % Initial input
t = tic;
for j=1:n_sim
    x0 = X0s(j,:)';
    [tV,y] = plant_cora.evaluate(x0,yC);
    sim1(j,:,:) = y';
end
tsim = toc(t);

%% Reachability analysis 1 (NNV, direct and approx-star NN)

outC = Star(yC,yC); % Initial input set
t = tic;
reachAll = Plant.simReach('direct', init_set, outC, reachStep, nSteps); % reduce the order (basic vectors) in order for the code to finish
treach = toc(t);

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
title('Average Model (2)');
saveas(f,'reachAM2_hw.png');


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
title('Average Model (2)');
saveas(f,'reachAM2_hw_I.png');

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
title('Average Model (2)');
saveas(f,'reachAM2_hw_V.png');

save('hwdata_'+string(controlPeriod)+'_'+string(n_sim)+'.mat','reachAll','tsim','treach','-v7.3');
