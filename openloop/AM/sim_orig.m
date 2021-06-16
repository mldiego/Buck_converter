% Reachability analysis of the average model
% Diego Manzanas, September 30th 2020
close all

%% Load plant model

C = 4.4e-6; % capacitor value
L = 5e-5; % inductor value
R = 4; % resistor value
% T = 0.00001;% sample time
T = 1e-5;
Vs = 10;% input (source) voltage
Vref = 6;% reference voltage
% Tmax = 1000*(1/R*C);% max simulation time
Du = Vref / Vs;% duty cycle 
rs=100e-3;% switching loss (?)
rL=520e-3;% inductor loss (?)      

% Define ss matrices
dim = 2;
A_avg = [0, -(1/L); (1/C), -(1/(R*C))];% switch closed
B_avg = [Vs*(Du/L); 0];
oC = eye(dim);
D = [0;0];

% Create plant
controlPeriod = 2.5000e-04; % controlPeriod = tfinal
nSteps = 500;
% controlPeriod = tfinal(2);
% nSteps = rSteps(2);
reachStep = controlPeriod/nSteps;
Plant = LinearODE(A_avg, B_avg, oC, D, controlPeriod,nSteps); % Linear ODE plant
plant_cora = LinearODE_cora(A_avg, B_avg, oC, D, reachStep, controlPeriod); % LinearODE cora plant
% nlPlant = NonLinearODE(2,1,@dynamicsAM,reachStep,controlPeriod,C); % Nonlinear ODE plant 


%% Define reachability parameters

lb = [0.0;0.0];
ub = [0.1;0.1];
init_set = Star(lb,ub);

%% Simulation (MATLAB & CORA)

% Sim (CORA)
n_sim = 25;
X0s = lb'+rand(n_sim,dim).*(ub'-lb'); % Set of random initial points to simulate
sim1 = zeros(n_sim,dim,nSteps+1); % Store all simulation data
yC = 1; % Initial input (If u = 0, the reach plots don't look appealing)
t = tic;
tvec = zeros(n_sim,nSteps+1);
for j=1:n_sim
    x0 = X0s(j,:)';
    [tV,y] = plant_cora.evaluate(x0,yC);
    sim1(j,:,:) = y';
    tvec(j,:) = tV';
end
tsim = toc(t);

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
    hold on;
end
xlabel('x_1')
ylabel('x_2');
title('Average Model');
% saveas(f,'reachAM_orig.png');

% Plot reach sets vs time (Current)
f = figure;
hold on;
for p=1:n_sim
    simP = sim1(p,:,:);
    nl = size(simP,3);
    simP = reshape(simP,[dim, nl]);
    plot(timeV,simP(1,:),'r');
end
xlabel('Time (seconds)')
ylabel('Current');
title('Average Model');
% saveas(f,'reachAM_orig_I.png');

% Plot reach sets vs time (Voltage)
f = figure;
hold on;
for p=1:n_sim
    simP = sim1(p,:,:);
    nl = size(simP,3);
    simP = reshape(simP,[dim, nl]);
    plot(timeV,simP(2,:),'r');
end
xlabel('Time (seconds)')
ylabel('Voltage');
title('Average Model');
% saveas(f,'reachAM_orig_V.png');

% save('Origdata_'+string(controlPeriod)+'_'+string(n_sim)+'.mat','reachAll','tsim','treach','-v7.3');
states = sim1;
save('buck_data.mat','states','tvec');
