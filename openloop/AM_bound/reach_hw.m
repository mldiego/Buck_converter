% Reachability analysis of the average model
% Diego Manzanas, September 30th 2020
close all;

%% Load plant model

% Parameters
C = 2.2e-3; % capacitor value
L = 2.65e-3; % inductor value
R = 10; % resistor value
T = 1/60000; % sample time
Vs = 100;% input (source) voltage
Vref = 48;% reference voltage
% Tmax = 1000*(1/R*C);% max simulation time
Du = Vref / Vs;% duty cycle        

% Define ss matrices
dim = 2;
A_avg = [0, -(1/L); (1/C), -(1/(R*C))];% switch closed
B_avg = [Vs*(Du/L); 0];
outC = eye(dim);
D = [0;0];
x0 = [0;0]; % Initial state to compute epsilon

% Create plant
controlPeriod = 0.3; % controlPeriod = tfinal
nSteps = 1000;
% controlPeriod = tfinal(1);
% nSteps = rSteps(1);
reachStep = controlPeriod/nSteps;
Plant = LinearODE(A_avg, B_avg, outC, D, controlPeriod,nSteps); % Linear ODE plant
plant_cora = LinearODE_cora(A_avg, B_avg, outC, D, reachStep, controlPeriod); % LinearODE cora plant
% nlPlant = NonLinearODE(2,1,@dynamicsAM,reachStep,controlPeriod,C); % Nonlinear ODE plant 

% How do we incorporate the bounded region?
alpha = max([1/L, 1/C, 1/(R*C), Vref/L]);
epsilon = alpha*T;
term1 = (norm(x0)+norm(B_avg)/norm(A_avg));
term2 = exp(epsilon*norm(A_avg)*(0.3-0))-1; % t0 = 0
diff_eq = term1*term2;
% Let's plot the norm bounds between average and hyrbid as a function of time
t = 0:T/10:T;
norm_func = (norm(x0)+norm(B_avg)/norm(A_avg)) * (exp(epsilon*norm(A_avg)*(t-0))-1);
figure;
plot(t,norm_func);
xlabel('Time (s)');
ylabel('Bound');
title('Bound Average Model');


%% Define reachability parameters

lb = [i_lb;v_lb];
ub = [i_ub;v_ub];
init_set = Star(lb,ub);

%% Simulation (MATLAB & CORA)

% Sim (CORA)
%n_sim = 25;
X0s = lb'+rand(n_sim,dim).*(ub'-lb'); % Set of random initial points to simulate
sim1 = zeros(n_sim,dim,nSteps+1); % Store all simulation data
yC = 1; % Initial input (percentage of V)
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
title('Average Model');
saveas(f,'reachAM_hw.png');

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
saveas(f,'reachAM_hw_I.png');

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
saveas(f,'reachAM_hw_V.png');

save('hwdata_'+string(controlPeriod)+'_'+string(n_sim)+'.mat','reachAll','tsim','treach','-v7.3');
