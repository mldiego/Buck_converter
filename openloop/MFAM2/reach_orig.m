% Open loop Reachability analysis of the average model
% Diego Manzanas, September 30th 2020
% clc;clear;close all
close all;

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
rs=100e-5;% switching loss 
rL=520e-5;% inductor loss

% index 1 avg of duty cycle
D_R1=1/(2*pi)*sin(2*pi*Du); %  Real part 
D_I1=1/(2*pi)*(cos(2*pi*Du)-1); % Imaginary part
fs=1/T;
ws=2*pi*fs;

% A matrices
dim = 6;
A1=[-1*(rs+rL)/L -1/L;1/C -1/(R*C)];
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
%controlPeriod = 2.5000e-04; % controlPeriod = tfinal
%nSteps = 300;
controlPeriod = tfinal(2);
nSteps = rSteps(2);
reachStep = controlPeriod/nSteps;
Plant = LinearODE(A, B, C, D, controlPeriod,nSteps); % Linear ODE plant
plant_cora = LinearODE_cora(A, B, C, D, reachStep, controlPeriod); % LinearODE cora plant
% nlPlant = NonLinearODE(6,1,@dynamicsMFAM,reachStep,controlPeriod,C); % Nonlinear ODE plant 


%% Define reachability parameters

lb = [i_lb;v_lb;0;0;0;0];
ub = [i_ub;v_ub;0;0;0;0];
init_set = Star(lb,ub);

%% Simulation

% Sim (CORA)
%n_sim = 25;
X0s = lb'+rand(n_sim,dim).*(ub'-lb'); % Set of random initial points to simulate
sim1 = zeros(n_sim,6,nSteps+1); % Store all simulation data
yC = 1; % Initial input (If u = 0, the reach plots don't look appealing)
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
title('MultiFrequency Average Model (2)');
saveas(f,'reachMFAM2_orig.png');

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
title('MultiFrequency Average Model (2)');
saveas(f,'reachMFAM2_orig_I.png');

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
title('MultiFrequency Average Model (2)');
saveas(f,'reachMFAM2_orig_V.png');

save('Origdata_'+string(controlPeriod)+'_'+string(n_sim)+'.mat','reachAll','tsim','treach','-v7.3');
