% Reachability analysis of the average model
% Diego Manzanas, September 30th 2020
clc;clear;close all

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
nSteps = 5;
reachStep = controlPeriod/nSteps;
Plant = LinearODE(A_avg, B_avg, C, D, controlPeriod,nSteps); % Linear ODE plant
plant_cora = LinearODE_cora(A_avg, B_avg, C, D, reachStep, controlPeriod); % LinearODE cora plant
nlPlant = NonLinearODE(2,1,@dynamicsAM,T/nSteps,T,eye(2)); % Nonlinear ODE plant 


%% Define reachability parameters

% N = 20; % Number of steps to simulate the system
% N = 10;
% N = 15;
N = 5;
% N = 30;

lb = [0;0];
ub = [0.05;0.05];
% ub = [0.2;0.2];
% init_set = Star(lb,ub);
% lb = [0;0]
ub1 = [0.05;0.05];
% reachPRM.init_set = init_set; % Initial set of the converter
% reachPRM.ref_input = Star([Vref;Vs],[Vref;Vs]); % Input reference to controller
% reachPRM.reachMethod = 'approx-star'; % Reachability method for NNCS
% reachPRM.numCores = 4; % Number of cores to be used for exact methods
% reachPRM.numSteps = 1; % Number of steps to simulate the system (forloop to N)


%% Simulation (MATLAB & CORA)

% Sim 1 (MATLAB)
x0 = [0.025;0.025];
t = 0;
dT = T;
sim1 = [x0];
for i=1:N
    Vout = x0(1);
    inCont = [Vref - Vout;Vref;x0];
    yC = Controller.evaluate(inCont);
    [yy,tt,x0] = Plant.simulate([yC;yC],[t T],x0);
    x0 = x0(:,2);
%     t = t+dT;
    sim1 = [sim1 x0];
end


% Sim 2 (CORA)
x0 = [0.025;0.025];
t = 0;
dT = T;
sim2 = [x0];
for i=1:100
    Vout = x0(1);
    inCont = [Vref - Vout;Vref;x0];
    yC = Controller.evaluate(inCont);
    [tV,y] = plant_cora.evaluate(x0,yC);
    x0 = y(end,:)';
    t = t+dT;
    sim2 = [sim2 x0];
end



%% Reachability analysis 1 (NNV, direct and exact + hypercubeHull)
disp(' ');
disp('---------------------------------------------------');
disp('Method 1 - NNV')
init_set = Star(lb,ub1);
plant1 = Plant;
try
    reachSet_1 = [init_set];
    reachAll_1 = [init_set];
    for i=1:N
        inNN = input_to_Controller(Vref,init_set);
        outC = Controller.reach(inNN,'exact-star');
        outC = outC.get_hypercube_hull(outC);
        outC = outC.toStar;
%         init_set = Plant.simReach('direct',init_set,outC,reachStep,100);
        init_set = plant1.simReach('direct', init_set, outC, reachStep, nSteps); % reduce the order (basic vectors) in order for the code to finish
        reachAll_1 = [reachAll_1 init_set];
        init_set = init_set(end);
%         init_set = init_set.orderReduction_box(100);
%         init_set = Star.get_convex_hull(init_set);
        reachSet_1 = [reachSet_1 init_set];
    end
catch e
    disp(' ');
    warning("Method 1 failed"); pause(0.01);
    fprintf(2,'THERE WAS AN ERROR. THE MESSAGE WAS:\n\n%s',getReport(e));
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Reachability analysis 2 (CORA-standard, approx)
disp(' ');
disp('---------------------------------------------------');
disp('Method 2 - CORA');
init_set = Star(lb,ub);
plant2 = plant_cora;
try
    reachSet_2 = [init_set];
    for i=1:N
        inNN = input_to_Controller(Vref,init_set);
        outC = Controller.reach(inNN,'approx-star');
        init_set = plant2.stepReachStar(init_set,outC);
%         init_set = Star.get_convex_hull(init_set);
        reachSet_2 = [reachSet_2 init_set];
    end
catch e
    disp(' ');
    warning("Method 2 failed"); pause(0.01);
    fprintf(2,'THERE WAS AN ERROR. THE MESSAGE WAS:\n\n%s',getReport(e));
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Reachability analysis 3 (CORA-adap, approx)
disp(' ');
disp('---------------------------------------------------');
disp('Method 3 - CORA');
init_set = Star(lb,ub);
plant3 = plant_cora;
plant3.set_linAlg('adap');
try
    reachSet_3 = [init_set];
    for i=1:N
        inNN = input_to_Controller(Vref,init_set);
        outC = Controller.reach(inNN,'approx-star');
        init_set = plant3.stepReachStar(init_set,outC);
%         init_set = Star.get_convex_hull(init_set);
        reachSet_3 = [reachSet_3 init_set];
    end
catch e
    disp(' ');
    warning("Method 3 failed"); pause(0.01);
    fprintf(2,'THERE WAS AN ERROR. THE MESSAGE WAS:\n\n%s',getReport(e));
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Reachability analysis 4 (CORA-standard, exact-convexHull)
disp(' ');
disp('---------------------------------------------------');
disp('Method 4 - CORA');
init_set = Star(lb,ub);
plant4 = plant_cora;
try
    reachSet_4 = [init_set];
    for i=1:N
        inNN = input_to_Controller(Vref,init_set);
        outC = Controller.reach(inNN,'exact-star');
%         if length(outC) > 1
% %             outC = Star.get_convex_hull(outC); % convex hull fails
% %             (cannot be converted to Box due to exitflag = -2 when
% %             converting Star to Zono for reachability analysis)
%             outC = Star.get_hypercube_hull(outC);
%             outC = outC.toStar;
%         end
%         init_set = plant_cora.stepReachStar(init_set,outC);
        last_set = init_set;
        init_set = [];
        for c=1:length(outC)
            init_set = [init_set plant4.stepReachStar(last_set,outC(c))];
        end
        if length(init_set)>1
            init_set = Star.get_convex_hull(init_set);
%             init_set = Star.get_hypercube_hull(init_set);
%             init_set = init_set.toStar;
        end
        reachSet_4 = [reachSet_4 init_set];
    end
catch e
    disp(' ');
    warning("Method 4 failed"); pause(0.01);
    fprintf(2,'THERE WAS AN ERROR. THE MESSAGE WAS:\n\n%s',getReport(e));
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Reachability analysis 5 (NNV, direct, approx)
disp(' ');
disp('---------------------------------------------------');
disp('Method 5 - NNV'); % Not really exact tho. Ranges estimmated in Plant.simReach (minkowski sum for plant reachability)
init_set = Star(lb,ub1);
plant5 = Plant;
try
    reachSet_5 = [init_set];
    reachAll_5 = [init_set];
    for i=1:N
        inNN = input_to_Controller(Vref,init_set);
        outC = Controller.reach(inNN,'approx-star');
        init_set = plant5.simReach('direct',init_set,outC,reachStep,nSteps);
        reachAll_5 = [reachAll_5 init_set];
        init_set = init_set(end);
%         init_set = Star.get_convex_hull(init_set);
        reachSet_5 = [reachSet_5 init_set];
    end
catch e
    disp(' ');
    warning('Method 5 failed'); pause(0.01);
    fprintf(2,'THERE WAS AN ERROR. THE MESSAGE WAS:\n\n%s',getReport(e));
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Reachability analysis 6 (nonlinear)
disp(' ');
disp('---------------------------------------------------');
disp('Method 6 - nonlinear')
init_set = Star(lb,ub);
try
    reachSet_6 = [init_set];
    for i=1:N
        inNN = input_to_Controller(Vref,init_set);
        outC = Controller.reach(inNN,'approx-star');
        init_set = nlPlant.stepReachStar(init_set,outC);
        reachSet_6 = [reachSet_6 init_set];
    end
catch e
    disp(' ');
    warning('Nonlinear method failed'); pause(0.01);
    fprintf(2,'THERE WAS AN ERROR. THE MESSAGE WAS:\n\n%s',getReport(e));
end




%% Visualize results

% Plot Simulations
figure;
hold on;
% plot(sim1(1,:),sim1(2,:),'b');
plot(sim2(1,:),sim2(2,:),'--m');
xlabel('x_1')
ylabel('x_2');
title('Simulations');
legend('nnv','cora');
% Plot Reach Sets
try
    figure;
    hold on;
    Star.plotBoxes_2D_noFill(reachAll_1,1,2,'b');
    Star.plotBoxes_2D_noFill(reachSet_1,1,2,'--m');
    xlabel('x_1')
    ylabel('x_2');
    title('Method 1');
catch
    disp('Method 1 failed, no plots')
end

try
    figure;
    hold on;
    Star.plotBoxes_2D_noFill(plant2.intermediate_reachSet,1,2,'b');
    Star.plotBoxes_2D_noFill(reachSet_2,1,2,'--m');
    xlabel('x_1')
    ylabel('x_2');
    title('Method 2');
catch
    disp('Method 2 failed, no plots')
end

try
    figure;
    hold on;
    Star.plotBoxes_2D_noFill(plant3.intermediate_reachSet,1,2,'b');
    Star.plotBoxes_2D_noFill(reachSet_3,1,2,'--m');
    xlabel('x_1')
    ylabel('x_2');
    title('Method 3');
catch
    disp('Method 3 failed, no plots')
end

try
    figure;
    hold on;
    Star.plotBoxes_2D_noFill(plant4.intermediate_reachSet,1,2,'b');
    Star.plotBoxes_2D_noFill(reachSet_4,1,2,'--m');
    xlabel('x_1')
    ylabel('x_2');
    title('Method 4');
catch
    disp('Method 4 failed, no plots')
end

try
    figure;
    hold on;
    Star.plotBoxes_2D_noFill(reachAll_5,1,2,'b');
    Star.plotBoxes_2D_noFill(reachSet_5,1,2,'--m');
    xlabel('x_1')
    ylabel('x_2');
    title('Method 5');
catch
    disp('Method 5 failed, no plots')
end

try
    figure;
    hold on;
    Star.plotBoxes_2D_noFill(nlPlant.intermediate_reachSet,1,2,'b');
    Star.plotBoxes_2D_noFill(reachSet_6,1,2,'--m');
    xlabel('x_1');
    ylabel('x_2');
    title('Nonlinear');
catch
    disp('NonLinear approx method failed, no plots')
end

%% Helper Functions

function inNN = input_to_Controller(Vref,init_set)
    l = length(init_set);
    inNN = [];
    for i = 1:l
        out1 = init_set(i).affineMap([1 0],-Vref); % input 1 (Vref - Vout)
        out1 = out1.affineMap(-eye(1),[]);
        out12 = out1.concatenate_with_vector(Vref); % input 1 + input 2
        out = out12.concatenate(init_set(i)); % add inputs 3 and 4 (Vout,Iout)
        inNN = [inNN out];
    end
end