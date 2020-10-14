%% Reachability analysis of the buck converter using NNV
% Load hybrid automata
buck = HybridA(5,1,buck_v1,2);

% Setup reachability parameters
steps = 9;
% Ts = 1.6667e-05 / 10; % 1/10 switching frequency
Ts = 0.0000001;
buck.set_tFinal(steps*Ts);
buck.set_timeStep(Ts);

% Execute reachability analysis star set NNV
inp_set = Star;
lb = [-0.09; 0; 0; 0.75; -0.09];
ub = [0.11; 0; 0; 0.75; 0.11];
init_set = Star(lb,ub);
S = buck.stepReachStar(init_set,inp_set);
Sall = buck.intermediate_reachSet;

figure;
Star.plotBoxes_2D_noFill(Sall,3,1,'b');

figure;
Star.plotBoxes_2D_noFill(Sall,3,5,'b');

figure;
Star.plotBoxes_2D_noFill(Sall,1,5,'b');

figure;
Star.plotBoxes_2D_noFill(Sall,3,2,'b');


%% CORA zonotope reachability
x0 = [0.01; 0; 0; 0.75; 0.01]; %initial state for simulation
R0 = zonotope([x0, diag([0.1, 0.0, 0.0, 0.0, 0.1])]); %initial state for reachability analysis
[buck.sysCORA] = reach(buck.sysCORA,buck.options);

%choose projection and plot------------------------------------------------
figure 
hold on
buck.options.projectedDimensions = [3 1];
buck.options.plotType = 'b';
plot(buck.sysCORA,'reachableSet',buck.options); %plot reachable set
plotFilled(R0,buck.options.projectedDimensions,'w','EdgeColor','k'); %plot initial set

figure 
hold on
buck.options.projectedDimensions = [3 5];
buck.options.plotType = 'b';
plot(buck.sysCORA,'reachableSet',buck.options); %plot reachable set
plotFilled(R0,buck.options.projectedDimensions,'w','EdgeColor','k'); %plot initial set

figure 
hold on
buck.options.projectedDimensions = [1 5];
buck.options.plotType = 'b';
plot(buck.sysCORA,'reachableSet',buck.options); %plot reachable set
plotFilled(R0,buck.options.projectedDimensions,'w','EdgeColor','k'); %plot initial set

figure 
hold on
buck.options.projectedDimensions = [3 2];
buck.options.plotType = 'b';
plot(buck.sysCORA,'reachableSet',buck.options); %plot reachable set
plotFilled(R0,buck.options.projectedDimensions,'w','EdgeColor','k'); %plot initial set

