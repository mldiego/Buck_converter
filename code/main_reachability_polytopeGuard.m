opt_controller = 0;

if opt_controller
load network.mat;
Layers = [];
W = network.weights;
b = network.bias;
n = length(b);
for i=1:n - 1
    bi = cell2mat(b(i));
    Wi = cell2mat(W(i));
    Li = Layer(Wi, bi, 'ReLU');
    Layers = [Layers Li];
end
bn = cell2mat(b(n));
Wn = cell2mat(W(n));
Ln = Layer(Wn, bn, 'Linear');

Layers = [Layers Ln];
F = FFNN(Layers);

% % x = [error, vref, vout, icurrent]
% lb = [-3; 5; 5; 0];
% ub = [3; 15; 15; 15];
% 
% % normalize input
% 
% V = Reduction.getVertices(lb, ub);
% 
%I = Polyhedron('V', V');
% 
%[R, t] = F.reach(I, 'exact', 4, []); % exact reach set
% save F.mat F; % save the verified network
% F.print('F.info'); % print all information to a file

end

%Plant = spaceex2cora('buck.xml','sys');

opt_copy = 1;

if opt_copy
spaceex2cora('buck_v1.xml','buckboost');
%cd([coraroot,'/models/SpaceExConverted/']);
copyfile([coraroot,filesep,'models',filesep,'SpaceExConverted',filesep,'buck_v1',filesep,'buck_v1.m'], [pwd(), filesep, 'buck_v1.m']);
end

HA = buck_v1;

options.x0 = [0.01; 0; 0; 0.01]; %initial state for simulation
options.R0 = zonotope([options.x0, diag([0.01, 0.0, 0.0, 0.01])]); %initial state for reachability analysis
options.startLoc = 1; %initial location
options.finalLoc = 0; %0: no final location
options.tStart = 0; %start time

steps = 5000;
%Ts = 5e-6;
Ts = 1.6667e-05 / 10; % 1/10 switching frequency

options.tFinal = steps * Ts; %final time


options.timeStep = Ts;
options.timeStepLoc{1} = Ts; %time step size for reachable set computation in location 1
options.timeStepLoc{2} = Ts; %time step size for reachable set computation in location 1

options.taylorTerms = 10;
options.zonotopeOrder = 20;
options.polytopeOrder = 10;
options.errorOrder=2;
options.reductionTechnique = 'girard';
options.isHyperplaneMap = 0;
options.guardIntersect = 'zonoGirard';%'polytope';
options.enclosureEnables = 5; %choose enclosure method(s)
options.originContained = 0;

N = 4;

%for i=1:N
%    
%end

options.uLoc{1} = 0; % no inputs
options.uLoc{2} = 0; % no inputs

options.uLocTrans{1} = 0; % no inputs
options.uLocTrans{2} = 0; % no inputs

options.Uloc{1} = zonotope(0); % no inputs
options.Uloc{2} = zonotope(0); % no inputs

opt_simulate = 1;

if opt_simulate
%simulate hybrid automaton
[HA] = simulate(HA,options);
end

[HA] = reach(HA,options);

%choose projection and plot------------------------------------------------
figure 
hold on
options.projectedDimensions = [3 1];
options.plotType = 'b';
plot(HA,'reachableSet',options); %plot reachable set
plotFilled(options.R0,options.projectedDimensions,'w','EdgeColor','k'); %plot initial set
if opt_simulate
plot(HA,'simulation',options); %plot simulation
end

figure 
hold on
options.projectedDimensions = [3 4];
options.plotType = 'b';
plot(HA,'reachableSet',options); %plot reachable set
plotFilled(options.R0,options.projectedDimensions,'w','EdgeColor','k'); %plot initial set
if opt_simulate
plot(HA,'simulation',options); %plot simulation
end

figure 
hold on
options.projectedDimensions = [1 4];
options.plotType = 'b';
plot(HA,'reachableSet',options); %plot reachable set
plotFilled(options.R0,options.projectedDimensions,'w','EdgeColor','k'); %plot initial set
if opt_simulate
plot(HA,'simulation',options); %plot simulation
end

figure 
hold on
options.projectedDimensions = [3 2];
options.plotType = 'b';
plot(HA,'reachableSet',options); %plot reachable set
plotFilled(options.R0,options.projectedDimensions,'w','EdgeColor','k'); %plot initial set
if opt_simulate
plot(HA,'simulation',options); %plot simulation
end