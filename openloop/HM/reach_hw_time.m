% Reachability analysis of the average model
% Diego Manzanas, October 12th 2020

%% Load Plant

% Load hybrid Automata
dim = 4;
HA = HybridA(dim,1,HM_hardware_time,3);
% HA.options.enclose = {'pca'}; % This fails sometimes and very
% Ha.options.guardIntersect = {'pancake'};
% conservative
HA.options.enclose = {'box'};
% HA.options.linAlg = 'adap';
HA.options.error = 0.001;
% HA.options.zonotopeOrder = 100;
T = 1/60000; % sample time
Vs = 100; % source voltage
Vref = 48; % reference voltage

%% Set reachability and simulation
controlPeriod = T;
% nSteps = 317;
nSteps = 523;
reachStep = controlPeriod/nSteps;
out_mat = [1 0 0 0; 0 1 0 0];
% tfinal = T*1800;
% mfac = 1800;
% mfac = 800;
mfac = 790;
% mfac = 800;
% tfinal = T*mfac;
% controlPeriod = tfinal(4);
controlPeriod = 0.011667;
tfinal = controlPeriod;
yC = 1;

i_lb = 0; % lower bound current
v_lb = 0; % lower bound voltage
i_ub = 0.2; % upper bound current
v_ub = 0.2; % upper bound voltage

lb = [i_lb;v_lb;0;0];
ub = [i_ub;v_ub;0;0];
init_set = Star(lb,ub);

HA.set_tFinal(controlPeriod); % set control period
HA.set_timeStep(reachStep); % reachability step

%%%%%%%%%%%%%%%% Simulation %%%%%%%%%%%%%%%%%%%
% Sim (CORA)
% n_sim = 100;
n_sim = 1;
Nt = nSteps*mfac+1;
X0s = lb'+rand(n_sim,dim).*(ub'-lb'); % Set of random initial points to simulate
sim1 = zeros(n_sim,dim,Nt); % Store all simulation data
simcell = cell(n_sim,dim);
t = tic;
for j=1:n_sim
    x0 = X0s(j,:)';
    [tV,y,loc] = HA.evaluate(yC,x0);
    simcell{j,1} = y;
    simcell{j,2} = loc;
    simcell{j,3} = tV;
end
tsim = toc(t);
disp('Finish Simulation in '+string(tsim)+' seconds');

% Plot only simulation
timeV = 0:reachStep:tfinal;
t = tic;
f = figure('visible','off');
hold on;
for p=1:n_sim
    temp = simcell{p};
    for i=1:length(temp)
        temp2 = temp{i};
        plot(temp2(:,1),temp2(:,2),'r');
    end
end
hold on;
xlabel('i')
ylabel('V');
title('Hybrid');
saveas(f,'reachHM_hw_time_simulation.png');

f = figure('visible','off');
hold on;
for p=1:n_sim
    temp = simcell{p};
    for i=1:length(temp)
        temp2 = temp{i};
        plot(temp2(:,4),temp2(:,1),'r');
    end
end
hold on;
xlabel('Time (s)')
ylabel('V');
title('Hybrid');
saveas(f,'reachHM_hw_timeI_simulation.png');
f = figure('visible','off');
hold on;
for p=1:n_sim
    temp = simcell{p};
    for i=1:length(temp)
        temp2 = temp{i};
        plot(temp2(:,4),temp2(:,2),'r');
    end
end
hold on;
xlabel('Time (s)')
ylabel('V');
title('Hybrid');
saveas(f,'reachHM_hw_timeV_simulation.png');
tviz = toc(t);
%%
%%%%%%%%%%%%%%%  Reachability  %%%%%%%%%%%%%%%%%
% HA.set_tFinal(tfinal); % set control period
% HA.set_timeStep(reachStep); % reachability step
inp_set = Star(yC,yC); % same mode as initial state
t = tic;
S = HA.stepReachStar(init_set,inp_set);
% [m,M] = S.getRanges;
% S2 = Star(m,M);
% S2 = HA.stepReachStar(S,inp_set);
treach = toc(t);
Sall = [init_set HA.intermediate_reachSet];
disp('Finish Reachability in '+string(treach)+' seconds');

%% Visualize results
timeV = 0:reachStep:tfinal;
t = tic;
f = figure('visible','off');
hold on;
for i=1:100:length(Sall)
    Star.plotBoxes_2D_noFill(Sall(i),1,2,'b');
end
for p=1:n_sim
    temp = simcell{p};
    for i=1:length(temp)
        temp2 = temp{i};
        plot(temp2(:,1),temp2(:,2),'r');
    end
end
hold on;
xlabel('i')
ylabel('V');
title('Hybrid');
saveas(f,'reachHM_hw_time.png');
f = figure('visible','off');
hold on;
for i=1:100:length(Sall)
    Star.plotBoxes_2D_noFill(Sall(i),4,1,'b');
end
for p=1:n_sim
    temp = simcell{p};
    for i=1:length(temp)
        temp2 = temp{i};
        plot(temp2(:,4),temp2(:,1),'r');
    end
end
hold on;
xlabel('Time (s)')
ylabel('V');
title('Hybrid');
saveas(f,'reachHM_hw_timeI.png');
f = figure('visible','off');
hold on;
for i=1:100:length(Sall)
    Star.plotBoxes_2D_noFill(Sall(i),4,2,'b');
end
for p=1:n_sim
    temp = simcell{p};
    for i=1:length(temp)
        temp2 = temp{i};
        plot(temp2(:,4),temp2(:,2),'r');
    end
end
hold on;
xlabel('Time (s)')
ylabel('V');
title('Hybrid');
saveas(f,'reachHM_hw_timeV.png');
tviz = toc(t);

disp('Total time = '+string(tviz+treach+tsim));
disp(' ');
disp('Time Viz = ' + string(tviz) + ' --- Time Reach = '+string(treach) + '  ---  Time Sim = '+string(tsim))
disp(' ');

save('hwdata_'+string(controlPeriod)+'_'+string(n_sim)+'.mat','reachAll','tsim','treach','-v7.3');
%% Helper Functions

function inNN = input_to_Controller(Vref,init_set,out_mat)
    l = length(init_set);
    inNN = [];
    for i = 1:l
        out1 = init_set(i).affineMap([0 1 0 0],-Vref); % input 1 (Vref - Vout)
        out1 = out1.affineMap(-eye(1),[]);
        out12 = out1.concatenate_with_vector(Vref); % input 1 + input 2
        out = out12.concatenate(init_set(i).affineMap(out_mat,[])); % add inputs 3 and 4 (Vout,Iout)
        inNN = [inNN out];
    end
end