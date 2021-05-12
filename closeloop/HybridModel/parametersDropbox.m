% University of Texas at Arlington
% 
%
% usage: first, open Simulink/StateFlow model dc2dc_dcm.slx / dc2dc_dcm.mdl
%        second: execute this file, parameters.m
% this sets up parameters for simulation, then simulates the models 
% and generates plots

clc  % clears command window
clear all; % warning, all results and workspace cleared


% source voltage noise for stateflow simulation
Vs_noise = 0;%0.1;

% initial current and voltage as per experiment
i0 = 0.1785;
v0 = 0.6666;

%PROTOTYPE BUCK IN THE LAB%%%%%%%%%%%%%%%%%%%
T = 1/60000;
Vs = 100;
Vref = 48;
C = 2.2e-3; % measured value 2.19e-3; %data sheet value 2.2e-3
L = 2.65e-3; % measured value 3.12e-3 ; %data sheet value 2.65e-3
R=10;
rs=3.5;
rL=520e-3;
Tmax = T*1800;% For Tmax=0.03
cr=1.265;%Correction applied to duty ratio to cater losses plecs 1.27, stateflow 1.265
%%%  END PROTOTYPE BUCK IN THE LAB  %%%%%%%%

% Define transition matrices

Ac_nom = [-1*(rs+rL)/L, -(1/L); (1/C), -(1/(R*C))];% switch closed
Bc_nom = [(1/L); 0];

Ao_nom = [-rL/L, -(1/L); (1/C), -(1/(R*C))];% switch open
Bo_nom = [0; 0];

Ad_nom = [0, 0; 0, -(1/(R*C))];%For DCM
Bd_nom = [0; 0];%For DCM
        
D = cr*Vref / Vs;% duty cycle

sys(1).Ac = Ac_nom;
sys(1).Bc = Bc_nom;
sys(1).Ao = Ao_nom;
sys(1).Bo = Bo_nom;
sys(1).Ad = Ad_nom;%For DCM
sys(1).Bd = Bd_nom;%For DCM

% set parameters used in stateflow simulation
a00c = sys(1).Ac(1,1);
a01c = sys(1).Ac(1,2);
a10c = sys(1).Ac(2,1);
a11c = sys(1).Ac(2,2);

b0c = sys(1).Bc(1);
b1c = sys(1).Bc(2);

a00o = sys(1).Ao(1,1);
a01o = sys(1).Ao(1,2);
a10o = sys(1).Ao(2,1);
a11o = sys(1).Ao(2,2);

b0o = sys(1).Bo(1);
b1o = sys(1).Bo(2);

%DCM

a00d = sys(1).Ad(1,1);
a01d = sys(1).Ad(1,2);
a10d = sys(1).Ad(2,1);
a11d = sys(1).Ad(2,2);

b0d = sys(1).Bd(1);
b1d = sys(1).Bd(2);
    
%simulate the model
[tout,yout]=sim('slsf_buck');
% [tout,yout]=sim('plecs_buck');
%plot the results
figure
plot(tout,yout(:,2),'b-','LineWidth',2);grid;
xlabel('Time,Sec');ylabel('Capacitor Voltage, Volts');
legend('Capacitor Voltage - Hybrid Automaton Stateflow')
figure
plot(tout,yout(:,3),'r-','LineWidth',2);grid;
xlabel('Time,Sec');ylabel('Inductor Current, Amps');
legend('Inductor Current - Hybrid Automaton Stateflow')
figure
plot(yout(:,2),yout(:,3),'g-','LineWidth',2);grid;
xlabel('Inductor Current, Amps');ylabel('Capacitor Voltage, Volts');