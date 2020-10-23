function HA = Buck_Converter(~)


%% Generated on 21-Oct-2020
% Based on the simulink model (old folder)

% % Define Buck converter parameters
% T_sw = 1/10000; % switching time period
% Vs = 30;%30
% % Generate avriable ref sequence for training 
% Vref = [10 10 10 11 12 13 11 10 9 8 6 7 8 9 10 10]; % choose 1?
% %
% C = 2.2e-3;
% L = 2.65e-3;
% R = 8;
% rs=100e-3;% switching loss 
% rL=520e-3;% inductor loss
% 
% %SYSTEM Matrix
% A = [-1*(rs+rL)/L, -1/L; 
%     1/C, -1/(R*C)];
%
% %Input matrices
% u1 = [Vs/L 0]';
% u2 = [0 0]';
%     
a00c = -2.339622641509434e+02;
a01c = -3.773584905660377e+02;
a10c = 4.545454545454545e+02;
a11c = -56.818181818181810;
    
b0c = 1.132075471698113e+04;
b1c = 0.0;
    
a00o = -1.962264150943396e+02;
a01o = -3.773584905660377e+02;
a10o = 4.545454545454545e+02;
a11o = -56.818181818181810;
    
b0o = 0.0;
b1o = 0.0;


%% Define modes 
% x = [i;v;mode;t];

%----------------------------State charging--------------------------------

%% equation (linear model)
%  i_dot = a00c*i + a01c*v+b0c; v_dot = a10c*i+a11c*v+b1c; mode_out = 2?

A = [a00c a01c 0 0;a10c a11c 0 0; 0 0 0 0; 0 0 0 0]; 
B = [0;0;1;0]; % 
k = [b0c; b1c; 0; 1]; % Constant added to each variable as x' = Ax +Bu + k
% C = [0 1 0 0;1 0 0 0;0 0 0 0;0 0 0 0]; % Outputs v & i

% dynamics = linearSys(A,B,k,C);
dynamics = linearSys(A,B,k);
%% equation (invariants)
% Do not really see any invariants in the Simulink model (could add
% something like v & i must be greater than or equal to 0)

invA = [-1 0 0 0;0 -1 0 0];
invb = [0;0];
invOpt = struct('A', invA, 'b', invb);
inv = mptPolytope(invOpt);

trans = {};
%% equation (reset)
%   nothing resets in this transition
resetA = eye(4);
resetb = [0;0;0;0];
reset = struct('A', resetA, 'b', resetb);

%% equation:
%   mode < 1, then move to discharging mode
guardA = [0 0 1 0];
guardb = [1];
% guardOpt = struct('A', guardA, 'b', guardb);
% guard = mptPolytope(guardOpt);
guard = conHyperplane(guardA,guardb);

trans{1} = transition(guard, reset, 2);

loc{1} = location('S1',inv, trans, dynamics);


%---------------------------State discharging------------------------------

%% equation:
% Notes on papers

A = [a00o a01o 0 0;a10o a11o 0 0; 0 0 0 0; 0 0 0 0]; 
B = [0;0;1;0]; % 
k = [b0o; b1o; 0; 1]; % Constant added to each variable as x' = Ax +Bu + k
% C = [0 1 0 0;1 0 0 0;0 0 0 0;0 0 0 0]; % Outputs v & i

% dynamics = linearSys(A,B,k,C);
dynamics = linearSys(A,B,k);

%% equation (invariant)
% Do not really see any invariants in the Simulink model (could add
% something like v & i must be greater than or equal to 0)

invA = [-1 0 0 0;0 -1 0 0];
invb = [0;0];
invOpt = struct('A', invA, 'b', invb);
inv = mptPolytope(invOpt);

trans = {};
%% equation (reset)
%   t' == 0
resetA = eye(4);
resetA(4,4) = 0;
resetb = [0;0;0;0];
reset = struct('A', resetA, 'b', resetb);

%% equation (guard)

% guard 1: if i<0, go to mode 3 (discontinuous)
guardA = [1 0 0 0];
guardb = [0];
% guardOpt = struct('A', guardA, 'b', guardb);
% guard = mptPolytope(guardOpt);
guard = conHyperplane(guardA,guardb);

trans{1} = transition(guard, reset, 3);

% guard 2: if mode >=1, go to mode 1 (charging)
guardA = [0 0 -1 0];
guardb = [-1];

guardOpt = struct('A', guardA, 'b', guardb);
guard = mptPolytope(guardOpt);

trans{2} = transition(guard, reset, 1);

loc{2} = location('S2',inv, trans, dynamics);


%----------------------------State discontinuous--------------------------------

%% equation (linear model)
%  i_dot = 0; v_dot = a11o*v+b1c; 

A = [0 0 0 0; 0 a11o 0 0; 0 0 0 0; 0 0 0 0]; 
B = [0;0;1;0]; % 
k = [0; b1c; 0; 1]; % Constant added to each variable as x' = Ax +Bu + k
% C = [0 1 0 0;1 0 0 0;0 0 0 0;0 0 0 0]; % Outputs v & i

% dynamics = linearSys(A,B,k,C);
dynamics = linearSys(A,B,k);

%% equation (invariants)
% Do not really see any invariants in the Simulink model (could add
% something like v & i must be greater than or equal to 0)
% Seems like this thing won't work without invariants...

invA = [-1 0 0 0;0 -1 0 0];
invb = [0;0];
invOpt = struct('A', invA, 'b', invb);
inv = mptPolytope(invOpt);

trans = {};
%% equation (reset)
%   t == 0
resetA = eye(4);
resetA(4,4) = 0;
resetb = [0;0;0;0];
reset = struct('A', resetA, 'b', resetb);

%% equation:
%   mode >= 1, then move to discharging mode
guardA = [0 0 -1 0];
guardb = [-1];
% guardOpt = struct('A', guardA, 'b', guardb);
% guard = mptPolytope(guardOpt);
guard = conHyperplane(guardA,guardb);

trans{1} = transition(guard, reset, 1);

loc{3} = location('S3',inv, trans, dynamics);


HA = hybridAutomaton(loc);


end