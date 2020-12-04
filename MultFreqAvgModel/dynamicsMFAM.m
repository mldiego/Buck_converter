function dx = dynamicsMFAM(x,u)
% set up parameters
C = 4.4e-006; % capacitor value
L = 50e-006; % inductor
R = 4;% resistor 
T = 0.00001;% sample time
Vs = 10;% input (source) voltage 10
Vref = 6;% desired output voltage 6
% Tmax = 1000*(1/R*C);% max simulation time
Du = Vref / Vs;% duty cycle

% index 1 avg of duty cycle
D_R1=1/(2*pi)*sin(2*pi*Du); %  Real part 
D_I1=1/(2*pi)*(cos(2*pi*Du)-1); % Imaginary part
fs=1/T;
ws=2*pi*fs;

% A matrices
A1=[0 -1/L;1/C -1/(R*C)];
A2=[0 0 0 0;0 0 0 0];
A3=[0 0;0 0;0 0;0 0];
A4=[0 ws -1/L 0;-ws 0 0 -1/L;1/C 0 -1/(R*C) ws;0 1/C -ws -1/(R*C)];

% State-space matrices
A=[A1 A2;A3 A4];
B=[Du/L;0;D_R1/L;D_I1/L;0;0];
B = Vs*B;
% C=eye(size(A));
% D=[0 0 0 0 0 0]';

% Initial state
% x0=[0 0 0 0 0 0]'; % [i_L0 v_C0 i_L1Real i_L1Imag v_C1Real v_C1Imag]

dx = A*x + B*u;

end

