function dx = dynamicsAM(x,u)
% Parameters
C = 4.4e-006; % capacitor value
L = 5e-5; % inductor value
R = 4; % resistor value
% T = 0.00001;% sample time
Vs = 10;% input (source) voltage
Vref = 6;% reference voltage
% Tmax = 1000*(1/R*C);% max simulation time
D = Vref / Vs;% duty cycle        

% % Define ss matrices
% A_avg = [0, -(1/L); (1/C), -(1/(R*C))];% switch closed
% B_avg = [(D/L); 0];
% C = eye(2);
% D = [0;0];

dx(1,1) = (-1/L)*x(2) + (D/L)*u;
dx(2,1) = (1/C)*x(1) - (1/(R*C))*x(2);


end

