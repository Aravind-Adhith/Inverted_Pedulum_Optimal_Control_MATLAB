clc
clear all
close all
pause('off')
m = 1;              % Mass of the pendulum in kg
M = 5;              % Mass of the cart in kg
L = 2;              % Length of pendulum center of mass in m
g = -10;            % Acceleration due to gravity down in m/s^2
d = 1;              % damping Ratio, 1 is critically damped

s = 1;              % Pendulum is up and vertical at s = 1
% Linearized A and B matrix about the above fixed values using Jacobian

A = [0              1                   0              0;...
     0           -d/M              -m*g/M              0;...
     0              0                   0              1;...
     0      -s*d/(M*L)     -s*(m+M)*g/(M*L)            0];

B = [0; 1/M; 0; s*1/(M*L)];

fprintf('<strong> Matrix A for the State Space Model is: </strong>');
disp(A);

fprintf('<strong> Matrix B for the State Space Model is: </strong>');
disp(B);

fprintf('<strong> Eigen Values of the State Space Model is: </strong>');

disp(eig(A));

% One of the eigen value is +ve, therefore currently our system is
% unstable!, which for us right now stability exists when the pendulum is
% upright
K_Controllability = ctrb(A,B);
Rank_K_Controllability = rank(K_Controllability);

disp('<strong> The rank of the controllability matrix is: </strong>');
disp(rank(ctrb(A,B)));  % is it controllable

if Rank_K_Controllability == 4
    disp('<strong> Thus, the LTI system is completely controllable </strong>');
end
%% Pole Placement Trial 1
P1 = [-10; -11; -12; -13]; % desired CL pole locations
K1 = place(A, B, P1); % proportional gains
A_closed1 = A - B*K1; % equivalent open-loop A for the closed-loop system
% Simulating
% Cartpend gives the State Space equations that are found using lagrange
% equations
tspan = 0:.001:30;
if(s==-1)
    y0 = [0; 0; 0; 0];
    [t,y] = ode45(@(t,y)cartpend(y,m,M,L,g,d,-K1*(y-[4; 0; 0; 0])),tspan,y0);
elseif(s==1)
    y0 = [3; 0; pi+.5; 0];
    [t,y] = ode45(@(t,y)cartpend(y,m,M,L,g,d,-K1*(y-[0; 0; pi; 0])),tspan,y0);
else 
end

plot(t,y)

