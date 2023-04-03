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
tspan = 0:.001:10;
if(s==-1)
    y0 = [0; 0; 0; 0];
    [t,y] = ode45(@(t,y)cartpend(y,m,M,L,g,d,-K1*(y-[4; 0; 0; 0])),tspan,y0);
elseif(s==1)
    y0 = [-3; 0; pi+.1; 0];
    [t,y] = ode45(@(t,y)cartpend(y,m,M,L,g,d,-K1*(y-[0; 0; pi; 0])),tspan,y0);
else 
end

myVideo = VideoWriter('PolePlace_Trial1'); % open video file
myVideo.FrameRate = 35;  
open(myVideo)

for k=1:100:length(t)
    drawcartpend_bw(y(k,:),m,M,L);
    pause(0.01)  % Pause and grab frame
    frame = getframe(gcf); % get frame
    writeVideo(myVideo, frame);
end
close(myVideo);


%% Pole Placement Trial 2
P2 = [-1; -1.1; -1.2; -1.3]; % desired CL pole locations
K2 = place(A, B, P2); % proportional gains
A_closed2 = A - B*K2; % equivalent open-loop A for the closed-loop system

tspan = 0:.001:10;
if(s==-1)
    y0 = [0; 0; 0; 0];
    [t,y] = ode45(@(t,y)cartpend(y,m,M,L,g,d,-K2*(y-[4; 0; 0; 0])),tspan,y0);
elseif(s==1)
    y0 = [-3; 0; pi+.1; 0];
    [t,y] = ode45(@(t,y)cartpend(y,m,M,L,g,d,-K2*(y-[0; 0; pi; 0])),tspan,y0);
else 
end

myVideo = VideoWriter('PolePlace_Trial2'); % open video file
myVideo.FrameRate = 35;  
open(myVideo)

for k=1:100:length(t)
    drawcartpend_bw(y(k,:),m,M,L);
    pause(0.01)  % Pause and grab frame
    frame = getframe(gcf); % get frame
    writeVideo(myVideo, frame);
end
close(myVideo);


%% TRIAL 3
P3 = [-2; -2.1; -2.3; -2.4]; % desired CL pole locations
K3 = place(A, B, P3); % proportional gains
A_closed3 = A - B*K3; % equivalent open-loop A for the closed-loop system

tspan = 0:.001:10;
if(s==-1)
    y0 = [0; 0; 0; 0];
    [t,y] = ode45(@(t,y)cartpend(y,m,M,L,g,d,-K3*(y-[4; 0; 0; 0])),tspan,y0);
elseif(s==1)
    y0 = [-3; 0; pi+.1; 0];
    [t,y] = ode45(@(t,y)cartpend(y,m,M,L,g,d,-K3*(y-[0; 0; pi; 0])),tspan,y0);
else 
end

myVideo = VideoWriter('PolePlace_Trial3'); % open video file
myVideo.FrameRate = 35;  
open(myVideo)

for k=1:100:length(t)
    drawcartpend_bw(y(k,:),m,M,L);
    pause(0.01)  % Pause and grab frame
    frame = getframe(gcf); % get frame
    writeVideo(myVideo, frame);
end
close(myVideo);


%% LQR Controller
% Q (State wighing matrix) is:
Q = diag([10 1 10 100]);

% R (Input weighing matrix) is:
R = .1; 
% Our control law is still u = -K*x
K_lqr = lqr(A,B,Q,R); % where K is my state feedback gain matrix

disp('<strong> Poles placed by LQR controller: </strong>');
disp(eig(A-B*K_lqr));

tspan = 0:.001:10;
if(s==-1)
    y0 = [0; 0; 0; 0];
    [t,y] = ode45(@(t,y)cartpend(y,m,M,L,g,d,-K_lqr*(y-[4; 0; 0; 0])),tspan,y0);
elseif(s==1)
    y0 = [-3; 0; pi+.1; 0];
    [t,y] = ode45(@(t,y)cartpend(y,m,M,L,g,d,-K_lqr*(y-[0; 0; pi; 0])),tspan,y0);
else
    
end
myVideo = VideoWriter('LQR_Trial'); % open video file
myVideo.FrameRate = 35;  
open(myVideo)

for k=1:100:length(t)
    drawcartpend_bw(y(k,:),m,M,L);
    pause(0.01)  % Pause and grab frame
    frame = getframe(gcf); % get frame
    writeVideo(myVideo, frame);
end
close(myVideo);


%%
C1 = [1 0 0 0]; % Assuming that we are just measuring the cart's position x 

O_Observability_x = obsv(A,C1); % Observability matrix with no outputs zero'd
Rank_O_Observability_x = rank(O_Observability_x);

disp('<strong> The rank of the observability matrix is: </strong>');
disp(Rank_O_Observability_x);
if Rank_O_Observability_x == 4
    disp('<strong> Thus, the LTI system is completely observable </strong>');
end

% Let's see if other measurements are observable or not 

C2 = [0 0 1 0]; % Assuming that we are just measuring the pendulum's angle theta 

O_Observability_theta = obsv(A,C2); % Observability matrix with no outputs zero'd
Rank_O_Observability_theta = rank(O_Observability_theta);

disp('<strong> The rank of the observability matrix is: </strong>');
disp(Rank_O_Observability_theta);
if Rank_O_Observability_theta == 4
    disp('<strong> Thus, the LTI system is completely observable </strong>');
else 
    disp('<strong> the LTI system is not completely observable </strong>');
end

% So we conclude that if we just calculate/measure theta then
% the system is not completely observable 
% Note: that is also the case for other measurements xdot, thetadot, if you
% check their ranks respectively


% Let's also check which measurement is best for observability if we omit x

C_omitX_measurexdot = [1 0 0];
A_omitX_measurexdot = A(2:end, 2:end);
B_omitX_measurexdot = B(2:end);

O_Observability_omitX_measurexdot = obsv(A_omitX_measurexdot,C_omitX_measurexdot); 
Rank_O_Observability_omitX_measurexdot = rank(O_Observability_omitX_measurexdot);

% Since it is full rank now if the rank is 3 because we have omitted one of
% the states
disp('<strong> The rank of the observability matrix is: </strong>');
disp(Rank_O_Observability_omitX_measurexdot);
if Rank_O_Observability_omitX_measurexdot == 3
    disp('<strong> Thus, the LTI system is completely observable </strong>');
else 
    disp('<strong> the LTI system is not completely observable </strong>');
end

C_omitX_measuretheta = [0 1 0];
A_omitX_measuretheta = A(2:end, 2:end);

O_Observability_omitX_measuretheta = obsv(A_omitX_measuretheta,C_omitX_measuretheta); 
Rank_O_Observability_omitX_measuretheta = rank(O_Observability_omitX_measuretheta);

disp('<strong> The rank of the observability matrix is: </strong>');
disp(Rank_O_Observability_omitX_measuretheta);
if Rank_O_Observability_omitX_measuretheta == 3
    disp('<strong> Thus, the LTI system is completely observable </strong>');
else 
    disp('<strong> the LTI system is not completely observable </strong>');
end

C_omitX_measurethetadot = [0 0 1];
A_omitX_measurethetadot = A(2:end, 2:end);

O_Observability_omitX_measurethetadot = obsv(A_omitX_measurethetadot,C_omitX_measurethetadot); 
Rank_O_Observability_omitX_measurethetadot = rank(O_Observability_omitX_measurethetadot);

disp('<strong> The rank of the observability matrix is: </strong>');
disp(Rank_O_Observability_omitX_measurethetadot);
if Rank_O_Observability_omitX_measurethetadot == 3
    disp('<strong> Thus, the LTI system is completely observable </strong>');
else 
    disp('<strong> the LTI system is not completely observable </strong>');
end

% Note: you will notice that if you calculate the gramian determinant at
% s=1 (Pendulum is upright), you will get an error because the system is
% unstable !! We can calculate gramian when the system is stable so we set
% s = -1 ! 

s_stable = -1;
A_checkgramian = [0              1                   0                            0;...
                  0           -d/M              -m*g/M                            0;...
                  0              0                   0                            1;...
                  0      -s_stable*d/(M*L)     -s_stable*(m+M)*g/(M*L)            0];

B_checkgramian = [0; 1/M; 0; s_stable*1/(M*L)];

A_checkgramian = A_checkgramian(2:end, 2:end);
B_checkgramian = B_checkgramian(2:end);

D_omitX_measurexdot = zeros(size(C_omitX_measurexdot,1),size(B_checkgramian,2));
sys_omitX_measurexdot = ss(A_checkgramian,B_checkgramian,C_omitX_measurexdot,D_omitX_measurexdot);
disp('<strong> Determinant of gramian if we just measure xdot </strong>');
disp(det(gram(sys_omitX_measurexdot,'o')));

D_omitX_measuretheta = zeros(size(C_omitX_measuretheta,1),size(B_checkgramian,2));
sys_omitX_measuretheta = ss(A_checkgramian,B_checkgramian,C_omitX_measuretheta,D_omitX_measuretheta);
disp('<strong> Determinant of gramian if we just measure theta </strong>');
disp(det(gram(sys_omitX_measuretheta,'o')));

D_omitX_measurethetadot = zeros(size(C_omitX_measurethetadot,1),size(B_checkgramian,2));
sys_omitX_measurethetadot = ss(A_checkgramian,B_checkgramian,C_omitX_measurethetadot,D_omitX_measurethetadot);
disp('<strong> Determinant of gramian if we just measure theta dot </strong>');
disp(det(gram(sys_omitX_measurethetadot,'o')));

%% Kalman Filter
% Trying to simulate for pendulum down
s_estimation = -1;

A_estimation = [0              1                   0              0;...
     0           -d/M              -m*g/M              0;...
     0              0                   0              1;...
     0      -s_estimation*d/(M*L)     -s_estimation*(m+M)*g/(M*L)            0];

B_estimation = [0; 1/M; 0; s_estimation*1/(M*L)];



%  Augmented system with disturbances and noise
Vd = .1*eye(4);         % disturbance covariance
Vn = 1;                 % noise covariance

% augment inputs to include disturbance and noise
BF = [B_estimation Vd 0*B_estimation];  % Takes an input u, d and n

% Note: This is big augmented state space system, which gives you the
% estimated state x

sysC = ss(A_estimation,BF,C1,[0 0 0 0 0 Vn]); 


% Note: this is the Full state System, which gives you true full-state (all
% 4 states)
% system with full state output, disturbance, no noise
sysFullOutput = ss(A_estimation, BF, eye(4), zeros(4,size(BF,2)));  % eye(4) because I want to measure all 4 states 

% alternatively, possible to design using "LQR" code
Kf = (lqr(A_estimation',C1',Vd,Vn))'; % Where we try to trick the lqr controller that we are giving in A and B (A' and C'), and our weight matrixes Q and R (Vd and Vn)

% Kalman filter estimator
% we have C as eye(4) because we want to estimate all 4 states and then our
% D is just 0
sysKF = ss(A_estimation-Kf*C1,[B_estimation Kf],eye(4),0*[B_estimation Kf]); 

dt = .01;
t = dt:dt:50;

uDIST = randn(4,size(t,2));
uNOISE = randn(size(t));
u = 0*t;                            % we start off with 0 input 
u(100:120) = 100;                   % impulse, then at some time forward input force
u(1500:1520) = -100;                % impulse, followed by backward input force

uAUG = [u; Vd*Vd*uDIST; uNOISE];

% Full state measurement with noise
[y,t] = lsim(sysC,uAUG,t);

% True Full state without noise
[xtrue,t] = lsim(sysFullOutput,uAUG,t);

% Simulating Kalman Filter estimate
[x,t] = lsim(sysKF,[u; y'],t);

% Plots 

figure
plot(t,y)
hold on
grid on
plot(t,xtrue(:,1),'r'); % No noise so we just want to know what the underlying full state is
plot(t,x(:,1),'k--'); % Kalman filter estimate of state x
hold off 
legend('Measurement y(just x here) with ton of noise', 'True Measurement y (just x here) with 0 noise', 'Kalman Filter estimate of measurement y (just x out of the 4 states) ');
xlabel('Time (seconds)');
ylabel('Measurement y(output) only x');
title('Measurement y but only measuring 1 of the 4 states i.e x here and comparing');

% In order to maximize the figure window in Windows
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0, 0.04, 1, 0.96]);


% Lets plot the Kalman Filter estimate and Trus Full state for all the 4
% states now
figure
plot(t, xtrue,'-', t, x,'--','LineWidth',2);
grid on 
xlabel('Time (seconds)');
ylabel('Measurement y (output) of all 4 states (x, xdot, theta and thetadot)');
title('Measurement Comparison of all 4 True states and the Kalman Filter estimate of each');

% In order to maximize the figure window in Windows
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0, 0.04, 1, 0.96]);
%% PP trial 1 Plot
P1 = [-0.3; -0.4; -0.5; -0.6]; % desired CL pole locations
K1 = place(A, B, P1); % proportional gains
A_closed1 = A - B*K1; % equivalent open-loop A for the closed-loop system


tspan = 0:.001:30;
if(s==-1)
    y0 = [0; 0; 0; 0];
    [t,y] = ode45(@(t,y)cartpend(y,m,M,L,g,d,-K1*(y-[4; 0; 0; 0])),tspan,y0);
elseif(s==1)
    y0 = [-3; 0; pi+.1; 0];
    [t,y] = ode45(@(t,y)cartpend(y,m,M,L,g,d,-K1*(y-[0; 0; pi; 0])),tspan,y0);
else 
end
u1=-K1.*y;
figure()
grid on; hold on;
plot(tspan,u1(:,1),'Linewidth',2)
plot(tspan,u1(:,2),'Linewidth',2)
plot(tspan,u1(:,4),'Linewidth',2)
title('Pole Placement 1')
xlabel('time')
ylabel('Control law u')
hl = legend('${x}$','$\dot{x}$','$\dot{\theta}$');
set(hl, 'Interpreter', 'latex');
%% PP trial 2 Plot
P2 = [-1; -1.1; -1.2; -1.3]; % desired CL pole locations
K2 = place(A, B, P2); % proportional gains
A_closed2 = A - B*K2; % equivalent open-loop A for the closed-loop system

tspan = 0:.001:10;
if(s==-1)
    y0 = [0; 0; 0; 0];
    [t,y] = ode45(@(t,y)cartpend(y,m,M,L,g,d,-K2*(y-[4; 0; 0; 0])),tspan,y0);
elseif(s==1)
    y0 = [-3; 0; pi+.1; 0];
    [t,y] = ode45(@(t,y)cartpend(y,m,M,L,g,d,-K2*(y-[0; 0; pi; 0])),tspan,y0);
else 
end
u2=-K2.*y;
figure()
grid on; hold on;
plot(tspan,u2(:,1),'Linewidth',2)
plot(tspan,u2(:,2),'Linewidth',2)
plot(tspan,u2(:,4),'Linewidth',2)
xlabel('time')
ylabel('Control law u for 3 states')
title('Pole Placement 2')
hl = legend('${x}$','$\dot{x}$','$\dot{\theta}$');
set(hl, 'Interpreter', 'latex');
%% PP trial 3 Plot
P3 = [-2; -2.1; -2.3; -2.4]; % desired CL pole locations
K3 = place(A, B, P3); % proportional gains
A_closed3 = A - B*K3; % equivalent open-loop A for the closed-loop system

tspan = 0:.001:10;
if(s==-1)
    y0 = [0; 0; 0; 0];
    [t,y] = ode45(@(t,y)cartpend(y,m,M,L,g,d,-K3*(y-[4; 0; 0; 0])),tspan,y0);
elseif(s==1)
    y0 = [-3; 0; pi+.1; 0];
    [t,y] = ode45(@(t,y)cartpend(y,m,M,L,g,d,-K3*(y-[0; 0; pi; 0])),tspan,y0);
else 
end
u3=-K2.*y;
figure()
grid on; hold on;
plot(tspan,u3(:,1),'Linewidth',2)
plot(tspan,u3(:,2),'Linewidth',2)
plot(tspan,u3(:,4),'Linewidth',2)
xlabel('time')
ylabel('Control law u for 3 states')
title('Pole Placement 3')
hl = legend('${x}$','$\dot{x}$','$\dot{\theta}$');
set(hl, 'Interpreter', 'latex');
%% LQR Plot
Q = diag([10 1 10 100]);

% R (Input weighing matrix) is:
R = .1; 
% Our control law is still u = -K*x
K_lqr = lqr(A,B,Q,R); % where K is my state feedback gain matrix

disp('<strong> Poles placed by LQR controller: </strong>');
disp(eig(A-B*K_lqr));

tspan = 0:.001:10;
if(s==-1)
    y0 = [0; 0; 0; 0];
    [t,y] = ode45(@(t,y)cartpend(y,m,M,L,g,d,-K_lqr*(y-[4; 0; 0; 0])),tspan,y0);
elseif(s==1)
    y0 = [-3; 0; pi+.1; 0];
    [t,y] = ode45(@(t,y)cartpend(y,m,M,L,g,d,-K_lqr*(y-[0; 0; pi; 0])),tspan,y0);
else
 
end
u_lqr=-K_lqr.*y;
figure()
grid on; hold on;
plot(tspan,u_lqr(:,1),'Linewidth',2)
plot(tspan,u_lqr(:,2),'Linewidth',2)
plot(tspan,u_lqr(:,4),'Linewidth',2)
xlabel('time')
ylabel('Control law u for 4 states')
title('LQR')
hl = legend('${x}$','$\dot{x}$','$\dot{\theta}$');
set(hl, 'Interpreter', 'latex');
