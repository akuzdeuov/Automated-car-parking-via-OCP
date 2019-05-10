% Author: Askat Kuzdeuov
% Date: 15 November 2018
% email: askat.kuzdeuov@nu.edu.kz

%% prepare the workspace
close all; clear; clc;

%% initialization
N = 20; % number of samples
T = 15; % motion time (sec)
sigma = 1.2; % distance between centers of the circles around the car (meters)
R = 1; % radius of circles around the car and also the radius of the two obstacles (meters)
x0 = [0,2,0.01]; % initial state (px, py, theta)
xT = [0,0,0];    % final state (px, py, theta)
x_constr = [-3.5 0; 3.5 0]; % coordinates of the two obstacles [x1 y1; x2 y2];

% lower bound and upper bound of the constraints
u1min = -0.5; % the minimum value of the velocity
u1max = 0.5;  % the maximum value of the velocity
u2min = -0.33; % the minimum value of the theta
u2max = 0.33;  % the maximum value of the theta
lb = [ones(N,1)*(u1min); ones(N,1)*(u2min)]; % lower bound 
ub = [ones(N,1)*(u1max); ones(N,1)*(u2max)]; % upper bound 
A = []; b = [];     % linear inequality constraints          
Aeq = []; beq = []; % linear equality constraints
%% function declaration
fun = @(u)objfun(u,N); % cost function using Lagrange term
mayer = @(u)mayer_term(u,x0,T,N,xT); % cost function using Mayer term 
nonlcon_mayer = @(u)confun(u,xT,T,R,N,sigma,x_constr,x0,false); % nonlinear constraints function for Mayer term
nonlcon_lagr = @(u)confun(u,xT,T,R,N,sigma,x_constr,x0,true);  % nonlinear constraints function for Lagrange term
options = optimoptions('fmincon','Algorithm','sqp','MaxFunctionEvaluations',1000000, 'MaxIterations', 10000000);
%% search until both term converge to local minimum.
% First of all we provide random control inputs (regarding defined limits) 
% to the cost function with Mayer term and try to bring the cost to zero.
% It is important to note that when we use only the Mayer term we do not
% use nonlinear equality constraints. Otherwise it tries to find solution 
% which lies on equality boundary (line).
% Then we take control inputs provided by the cost function with Mayer term
% and give them as an initial control inputs to the cost function with
% Lagrange term. It should provide more feasible values than random values.
% We continue the process until both functions find local minimum that satisfies the constraints.
flag_mayer = false;
flag_lagrange = false;
while flag_mayer~=1 || flag_lagrange~=1
    u1 = u1min + rand(1,N)*(u1max - u1min);
    u2 = u2min + rand(1,N)*(u2max - u2min);
    u0 = [u1'; u2'];
    [u_mayer,fval_mayer,flag_mayer,output_mayer] = fmincon(mayer,u0,A,b,Aeq,beq,lb,ub,nonlcon_mayer,options);
    [u_lagrange,fval_lagrange,flag_lagrange,output_lagrange] = fmincon(fun,u_mayer,A,b,Aeq,beq,lb,ub,nonlcon_lagr,options);
end

%% prepare data for vizualization
t = 0:T/N:T;
% reshape a vector 20x1 -> u_lagrange = [u1(1) u1(2) ... u1(10) u2(1) u2(2) ... u2(10)]
% to a matrix 10x2 -> u = [u1(1) u1(2) ... u1(10); u2(1) u2(2) ... u2(10)]
u_lag = reshape(u_lagrange,[N, 2]);
u_may = reshape(u_mayer,[N, 2]);
% calculate all states using control inputs u
x_lag = disc_dynamics(u_lag,x0,T,N);
x_may = disc_dynamics(u_may,x0,T,N);
%% plot control input variables provided by the cost function with Mayer term  
figure(1)
plot(t(1:end-1), u_may(:,1), 'Color', 'r', 'LineWidth',2)
hold on 
plot(t(1:end-1), u_may(:,2), 'Color', 'g', 'LineWidth',2)
hold off
grid on
axis([0 t(end) -0.7 0.7]);
legend({'u1','u2'},'Location','southeast')
xlabel('time, sec'); ylabel('Control inputs')
title('Cost function using Mayer term')

%% plot state variables for the cost function with Mayer term 
figure(2)
plot(t, x_may(:,1), 'Color', 'r', 'LineWidth',2)
hold on 
plot(t, x_may(:,2), 'Color', 'g', 'LineWidth',2)
hold on 
plot(t, x_may(:,3), 'Color', 'b', 'LineWidth',2)
hold off
grid on
legend({'x1','x2','x3'},'Location','southeast')
xlabel('time, sec'); ylabel('States')
title('Cost function using only Mayer term')

%% animation of the whole motion for the cost function with Mayer term 
figure(3)
vizualization(x_may,R,sigma,N,x_constr,xT);
title('Cost function using only Mayer term')

%% plot control input variables for the case when we used 
% control inputs given by Mayer cost function as an initial control input 
% for Lagrange cost function 
figure(4)
plot(t(1:end-1), u_lag(:,1), 'Color', 'r', 'LineWidth',2)
hold on 
plot(t(1:end-1), u_lag(:,2), 'Color', 'g', 'LineWidth',2)
hold off
grid on
axis([0 t(end) -0.7 0.7]);
legend({'u1','u2'},'Location','southeast')
xlabel('time, sec'); ylabel('Control inputs')
title('Cost function using Lagrange term')

%% plot state variables for the case when we used 
% control inputs given by Mayer cost function as an initial control input 
% for Lagrange cost function 
figure(5)
plot(t, x_lag(:,1), 'Color', 'r', 'LineWidth',2)
hold on 
plot(t, x_lag(:,2), 'Color', 'g', 'LineWidth',2)
hold on 
plot(t, x_lag(:,3), 'Color', 'b', 'LineWidth',2)
hold off
grid on
legend({'x1','x2','x3'},'Location','southeast')
xlabel('time, sec'); ylabel('States')
title('Cost function using Lagrange term')
%% animation of the whole motion for the case when we used 
% control inputs given by Mayer cost function as an initial control input 
% for Lagrange cost function 
figure(6)
vizualization(x_lag,R,sigma,N,x_constr,xT);
title('Cost function using Lagrange term')