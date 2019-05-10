function [c, ceq] = confun(ui,xT,T,R,N,sigma,x_constr,x0,add_eq)
%--------------------------------------------------------------------------
% ui - control inputs [u1(1) u1(2) ... u1(10) u2(1) u2(2) ... u2(10)]
% xT - the final state [px py theta] or [x1 x2 x3]
% T - motion time (sec)
% R - radius of the obstacles and circles on the car (meter)
% N - number of samples
% sigma - distance between centers of the circles on the car (meter)
% x_constr - coordinates of the two obstacles [x1 y1; x2 y2] (meter)
% x0 - initial state [px py theta] or [x1 x2 x3]
% add_eq - add nonlinear equality constraints (true/false)
%--------------------------------------------------------------------------

Rt = 2*R;
% reshape a vector 20x1 -> ui = [u1(1) u1(2) ... u1(10) u2(1) u2(2) ... u2(10)]
% to a matrix 10x2 -> u = [u1(1) u1(2) ... u1(10); u2(1) u2(2) ... u2(10)]
u = reshape(ui,[N,2]);
% here we obtain states for all N states
x = disc_dynamics(u,x0,T,N);
% number of rows of a nonlinear inequality constraint vector.
% each circle on the car must not collide with two obstacles, we have 
% 3 circles on the car and 2 obstacles and N states, therefore:
c_r = 6*N;
c = zeros(c_r,1);
% add counter to fill c vector
count = 1;
for ind = 2:N+1
    % -(x1 - sigma*cos(x3) - x_obs1)^2 - (y1 - sigma*sin(x3) - y_obs1)^2 +
    % Rt^2 <= 0
    c(count,1) = -(x(ind,1) - sigma*cos(x(ind,3)) - x_constr(1,1))^2 - (x(ind,2) - sigma*sin(x(ind,3)) - x_constr(1,2))^2 + Rt^2;
    count = count + 1;
    % -(x1 - sigma*cos(x3) - x_obs2)^2 - (y1 - sigma*sin(x3) - y_obs2)^2 +
    % Rt^2 <= 0
    c(count,1) = -(x(ind,1) - sigma*cos(x(ind,3)) - x_constr(2,1))^2 - (x(ind,2) - sigma*sin(x(ind,3)) - x_constr(2,2))^2 + Rt^2;
    count = count + 1;
    % -(x1 - x_obs1)^2 - (y1 - y_obs1)^2 + Rt^2 <= 0
    c(count,1) = -(x(ind,1) - x_constr(1,1))^2 - (x(ind,2)- x_constr(1,2))^2 + Rt^2;
    count = count + 1;
    % -(x1 - x_obs2)^2 - (y1 - y_obs2)^2 + Rt^2 <= 0
    c(count,1) = -(x(ind,1) - x_constr(2,1))^2 - (x(ind,2) - x_constr(2,2))^2 + Rt^2;
    count = count + 1;
    % -(x1 + sigma*cos(x3) - x_obs1)^2 - (y1 + sigma*sin(x3) - y_obs1)^2 +
    % Rt^2 <= 0
    c(count,1) = -(x(ind,1) + sigma*cos(x(ind,3)) - x_constr(1,1))^2 - (x(ind,2) + sigma*sin(x(ind,3)) - x_constr(1,2))^2 + Rt^2;
    count = count + 1;
    % -(x1 + sigma*cos(x3) - x_obs2)^2 - (y1 + sigma*sin(x3) - y_obs2)^2 +
    % Rt^2 <= 0
    c(count,1) = -(x(ind,1) + sigma*cos(x(ind,3)) - x_constr(2,1))^2 - (x(ind,2) + sigma*sin(x(ind,3)) - x_constr(2,2))^2 + Rt^2;
    count = count + 1;
end

% nonlinear equality constraint for the final state
if add_eq
    ceq = x(N+1,:) - xT;
else
    ceq = [];
end

end