function x = disc_dynamics(u,x0,T,N)
%--------------------------------------------------------------------------
% u = [u1(1) u1(2) ... u1(10); u2(1) u2(2) ... u2(10)] control inputs
% x0 - initial state [px py theta] or [x1 x2 x3]
% T - motion time (sec)
% N - number of samples
%--------------------------------------------------------------------------

% create a state matrix (N+1)x3 for N states [x1 x2 x3] and plus the inital
% state
x = zeros(N+1,3);
% the first row contains initial values
x(1,:) = x0;
% sampling time (sec)
dt = T/N;
% here we calculate all states:
% x1(k+1) = x1(k) + dt*u1(k)*cos(x3(k))
% x2(k+1) = x2(k) + dt*u1(k)*sin(x3(k))
% x3(k+1) = x3(k) + u1(k)*u2(k)
for ind = 2:N+1
    x(ind,1) = x(ind-1,1) + dt*u(ind-1,1)*cos(x(ind-1,3));
    x(ind,2) = x(ind-1,2) + dt*u(ind-1,1)*sin(x(ind-1,3));
    x(ind,3) = x(ind-1,3) + u(ind-1,1)*u(ind-1,2);
end
end