function mayer = mayer_term(ui,x0,T,N,xT)
%--------------------------------------------------------------------------
% ui - control inputs [u1(1) u1(2) ... u1(10) u2(1) u2(2) ... u2(10)]
% x0 - initial state [px py theta] or [x1 x2 x3]
% T - motion time (sec)
% N - number of samples
% xT - the final state [px py theta] or [x1 x2 x3]
%--------------------------------------------------------------------------

% reshape a vector 20x1 -> ui = [u1(1) u1(2) ... u1(10) u2(1) u2(2) ... u2(10)]
% to a matrix 10x2 -> u = [u1(1) u1(2) ... u1(10); u2(1) u2(2) ... u2(10)]
u = reshape(ui,[N,2]);
% here we obtain states for all N states
x = disc_dynamics(u,x0,T,N);
% calculate the cost function
mayer = (x(N+1,1) - xT(1))^2 + (x(N+1,2) - xT(2))^2 + (x(N+1,3) - xT(3))^2;
end