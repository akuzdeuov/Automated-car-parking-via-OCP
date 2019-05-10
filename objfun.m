function fun = objfun(ui,N)
%--------------------------------------------------------------------------
% ui - control inputs [u1(1) u1(2) ... u1(10) u2(1) u2(2) ... u2(10)]
% N - number of samples
%--------------------------------------------------------------------------

% reshape vector 20x1 -> ui = [u1(1) u1(2) ... u1(10) u2(1) u2(2) ... u2(10)]
% to a matrix 10x2 -> u = [u1(1) u1(2) ... u1(10); u2(1) u2(2) ... u2(10)]
u = reshape(ui,[N,2]);
% initial value of the cost function
fun = 0;
% here we calculate the cost fucntion -> f = u1(1)^2 + u2(1)^2 + u1(N)^2 + u2(N)^2 
for ind = 1:N
    fun = fun + u(ind,1)^2 + u(ind,2)^2;
end

end