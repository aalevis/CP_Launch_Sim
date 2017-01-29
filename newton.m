function [eta] = newton(f,df,eta)

% Newton iteration to solve for eta
% INPUTS
% f = equation - burnout velocity
% df = derivative of f
% eta = guess the value of eta
% OUTPUT
% eta = actual value of eta

ratio = 1;
error = 1e-8;
count = 1;

while abs(ratio) > error && count < 5000
   count = count + 1;
   func = eval(f);
   dfunc = eval(df);
   ratio = func/dfunc;
   eta = eta - ratio;
end



end

