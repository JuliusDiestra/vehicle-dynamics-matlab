function X_next = rk4(f,X,U,Ts)
% RK4 Next discrete step calculation using Runge-Kutta 4.
%   RK4(f,X,U,Ts)   : Next discrete step calculation using Runge-Kutta 4.
%
%   Inputs:
%       f           : State-space equation dX = f(X,U) 
%       X           : Current state Value.
%       U           : Current input Value
%       Ts          : Sampling time.
%
%   Output:
%       X_next      : Next state value. t_next = t + Ts
%   Author : Julius D.
    k1 =f(X,U);
    k2 = f(X+Ts/2*k1,U);
    k3 = f(X+Ts/2*k2,U);
    k4 = f(X+Ts*k3,U);
    X_next = X + Ts/6*(k1+2*k2+2*k3+k4);
end

