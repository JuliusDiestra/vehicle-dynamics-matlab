function df_dt = dfdt(f,q,dqdt)
% DFDT Estimation of df/dt while 'f' depends on generalized coordinates 'q'. Chain rule is applied
%   df_dt = DFDT(q)     : Estimation is done by using chain rule df/dt = (df/dq)(dq/dt) 
%
%   Inputs:
%       f           : Function express as a function of 'q'. Data type : sym
%       q           : Vertical symbolic vector. Data type : sym
%       dqdt        : Vertical symbolic vector of the derivative expression of 'q' vector. 
%   Output:
%       df_dt       : Vertical symbolic vector of the derivative of 'f' applying chain rule. df_dt is expressed as a function of 'q' and 'dqdt', not time.
%   Author : Julius D.
    df_dt = jacobian(f,q)*dqdt;
end