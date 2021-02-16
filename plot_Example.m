
% Import our packaget Marsvin Tech library
import mt.*                 % Import our package

%% Vehicle Parameter
m = 2200;   	% The vehicle weight, Gross vehicle mass. Unit: [kg].
l = 3;          % The vehicle wheelbase. Unit: [m] 
l1 = 1.2;       % Distance between front axle and centre of gravity. Unit: [m] 
l2 = l - l1;    % Distance between rear axle and centre of gravity. Unit: [m]
J = 4300;       % Vehicle inertia around z-axis. Unit: [Nm/s2] 
Cy1 = 150540;   % The vehicle front cornering stiffness. Unit: [N/rad]
Cy2 = 122380;   % The vehicle rear cornering stiffness. Unit: [N/rad]
%% 
params = [Cy1 Cy2 l1 l2 m J]; 
syms vy d_psi delta
X = [vy d_psi];
U = delta;
vx = 12;
dX = mt.ss.carNonlinearVxConstant(X,U,vx,params);

f = @ (X,U) dX;
