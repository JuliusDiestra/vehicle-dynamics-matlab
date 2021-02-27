% Import our packaget Marsvin Tech library
clear;clc;
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
params = [Cy1 Cy2 l1 l2 m J]'; 
syms vy d_psi delta real
X = [vy d_psi]';
U = delta;
vx = 12;
%%
% Define 
f = @(X,delta) mt.ss.carNonlinearVxConstant(X,delta,vx,params);
% Loop to plot
Ts = 0.1;
% param
l1 = 1.5;
l2 = 1.5;
lf = 2;
lr = 2;
w = 2;
param = [l1 l2 w lf lr];
% constant steering 
% delta_v = 35*pi/180;
% Initial conditions
X_current = [0,0]';
r = [0 0]';
psi = 0;
trajectory = [];
delta_c = 35*pi/180;
delta_v = zeros(1,120);
for j = 1:120
    time = (j-1)*Ts;
    if time < 4.5
        delta_v(j) = delta_c*sin(2*pi*0.5*j*Ts);
    else
        delta_v(j) = delta_c;
    end
    % Plot car
    figure(1)
    hold off;
    mt.tools.plotCar(r,psi,param);
    hold on;
    trajectory = [trajectory r];
    plot(trajectory(1,:),trajectory(2,:));
    xlim([-5 55])
    ylim([-5 55])
    xlabel('x-axis [m]','FontSize',14)
    ylabel('y-axis [m]','FontSize',14)
    title('Car Position : Constant longitudinal velocity','FontSize',14)
    figure(2)
    plot(time,delta_v(j),'*')
    title('Steering','FontSize',14)
    xlabel('time [s]','FontSize',14)
    ylabel('Steering [rad]','FontSize',14)
    hold on
    % Calculate next state
    X_next = mt.tools.rk4(f,X_current,delta_v(j),Ts);
    dr = mt.tools.Rz(psi,2)*[vx;X_next(1)];
    d_psi = X_next(2);
    r = dr*Ts + r;
    psi = d_psi*Ts + psi; 
    X_current = X_next;
    pause(0.2)
end
figure(2)
hold off