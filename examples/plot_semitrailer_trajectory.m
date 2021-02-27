% Import our packaget Marsvin Tech library
clear;clc;
import mt.*                 % Import our package
%% Semitrailer Parameter
% m1                : Mass of Unit 1. Unit: [Kg]
% m2                : Mass of Unit 2. Unit:[Kg]
% J1                : Inertia around z-axis of Unit 1. Unit: [Kg.m^2]
% J2                : Inertia around z-axis of Unit 2. Unit: [Kg.m^2]
% l11               : Distance from CoG of Unit 1 to Tyre #1. Unit: [m]
% l12               : Distance from CoG of Unit 1 to Tyre #2. Unit: [m]
% l13               : Distance from CoG of Unit 1 to Tyre #3. Unit: [m]
% l1r               : Distance from Tyre #3 to Joint Connection Point. Unit: [m]
% l21               : Distance from CoG of Unit 2 to Tyre #1. Unit: [m]
% l22               : Distance from CoG of Unit 2 to Tyre #2. Unit: [m]
% l23               : Distance from CoG of Unit 2 to Tyre #3. Unit: [m]
% l2r               : Distance from Tyre #1 to Joint Connection Point. Unit: [m]
% l1c1              : Distance from CoG of Uni 1 to Joint Connection Point. Unit: [m]
% l2c1              : Distance from CoG of Uni 2 to Joint Connection Point. Unit: [m]
Cy11 = 35e4;
Cy12 = 20e4;
Cy13 = 20e4;
Cy21 = 13e4;
Cy22 = 13e4;
Cy23 = 13e4;
m1  = 9841;
m2  = 9900;
J1  = 3.7396*10^3;
J2  = 1.1119*10^5;
l11  = 1.4540;
l12  = 3.0-l11;
l13  = 4.37-l11;
l1c1 = 3.4-l11;
l2c1 = 6.55+1.2825;
l21 = 1.2825;
l22 = 1.31 - l21;
l23 = 2.62 - l21;
%% 
params = [Cy11 Cy12 Cy13 Cy21 Cy22 Cy23 l11 l12 l13 l21 l22 l23 l1c1 l2c1 m1 m2 J1 J2]'; 
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