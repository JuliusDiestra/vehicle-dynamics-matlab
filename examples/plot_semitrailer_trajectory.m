% Import our packaget Marsvin Tech library
clear;clc;
close all;
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
syms vy1 d_psi1 d_deltaPsi1 psi1 deltaPsi1 delta11 real
X = [vy1 d_psi1 d_deltaPsi1 psi1 deltaPsi1]';
U = delta11;
vx1 = 15;
%% State-space equation
% Define 
f = @(X,delta11) mt.ss.semitrailerNonlinearVxConstant(X,delta11,vx1,params);
%% Loop to plot
Ts = 0.1;
% Unit 1 - Dimensions
lf = l11 + 0.5;
lr = l13 + 0.4;
w = 3;
param_unit_one = [l11 l13 w lf lr];
% Unit 2 - Dimensions
lf = l2c1;
lr = l23 + 0.4;
w = 3;
param_unit_two = [l2c1 l23 w lf lr];
%####################
% Initial conditions
%####################
X_current = [0,0,0,0,0]';
r1 = [0,0]';
r2 = [-l1c1-l2c1,0]';
psi1 = 0;
psi2 = 0;
deltaPsi1 = 0;
trajectory_one = [];
trajectory_two = [];
trajectory_one = [trajectory_one r1];
trajectory_two = [trajectory_two r2];
delta_c = 15*pi/180;
delta_v = zeros(1,120);
figure(1)
set(gcf, 'Position',  [50, 600, 1800, 500])
%set(gcf, 'Position',  [50, 300, 900, 900])
hold on
xlabel('x-axis [m]','FontSize',14)
ylabel('y-axis [m]','FontSize',14)
title('Semitrailer Position (Unloaded) : Constant longitudinal velocity','FontSize',18)
xlim([-15 145])
ylim([-10 10])
% xlim([-15 105])
% ylim([-60 60])
h1 = mt.tools.plotCar(r1,psi1,param_unit_one,'red');
h2 = mt.tools.plotCar(r2,psi2,param_unit_two,'blue');
figure(2)
hold on;
set(gcf, 'Position',  [50, 10, 1800, 500])
%set(gcf, 'Position',  [950, 300, 900, 900])
title('Steering','FontSize',14)
xlabel('time [s]','FontSize',14)
ylabel('Steering [rad]','FontSize',14)
for j = 1:100
    % Calculate psi2
    psi2 = psi1 - deltaPsi1;
    % Calculate r2
    r2 = r1 + mt.tools.Rz(psi1,2)*[-l1c1;0] - mt.tools.Rz(psi2,2)*[l2c1;0];
    time = (j-1)*Ts;
    if time < 1
        delta_v(j) = 0;
    elseif time < 3
        delta_v(j) = delta_c*sin(2*pi*0.5*j*Ts - pi);
    else
        delta_v(j) = 0;
    end
    % Plot car
    figure(1)
    delete(h1)
    delete(h2)
    % hold off;
    h1 = mt.tools.plotCar(r1,psi1,param_unit_one,'red');
    % pause(1)
    h2 = mt.tools.plotCar(r2,psi2,param_unit_two,'blue');
    % pause(1)
    trajectory_one = [trajectory_one r1];
    trajectory_two = [trajectory_two r2];
    plot(trajectory_one(1,:),trajectory_one(2,:),'Color','red');
    plot(trajectory_two(1,:),trajectory_two(2,:),'Color','blue');
    hold on;
    % Plot steering
    figure(2)
    plot(time,delta_v(j),'*')
    % Calculate next state
    X_next = mt.tools.rk4(f,X_current,delta_v(j),Ts);
    dr1 = mt.tools.Rz(psi1,2)*[vx1;X_next(1)];
    d_psi1 = X_next(2);
    d_deltaPsi1 = X_next(3);
    r1 = dr1*Ts + r1;
    psi1 = d_psi1*Ts + psi1;
    deltaPsi1 = d_deltaPsi1*Ts + deltaPsi1;
    X_current = X_next;
    pause(0.2)
end
figure(2)
hold off