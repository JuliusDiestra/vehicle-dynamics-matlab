%##########################################################################
%                      		MARSVIN-TECH
%##########################################################################
%##########################################################################
% Project: Car modelling
% Description : Car model using single axle bicycle model using Euler-Lagrange equation.
% Documentation: car_modelling.tex / car_modelling.pdf
% Author : Julius Diestra
%##########################################################################
clear all;clc;
%##############################
% States
%##############################
%##########################################################################
% r = [x;y]             : Position of CoG of car w.r.t Frame {E}. Unit: [m]   
% d_r = [d_x;d_y]       : Velocity vector of CoG of car w.r.t. Frame {E}. Unit: [m/s]
% dd_r = [dd_x;dd_y]    : Acceleration vector of CoG of car w.r.t. Frame {E}. Unit: [m/s]
% psi                   : Inclination of Unit 1 w.r.t x-axis of Frame {E}. Unit: [rad]
% [vx;vy]               : Velocity vector of CoG of car w.r.t. Frame {V}. Unit: [m/s]
% [d_vx;d_vy]           : Acceleration vector of CoG of car w.r.t. Frame {V}. Unit: [m/s^2]
%##########################################################################
syms x y psi delta real
syms d_x d_y d_psi d_delta real
syms dd_x dd_y dd_psi dd_delta real
syms vx vy d_vx d_vy real
%##############################
% Inputs
%##############################
%##########################################################################
% delta                 : Wheel steer angle of car. Unit: [rad]
% Fp2                   : Propulsion force on axle 2. Unit: [N]
% d_delta               : Wheel steer angle rate of car. Unit: [rad]
% d_Fp2                 : Propulsion force rate on axle 2. Unit: [N/s]
%##########################################################################
syms delta Fp2 Fb1 Fb2 d_delta d_Fp2 Fxw2 real
%##############################
% Parameters
%##############################
%##########################################################################
% l1                    : Distance between front axle and centre of gravity. Unit: [m]
% l2                    : Distance between rear axle and centre of gravity. Unit: [m]
% Cy1                   : Cornering stiffness axle 1. Unit: [N/rad]
% Cy2                   : Cornering stiffness axle 2. Unit: [N/rad]
% m                     : Vehicle mass. Unit: [Kg]
% J                     : Inertia around z-axis of vehicle Unit. Unit: [Kg.m^2]
% wheel_base            : Distance between axle 1 and axle 2. Unit: [m]
% width                 : Width of the car. Unit: [m]
% theta_r               : Road inclination. Unit: [rad]
%##########################################################################
syms l1 l2 Cy1 Cy2 m J wheel_base width rho_air Av Cd g theta_r real
%##############################
% Generalized coordinates
%##############################
q = [x y psi]';             % Eq. (12)
dq = [d_x d_y d_psi]';      % Eq. (12)
%##############################
% Rotation matrices:
%##############################
% Frame {E} : Axis system fixed in the earth frame or reference frame.
% Frame {V} : Axis system fixed in the center of gravity (CoG) of the car.
% Frame {W} : Axis system fixed in the front wheel.
% Frame {F} : Axis system fixed in the front axle (axle 1).
% Frame {R} : Axis system fixed in the rear axle (axle 2).
% R_EV      : Rotation matrix for Frame {V} w.r.t. Frame {E}
% R_EW      : Rotation matrix for Frame {W} w.r.t. Frame {E}
Rz = @(angle) [cos(angle) -sin(angle);
        sin(angle) cos(angle)];
R_EV = Rz(psi);         % Eq. (1)
R_EW = Rz(psi+delta);   % Eq. (2)
R_ER = R_EV;
R_EF = R_EV;
%############################################################
% Declaring Postion Vectors for the Tyres in Fixed Frame of the vehicle
% i.e. The vector position of the tyres in Unit 1 are w.r.t. Fixed Frame of
% Unit 1. For Unit 2, w.r.t. Fixed Frame of Unit 2.
% NOTE: Change this if the CoG CHANGE !!!!
%############################################################
% r_V1 : Position of front axle in Frame {V}
% r_V2 : Position of rear axle in Frame {V}
r_V1 = [l1;0];  % Eq. (17)
r_V2 = [-l2;0]; % Eq. (18)
%############################################################
% Position and Velocities of Vehicle CoG
%############################################################
% Position of CoG of Unit 1 and 2 w.r.t. Inertial Frame {E}
r = [x y]';
% Velocity of CoG of Unit 1 and 2 w.r.t. Inertial Frame {E}
% Note: Chain rule was used to obtain the expression:
%       d_r1 = (dr1/dt) = (dr1/dq)*(dq/dt)
%       d_r2 = (dr2/dt) = (dr2/dq)*(dq/dt)
d_r = jacobian(r,q)*dq;     % Eq. (14)
%############################################################
% Position and Velocities of Tyres w.r.t Inertial and Fixed Frame
%############################################################
% r1 : Position of Unit 1 Axle 1 w.r.t. Frame {E}
% r2 : Position of Unit 1 Axle 2 w.r.t Frame {E}
% v1 : Translational velocity vector of vehicle unit 1 axle 1. Unit: [m/s]
% v2 : Translational velocity vector of vehicle unit 1 axle 2. Unit: [m/s]
r1 = r + R_EV*r_V1;     % Eq. (19)
r2 = r + R_EV*r_V2;     % Eq. (20)
v1 = jacobian(r1,q)*dq; % Eq. (25)
v2 = jacobian(r2,q)*dq; % Eq. (26)
%##############################
%  Kinetic Energy Calculation
%##############################
T = 1/2 *m*(d_r')*d_r + 1/2*J*d_psi^2;  % Eq. (13)
%##############################
% Lagrange Equation
%##############################
T_q = jacobian(T,q)';
T_dq = jacobian(T,dq)';
%##############################
% External Forces
%##############################
% BODY :
% F_air_1           : Aerodynamic drag in a body fixed frame. Unit: [N]
% F_grav_1          : Gravitational force of vehicle Unit 1. Unit: [N]
% [Fxv1,Fyv1]       : Body forces of vehicle Unit 1 in a body fixed frame (Frame{1}). Unit: [N]
% [Fxv2,Fyv2]       : Body forces of vehicle Unit 2 in a body fixed frame (Frame{2}). Unit: [N]
% F1                : Body force vector of vehicle Unit 1. Unit: [N]
% F2                : Body force vector of vehicle Unit 2. Unit: [N]
%-----------
F_air_1 = -0.5*rho_air*Av*Cd*vx^2;
F_grav_1 = -m*g*sin(theta_r);
Fxv1 = F_air_1 + F_grav_1;
Fyv1 = 0;
Fyv2 = 0;
F = R_EV*[Fxv1;Fyv1];
%-----------
% TYRES : This forces are in function of "vx1" and "vy1" instead of "d_x1"
% and "d_y1".
% [vxw1,vyw1]     : Translational velocities of vehicle front axle in a wheel fixed frame. Unit: [m/s]
% [vxw2,vyw2]     : Translational velocities of vehicle rear axle in a wheel fixed frame. Unit: [m/s]
%-------------
%######################
Vw1=simplify(R_EW'*v1);
Vw2=simplify(R_EV'*v2);
%######################
vxw1 = Vw1(1);
vyw1 = Vw1(2);
vxw2 = Vw2(1);
vyw2 = Vw2(2);
% [Fxw1,Fyw1]       : Tyre forces of Unit 1 Axle 1 in Wheel Fixed Frame (Frame {H}). Unit: [N]
% [Fxw2,Fyw2]    	: Tyre forces of Unit 1 Axle 2 in Wheel Fixed Frame (Frame {1}). Unit: [N]
% Fb1               : Brake force vehicle Unit 1 Axle 1 in a wheel fixed frame. Unit: [N]
% Fb2               : Brake force vehicle Unit 1 Axle 2 in a wheel fixed frame. Unit: [N]
% Sy1,Sy2           : Lateral tyre slip of vehicel unit 1 axle 1 and 2.
% Cy1,Cy2           : Cornering stiffness of vehicel axle 1 and 2 . Unit: [N/rad]
%------------
% Note: Rolling resistances IS NOT INCLUDED HERE !!
% Fzv11,Fzv12,Fzv13 : Vertical force of vehicle unit 1 axle 1,2,3 in a body fixed frame. Unit: [N]  
% Fzv21,Fzv22,Fzv23 : Vertical force of vehicle unit 2 axle 1,2,3 in a body fixed frame. Unit: [N] 
% Cr                : Rolling resistance coefficient.
% Lateral Tyre Slips
Sy1 = vyw1/vxw1;
Sy2 = vyw2/vxw2;
% Wheel 1
Fxw1 = 0;
Fyw1 = - Cy1*Sy1;
% Wheel 2
Fyw2 = - Cy2*Sy2;
% F1                : Tyre force vector of vehicle Axle 1. Unit: [N]
% F2                : Tyre force vector of vehicle Axle 2. Unit: [N]
F1 = R_EW*[Fxw1;Fyw1];
F2 = R_EV*[Fxw2;Fyw2];
% Generalized forces calculation
Q = (F1')*jacobian(r1,q) + (F2')*jacobian(r2,q) + (F')*jacobian(r,q);   % Eq. (15)
Q = simplify(Q');
%##############################
% Dynamics of the system
%##############################
% ddq = [dd_x;dd_y;dd_psi]
ddq = inv(jacobian(T_dq,dq))*(-jacobian(T_dq,q)*dq +T_q+Q);             % Eq. (11)
%% Changing ddq -> ddq_v
%   Remember that: 
%       [dx1 dy1]' = R_v*[u v]'
%       [ddx1 ddy1]' = dotR_v*[u v] + R_v*[du dv]'
% ddq_v : Represents ddq. We express all d_x1 and d_y1 in function of vx1,
% vy1 and psi1 using: [d_x1 d_y1]' = R_v1*[vx1 vy1]'
ddq_v = subs(ddq,[d_x d_y],[vx*cos(psi)-vy*sin(psi), vy*cos(psi)+vx*sin(psi)]);
% Derivation in function of time of rotation matrix R_E1
dotR_EV =  [-sin(psi)*d_psi,-cos(psi)*d_psi;
        cos(psi)*d_psi,-sin(psi)*d_psi];
% Converting [dd_x1 dd_y1]' to [d_u d_v]' : 
ddq_v(1:2) = R_EV'*(ddq_v(1:2) - dotR_EV*[vx;vy]);
%% Simplifications on model
theta_r_value = 0;          % No inclination in the road.
rho_air_value = 0;          % No considering aerodynamics.
Fb1_value = 0;              % No breaking in the first axle.
Fb2_value = 0;              % We will consider propulsion and breaking in Fp2;
ddq_v = simplify(subs(ddq_v,[theta_r,rho_air,Fb1,Fb2],[theta_r_value,rho_air_value,Fb1_value,Fb2_value]));
dotX = ddq_v;
%% Assuming constant velocity
% Two main changes in the model :
Fxw2 = 0;                                   % Longitudinal velocity it is "forced" to be zero
F1 = R_EV*[Fxw1;Fyw1*cos(delta)];           % Force applied on front wheel. The longitudinal force generated by steering is neglected.
Q_vx = (F1')*jacobian(r1,q) + (F2')*jacobian(r2,q) + (F')*jacobian(r,q);
Q_vx = simplify(Q_vx');
%##############################
% Dynamics of the system
%##############################
% ddq = [dd_x;dd_y;dd_psi]
ddq_vx = inv(jacobian(T_dq,dq))*(-jacobian(T_dq,q)*dq +T_q+Q_vx);
ddq_v_vx = subs(ddq_vx,[d_x d_y],[vx*cos(psi)-vy*sin(psi), vy*cos(psi)+vx*sin(psi)]);
% Derivation in function of time of rotation matrix R_E1
dotR_EV =  [-sin(psi)*d_psi,-cos(psi)*d_psi;
        cos(psi)*d_psi,-sin(psi)*d_psi];
% Converting [dd_x1 dd_y1]' to [d_u d_v]' : 
ddq_v_vx(1:2) = R_EV'*(ddq_v_vx(1:2) - dotR_EV*[vx;vy]);
dotX_vx_constant = simplify(ddq_v_vx(2:3));
%% Linearized
syms vx vy d_psi delta Fxw2 real        % Define symbolics again just in case.
X = [vx vy d_psi]';
U = [delta Fxw2]';
X_vx = [vy d_psi]';
U_vx = delta;
A = simplify(subs(jacobian(dotX,X),[X' U'],[vx 0 0 0 0]));
B = simplify(subs(jacobian(dotX,U),[X' U'],[vx 0 0 0 0]));
dotX_lin = simplify(A*X + B*U);
A_vx = simplify(subs(jacobian(dotX_vx_constant,X_vx),[X_vx' U_vx'],[0 0 0]));
B_vx = simplify(subs(jacobian(dotX_vx_constant,U_vx),[X_vx' U_vx'],[0 0 0]));
dotX_vx_constant_lin = simplify(A_vx*X_vx + B_vx*U_vx);
%% Generating function files with results.
% Note : After generating, some changes were done like: "help comment" and
% change of name of inputs. I changed from in1,in2,in3 to X,U,params.
fprintf('########################### \n');
fprintf('Generating code \n')
fprintf('########################### \n');
% Nonlinear model files
matlabFunction(dotX,'File','carNonlinear','Vars',{[vx vy d_psi]',[delta Fxw2]',[Cy1 Cy2 l1 l2 m J]'},'Output',{'dotX'});
matlabFunction(dotX_vx_constant,'File','carNonlinearVxConstant','Vars',{[vy d_psi]',delta,vx,[Cy1 Cy2 l1 l2 m J]'},'Output',{'dotX'});
% Linear model files
matlabFunction(dotX_lin,A,B,'File','carLinear','Vars',{[vx vy d_psi]',[delta Fxw2]',[Cy1 Cy2 l1 l2 m J]'},'Output',{'dotX','A','B'});
matlabFunction(dotX_vx_constant_lin,A_vx,B_vx,'File','carLinearVxConstant','Vars',{[vy d_psi]',delta,vx,[Cy1 Cy2 l1 l2 m J]'},'Output',{'dotX','A','B'});

