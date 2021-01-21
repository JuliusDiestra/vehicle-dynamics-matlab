%##########################################################################
%                      	SEMI-TRAILER MODELING
%##########################################################################
%##########################################################################
% Description : Car model using single axle bicycle model using
% Euler-Lagrange equation.
%##########################################################################
clear all;clc;
import mt.*             % Import our package
%% Code starts !
fprintf('########################### \n');
fprintf('Script start ! \n')
fprintf('########################### \n');
%% Declaration of all the symbolics:
%#########################################
%                   Symbolics
%#########################################
%#############################
% STATES AND ITS DERIVATIVES
%############################
% p1 = [x1,y1]'     : Position of Unit 1 w.r.t Frame {E}. Unit: [m]   
% p2 = [x2,y2]'     : Position of Unit 2 w.r.t Frame {E}. Unit: [m]
% psi1              : Inclination of Unit 1 w.r.t x-axis of Frame {E}. Unit: [rad]
% psi2              : Inclination of Unit 2 w.r.t x-axis of Frame {E}. Unit: [rad]
% deltaPsi1         : Angle between Unit 1 and Unit 2. Unit: [rad]
% d_x1,d_y1,d_x2,d_y2                   : Velocities in Frame {E}. Unit: [m/s]
% d_p1 = [d_x1,d_y1]'                   : Vector velocity of Unit 1 w.r.t. Frame {E}
% d_p2 = [d_x2,d_y2]'                   : Vector velocity of Unit 2 w.r.t. Frame {E}
% d_psi1,d_psi2,d_deltaPsi1             : Angular Velocities. Unit: [rad/s] 
% dd_x1,dd_y1,dd_x2,dd_y2               : Accelerations in Frame {E}. Unit: [m/s^2]  
% dd_psi1,dd_psi2,dd_deltaPsi1          : Angular Accelerations. Unit: [rad/s^2]
% [vx1 vy1]'        : Velocity of Unit 1 w.r.t Frame {1}. Unit: [m/s]
% [d_vx1 d_vy1]'    : Acceleration of Unit 1 w.r.t Frame {1}. Unit: [m/s^2]
syms x1 y1 x2 y2 psi1 psi2 deltaPsi1 real
syms d_x1 d_y1 d_x2 d_y2 d_psi1 d_psi2 d_deltaPsi1 real
syms dd_x1 dd_y1 dd_x2 dd_y2 dd_psi1  dd_psi2 dd_deltaPsi1 real
syms vx1 vy1 d_vx1 d_vy1 real
%############
% INPUTS
%############
% delta11           : Steering angle. Unit: [rad]
% Fp12              : Propulsion force vehicle unit 1 axle 2 in a wheel fixed frame. Unit: [N]
% Fb11              : Brake force vehicle Unit 1 Axle 1 in a wheel fixed frame. Unit: [N]
% Fb12              : Brake force vehicle Unit 1 Axle 2 in a wheel fixed frame. Unit: [N]
% Fb13              : Brake force vehicle Unit 1 Axle 3 in a wheel fixed frame. Unit: [N]
% Fb21              : Brake force vehicle Unit 2 Axle 1 in a wheel fixed frame. Unit: [N]
% Fb22              : Brake force vehicle Unit 2 Axle 2 in a wheel fixed frame. Unit: [N]
% Fb23              : Brake force vehicle Unit 2 Axle 3 in a wheel fixed frame. Unit: [N]
% Sy11,Sy12,Sy13    : Lateral tyre slip of vehicel unit 1 axle 1,2 and 3.
syms delta11 Fp12 Fb11 Fb12 Fb13 Fb21 Fb22 Fb23 Fxw12 real
syms d_delta11 d_Fp12 real
%####################
% BASIC PARAMETERS
%####################
% m1                : Mass of Unit 1. Unit: [Kg]
% m2                : Mass of Unit 2. Unit:[Kg]
% J1                : Inertia around z-axis of Unit 1. Unit: [Kg.m^2]
% J2                : Inertia around z-axis of Unit 2. Unit: [Kg.m^2]
% Cy11              : Cornering stiffness of Tyre #1. Unit: [N/rad]
% Cy12              : Cornering stiffness of Tyre #2. Unit: [N/rad]
% Cy13              : Cornering stiffness of Tyre #3. Unit: [N/rad]
% Cy21              : Cornering stiffness of Tyre #4. Unit: [N/rad]
% Cy22              : Cornering stiffness of Tyre #5. Unit: [N/rad]
% Cy23              : Cornering stiffness of Tyre #6. Unit: [N/rad]
% Sy11              : Lateral tyre slip of Tyre #1. Unit: [non]
% Sy12              : Lateral tyre slip of Tyre #2. Unit: [non]
% Sy13              : Lateral tyre slip of Tyre #3. Unit: [non]
% Sy21              : Lateral tyre slip of Tyre #4. Unit: [non]
% Sy22              : Lateral tyre slip of Tyre #5. Unit: [non]
% Sy23              : Lateral tyre slip of Tyre #6. Unit: [non]
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
syms m1 m2 J1 J2 real
syms Cy11 Cy12 Cy13 Cy21 Cy22 Cy23 Sy11 Sy12 Sy13 Sy21 Sy22 Sy23 real
syms Cr1 Cr2 Cr3 Cr4 Cr5 Cr6 Fz1 Fz2 Fz3 Fz4 Fz5 Fz6 real
syms l11 l12 l13 l1r l21 l22 l23 l2r l1c1 l2c1 real
%###########################
% COMPLEMENTARY PARAMETERS
%###########################
% theta_R           : Road uphill slope at Unit 1 first axle. Unit: [rad]
% rho_air           : Air density. Unit: [Kg/m^3]
% Av                : Vehicle front cross-sectional area. Unit: [m^2]
% Cd                : Air drag coefficient. Unit: [Non]
% g                 : Gravitational acceleration. Unit: [m/s^2]
syms theta_r rho_air Av Cd g real
%####################################
% Position based on Road Coordinates
%####################################
syms psiR11 psiR23 dr11 ch11 dr23 ch23 sr11 sr23 real
%% 
%##############################
% Generalized coordinates
%##############################
q = [x1 y1 psi1 deltaPsi1]';
dq = [d_x1 d_y1 d_psi1 d_deltaPsi1]';
%##############################    
%   Inclination of Unit 2
%##############################    
psi2 = psi1 - deltaPsi1;
d_psi2 = d_psi1 - d_deltaPsi1;
dd_psi2 = dd_psi1 - dd_deltaPsi1;
%##############################
% Rotation matrices:
%##############################
% R_v1  : Rotation matrix for vehicle Unit 1 w.r.t. Inertial Frame {E}
% R_v2  : Rotation matrix for vehicle Unit 2 w.r.t. Inertial Frame {E}
% R_w11 : Rotation matrix for vehicle Unit 1 Axle 1 w.r.t. Inertial Frame {E}
R_v1 = [cos(psi1) -sin(psi1);
        sin(psi1) cos(psi1)];
R_v2 = [cos(psi2) -sin(psi2);
        sin(psi2) cos(psi2)];
R_w11 = [cos(psi1+delta11) -sin(psi1+delta11);
        sin(psi1+delta11) cos(psi1+delta11)];
%############################################################
% Declaring Postion Vectors for the Tyres in Fixed Frame
% i.e. The vector position of the tyres in Unit 1 are w.r.t. Fixed Frame of
% Unit 1. For Unit 2, w.r.t. Fixed Frame of Unit 2.
% NOTE: Change this if the CoG CHANGE !!!!
%############################################################
L11 = [l11;0];
L12 = [-l12;0];
L13 = [-l13;0];
L1c1 = [-l1c1;0];
L21 = [l21;0];
L22 = [-l22;0];
L23 = [-l23;0];
L2c1 = [l2c1;0];
%############################################################
% Position and Velocities of Unit 1 and 2
%############################################################
% Position of CoG of Unit 1 and 2 w.r.t. Inertial Frame {E}
r1 = [x1 y1]';
r2 = r1 - R_v2*L2c1 + R_v1*L1c1;
% Velocity of CoG of Unit 1 and 2 w.r.t. Inertial Frame {E}
% Note: Chain rule was used to obtain the expression:
%       d_r1 = (dr1/dt) = (dr1/dq)*(dq/dt)
%       d_r2 = (dr2/dt) = (dr2/dq)*(dq/dt)
d_r1 = jacobian(r1,q)*dq;    
d_r2 = jacobian(r2,q)*dq;
%matlabFunction(d_r2,'File','d_r2_Fun');
%############################################################
% Position and Velocities of Tyres w.r.t Inertial and Fixed Frame
%############################################################
% r11 : Position of Unit 1 Axle 1 w.r.t. Frame {E}
% r12 : Position of Unit 1 Axle 2 w.r.t Frame {E}
% r13 : Position of Unit 1 Axle 3 w.r.t Frame {E}
% r21 : Position of Unit 2 Axle 1 w.r.t Frame {E}
% r22 : Position of Unit 2 Axle 2 w.r.t Frame {E}
% r23 : Position of Unit 2 Axle 3 w.r.t Frame {E}
% v11 : Translational velocity vector of vehicle unit 1 axle 1. Unit: [m/s]
% v12 : Translational velocity vector of vehicle unit 1 axle 2. Unit: [m/s]
% v13 : Translational velocity vector of vehicle unit 1 axle 3. Unit: [m/s]
% v21 : Translational velocity vector of vehicle unit 2 axle 1. Unit: [m/s]
% v22 : Translational velocity vector of vehicle unit 2 axle 2. Unit: [m/s]
% v23 : Translational velocity vector of vehicle unit 2 axle 3. Unit: [m/s]
r11 = r1 + R_v1*L11;
r12 = r1 + R_v1*L12;
r13 = r1 + R_v1*L13;
r21 = r2 + R_v2*L21;
r22 = r2 + R_v2*L22;
r23 = r2 + R_v2*L23;
v11 = jacobian(r11,q)*dq;
v12 = jacobian(r12,q)*dq;
v13 = jacobian(r13,q)*dq;
v21 = jacobian(r21,q)*dq;
v22 = jacobian(r22,q)*dq;
v23 = jacobian(r23,q)*dq;
%##############################
%  Kinetic Energy Calculation
%##############################
T = 1/2 *m1*(d_r1')*d_r1 + 1/2*m2*(d_r2')*d_r2 + 1/2*J1*d_psi1^2 + 1/2*J2*d_psi2^2;
%##############################
% Lagrange Equation
%##############################
T_q = jacobian(T,q)';
T_dq = jacobian(T,dq)';
%%
%##############################
% External Forces
%##############################
% BODY :
% F_air_1           : Aerodynamic drag in a body fixed frame. Unit: [N]
% F_grav_1          : Gravitational force of vehicle Unit 1. Unit: [N]
% F_grav_2          : Gravitational force of vehicle Unit 2. Unit: [N]
% [Fxv1,Fyv1]       : Body forces of vehicle Unit 1 in a body fixed frame (Frame{1}). Unit: [N]
% [Fxv2,Fyv2]       : Body forces of vehicle Unit 2 in a body fixed frame (Frame{2}). Unit: [N]
% F1                : Body force vector of vehicle Unit 1. Unit: [N]
% F2                : Body force vector of vehicle Unit 2. Unit: [N]
%-----------
F_air_1 = -0.5*rho_air*Av*Cd*vx1^2;
F_grav_1 = -m1*g*sin(theta_r);
F_grav_2 = -m2*g*sin(theta_r);
Fxv1 = F_air_1 + F_grav_1;
Fxv2 = F_grav_2;
Fyv1 = 0;
Fyv2 = 0;
F1 = R_v1*[Fxv1;Fyv1];
F2 = R_v2*[Fxv2;Fyv2];
%-----------
% TYRES : This forces are in function of "vx1" and "vy1" instead of "d_x1"
% and "d_y1".
% [vxw11,vyw11]     : Translational velocities of vehicle unit 1 axle 1 in a wheel fixed frame. Unit: [m/s]
% [vxw12,vyw12]     : Translational velocities of vehicle unit 1 axle 2 in a wheel fixed frame. Unit: [m/s]
% [vxw13,vyw13]     : Translational velocities of vehicle unit 1 axle 3 in a wheel fixed frame. Unit: [m/s]
% [vxw21,vyw21]     : Translational velocities of vehicle unit 2 axle 1 in a wheel fixed frame. Unit: [m/s]
% [vxw22,vyw22]     : Translational velocities of vehicle unit 2 axle 2 in a wheel fixed frame. Unit: [m/s]
% [vxw23,vyw23]     : Translational velocities of vehicle unit 2 axle 3 in a wheel fixed frame. Unit: [m/s]
%-------------
% Vw11=simplify(subs(R_w11'*v11,[d_x1,d_y1],[vx1*cos(psi1)-vy1*sin(psi1),vx1*sin(psi1)+vy1*cos(psi1)]));
% Vw12=simplify(subs(R_v1'*v12,[d_x1,d_y1],[vx1*cos(psi1)-vy1*sin(psi1),vx1*sin(psi1)+vy1*cos(psi1)]));
% Vw13=simplify(subs(R_v1'*v13,[d_x1,d_y1],[vx1*cos(psi1)-vy1*sin(psi1),vx1*sin(psi1)+vy1*cos(psi1)]));
% Vw21=simplify(subs(R_v2'*v21,[d_x1,d_y1],[vx1*cos(psi1)-vy1*sin(psi1),vx1*sin(psi1)+vy1*cos(psi1)]));
% Vw22=simplify(subs(R_v2'*v22,[d_x1,d_y1],[vx1*cos(psi1)-vy1*sin(psi1),vx1*sin(psi1)+vy1*cos(psi1)]));
% Vw23=simplify(subs(R_v2'*v23,[d_x1,d_y1],[vx1*cos(psi1)-vy1*sin(psi1),vx1*sin(psi1)+vy1*cos(psi1)]));
%######################
Vw11=simplify(R_w11'*v11);
Vw12=simplify(R_v1'*v12);
Vw13=simplify(R_v1'*v13);
Vw21=simplify(R_v2'*v21);
Vw22=simplify(R_v2'*v22);
Vw23=simplify(R_v2'*v23);
%######################
vxw11 = Vw11(1);
vyw11 = Vw11(2);
vxw12 = Vw12(1);
vyw12 = Vw12(2);
vxw13 = Vw13(1);
vyw13 = Vw13(2);
vxw21 = Vw21(1);
vyw21 = Vw21(2);
vxw22 = Vw22(1);
vyw22 = Vw22(2);
vxw23 = Vw23(1);
vyw23 = Vw23(2);
% [Fxw11,Fyw11]     : Tyre forces of Unit 1 Axle 1 in Wheel Fixed Frame (Frame {H}). Unit: [N]
% [Fxw12,Fyw12]    	: Tyre forces of Unit 1 Axle 2 in Wheel Fixed Frame (Frame {1}). Unit: [N]
% [Fxw13,Fyw13]   	: Tyre forces of Unit 1 Axle 3 in Wheel Fixed Frame (Frame {1}). Unit: [N]
% [Fxw21,Fyw21]    	: Tyre forces of Unit 2 Axle 1 in Wheel Fixed Frame (Frame {2}). Unit: [N]
% [Fxw22,Fyw22]     : Tyre forces of Unit 2 Axle 2 in Wheel Fixed Frame (Frame {2}). Unit: [N]
% [Fxw23,Fyw23]     : Tyre forces of Unit 2 Axle 3 in Wheel Fixed Frame (Frame {2}). Unit: [N]
% Fp12              : Propulsion force vehicle unit 1 axle 2 in a wheel fixed frame. Unit: [N]
% Fb11              : Brake force vehicle Unit 1 Axle 1 in a wheel fixed frame. Unit: [N]
% Fb12              : Brake force vehicle Unit 1 Axle 2 in a wheel fixed frame. Unit: [N]
% Fb13              : Brake force vehicle Unit 1 Axle 3 in a wheel fixed frame. Unit: [N]
% Fb21              : Brake force vehicle Unit 2 Axle 1 in a wheel fixed frame. Unit: [N]
% Fb22              : Brake force vehicle Unit 2 Axle 2 in a wheel fixed frame. Unit: [N]
% Fb23              : Brake force vehicle Unit 2 Axle 3 in a wheel fixed frame. Unit: [N]
% Sy11,Sy12,Sy13    : Lateral tyre slip of vehicel unit 1 axle 1,2 and 3.
% Sy21,Sy22,Sy23    : Lateral tyre slip of vehicel unit 2 axle 1,2 and 3.
% Cy11,Cy12,Cy13    : Cornering stiffness of vehicel unit 1 axle 1,2 and 3. Unit: [N/rad]
% Cy21,Cy22,Cy23    : Cornering stiffness of vehicel unit 2 axle 1,2 and 3. Unit: [N/rad]
%------------
% Note: Rolling resistances IS NOT INCLUDED HERE !!
% Fzv11,Fzv12,Fzv13 : Vertical force of vehicle unit 1 axle 1,2,3 in a body fixed frame. Unit: [N]  
% Fzv21,Fzv22,Fzv23 : Vertical force of vehicle unit 2 axle 1,2,3 in a body fixed frame. Unit: [N] 
% Cr                : Rolling resistance coefficient.
%------------
% Lateral Tyre Slips
Sy11 = vyw11/vxw11;
Sy12 = vyw12/vxw12;
Sy13 = vyw13/vxw13;
Sy21 = vyw21/vxw21;
Sy22 = vyw22/vxw22;
Sy23 = vyw23/vxw23;
% Wheel 11
Fxw11 = - Fb11;
Fyw11 = - Cy11*Sy11;
% Wheel 12
%Fxw12 = Fp12 - Fb12;
Fyw12 = - Cy12*Sy12;
% Wheel 13
Fxw13 = - Fb13;
Fyw13 = - Cy13*Sy13;
% Wheel 21
Fxw21 = - Fb21;
Fyw21 = - Cy21*Sy21;
% Wheel 22
Fxw22 = - Fb22;
Fyw22 = - Cy22*Sy22;
% Wheel 23
Fxw23 = - Fb23;
Fyw23 = - Cy23*Sy23;
% F11 : Tyre force vector of vehicle Unit 1 Axle 1. Unit: [N]
% F12 : Tyre force vector of vehicle Unit 1 Axle 2. Unit: [N]
% F13 : Tyre force vector of vehicle Unit 1 Axle 3. Unit: [N]
% F21 : Tyre force vector of vehicle Unit 2 Axle 1. Unit: [N]
% F22 : Tyre force vector of vehicle Unit 2 Axle 2. Unit: [N]
% F23 : Tyre force vector of vehicle Unit 2 Axle 3. Unit: [N]
F11 = R_w11*[Fxw11;Fyw11];
F12 = R_v1*[Fxw12;Fyw12];
F13 = R_v1*[Fxw13;Fyw13];
F21 = R_v2*[Fxw21;Fyw21];
F22 = R_v2*[Fxw22;Fyw22];
F23 = R_v2*[Fxw23;Fyw23];

Q = (F11')*jacobian(r11,q) + (F12')*jacobian(r12,q) + ...
    (F13')*jacobian(r13,q) + (F21')*jacobian(r21,q) + ...
    (F22')*jacobian(r22,q) + (F23')*jacobian(r23,q) + ...
    (F1')*jacobian(r1,q) + (F2')*jacobian(r2,q);
Q = simplify(Q');
%##############################
% Dynamics of the system
%##############################
ddq = inv(jacobian(T_dq,dq))*(-jacobian(T_dq,q)*dq +T_q+Q);
%   Remember that: 
%       [dx1 dy1]' = R_E1*[u v]'
%       [ddx1 ddy1]' = dotR_E1*[u v] + R_E1*[du dv]'

% ddq_v : Represents ddq. We express all d_x1 and d_y1 in function of vx1,
% vy1 and psi1 using: [d_x1 d_y1]' = R_v1*[vx1 vy1]'
ddq_v = subs(ddq,[d_x1 d_y1],[vx1*cos(psi1)-vy1*sin(psi1), vy1*cos(psi1)+vx1*sin(psi1)]);
% Derivation in function of time of rotation matrix R_E1
dotR_v1 =  [-sin(psi1)*d_psi1,-cos(psi1)*d_psi1;
        cos(psi1)*d_psi1,-sin(psi1)*d_psi1];
% Converting [dd_x1 dd_y1]' to [d_vx1 d_vy1]' : 
% ddq_v = [d_vx1 d_vy1 dd_psi1 dd_deltaPsi1]
ddq_v(1:2) = R_v1'*(ddq_v(1:2) - dotR_v1*[vx1;vy1]);
%% Simplifications on model
Fb11_value = 0;         % No breaking in the unit 1 axle 1.
Fb12_value = 0;         % No breaking in the unit 1 axle 2.
Fb13_value = 0;         % No breaking in the unit 1 axle 3.
Fb21_value = 0;         % No breaking in the unit 2 axle 1.
Fb22_value = 0;         % No breaking in the unit 2 axle 1.
Fb23_value = 0;         % No breaking in the unit 2 axle 3.    
Av_value = 0;           % No aerodynamics considered.
Cd_value = 0;           % No aerodynamics considered.
rho_air_value = 0;      % No aerodynamics considered.
theta_r_value = 0;      % No inclination in the road.
ddq_v = simplify(subs(ddq_v,...
                [theta_r,rho_air,Fb11,Fb12,Fb13,Fb21,Fb22,Fb23],...
                [theta_r_value,rho_air_value,Fb11_value,Fb12_value,Fb13_value,Fb21_value,Fb22_value,Fb23_value]));
%% Nonlinear State-Space Model
% X = [vx1 vy1 d_psi1 d_deltaPsi1 psi1 deltaPsi1]
dotX = [ddq_v;d_psi1;d_deltaPsi1];          % Nonlinear model: dX = f(X,U)
%% Assuming constant velocity
%Fxw12 = 0;                              % Longitudinal velocity it is "forced" to be zero
F11 = R_v1*[Fxw11;Fyw11*cos(delta11)];  % Force applied on front wheel. The longitudinal force generated by steering is neglected.  
Q_vx = (F11')*jacobian(r11,q) + (F12')*jacobian(r12,q) + ...
    (F13')*jacobian(r13,q) + (F21')*jacobian(r21,q) + ...
    (F22')*jacobian(r22,q) + (F23')*jacobian(r23,q);
Q_vx = simplify(Q_vx');
Q_vx = simplify(subs(Q_vx,Fxw12,0));
%##############################
% Dynamics of the system
%##############################
ddq_vx = (jacobian(T_dq,dq))\(-jacobian(T_dq,q)*dq +T_q+Q_vx);
%   Remember that: 
%       [dx1 dy1]' = R_E1*[u v]'
%       [ddx1 ddy1]' = dotR_E1*[u v] + R_E1*[du dv]'
% ddq_v : Represents ddq. We express all d_x1 and d_y1 in function of vx1,
% vy1 and psi1 using: [d_x1 d_y1]' = R_v1*[vx1 vy1]'
ddq_v_vx = subs(ddq_vx,[d_x1 d_y1],[vx1*cos(psi1)-vy1*sin(psi1), vy1*cos(psi1)+vx1*sin(psi1)]);
% Derivation in function of time of rotation matrix R_E1
dotR_v1 =  [-sin(psi1)*d_psi1,-cos(psi1)*d_psi1;
        cos(psi1)*d_psi1,-sin(psi1)*d_psi1];
% Converting [dd_x1 dd_y1]' to [d_u d_v]' : 
ddq_v_vx(1:2) = R_v1'*(ddq_v_vx(1:2) - dotR_v1*[vx1;vy1]);
%% Simplifications on model
Fb11_value = 0;         % No breaking in the unit 1 axle 1.
Fb12_value = 0;         % No breaking in the unit 1 axle 2.
Fb13_value = 0;         % No breaking in the unit 1 axle 3.
Fb21_value = 0;         % No breaking in the unit 2 axle 1.
Fb22_value = 0;         % No breaking in the unit 2 axle 1.
Fb23_value = 0;         % No breaking in the unit 2 axle 3.    
Av_value = 0;           % No aerodynamics considered.
Cd_value = 0;           % No aerodynamics considered.
rho_air_value = 0;      % No aerodynamics considered.
theta_r_value = 0;      % No inclination in the road.
ddq_v_vx = simplify(subs(ddq_v_vx,...
                [theta_r,rho_air,Fb11,Fb12,Fb13,Fb21,Fb22,Fb23],...
                [theta_r_value,rho_air_value,Fb11_value,Fb12_value,Fb13_value,Fb21_value,Fb22_value,Fb23_value]));
dotX_vx_constant = [ddq_v_vx(2:4);d_psi1;d_deltaPsi1];            
%% Linearized Models
fprintf('########################### \n');
fprintf('Linearization \n')
fprintf('########################### \n');
 % Define symbolic of states and inputs. Just in case !
syms vx1 vy1 d_psi1 d_deltaPsi1 psi1 deltaPsi1 real     % States : X = [vx1 vy1 d_psi1 d_deltaPsi1 psi1 deltaPsi1]
syms delta11 Fxw12 real                                 % Inputs: U = [delta11 Fxw12]
[dotX_lin,A,B] = mt.tools.linearizeStateSpace(dotX,[vx1 vy1 d_psi1 d_deltaPsi1 psi1 deltaPsi1]',[delta11 Fxw12]',[vx1 0 0 0 0 0]',[0 0]');
[dotX_vx_constant_lin,A_vx,B_vx] = mt.tools.linearizeStateSpace(dotX_vx_constant,[vy1 d_psi1 d_deltaPsi1 psi1 deltaPsi1]',delta11,[0 0 0 0 0]',0);
%% Generating function files with results.
% Note : After generating, some changes were done like: "help comment" and
% change of name of inputs. I changed from in1,in2,in3 to X,U,params.
fprintf('########################### \n');
fprintf('Generating code \n')
fprintf('########################### \n');
% Nonlinear model files
% params = [Cy11 Cy12 Cy13 Cy21 Cy22 Cy23 l11 l12 l13 l21 l22 l23 l1c1 l2c1 m1 m2 J1 J2]
%matlabFunction(dotX,'File','semitrailerNonlinear','Vars',{[vx1 vy1 d_psi1 d_deltaPsi1 psi1 deltaPsi1],[delta11 Fxw12],[Cy11 Cy12 Cy13 Cy21 Cy22 Cy23 l11 l12 l13 l21 l22 l23 l1c1 l2c1 m1 m2 J1 J2]},'Output',{'dotX'});
%matlabFunction(dotX_vx_constant,'File','semitrailerNonlinearVxConstant','Vars',{[vy1 d_psi1 d_deltaPsi1 psi1 deltaPsi1],delta11,vx1,[Cy11 Cy12 Cy13 Cy21 Cy22 Cy23 l11 l12 l13 l21 l22 l23 l1c1 l2c1 m1 m2 J1 J2]},'Output',{'dotX'});
% Linear model files
matlabFunction(dotX_lin,A,B,'File','semitrailerLinear','Vars',{[vx1 vy1 d_psi1 d_deltaPsi1 psi1 deltaPsi1],[delta11 Fxw12],[Cy11 Cy12 Cy13 Cy21 Cy22 Cy23 l11 l12 l13 l21 l22 l23 l1c1 l2c1 m1 m2 J1 J2]},'Output',{'dotX','A','B'});
matlabFunction(dotX_vx_constant_lin,A_vx,B_vx,'File','semitrailerLinearVxConstant','Vars',{[vy1 d_psi1 d_deltaPsi1 psi1 deltaPsi1],delta11,vx1,[Cy11 Cy12 Cy13 Cy21 Cy22 Cy23 l11 l12 l13 l21 l22 l23 l1c1 l2c1 m1 m2 J1 J2]},'Output',{'dotX','A','B'});


