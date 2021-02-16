function [wheels,borders] = wagon(r,psi,param)
% WAGON Calculation of wheels and borders position with respect of Frame {E} (reference Frame).
%   WAGON(r,psi,param)  : Calculation of wheels and borders position using
%   current vehicle position, inclination and parameters of the vehicle.
%
%   Inputs:
%       r       : Position vector of Center of Gravity of vehicle 
%       psi     : Angle of Frame {V} with respect to Frame {E}. Data type : sym or double
%       param   : [l1,l2,w,lf,lr]
%                   l1 : Distance between front axle and centre of gravity. Unit: [m]
%                   l2 : Distance between rear axle and centre of gravity. Unit: [m]
%                   lf : Distance between front part and center of gravity. Unit : [m]
%                   lr : Distance between rear part and center of gravity. Unit : [m]
%   Output:
%       wheels  = [r1l,r1r,r2l,r2r]
%                   r1l : Position vector of left front wheel/axle.
%                   r1r : Position vector of right front wheel/axle.
%                   r2l : Position vector of left rear wheel/axle.
%                   r2r : Position vector of right rear wheel/axle.
%       borders = [r1bl,r1br,r2bl,r2br]
%                   r1bl : Position vector of left front border.
%                   r1br : Position vector of right front border.
%                   r2bl : Position vector of left rear border.
%                   r2br : Position vector of right rear border.
%   Author : Julius D.

    % Assign parameters
    l1 = param(1);
    l2 = param(2);
    w = param(3);
    lf = param(4);
    lr = param(5);
    % Wheel positions w.r.t Frame {V} (Vehicle Framework)
    r_V1l = [l1;w/2];
    r_V1r = [l1;-w/2];
    r_V2l = [-l2;w/2];
    r_V2r = [-l2;-w/2];
    % Borders position w.r.t. Frame {V} (Vehicle Framework)
    r_V1bl = [lf;w/2];
    r_V1br = [lf;-w/2];
    r_V2bl = [-lr;w/2];
    r_V2br = [-lr;-w/2];
    % Define Homogeneous matrix of Frame {V} (vehicle axis) with respect to
    % Frame {E} (reference frame)
    R_EV = [cos(psi),-sin(psi);
            sin(psi),cos(psi)];
    H_EV = [R_EV,r;
            zeros(1,2),1];
    % Transformation wheels/axle
    tilde_r1l = H_EV*[r_V1l;1];
    tilde_r1r = H_EV*[r_V1r;1];
    tilde_r2l = H_EV*[r_V2l;1];
    tilde_r2r = H_EV*[r_V2r;1];
    r1l = tilde_r1l(1:3);
    r1r = tilde_r1r(1:3);
    r2l = tilde_r2l(1:3);
    r2r = tilde_r2r(1:3);
    % Transformation border
    tilde_r1bl = H_EV*[r_V1bl;1];
    tilde_r1br = H_EV*[r_V1br;1];
    tilde_r2bl = H_EV*[r_V2bl;1];
    tilde_r2br = H_EV*[r_V2br;1];
    r1bl = tilde_r1bl(1:3);
    r1br = tilde_r1br(1:3);
    r2bl = tilde_r2bl(1:3);
    r2br = tilde_r2br(1:3);
    % Outputs
    wheels = [r1l,r1r,r2l,r2r];
    borders = [r1bl,r1br,r2bl,r2br];
end