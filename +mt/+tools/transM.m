function r_frameE = transM(r_frameV,angle_EV,r_EV)
% TRANSM Calculation of transformation matrix of a define Frame {V} with respect to a reference Frame {E}.
%   TRANSM(r_frameV,angle_EV,r_EV)  : Transformation of vector r_frameV expressed in Frame V to a representation in Frame E.
%
%   Inputs:
%       r_frameV    : Vector expressed in Frame {V}. length(r_frameV) = {2,3}. Data type : sym or double
%       angle_EV    : Angle of Frame {V} with respect to Frame {E}. Data type : sym or double
%       r_EV        : Position vector of origin of Frame {V} with respect to Frame {E}. length(r_EV) = {2,3}. Data type : sym or double
%   Output:
%       r_frameE    : Vector expressed in Frame {E}. length(r_frameV) = {2,3}. Data type : sym or double
%   Author : Julius D.
    dimSet = [2;3];
    % Conditions:
    assert(any(dimSet == length(r_frameV)),'The dimension of the first input should be dim = {2,3}');
    assert(or(isa(angle_EV,'sym'),isa(angle_EV,'double')),'The input psi is type %s, not symbolic or double',class(angle));
    assert(any(dimSet == length(r_EV)),'The dimension of the second input should be dim = {2,3}');
    assert(length(r_frameV)==length(r_EV),'Vector inputs should have same dimensions.')
    % Calculations:
    R_EV = [cos(angle),-sin(angle);sin(angle),cos(angle)];
    r_frameE = r_EV + R_EV*r_frameV;
end