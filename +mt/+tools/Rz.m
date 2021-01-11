function R = Rz(angle,dim)
% RZ Calculation of rotation matrix around axis Z.
%   Rz(angle)      	: Rotation matrix using 'psi' angle around Z-axis. Dimension 2x2
%   RZ(angle,dim)   : Rotation matrix using 'psi' angle around Z-axis. Dimension dim x dim 
%
%   Inputs:
%       angle       : Angle of rotation. Data type : 'sym' or 'double'
%       dim         : Dimension of rotation matrix. dim = {2,3}. Data type : double
%   Output:
%       R           : Rotation matrix.
%   Author : Julius D.
    dimSet = [2;3];
    switch nargin
        case 1
            % Check input data type
            assert(or(isa(angle,'sym'),isa(angle,'double')),'The input psi is type %s, not symbolic or double',class(angle));
            % Estimate R
            R = [cos(angle),-sin(angle);
                 sin(angle),cos(angle)];
        case 2
            % Check input data type
            assert(or(isa(angle,'sym'),isa(angle,'double')),'The input psi is type %s, not symbolic or double',class(angle));
            assert(isa(dim,'double'),'The input dim is type %s, not double',class(dim));
            % Check if dim is on dimSet
            assert(any(dimSet == dim),'The input dim should be dim = {2,3}');
            % Estimatime R
            if (dim == 2)
                R = [cos(angle),-sin(angle);
                    sin(angle),cos(angle)];
            else
                R = [cos(angle),-sin(angle),0;
                    sin(angle),cos(angle),0;
                    0,0,1];                
            end
        otherwise
            error('Too many input arguments !');
    end
end