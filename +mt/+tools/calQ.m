
function Q = calQ(q,F,r,angles)
% CALQ Calculation of generalized force of a system as a function of the generalized coordinates.
%   d_q = CALQ(q,F,r,angles)  : Returns a same size vector with 'd_' added to each element.
%
%   Inputs:
%       q       : Symbolic vertical vector of generalized coordinates.
%       F       : Matrix of forces applied on body. F = [F1 F2 F3 ..]. F1 is vertical vector.   
%       r       : Matrix of position vectors of the forces applied. r = [r1 r2 r3 ...].
%       angles  : Vector of angles of each frame for each force vector frame. 
%   Output:
%       Q       : Generalized force of system as a function of generalized coordinates.
%   Author : Julius D.
    
    % Conditions
    % 1) Check same row and columns of F,r and angles and same number of rows of q.
    % 2) Inputs should be sym or double
    n = size(F,2);
    Q = 0;
    for j = 1:n
        Fj = F(:,j);
        Fj_frameE = mt.tools.Rz(angles(j))*Fj;
        rj = r(:,j);
        Q = Q + (Fj_frameE')*jacobian(rj,q);
        Q = simplify(Q);
    end    
end