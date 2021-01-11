function d_q = dtGen(q)
% DTGEN Generation of symbolic derivative of vector.
%   d_q = DTGEN(q)  : Returns a same size vector with 'd_' added to each element.
%
%   Inputs:
%       q    : Symbolic vertical vector.
%   Output:
%       d_q    : Symbolic vertical vector with 'd_' added to each element.
%   Author : Julius D.

    % Conditions:
    % 1. Check if input q is a symbolic
    assert(isa(q,'sym'),'Input is not a symbolic vector');
    % 2. Check if input is a vertical vector
    assert(and(size(q,1)>=1,size(q,2)==1),'Input should be a vertical vector');
    % Output generation
    d_q = sym('d_q',[length(q),1]);
    for j = 1:length(q)
        string = strcat('d_',string(q(j)));
        d_q(j) = sym(string);
    end
end