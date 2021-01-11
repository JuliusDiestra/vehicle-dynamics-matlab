function [dotX_lin,A,B] = linearizeStateSpace(dotX,X,U,X0,U0)
% LINEARIZESTATESPACE Linearize nonlinear state-space model around an stationary-point
%   LINEARIZESTATESPACE(dotX,X,U,X0,U0)    : dotX is linearized around (X0,U0)
%   LINEARIZESTATESPACE(dotX,X,U)          : dotX is linearized around (0,0)
%
%   Inputs:
%       dotX    : Symbolic expression of dotX = dX/dt = f(X,U)
%       X       : Symbolic expression of state.
%       U       : Symbolic expression of input.
%       X0      : Stationary point of state X.
%       U0      : Stationary point of input U.
%   Output:
%       dotX_lin    : Symbolic expression of linearized dotX : dotX = A*X +B*U
%       A           : Numerical matrix of dotX = A*X + B*U
%       B           : Numerical matrix of dotX = A*X + B*U
%   Author : Julius D.
    switch nargin
        case 5
            assert(isa(dotX,'sym'),'The inputs dotX is type %s, not symbolic',class(dotX));
            assert(isa(X,'sym'),'The inputs X is type %s, not symbolic',class(X));
            assert(isa(U,'sym'),'The inputs U is type %s, not symbolic',class(U));
            assert(or(isa(X0,'double'),isa(X0,'sym')),'The inputs X0 is type %s, not numerical',class(X0));
            assert(or(isa(U0,'double'),isa(U0,'sym')),'The inputs U0 is type %s, not numerical',class(U0));
            A = simplify(subs(jacobian(dotX,X),[X' U'],[X0' U0']));
            B = simplify(subs(jacobian(dotX,U),[X' U'],[X0' U0']));
        case 3
            assert(isa(dotX,'sym'),'The inputs dotX is type %s, not symbolic',class(dotX));
            assert(isa(X,'sym'),'The inputs X is type %s, not symbolic',class(X));
            assert(isa(U,'sym'),'The inputs U is type %s, not symbolic',class(U));
            X0 = zeros(length(X),1);
            U0 = zeros(length(U),1);
            A = simplify(subs(jacobian(dotX,X),[X' U'],[X0' U0']));
            B = simplify(subs(jacobian(dotX,U),[X' U'],[X0' U0']));
        otherwise
            A = 0;
            B = 0;
            error('Not enough inputs arguments !');
    end
    dotX_lin = simplify(A*X+B*U);
end