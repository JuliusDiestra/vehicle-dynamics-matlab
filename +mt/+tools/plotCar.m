function h1 = plotCar(r,psi,param,color)
% PLOTCAR Plotting minimalistic car shape.
%   PLOTCAR(r,psi,param,color)  : Plotting minimalistic car shape.
%
%   Inputs:
%       r       : Position vector of Center of Gravity of vehicle 
%       psi     : Angle of Frame {V} with respect to Frame {E}. Data type : sym or double
%       param   : [l1,l2,w,lf,lr]
%                   l1 : Distance between front axle and centre of gravity. Unit: [m]
%                   l2 : Distance between rear axle and centre of gravity. Unit: [m]
%                   lf : Distance between front part and center of gravity. Unit : [m]
%                   lr : Distance between rear part and center of gravity. Unit : [m]
%       color   : Car color. 
%   Author : Julius D.
switch nargin
    case 3
        [wheels,borders] = mt.tools.wagon(r,psi,param);
        b1 = borders(:,1);
        b2 = borders(:,2);
        b3 = borders(:,3);
        b4 = borders(:,4);
        figure(1)
        h1 = plot([b1(1) b2(1) b4(1) b3(1) b1(1)], [b1(2) b2(2) b4(2) b3(2) b1(2)]);
    case 4
        [wheels,borders] = mt.tools.wagon(r,psi,param);
        b1 = borders(:,1);
        b2 = borders(:,2);
        b3 = borders(:,3);
        b4 = borders(:,4);
        figure(1)
        h1 = plot([b1(1) b2(1) b4(1) b3(1) b1(1)], [b1(2) b2(2) b4(2) b3(2) b1(2)],'Color',color);
    otherwise
        error('Too many input arguments !');
end

end