function [c, ceq] = cartesianEESphere7DoFsConstraint(LWR,q,x_ee,radius)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% Nonlinear constraint
%c = [];
c = (LWR.fkine(q).t(1) - x_ee(1))^2 + (LWR.fkine(q).t(2) - x_ee(2))^2 + (LWR.fkine(q).t(3) - x_ee(3))^2 - radius^2;
% x axis constraint
%ceq = planar_arm.fkine(q).t(1) - x_ee(1);
% y axis constraint
%ceq = planar_arm.fkine(q).t(2) - x_ee(2);

% xyz constraint
ceq = [];
%ceq = LWR.fkine(q).t - x_ee;

end
