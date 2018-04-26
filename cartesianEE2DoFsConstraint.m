function [c, ceq] = cartesianEE2DoFsConstraint(q)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% Model
l1 = Link('d', 0, 'a', 1, 'alpha', 0);
l2 = Link('d', 0, 'a', 1, 'alpha', 0);
planar_arm = SerialLink([l1 l2], 'name', '2 DoFs Planar Arm');

% Initial configuration
%q0 = [5/12*pi 1/5*pi];
q0 = [2/12*pi 1/2*pi];
x_ee = planar_arm.fkine(q0).t;

% Nonlinear constraint
c = [];
% x axis constraint
ceq = planar_arm.fkine(q).t(1) - x_ee(1);
% y axis constraint
%ceq = planar_arm.fkine(q).t(2) - x_ee(2);

end

