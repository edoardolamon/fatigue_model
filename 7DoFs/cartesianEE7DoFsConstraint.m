function [c, ceq] = cartesianEE7DoFsConstraint(q)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% Model
mdl_LWR;
for i=1:7
    LWR.links(i).m = 2.0;
    LWR.links(i).Jm = 0;
    LWR.links(i).G = 1;
end

% Initial configuration
% q0 = [0 0 0 0 0 0 0];
% x_ee = LWR.fkine(q0).t;
% x_ee = [0.4; 0.6; -0.2];
x_ee = [0.0; 0.0; 0.79];

% Nonlinear constraint
c = [];
% x axis constraint
%ceq = planar_arm.fkine(q).t(1) - x_ee(1);
% y axis constraint
%ceq = planar_arm.fkine(q).t(2) - x_ee(2);

% xyz constraint
ceq = LWR.fkine(q).t - x_ee;
%everything here is completely wrong thank you bye
end

