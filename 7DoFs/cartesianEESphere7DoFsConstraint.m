function [c, ceq] = cartesianEESphere7DoFsConstraint(LWR,q,x_ee,radius)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% Model
% mdl_kukaLWR;
% for i=1:7
%     LWR.links(i).m = 2.0;
%     LWR.links(i).Jm = 0;
%     LWR.links(i).G = 1;
% end

% Initial configuration
% q0 = [0 0 0 0 0 0 0];
% x_ee = LWR.fkine(q0).t;
% x_ee = [0.4; 0.3; 0.2];
%x_ee = [0.0; 0.79; 0.0];

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
