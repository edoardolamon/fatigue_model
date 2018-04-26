close all
clear all

%% Model
l1 = Link('d', 0, 'a', 1, 'alpha', 0);
l2 = Link('d', 0, 'a', 1, 'alpha', 0);
l3 = Link('d', 0, 'a', 1, 'alpha', 0);

planar_arm = SerialLink([l1 l2 l3], 'name', '3 DoFs Planar Arm');

% initial configuration
%q0 = [0.0 0.0 0.0];
q0 = [1.3 0 0];
%q0 = [-1.4 -1 -0.6];
x0 = planar_arm.fkine(q0).t;
f_ext = zeros(6,1);
f_ext(1) = 0;
f_ext(2) = 1;

%J = planar_arm.jacobe(q0);
% mdl_planar3
% J = p3.jacobe(q0);

planar_arm.plot(q0);

%% vectorial fatigue model
%capacity = eye(3);
capacity = ones(3,1);
T = 1; 
lambda = 0;

fatigue = @(q)(1 - exp(-T*capacity'*abs(planar_arm.jacobe(q)'* f_ext)));
%fatigue = @(q)(1 - exp(-T*capacity'*abs(planar_arm.jacobe(q)'* f_ext) + lambda*(sum(x0 - planar_arm.fkine(q).t)^2)));

fatigue0 = fatigue(q0);

q_tmp = 0:0.01:2*pi;
q_tmp = [zeros(1,size(q_tmp,2)); q_tmp; zeros(1,size(q_tmp,2))];
% fatigue_tmp = zeros(size(q_tmp,2),1);
% for i=1:size(q_tmp,2)
%     fatigue_tmp(i) = fatigue(q_tmp(:,i)); 
% end
% figure
% grid on
% plot(q_tmp(1,:), fatigue_tmp)

%% optimization
[q_opt, fatigue_opt] = fminsearch(fatigue,q0);
%options = optimset('PlotFcns',@optimplotfval);
%[q_opt, fatigue_opt] = fminsearch(fatigue,q0,options);
x_opt = planar_arm.fkine(q_opt).t;
planar_arm.plot(q_opt)

disp(["Initial conf:" q0 "Fatigue:" fatigue0 "Cart pos:" x0']);
disp(["Optimal conf:" q_opt "Fatigue:" fatigue_opt "Cart pos:" x_opt']);

