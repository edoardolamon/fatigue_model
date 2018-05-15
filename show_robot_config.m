close all
clear all
clc

%% Import saved data

addpath('7DoFs')
%data_path = 'data/7DoFs_opt_conf_2018_05_08_10_19_20.mat';
%data_path = 'data/7DoFs_opt_conf_2018_05_08_11_55_40.mat';
data_path = 'data/7DoFs_opt_conf_2018_05_15_11_27_06.mat';
load(data_path);

%% computations
x0_fkine = LWR.fkine(q0).t;
[~, c_constr0] = cartesianEE7DoFsConstraint(LWR, q0, x_ee);

x_opt_constr_sqp = LWR.fkine(q_opt_constr_sqp).t;
[~, c_constr_sqp] = cartesianEE7DoFsConstraint(LWR, q_opt_constr_sqp, x_ee);

x_opt_constr_sqp_sphere = LWR.fkine(q_opt_constr_sqp_sphere).t;
[c_constr_sqp_sphere, ~] = cartesianEESphere7DoFsConstraint(LWR, q_opt_constr_sqp_sphere, x_ee, radius);

x_min_eff = LWR.fkine(q_min_eff).t;
[~, c_min_eff] = cartesianEE7DoFsConstraint(LWR, q_min_eff, x_ee);
fatigue_min_eff = fatigue7DoFs(LWR,q_min_eff,f_ext,duration,capacity);

x_min_eff_sphere = LWR.fkine(q_min_eff_sphere).t;
[c_min_eff_sphere, ~] = cartesianEESphere7DoFsConstraint(LWR, q_min_eff_sphere, x_ee, radius);
fatigue_min_eff_sphere = fatigue7DoFs(LWR,q_min_eff_sphere,f_ext,duration,capacity);

f_ext_scaled = 0.4/norm(f_ext)*f_ext;

% q0 = zeros(1,7);
%% Plots
subplot(2,3,[1 2 4 5]);
disp('SHOWING INITIAL CONFIGURATION')
LWR.plot(q0);
hold on
h = quiver3(x_ee(1), x_ee(2), x_ee(3), f_ext_scaled(1), f_ext_scaled(2), f_ext_scaled(3));

subplot(2,3,3);
fatigue_plot = bar3([fatigue0_vec]);%, fatigue_vec_opt, fatigue_vec_opt_sphere, fatigue_vec_min_eff, fatigue_vec_min_eff_sphere]);
title('Fatigue of every configuration')

subplot(2,3,6);
tau_plot = bar3(abs([tau0]));% ,tau_opt, tau_opt_sphere, tau_min_eff, tau_min_eff_sphere]));
title('Absolute value of torque of every configuration')

pause;

delete(h);
disp('SHOWING FATIGUE-BASED SQP CONFIGURATION WITH POINT CONSTRAINT')
subplot(2,3,[1 2 4 5]);
LWR.plot(q_opt_constr_sqp);
hold on
h = quiver3(x_opt_constr_sqp(1), x_opt_constr_sqp(2), x_opt_constr_sqp(3), f_ext_scaled(1), f_ext_scaled(2), f_ext_scaled(3));

subplot(2,3,3);
%hold on
fatigue_plot = bar3([fatigue0_vec, fatigue_vec_opt], 'grouped'); %, fatigue_vec_opt_sphere, fatigue_vec_min_eff, fatigue_vec_min_eff_sphere]);
%bar3(fatigue_vec_opt, 'grouped', 'r');
title('Fatigue of every configuration')

subplot(2,3,6);
tau_plot = bar3(abs([tau0 ,tau_opt]), 'grouped'); %, tau_opt_sphere, tau_min_eff, tau_min_eff_sphere]));
title('Absolute value of torque of every configuration')

pause;

disp('RELAXING POINT CONSTRAINT')
[x, y, z] = sphere;
subplot(2,3,[1 2 4 5]);
s1 = mesh(radius*x+x_ee(1), radius*y+x_ee(2), radius*z+x_ee(3));
alpha 0.5
pause;

disp('SHOWING SQP CONFIGURATION WITH SPHERE CONSTRAINT')
delete(h); 
subplot(2,3,[1 2 4 5]);
LWR.plot(q_opt_constr_sqp_sphere);
h = quiver3(x_opt_constr_sqp_sphere(1), x_opt_constr_sqp_sphere(2), x_opt_constr_sqp_sphere(3), f_ext_scaled(1), f_ext_scaled(2), f_ext_scaled(3));

subplot(2,3,3);
% alpha 0.5
fatigue_plot = bar3([fatigue0_vec, fatigue_vec_opt, fatigue_vec_opt_sphere],'grouped');%, fatigue_vec_min_eff, fatigue_vec_min_eff_sphere]);
%bar3(fatigue_vec_opt_sphere,'grouped', 'g');%,
title('Fatigue of every configuration')

subplot(2,3,6);
tau_plot = bar3(abs([tau0 ,tau_opt, tau_opt_sphere]),'grouped');%, tau_min_eff, tau_min_eff_sphere]));
title('Absolute value of torque of every configuration')
pause;

delete(h);
%delete(s1);
disp('SHOWING TORQUE-BASED SQP CONFIGURATION WITH POINT CONSTRAINT')
subplot(2,3,[1 2 4 5]);
LWR.plot(q_min_eff);
hold on
h = quiver3(x_min_eff(1), x_min_eff(2), x_min_eff(3), f_ext_scaled(1), f_ext_scaled(2), f_ext_scaled(3));

subplot(2,3,3);
fatigue_plot = bar3([fatigue0_vec, fatigue_vec_opt, fatigue_vec_opt_sphere, fatigue_vec_min_eff], 'grouped');%, fatigue_vec_min_eff_sphere]);
title('Fatigue of every configuration')

subplot(2,3,6);
tau_plot = bar3(abs([tau0 ,tau_opt, tau_opt_sphere, tau_min_eff]), 'grouped');%, tau_min_eff_sphere]));
title('Absolute value of torque of every configuration')

pause;

% [x, y, z] = sphere;
% s1 = mesh(radius*x+x_ee(1), radius*y+x_ee(2), radius*z+x_ee(3));
% alpha 0.5

disp('SHOWING SQP CONFIGURATION WITH SPHERE CONSTRAINT')
delete(h);
subplot(2,3,[1 2 4 5]);
LWR.plot(q_min_eff_sphere);
h = quiver3(x_min_eff_sphere(1), x_min_eff_sphere(2), x_min_eff_sphere(3), f_ext_scaled(1), f_ext_scaled(2), f_ext_scaled(3));

% figure
subplot(2,3,3);
fatigue_plot = bar3([fatigue0_vec, fatigue_vec_opt, fatigue_vec_opt_sphere, fatigue_vec_min_eff, fatigue_vec_min_eff_sphere], 'grouped');
title('Fatigue of every configuration')

%figure
subplot(2,3,6);
tau_plot = bar3(abs([tau0 ,tau_opt, tau_opt_sphere, tau_min_eff, tau_min_eff_sphere]), 'grouped');
title('Absolute value of torque of every configuration')

%% results
disp('----------------------------------RESULTS-------------------------------------')
disp(['Initial configuration: ', num2str(q0)]);
disp(['Fatigue: ' num2str(fatigue0) ]);
disp(['Sum of squared torques: ' num2str(tau0_sum) ]);
disp(['Torques: ' num2str(tau_opt') ]);
disp(['Cart pos: ' num2str(x0_fkine')]);
disp(['Constraint value: ' num2str(c_constr0')]);
disp(' ');
disp(['Fatigue-based optimal configuration (point const): ', num2str(q_opt_constr_sqp)]);
disp(['Fatigue: ' num2str(fatigue_opt_constr_sqp) ]);
disp(['Sum of squared torques: ' num2str(tau_opt_sum) ]);
%disp(['Torques: ' num2str(tau_opt') ]);
disp(['Cart pos: ' num2str(x_opt_constr_sqp')]);
disp(['Constraint value: ' num2str(c_constr_sqp')]);
disp(' ');
disp(['Fatigue-based optimal configuration (sphere const): ', num2str(q_opt_constr_sqp_sphere)]);
disp(['Fatigue: ' num2str(fatigue_opt_constr_sqp_sphere) ]);
disp(['Sum of squared torques: ' num2str(tau_opt_sum_sphere) ]);
%disp(['Torques: ' num2str(tau_opt_sphere') ]);
disp(['Cart pos: ' num2str(x_opt_constr_sqp_sphere')]);
disp(['Constraint value: ' num2str(c_constr_sqp_sphere')]);
disp(' ');
disp(['Torque-based optimal configuration (point const): ', num2str(q_min_eff)]);
disp(['Fatigue: ' num2str(fatigue_min_eff) ]);
disp(['Sum of squared torques: ' num2str(tau_min_eff_sum) ]);
%disp(['Torques: ' num2str(tau_min_eff') ]);
disp(['Cart pos: ' num2str(x_min_eff')]);
disp(['Constraint value: ' num2str(c_min_eff')]);
disp(' ');
disp(['Torque-based optimal configuration (sphere const): ', num2str(q_min_eff_sphere)]);
disp(['Fatigue: ' num2str(fatigue_min_eff_sphere) ]);
disp(['Sum of squared torques: ' num2str(tau_min_eff_sum_sphere) ]);
%disp(['Torques: ' num2str(tau_min_eff_sphere') ]);
disp(['Cart pos: ' num2str(x_min_eff_sphere')]);
disp(['Constraint value: ' num2str(c_min_eff_sphere')]);
disp('------------------------------------------------------------------------------')