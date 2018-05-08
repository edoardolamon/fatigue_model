close all
clear all
clc

%% Import saved data

addpath('7DoFs')
%data_path = 'data/7DoFs_opt_conf_2018_05_08_10_19_20.mat';
data_path = 'data/7DoFs_opt_conf_2018_05_08_11_55_40.mat';
load(data_path);

%% computations 
x_opt_constr_sqp = LWR.fkine(q_opt_constr_sqp).t;
[~, c_constr_sqp] = cartesianEE7DoFsConstraint(LWR, q_opt_constr_sqp, x_ee);

x_opt_constr_sqp_sphere = LWR.fkine(q_opt_constr_sqp_sphere).t;
[c_constr_sqp_sphere, ~] = cartesianEESphere7DoFsConstraint(LWR, q_opt_constr_sqp_sphere, x_ee, radius);

f_ext_scaled = 0.4/norm(f_ext)*f_ext;

%% display
disp('SHOWING SQP CONFIGURATION WITH POINT CONSTRAINT')
LWR.plot(q_opt_constr_sqp);
hold on
h = quiver3(x_opt_constr_sqp(1), x_opt_constr_sqp(2), x_opt_constr_sqp(3), f_ext_scaled(1), f_ext_scaled(2), f_ext_scaled(3));
pause;

disp('RELAXING POINT CONSTRAINT')
[x, y, z] = sphere;
s1 = mesh(radius*x+x_ee(1), radius*y+x_ee(2), radius*z+x_ee(3));
alpha 0.5
pause;

disp('SHOWING SQP CONFIGURATION WITH SPHERE CONSTRAINT')
delete(h);
LWR.plot(q_opt_constr_sqp_sphere);
h = quiver3(x_opt_constr_sqp_sphere(1), x_opt_constr_sqp_sphere(2), x_opt_constr_sqp_sphere(3), f_ext_scaled(1), f_ext_scaled(2), f_ext_scaled(3));
pause;

disp('----------------------------------RESULTS-------------------------------------')
disp(['Optimized sqp configuration (point const): ', num2str(q_opt_constr_sqp)]);
disp(['Fatigue: ' num2str(fatigue_opt_constr_sqp) ]);
disp(['Cart pos: ' num2str(x_opt_constr_sqp')]);
disp(['Constraint value: ' num2str(c_constr_sqp')]);
disp(' ');
disp(['Optimized sqp configuration (sphere const): ', num2str(q_opt_constr_sqp_sphere)]);
disp(['Fatigue: ' num2str(fatigue_opt_constr_sqp_sphere) ]);
disp(['Cart pos: ' num2str(x_opt_constr_sqp_sphere')]);
disp(['Constraint value: ' num2str(c_constr_sqp_sphere')]);
disp('------------------------------------------------------------------------------')