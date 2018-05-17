close all
clear all

curr_path = pwd;
if (curr_path(end-4:end) ~= '7DoFs')
    cd('7DoFs')
end

if (exist('h','var'))
    delete(h)
    delete(s1)
    alpha 1
end

%% Model

mdl_kukaLWR

f_ext = zeros(6,1);
f_ext(1) = 40;
f_ext(2) = 0;
f_ext(3) = 0;
f_ext_scaled = 0.4/norm(f_ext)*f_ext;

duration = 1;
capacity = 10 * ones(n_dofs,1);
% capacity = [10, 10, 10, 5, 1, 1, 1];
% capacity = [1, 1, 1, 1, 10, 10, 10];

% q0 = zeros(1,n_dofs);
% affine0 = LWR.fkine(q0);

%% Optimization

% constraints
q_lb = -3/4*pi*ones(n_dofs,1);
q_ub = 3/4*pi*ones(n_dofs,1);
% q_lb = [];
% q_ub = [];
A = [];
b = [];
Aeq = [];
beq = [];
% x_ee = rand(3,1);
% affine0.t = x_ee;
% % q0 = LWR.ikcon(affine0);
% q0 = LWR.ikinem(affine0);
q0 = 1.5*(rand(1, n_dofs) - 1);
x_ee = LWR.fkine(q0).t;
% x_ee = [0.4; 0.3; 0.2];
radius = 0.19;
% nonlcon = [];
cartPointCon = @(q) cartesianEE7DoFsConstraint(LWR,q,x_ee);
cartSphereCon = @(q) cartesianEESphere7DoFsConstraint(LWR,q,x_ee,radius);

% optimization with 'sqp'
options_sqp = optimoptions(@fmincon, 'Algorithm', 'sqp', 'Display', 'off');
trials = 1;
% change_counter = 0;
% fatigue_opt_constr_sqp = 10000000;
% fatigue_opt_constr_sqp_sphere = 1000000;
% tau_min_eff = 1000000000;
% tau_min_eff_sphere = 100000000;


disp('GLOBAL SEARCH ...')
% Search for a global optimum
for i=1:trials
    
    % random initial condition
    %q0 = rand(1,n_dofs) - 0.5;
%     fatigue0 = fatigue7DoFs(LWR,q0,f_ext,duration,capacity);

    % fatigue-based configuration with cartesian point constraint
    [q_opt_constr_sqp_tmp, fatigue_opt_constr_sqp_tmp] = fmincon(@(q)fatigue7DoFs(LWR,q,f_ext,duration,capacity),q0,A,b,Aeq,beq,q_lb,q_ub,cartPointCon,options_sqp);
    %if (fatigue_opt_constr_sqp_tmp < fatigue_opt_constr_sqp)
        fatigue_opt_constr_sqp = fatigue_opt_constr_sqp_tmp;
        q_opt_constr_sqp = q_opt_constr_sqp_tmp;
%         change_counter = change_counter + 1;
    %end
    
    % fatigue-based configuration with sphere point constraint
    [q_opt_constr_sqp_sphere_tmp, fatigue_opt_constr_sqp_sphere_tmp] = fmincon(@(q)fatigue7DoFs(LWR,q,f_ext,duration,capacity),q0,A,b,Aeq,beq,q_lb,q_ub,cartSphereCon,options_sqp);
    %if (fatigue_opt_constr_sqp_sphere_tmp < fatigue_opt_constr_sqp_sphere)
        fatigue_opt_constr_sqp_sphere = fatigue_opt_constr_sqp_sphere_tmp;
        q_opt_constr_sqp_sphere = q_opt_constr_sqp_sphere_tmp;
   % end
    
    % torque-based configuration with cartesian point constraint
    [q_min_eff_tmp, tau_min_eff_sum_tmp, tau_min_eff_tmp] = fmincon(@(q)torque7DoFs(LWR,q,f_ext),q0,A,b,Aeq,beq,q_lb,q_ub,cartPointCon,options_sqp);
    %if (tau_min_eff_tmp < tau_min_eff)
        tau_min_eff = tau_min_eff_tmp;
        tau_min_eff_sum = tau_min_eff_sum_tmp;
        q_min_eff = q_min_eff_tmp;
    %end
    
    % torque-based configuration with sphere point constraint
    [q_min_eff_sphere_tmp, tau_min_eff_sum_sphere_tmp, tau_min_eff_sphere_tmp] = fmincon(@(q)torque7DoFs(LWR,q,f_ext),q0,A,b,Aeq,beq,q_lb,q_ub,cartSphereCon,options_sqp);
    %if (tau_min_eff_sphere_tmp < tau_min_eff_sphere)
        tau_min_eff_sphere = tau_min_eff_sphere_tmp;
        tau_min_eff_sum_sphere = tau_min_eff_sum_sphere_tmp;
        q_min_eff_sphere = q_min_eff_sphere_tmp;
   % end
    
    disp(['Trial ' num2str(i) ' computed.'])
end

% radius_tmp = 0.01:0.02:0.2;
% 
% for i=1:trials
%     q0 = rand(1,n_dofs) - 0.5;
%     fatigue0 = fatigue7DoFs(LWR,q0,f_ext,duration,capacity);
%     [q_opt_constr_sqp_tmp, fatigue_opt_constr_sqp_tmp] = fmincon(@(q)fatigue7DoFs(LWR,q,f_ext,duration,capacity),q0,A,b,Aeq,beq,q_lb,q_ub,cartPointCon,options_sqp);
%     if (fatigue_opt_constr_sqp_tmp < fatigue_opt_constr_sqp)
%         fatigue_opt_constr_sqp = fatigue_opt_constr_sqp_tmp;
%         q_opt_constr_sqp = q_opt_constr_sqp_tmp;
%         change_counter = change_counter + 1;
%     end
%     for j=1:length(radius_tmp)
%        cartSphereCon = @(q) cartesianEESphere7DoFsConstraint(LWR,q,x_ee,radius_tmp(j));
%        [q_opt_constr_sqp_sphere_tmp, fatigue_opt_constr_sqp_sphere_tmp] = fmincon(@(q)fatigue7DoFs(LWR,q,f_ext,duration,capacity),q_opt_constr_sqp,A,b,Aeq,beq,q_lb,q_ub,cartSphereCon,options_sqp);
%        if (fatigue_opt_constr_sqp_sphere_tmp < fatigue_opt_constr_sqp_sphere)
%            fatigue_opt_constr_sqp_sphere = fatigue_opt_constr_sqp_sphere_tmp;
%            q_opt_constr_sqp_sphere = q_opt_constr_sqp_sphere_tmp;
%            radius_final = radius_tmp(j);
%        end
%     end
%     disp(['Trial ' num2str(i) ' computed.'])
% end

disp('DONE !')

% disp('LOCAL SEARCH ...')
% % Refined search for a local minimum from the previous
% [q_opt_constr_sqp_tmp, fatigue_opt_constr_sqp_tmp] = fmincon(@fatigue7DoFs,q_opt_constr_sqp,A,b,Aeq,beq,q_lb,q_ub,nonlcon,options_sqp);
% if (fatigue_opt_constr_sqp_tmp < fatigue_opt_constr_sqp)
%         diff = fatigue_opt_constr_sqp - fatigue_opt_constr_sqp_tmp;
%         disp(['Last search fatigue difference : ', num2str(diff)])
%         fatigue_opt_constr_sqp = fatigue_opt_constr_sqp_tmp;
%         q_opt_constr_sqp = q_opt_constr_sqp_tmp;
%         change_counter = change_counter + 100;
% else
%     disp('Already in a local minimum.')
% end
% disp('DONE !')

%% further computations
[tau0_sum, tau0] = torque7DoFs(LWR, q0, f_ext);
[fatigue0, fatigue0_vec] = fatigue7DoFs(LWR, q0, f_ext, duration, capacity);
x0_fkine = LWR.fkine(q0).t;
[~, c_constr0] = cartesianEE7DoFsConstraint(LWR, q0, x_ee);

x_opt_constr_sqp = LWR.fkine(q_opt_constr_sqp).t;
[~, c_constr_sqp] = cartesianEE7DoFsConstraint(LWR, q_opt_constr_sqp, x_ee);
[tau_opt_sum, tau_opt] = torque7DoFs(LWR, q_opt_constr_sqp, f_ext);
[~, fatigue_vec_opt] = fatigue7DoFs(LWR, q_opt_constr_sqp, f_ext, duration, capacity);

x_opt_constr_sqp_sphere = LWR.fkine(q_opt_constr_sqp_sphere).t;
[c_constr_sqp_sphere, ~] = cartesianEESphere7DoFsConstraint(LWR, q_opt_constr_sqp_sphere, x_ee, radius);
[tau_opt_sum_sphere, tau_opt_sphere] = torque7DoFs(LWR, q_opt_constr_sqp_sphere, f_ext);
[~, fatigue_vec_opt_sphere] = fatigue7DoFs(LWR, q_opt_constr_sqp_sphere, f_ext, duration, capacity);

x_min_eff = LWR.fkine(q_min_eff).t;
[~, c_min_eff] = cartesianEE7DoFsConstraint(LWR, q_min_eff, x_ee);
[fatigue_min_eff, fatigue_vec_min_eff] = fatigue7DoFs(LWR, q_min_eff, f_ext, duration, capacity);
[tau_min_eff_sum, tau_min_eff] = torque7DoFs(LWR, q_min_eff, f_ext);

x_min_eff_sphere = LWR.fkine(q_min_eff_sphere).t;
[c_min_eff_sphere, ~] = cartesianEESphere7DoFsConstraint(LWR, q_min_eff_sphere, x_ee, radius);
[fatigue_min_eff_sphere, fatigue_vec_min_eff_sphere] = fatigue7DoFs(LWR,q_min_eff_sphere,f_ext,duration,capacity);
[tau_min_eff_sum_sphere, tau_min_eff_sphere] = torque7DoFs(LWR, q_min_eff_sphere, f_ext);

%% Plots
subplot(2,3,[1 2 4 5]);
disp('SHOWING INITIAL CONFIGURATION')
LWR.plot(q0);
hold on
h = quiver3(x_ee(1), x_ee(2), x_ee(3), f_ext_scaled(1), f_ext_scaled(2), f_ext_scaled(3));

% subplot(2,3,3);
% fatigue_plot = bar3([fatigue0_vec]);%, fatigue_vec_opt, fatigue_vec_opt_sphere, fatigue_vec_min_eff, fatigue_vec_min_eff_sphere]);
% title('Fatigue of every configuration')

pause;

delete(h);
disp('SHOWING FATIGUE-BASED SQP CONFIGURATION WITH POINT CONSTRAINT')
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

delete(h);
%delete(s1);
disp('SHOWING TORQUE-BASED SQP CONFIGURATION WITH POINT CONSTRAINT')
LWR.plot(q_min_eff);
hold on
h = quiver3(x_min_eff(1), x_min_eff(2), x_min_eff(3), f_ext_scaled(1), f_ext_scaled(2), f_ext_scaled(3));
pause;

% [x, y, z] = sphere;
% s1 = mesh(radius*x+x_ee(1), radius*y+x_ee(2), radius*z+x_ee(3));
% alpha 0.5

disp('SHOWING SQP CONFIGURATION WITH SPHERE CONSTRAINT')
delete(h);
LWR.plot(q_min_eff_sphere);
h = quiver3(x_min_eff_sphere(1), x_min_eff_sphere(2), x_min_eff_sphere(3), f_ext_scaled(1), f_ext_scaled(2), f_ext_scaled(3));
pause;

% figure
subplot(2,3,3);
fatigue_plot = bar3([fatigue0_vec, fatigue_vec_opt, fatigue_vec_opt_sphere, fatigue_vec_min_eff, fatigue_vec_min_eff_sphere]);
title('Fatigue of every configuration')
%figure
subplot(2,3,6);
tau_plot = bar3(abs([tau0 ,tau_opt, tau_opt_sphere, tau_min_eff, tau_min_eff_sphere]));
title('Absolute value of torque of every configuration')

%% saving data
data_name = ['../data/7DoFs_opt_conf_', datestr(now,'yyyy_mm_dd_HH_MM_SS'), '.mat'];
save(data_name, 'LWR', 'f_ext', 'x_ee', 'radius', 'q_opt_constr_sqp', 'q_opt_constr_sqp_sphere', ...
'fatigue_opt_constr_sqp', 'fatigue_opt_constr_sqp_sphere', 'duration', 'capacity', ...
'q_min_eff', 'q_min_eff_sphere', 'fatigue_min_eff', 'fatigue_min_eff_sphere', ... 
'tau_min_eff', 'tau_min_eff_sphere','tau_opt_sum', 'tau_opt', 'tau_opt_sum_sphere', 'tau_opt_sphere', ...
'tau_min_eff_sum', 'tau_min_eff_sum_sphere', 'q0', 'fatigue0', 'tau0_sum', ... 
'fatigue0_vec', 'fatigue_vec_opt', 'fatigue_vec_opt_sphere', 'fatigue_vec_min_eff', 'fatigue_vec_min_eff_sphere',...
'tau0' ,'tau_opt', 'tau_opt_sphere', 'tau_min_eff', 'tau_min_eff_sphere');
disp('Results saved')

%% results
disp('----------------------------------RESULTS-------------------------------------')
disp(['Initial configuration: ', num2str(q0)]);
disp(['Fatigue: ' num2str(fatigue0) ]);
disp(['Sum of squared torques: ' num2str(tau0_sum) ]);
%disp(['Torques: ' num2str(tau_opt') ]);
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