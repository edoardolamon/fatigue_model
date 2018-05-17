close all
clear all

%% Model

mdl_kukaLWR

%first task
f_ext = zeros(6,1);
f_ext(1) = 40;
f_ext(2) = 0;
f_ext(3) = 0;
f_ext_scaled = 0.4/norm(f_ext)*f_ext;

duration = 1;
capacity = 10 * ones(n_dofs,1);
% capacity = [10, 10, 10, 5, 1, 1, 1];
% capacity = [1, 1, 1, 1, 10, 10, 10];

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
q1 = (rand(1, n_dofs) - 1.5);
x_ee = LWR.fkine(q1).t;
% x_ee = [0.4; 0.3; 0.2];
radius = 0.19;
% nonlcon = [];
cartPointCon = @(q) cartesianEE7DoFsConstraint(LWR,q,x_ee);
cartSphereCon = @(q) cartesianEESphere7DoFsConstraint(LWR,q,x_ee,radius);

% optimization with 'sqp'
options_sqp = optimoptions(@fmincon, 'Algorithm', 'sqp', 'Display', 'off');
trials = 1;
change_counter = 0;
fatigue_opt_constr_sqp = 1000;
fatigue_opt_constr_sqp_sphere = 1000;
tau_min_eff = 10000;
tau_min_eff_sphere = 10000;

disp('FATIGUE-BASED OPTIMIZATION of TASK 1')
% fatigue-based configuration with cartesian point constraint
[q_opt_task1, fatigue_opt_task1] = fmincon(@(q)fatigue7DoFs(LWR,q,f_ext,duration,capacity),q1,A,b,Aeq,beq,q_lb,q_ub,cartPointCon,options_sqp);
%if (fatigue_opt_constr_sqp_tmp < fatigue_opt_constr_sqp)
%     fatigue_opt_task1 = fatigue_opt_constr_sqp_tmp;
%     q_opt_task1 = q_opt_constr_sqp_tmp;
%end

%% further computations

[tau1_sum, tau0] = torque7DoFs(LWR, q1, f_ext);
[fatigue1, fatigue1_vec] = fatigue7DoFs(LWR, q1, f_ext, duration, capacity);
x1_fkine = LWR.fkine(q1).t;
[~, c_constr1] = cartesianEE7DoFsConstraint(LWR, q1, x_ee);

x_opt_task1 = LWR.fkine(q_opt_task1).t;
[~, c_task1] = cartesianEE7DoFsConstraint(LWR, q_opt_task1, x_ee);
[tau_opt_sum_task1, tau_opt_task1] = torque7DoFs(LWR, q_opt_task1, f_ext);
[~, fatigue_vec_opt_task1] = fatigue7DoFs(LWR, q_opt_task1, f_ext, duration, capacity);

% x_opt_constr_sqp_sphere = LWR.fkine(q_opt_constr_sqp_sphere).t;
% [c_constr_sqp_sphere, ~] = cartesianEESphere7DoFsConstraint(LWR, q_opt_constr_sqp_sphere, x_ee, radius);
% [tau_opt_sum_sphere, tau_opt_sphere] = torque7DoFs(LWR, q_opt_constr_sqp_sphere, f_ext);
% [~, fatigue_vec_opt_sphere] = fatigue7DoFs(LWR, q_opt_constr_sqp_sphere, f_ext, duration, capacity);

%% Second task

%first task
f_ext = zeros(6,1);
f_ext(1) = 0;
f_ext(2) = 40;
f_ext(3) = 0;
f_ext_scaled = 0.4/norm(f_ext)*f_ext;

q2 = (rand(1, n_dofs) - 1.5);
x_ee = LWR.fkine(q2).t;
cartPointCon = @(q) cartesianEE7DoFsConstraint(LWR,q,x_ee);

% fatigue-based configuration with cartesian point constraint
disp('FATIGUE-BASED OPTIMIZATION of TASK 2')
[q_opt_constr_sqp_tmp, fatigue_opt_constr_sqp_tmp] = fmincon(@(q)fatigue7DoFs(LWR,q,f_ext,duration,capacity),q2,A,b,Aeq,beq,q_lb,q_ub,cartPointCon,options_sqp);
%if (fatigue_opt_constr_sqp_tmp < fatigue_opt_constr_sqp)
    fatigue_opt_task2 = fatigue_opt_constr_sqp_tmp;
    q_opt_task2 = q_opt_constr_sqp_tmp;
%end

% fatigue-based configuration with cartesian point constraint considering
% the previous fatigue
disp('FATIGUE-BASED OPTIMIZATION of TASK 2 (considering fatigue on TASK 1)')
[q_opt_constr_sqp_tmp, fatigue_opt_constr_sqp_tmp] = fmincon(@(q)fatigue7DoFs(LWR,q,f_ext,duration,capacity,fatigue_vec_opt_task1),q2,A,b,Aeq,beq,q_lb,q_ub,cartPointCon,options_sqp);
% if (fatigue_opt_constr_sqp_tmp < fatigue_opt_constr_sqp)
    fatigue_opt_task2_stack = fatigue_opt_constr_sqp_tmp;
    q_opt_task2_stack = q_opt_constr_sqp_tmp;
% end

[tau2_sum, tau2] = torque7DoFs(LWR, q2, f_ext);
[fatigue2, fatigue2_vec] = fatigue7DoFs(LWR, q2, f_ext, duration, capacity);
x2_fkine = LWR.fkine(q2).t;
[~, c_constr2] = cartesianEE7DoFsConstraint(LWR, q2, x_ee);

x_opt_task2 = LWR.fkine(q_opt_task2).t;
[~, c_task2] = cartesianEE7DoFsConstraint(LWR, q_opt_task2, x_ee);
[tau_opt_sum_task2, tau_opt_task2] = torque7DoFs(LWR, q_opt_task2, f_ext);
[~, fatigue_vec_opt_task2] = fatigue7DoFs(LWR, q_opt_task2, f_ext, duration, capacity);

x_opt_task2_stack = LWR.fkine(q_opt_task2_stack).t;
[~, c_task2_stack] = cartesianEE7DoFsConstraint(LWR, q_opt_task2_stack, x_ee);
[tau_opt_sum_task2_stack, tau_opt_task2_stack] = torque7DoFs(LWR, q_opt_task2_stack, f_ext);
[~, fatigue_vec_opt_task2_stack] = fatigue7DoFs(LWR, q_opt_task2_stack, f_ext, duration, capacity, fatigue_vec_opt_task1);

%% Plot

subplot(2,3,1)
bar3([fatigue1_vec, fatigue_vec_opt_task1], 'grouped');
title('Fatigue of task 1')
subplot(2,3,2)
bar3([fatigue2_vec, fatigue_vec_opt_task2], 'grouped');
title('Fatigue of task 2')
subplot(2,3,3)
bar3([fatigue_vec_opt_task2, fatigue_vec_opt_task2_stack], 'grouped');
title('Comparison between optimization of task 2 with different initial fatigue condition')
subplot(2,3,4)
bar3([fatigue_vec_opt_task1, fatigue_vec_opt_task2], 'grouped');
title('Comparison between task 1 and 2')
subplot(2,3,5)
bar3([fatigue_vec_opt_task1, fatigue_vec_opt_task2_stack ], 'grouped');
title('Comparison between task 1 and 2_stacked')
subplot(2,3,6)
bar3([fatigue_vec_opt_task1, (fatigue_vec_opt_task2_stack - fatigue_vec_opt_task1) ], 'grouped');
title('Comparison between task 1 and relative 2_stacked')

%% results
disp('----------------------------------RESULTS-------------------------------------')
disp(['Initial configuration: ', num2str(q1)]);
disp(['Fatigue: ' num2str(fatigue1) ]);
disp(['Sum of squared torques: ' num2str(tau1_sum) ]);
%disp(['Torques: ' num2str(tau_opt') ]);
disp(['Cart pos: ' num2str(x1_fkine')]);
disp(['Constraint value: ' num2str(c_constr1')]);
disp(' ');
disp(['Fatigue-based optimal configuration : ', num2str(q_opt_task1)]);
disp(['Fatigue: ' num2str(fatigue_opt_task1) ]);
disp(['Sum of squared torques: ' num2str(tau_opt_sum_task2) ]);
%disp(['Torques: ' num2str(tau_opt') ]);
disp(['Cart pos: ' num2str(x_opt_task1')]);
disp(['Constraint value: ' num2str(c_task1')]);
disp(' ');
disp(['Second configuration: ', num2str(q2)]);
disp(['Fatigue: ' num2str(fatigue2) ]);
disp(['Sum of squared torques: ' num2str(tau2_sum) ]);
%disp(['Torques: ' num2str(tau_opt') ]);
disp(['Cart pos: ' num2str(x2_fkine')]);
disp(['Constraint value: ' num2str(c_constr2')]);
disp(' ');
disp(['Fatigue-based second optimal configuration: ', num2str(q_opt_task2)]);
disp(['Fatigue: ' num2str(fatigue_opt_task2) ]);
disp(['Sum of squared torques: ' num2str(tau_opt_sum_task2) ]);
%disp(['Torques: ' num2str(tau_opt') ]);
disp(['Cart pos: ' num2str(x_opt_task2')]);
disp(['Constraint value: ' num2str(c_task2')]);
disp(' ');
disp(['Fatigue-based second optimal configuration (task stacked): ', num2str(q_opt_task2_stack)]);
disp(['Fatigue: ' num2str(fatigue_opt_task2_stack) ]);
disp(['Sum of squared torques: ' num2str(tau_opt_sum_task2_stack) ]);
%disp(['Torques: ' num2str(tau_opt') ]);
disp(['Cart pos: ' num2str(x_opt_task2_stack')]);
disp(['Constraint value: ' num2str(c_task2_stack')]);
