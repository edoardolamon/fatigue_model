close all
clear all

%% Model
l1 = Link('d', 0, 'a', 1, 'alpha', 0);
l2 = Link('d', 0, 'a', 1, 'alpha', 0);

planar_arm = SerialLink([l1 l2], 'name', '2 DoFs Planar Arm');

% initial configuration
q_1 = 5/12*pi; 
q0_2 = 1/5*pi;
q0 = [q_1 q0_2]; 
disp("INITIAL CONFIGURATION")
planar_arm.plot(q0);
x0 = planar_arm.fkine(q0).t;

% f_ext = zeros(6,1);
% f_ext(1) = 0;
% f_ext(2) = 1;


%% Scalar fatigue model

% T = 1;
% capacity = ones(2,1);
fatigue0 = scalarFatigue2DoFs(q0_2);

%% Optimization
pause; %(sec);
[q_opt_2, fatigue_opt] = fminsearch(@scalarFatigue2DoFs,q0_2);
q_opt = [q_1 q_opt_2];
x_opt = planar_arm.fkine(q_opt).t;

sec = 5;
disp("OPTIMIZED CONFIGURATION")
planar_arm.plot(q_opt);

%% constrained optimization
% constraints
q_lb = -3/4*pi*ones(2,1);
q_ub = 3/4*pi*ones(2,1);
A = [];
b = [];
Aeq = [];
beq = [];
nonlcon = [];
%options = optimset('PlotFcns',@optimplotfval);


% optimization with 'interior-set'
pause; %(sec);
[q_opt_constr_2, fatigue_opt_constr] = fmincon(@scalarFatigue2DoFs,q0_2,A,b,Aeq,beq,q_lb,q_ub) %,nonlcon,options);
q_opt_constr = [q_1 q_opt_constr_2];
x_opt_constr = planar_arm.fkine(q_opt_constr).t;
disp("INTERIOR-SET CONFIGURATION")
planar_arm.plot(q_opt_constr);

% optimization with 'sqp'
pause; %(sec);
options_sqp = optimoptions(@fmincon, 'Algorithm', 'sqp');
[q_opt_constr_sqp_2, fatigue_opt_constr_sqp] = fmincon(@scalarFatigue2DoFs,q0_2,A,b,Aeq,beq,q_lb,q_ub,nonlcon,options_sqp);
q_opt_constr_sqp = [q_1 q_opt_constr_sqp_2];
x_opt_constr_sqp = planar_arm.fkine(q_opt_constr_sqp).t;
disp("SQP CONFIGURATION")
planar_arm.plot(q_opt_constr_sqp);

% optimization with 'active-set'
pause; %(sec);
options_activeSet = optimoptions(@fmincon, 'Algorithm', 'active-set');
[q_opt_constr_activeSet_2, fatigue_opt_constr_activeSet] = fmincon(@scalarFatigue2DoFs,q0_2,A,b,Aeq,beq,q_lb,q_ub,nonlcon,options_activeSet);
q_opt_constr_activeSet = [q_1 q_opt_constr_activeSet_2];
x_opt_constr_activeSet = planar_arm.fkine(q_opt_constr_activeSet).t;
disp("ACTIVE-SET CONFIGURATION")
planar_arm.plot(q_opt_constr_activeSet);

%% Results 
disp("------------------------------Results ----------------------------------------")
disp(["Initial conf:         " q0 "Fatigue:" fatigue0 "Cart pos:" x0']);
disp(["Opt conf:             " q_opt "Fatigue:" fatigue_opt "Cart pos:" x_opt']);
disp(["Opt conf interior-set:" q_opt_constr "Fatigue:" fatigue_opt_constr "Cart pos:" x_opt_constr']);
disp(["Opt conf sqp:         " q_opt_constr_sqp "Fatigue:" fatigue_opt_constr_sqp "Cart pos:" x_opt_constr_sqp']);
disp(["Opt conf active-set:  " q_opt_constr_activeSet "Fatigue:" fatigue_opt_constr_activeSet "Cart pos:" x_opt_constr_activeSet']);
