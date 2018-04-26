% close all
% clear all


if (exist('h','var'))
    delete(h)
end

%% Model
mdl_LWR
n_dofs = size(links,2);

% q0 = rand(1,n_dofs);
% q0 = [0 0 0 0 0 0 0];

% disp("INITIAL CONFIGURATION")
% LWR.plot(q0);

% affineFromBaseToEE = LWR.fkine(q0)
% x0 = affineFromBaseToEE.t;

f_ext = zeros(6,1);
f_ext(1) = 0;
f_ext(2) = 0;
f_ext(3) = 1;

f_ext_scaled = 0.4*f_ext;


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
% nonlcon = [];
nonlcon = @cartesianEE7DoFsConstraint;

% optimization with 'sqp'
options_sqp = optimoptions(@fmincon, 'Algorithm', 'sqp', 'Display', 'off');
trials = 10;
fatigue_opt_constr_sqp = 1000;

for (i=1:trials)
    q0 = rand(1,n_dofs) - 0.5;
    fatigue0 = fatigue7DoFs(q0);
    [q_opt_constr_sqp_tmp, fatigue_opt_constr_sqp_tmp] = fmincon(@fatigue7DoFs,q0,A,b,Aeq,beq,q_lb,q_ub,nonlcon,options_sqp);
    if (fatigue_opt_constr_sqp_tmp < fatigue_opt_constr_sqp)
        fatigue_opt_constr_sqp = fatigue_opt_constr_sqp_tmp;
        q_opt_constr_sqp = q_opt_constr_sqp_tmp;
    end
end

x_opt_constr_sqp = LWR.fkine(q_opt_constr_sqp).t;
[~, c_constr_sqp] = cartesianEE7DoFsConstraint(q_opt_constr_sqp);
disp("SQP CONFIGURATION")
LWR.plot(q_opt_constr_sqp);
hold on
h = quiver3(x_opt_constr_sqp(1), x_opt_constr_sqp(2), x_opt_constr_sqp(3), f_ext_scaled(1), f_ext_scaled(2), f_ext_scaled(3));

disp('----------------------------------RESULTS-------------------------------------')
disp(['Optimized sqp configuration: ', num2str(q_opt_constr_sqp)]);
disp(['Fatigue: ' num2str(fatigue_opt_constr_sqp) ]);
disp(['Cart pos: ' num2str(x_opt_constr_sqp')]);
disp(['Constraint value: ' num2str(c_constr_sqp')]);
disp('------------------------------------------------------------------------------')