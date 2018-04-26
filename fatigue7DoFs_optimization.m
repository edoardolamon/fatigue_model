% close all
% clear all

t = [0:.01:1]';

%% Model
mdl_LWR
n_dofs = size(links,2);

% initial configuration
affineFromBaseToEE = LWR.fkine(qz);
xz = affineFromBaseToEE.t;
x0 = [0.4; 0.6; -0.2];
affineFromBaseToEE.t = x0;
q0 = LWR.ikcon(affineFromBaseToEE);
% q0 = rand(1,n_dofs);
% q0 = [0 0 0 0 0 0 0];

disp("INITIAL CONFIGURATION")
LWR.plot(q0);

% affineFromBaseToEE = LWR.fkine(q0)
% x0 = affineFromBaseToEE.t;

f_ext = zeros(6,1);
f_ext(1) = 1;
f_ext(2) = 0;
f_ext(3) = 0;

f_ext_scaled = 0.4*f_ext;

% f_ext_ee = [affineFromBaseToEE.R zeros(3); zeros(3) affineFromBaseToEE.R]*f_ext;

% Show the force vector
hold on
h = quiver3(x0(1), x0(2), x0(3), f_ext_scaled(1), f_ext_scaled(2), f_ext_scaled(3));
% h.DisplayName = 'force';

%% Scalar fatigue model

fatigue0 = fatigue7DoFs(q0);

%% Optimization
pause; %(sec);
% h.Visible = 'off';
delete(h);
[q_opt, fatigue_opt] = fminsearch(@fatigue7DoFs,q0);
affineFromBaseToEE = LWR.fkine(q_opt);
x_opt = affineFromBaseToEE.t;

disp("OPTIMIZED CONFIGURATION")
traj = jtraj(q0, q_opt, t);

LWR.plot(q_opt);
hold on
h = quiver3(x_opt(1), x_opt(2), x_opt(3), f_ext_scaled(1), f_ext_scaled(2), f_ext_scaled(3));

%% constrained optimization
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
%options = optimset('PlotFcns',@optimplotfval);

% optimization with 'interior-set'
pause; %(sec);
delete(h);
[q_opt_constr, fatigue_opt_constr] = fmincon(@fatigue7DoFs,q0,A,b,Aeq,beq,q_lb,q_ub); %,nonlcon,options);
x_opt_constr = LWR.fkine(q_opt_constr).t;
[~, c_constr] = cartesianEE7DoFsConstraint(q_opt_constr);
disp("INTERIOR-POINT CONFIGURATION")
LWR.plot(q_opt_constr);
h = quiver3(x_opt_constr(1), x_opt_constr(2), x_opt_constr(3), f_ext_scaled(1), f_ext_scaled(2), f_ext_scaled(3));

% optimization with 'sqp'
pause; %(sec);
delete(h);
options_sqp = optimoptions(@fmincon, 'Algorithm', 'sqp', 'MaxFunctionEvaluations', 1000);
[q_opt_constr_sqp, fatigue_opt_constr_sqp] = fmincon(@fatigue7DoFs,q0,A,b,Aeq,beq,q_lb,q_ub,nonlcon,options_sqp);
x_opt_constr_sqp = LWR.fkine(q_opt_constr_sqp).t;
[~, c_constr_sqp] = cartesianEE7DoFsConstraint(q_opt_constr_sqp);
disp("SQP CONFIGURATION")
LWR.plot(q_opt_constr_sqp);
h = quiver3(x_opt_constr_sqp(1), x_opt_constr_sqp(2), x_opt_constr_sqp(3), f_ext_scaled(1), f_ext_scaled(2), f_ext_scaled(3));

% optimization with 'active-set'
pause; %(sec);
delete(h);
options_activeSet = optimoptions(@fmincon, 'Algorithm', 'active-set');
[q_opt_constr_activeSet, fatigue_opt_constr_activeSet] = fmincon(@fatigue7DoFs,q0,A,b,Aeq,beq,q_lb,q_ub,nonlcon,options_activeSet);
x_opt_constr_activeSet = LWR.fkine(q_opt_constr_activeSet).t;
[~, c_constr_activeSet] = cartesianEE7DoFsConstraint(q_opt_constr_activeSet);
disp("ACTIVE-SET CONFIGURATION")
LWR.plot(q_opt_constr_activeSet);
h = quiver3(x_opt_constr_activeSet(1), x_opt_constr_activeSet(2), x_opt_constr_activeSet(3), f_ext_scaled(1), f_ext_scaled(2), f_ext_scaled(3));

pause;
delete(h);

%% Results 
disp(' ');
disp('------------------------------Results ----------------------------------------')
disp(['Initial configuration:                  ', num2str(q0)]);
disp(['Fatigue: ' num2str(fatigue0) ]);
disp(['Cart pos: ' num2str(x0')]);
disp('------------------------------------------------------------------------------')
disp(['Optimized nonconstrained configuration: ', num2str(q_opt)]);
disp(['Fatigue: ' num2str(fatigue_opt) ]);
disp(['Cart pos: ' num2str(x_opt')]);
disp('------------------------------------------------------------------------------')
disp(['Optimized interior-point configuration: ', num2str(q_opt_constr)]);
disp(['Fatigue: ' num2str(fatigue_opt_constr) ]);
disp(['Cart pos: ' num2str(x_opt_constr')]);
disp(['Constraint value: ' num2str(c_constr')]);
disp('------------------------------------------------------------------------------')
disp(['Optimized sqp configuration:            ', num2str(q_opt_constr_sqp)]);
disp(['Fatigue: ' num2str(fatigue_opt_constr_sqp) ]);
disp(['Cart pos: ' num2str(x_opt_constr_sqp')]);
disp(['Constraint value: ' num2str(c_constr_sqp')]);
disp('------------------------------------------------------------------------------')
disp(['Optimized active-set configuration:     ', num2str(q_opt_constr_activeSet)]);
disp(['Fatigue: ' num2str(fatigue_opt_constr_activeSet) ]);
disp(['Cart pos: ' num2str(x_opt_constr_activeSet')]);
disp(['Constraint value: ' num2str(c_constr_activeSet')]);
disp('------------------------------------------------------------------------------')
