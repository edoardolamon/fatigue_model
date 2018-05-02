% close all
% clear all

t = [0:.01:1]'; 

%% Model
l1 = Link('d', 0, 'a', 1, 'alpha', 0);
l2 = Link('d', 0, 'a', 1, 'alpha', 0);
l1.m = 1;
l2.m = 1;

planar_arm = SerialLink([l1 l2], 'name', '2 DoFs Planar Arm');
planar_arm.qlim =  [-3/4*pi*ones(2,1)  3/4*pi*ones(2,1)];

% initial configuration
% q0 = [0*pi 0*pi];
% q0 = [-pi/2 0];
q0 = [5/12*pi 1/5*pi];
% q0 = [2/12*pi 1/2*pi];
% q0 = [2/12*pi -1/6*pi];
disp("INITIAL CONFIGURATION")
planar_arm.plot(q0);
x0 = planar_arm.fkine(q0).t;

f_ext = zeros(6,1);
f_ext(1) = 1;
f_ext(2) = 0;

% Show the force vector
hold on
h = quiver3(x0(1), x0(2), x0(3), f_ext(1), f_ext(2), f_ext(3));
h.DisplayName = 'force';

%% Scalar fatigue model

% T = 1;
% capacity = ones(2,1);
fatigue0 = fatigue2DoFs(q0);

%% Optimization
pause; %(sec);
% h.Visible = 'off';
delete(h);
[q_opt, fatigue_opt] = fminsearch(@fatigue2DoFs,q0);
x_opt = planar_arm.fkine(q_opt).t;


sec = 5;
disp("OPTIMIZED CONFIGURATION")
traj = jtraj(q0, q_opt, t);

planar_arm.plot(q_opt);
h = quiver3(x_opt(1), x_opt(2), x_opt(3), f_ext(1), f_ext(2), f_ext(3));
%for (i=1;i<)
%planar_arm.plot(q_opt);

%% constrained optimization
% constraints
q_lb = -3/4*pi*ones(2,1);
q_ub = 3/4*pi*ones(2,1);
% q_lb = [];
% q_ub = [];
A = [];
b = [];
Aeq = [];
beq = [];
nonlcon = [];
% nonlcon = @cartesianEE2DoFsConstraint;
%options = optimset('PlotFcns',@optimplotfval);

% optimization with 'interior-set'
pause; %(sec);
delete(h);
[q_opt_constr, fatigue_opt_constr] = fmincon(@fatigue2DoFs,q0,A,b,Aeq,beq,q_lb,q_ub); %,nonlcon,options);
x_opt_constr = planar_arm.fkine(q_opt_constr).t;
[~, c_constr] = cartesianEE2DoFsConstraint(q_opt_constr);
disp("INTERIOR-SET CONFIGURATION")
planar_arm.plot(q_opt_constr);
h = quiver3(x_opt_constr(1), x_opt_constr(2), x_opt_constr(3), f_ext(1), f_ext(2), f_ext(3));

% optimization with 'sqp'
pause; %(sec);
delete(h);
options_sqp = optimoptions(@fmincon, 'Algorithm', 'sqp');
[q_opt_constr_sqp, fatigue_opt_constr_sqp] = fmincon(@fatigue2DoFs,q0,A,b,Aeq,beq,q_lb,q_ub,nonlcon,options_sqp);
x_opt_constr_sqp = planar_arm.fkine(q_opt_constr_sqp).t;
[~, c_constr_sqp] = cartesianEE2DoFsConstraint(q_opt_constr_sqp);
disp("SQP CONFIGURATION")
planar_arm.plot(q_opt_constr_sqp);
h = quiver3(x_opt_constr_sqp(1), x_opt_constr_sqp(2), x_opt_constr_sqp(3), f_ext(1), f_ext(2), f_ext(3));

% optimization with 'active-set'
pause; %(sec);
delete(h);
options_activeSet = optimoptions(@fmincon, 'Algorithm', 'active-set');
[q_opt_constr_activeSet, fatigue_opt_constr_activeSet] = fmincon(@fatigue2DoFs,q0,A,b,Aeq,beq,q_lb,q_ub,nonlcon,options_activeSet);
x_opt_constr_activeSet = planar_arm.fkine(q_opt_constr_activeSet).t;
[~, c_constr_activeSet] = cartesianEE2DoFsConstraint(q_opt_constr_activeSet);
disp("ACTIVE-SET CONFIGURATION")
planar_arm.plot(q_opt_constr_activeSet);
h = quiver3(x_opt_constr_activeSet(1), x_opt_constr_activeSet(2), x_opt_constr_activeSet(3), f_ext(1), f_ext(2), f_ext(3));

% optimization with 'trust-region-reflective'
% options_trf = optimoptions(@fmincon, 'Algorithm', 'trust-region-reflective');
% [q_opt_constr_trf, fatigue_opt_constr_trf] = fmincon(@fatigue2DoFs,q0,A,b,Aeq,beq,q_lb,q_ub,nonlcon,options_trf);
% x_opt_constr_trf = planar_arm.fkine(q_opt_constr_trf).t;
% pause; %(sec);
% disp("TRUST REGION REFLECTIVE CONFIGURATION")
% planar_arm.plot(q_opt_constr_trf);

pause;
delete(h);

%% Results 
disp(' ');
disp("------------------------------Results ----------------------------------------")
disp(["Initial conf:         " q0 "Fatigue:" fatigue0 "Cart pos:" x0']);
disp(["Opt conf:             " q_opt "Fatigue:" fatigue_opt "Cart pos:" x_opt']);
disp(["Opt conf interior-set:" q_opt_constr "Fatigue:" fatigue_opt_constr "Cart pos:" x_opt_constr' "Cart constr" c_constr]);
disp(["Opt conf sqp:         " q_opt_constr_sqp "Fatigue:" fatigue_opt_constr_sqp "Cart pos:" x_opt_constr_sqp' "Cart constr" c_constr_sqp]);
disp(["Opt conf active-set:  S" q_opt_constr_activeSet "Fatigue:" fatigue_opt_constr_activeSet "Cart pos:" x_opt_constr_activeSet' "Cart constr" c_constr_activeSet]);
% disp(["Opt costr trust-region-reflective:" q_opt_constr_trf "Fatigue:" fatigue_opt_constr_trf "Cart pos:" x_opt_constr_trf']);
