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
capacity = ones(7,1);
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
%x_ee = [0.4; 0.3; 0.2];
x_ee = rand(3,1);
radius = 0.19;
% nonlcon = [];
cartPointCon = @(q) cartesianEE7DoFsConstraint(LWR,q,x_ee);
cartSphereCon = @(q) cartesianEESphere7DoFsConstraint(LWR,q,x_ee,radius);

% optimization with 'sqp'
options_sqp = optimoptions(@fmincon, 'Algorithm', 'sqp', 'Display', 'off');
trials = 10;
fatigue_opt_constr_sqp = 1000;
fatigue_opt_constr_sqp_sphere = 1000;
change_counter = 0;


disp('GLOBAL SEARCH ...')
% Search for a global optimum
for i=1:trials
    q0 = rand(1,n_dofs) - 0.5;
    fatigue0 = fatigue7DoFs(LWR,q0,f_ext,duration,capacity);
    [q_opt_constr_sqp_tmp, fatigue_opt_constr_sqp_tmp] = fmincon(@(q)fatigue7DoFs(LWR,q,f_ext,duration,capacity),q0,A,b,Aeq,beq,q_lb,q_ub,cartPointCon,options_sqp);
    if (fatigue_opt_constr_sqp_tmp < fatigue_opt_constr_sqp)
        fatigue_opt_constr_sqp = fatigue_opt_constr_sqp_tmp;
        q_opt_constr_sqp = q_opt_constr_sqp_tmp;
        change_counter = change_counter + 1;
    end
    [q_opt_constr_sqp_sphere_tmp, fatigue_opt_constr_sqp_sphere_tmp] = fmincon(@(q)fatigue7DoFs(LWR,q,f_ext,duration,capacity),q_opt_constr_sqp,A,b,Aeq,beq,q_lb,q_ub,cartSphereCon,options_sqp);
    if (fatigue_opt_constr_sqp_sphere_tmp < fatigue_opt_constr_sqp_sphere)
        fatigue_opt_constr_sqp_sphere = fatigue_opt_constr_sqp_sphere_tmp;
        q_opt_constr_sqp_sphere = q_opt_constr_sqp_sphere_tmp;
    end
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

x_opt_constr_sqp = LWR.fkine(q_opt_constr_sqp).t;
[~, c_constr_sqp] = cartesianEE7DoFsConstraint(LWR, q_opt_constr_sqp, x_ee);

x_opt_constr_sqp_sphere = LWR.fkine(q_opt_constr_sqp_sphere).t;
[c_constr_sqp_sphere, ~] = cartesianEESphere7DoFsConstraint(LWR, q_opt_constr_sqp_sphere, x_ee, radius);

%%
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

%% saving data
data_name = ['../data/7DoFs_opt_conf_', datestr(now,'yyyy_mm_dd_HH_MM_SS'), '.mat'];
save(data_name, 'LWR', 'f_ext', 'x_ee', 'radius', 'q_opt_constr_sqp', 'q_opt_constr_sqp_sphere', ...
'fatigue_opt_constr_sqp', 'fatigue_opt_constr_sqp_sphere', 'duration', 'capacity');

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