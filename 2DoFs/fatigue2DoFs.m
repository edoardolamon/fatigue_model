function f = fatigue2DoFs(q)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% Model
l1 = Link('d', 0, 'a', 1, 'alpha', 0);
l2 = Link('d', 0, 'a', 1, 'alpha', 0);
l1.m = 1;
l2.m = 1;
planar_arm = SerialLink([l1 l2], 'name', '2 DoFs Planar Arm');
planar_arm.qlim =  [-3/4*pi*ones(2,1)  3/4*pi*ones(2,1)];


% External Force
f_ext = zeros(6,1);
f_ext(1) = 1;
f_ext(2) = 0;

% Fatigue model
T = 1;
capacity = ones(2,1);
f_th = 0.5 * ones(2,1);

% gravity
% p1 = [planar_arm.a(1)*cos(q(1)); planar_arm.a(1)*sin(q(1))]; 
% p2 = [planar_arm.a(1)*cos(q(1)) + planar_arm.a(2)*cos(q(1)+q(2)); planar_arm.a(1)*sin(q(1)) + planar_arm.a(2)*sin(q(1) + q(2))];
g = [0; 9.81; 0]; 
% link_mass = [1 1];
% tau_grav = [link_mass(1)*g'*p1; link_mass(2)*g'*p2];
tau_grav = planar_arm.gravload(q,g)';
lambda = 0.1;

% tau = planar_arm.jacob0(q)'* f_ext;
% tau = tau_grav;
tau = planar_arm.jacob0(q)'* f_ext + lambda*tau_grav;
f1 = 1 - exp(-abs(tau(1)*T/capacity(1)));
f2 = 1 - exp(-abs(tau(2)*T/capacity(2)));


f = 0.5*([f1;f2]'*[f1;f2]);
% if (f1 > f_th(1))
%     f = f + 999;
% elseif (f2 > f_th(2))
%     f = f + 999;
% end
      
end

