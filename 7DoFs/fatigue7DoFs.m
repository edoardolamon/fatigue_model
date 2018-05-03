function f_tot = fatigue7DoFs(q)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% Model
mdl_kukaLWR
for i=1:7
    LWR.links(i).m = 2.0;
    LWR.links(i).Jm = 0;
    LWR.links(i).G = 1;
end

% External Force
f_ext = zeros(6,1);
f_ext(1) = 0;
f_ext(2) = 0;
f_ext(3) = 1;

% Fatigue model
T = 1;
capacity = ones(7,1);
f_th = 1 * ones(7,1);
f = zeros(7,1);

% gravity
% p1 = [planar_arm.a(1)*cos(q(1)); planar_arm.a(1)*sin(q(1))]; 
% p2 = [planar_arm.a(1)*cos(q(1)) + planar_arm.a(2)*cos(q(1)+q(2)); planar_arm.a(1)*sin(q(1)) + planar_arm.a(2)*sin(q(1) + q(2))];
% g = [0; 9.81; 0]; 
% link_mass = [1 1];
% tau_grav = [link_mass(1)*g'*p1; link_mass(2)*g'*p2];
tau_grav = LWR.gravload(q)';
% tau_grav = zeros(7,1);
% 
% for (i=1:7)
%     
% end
lambda = 0.1;

%tau = LWR.jacob0(q)'* f_ext;
% tau = tau_grav;
tau = LWR.jacob0(q)'* f_ext + lambda*tau_grav;
f_tot = 0;

for i=1:7
    f(i) = 1 - exp(-abs(tau(i)*T/capacity(i)));
    f_tot = f_tot + 1/2 *f(i)^2;
    if f(i) > f_th(i)
        f_tot = f_tot + 999;
    end
end

%f = 0.5*([f1;f2]'*[f1;f2]);
% if (f1 > f_th(1))
%     f = f + 999;
% elseif (f2 > f_th(2))
%     f = f + 999;
% end

%f_tot = 1/2*(f')*(f);


end

