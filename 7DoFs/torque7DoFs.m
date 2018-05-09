function [tau_tot, tau] = torque7DoFs(LWR, q, f_ext)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% Fatigue model
% fat_th = 1 * ones(7,1);
% fat = zeros(7,1);

% gravity
tau_grav = LWR.gravload(q)';
% tau_grav = zeros(7,1);
lambda = 1;

% torque
% tau = LWR.jacob0(q)'* f_ext;
% tau = tau_grav;
tau = LWR.jacob0(q)'* f_ext + lambda*tau_grav;
% fat_tot = 0;
% 
% for i=1:7
%     fat(i) = 1 - exp(-abs(tau(i)*duration/capacity(i)));
%     fat_tot = fat_tot + 1/2 *fat(i)^2;
%     if fat(i) > fat_th(i)
%         fat_tot = fat_tot + 999;
%     end
% end

tau_tot = 0.5*(tau')*(tau);

%f = 0.5*([f1;f2]'*[f1;f2]);
% if (f1 > f_th(1))
%     f = f + 999;
% elseif (f2 > f_th(2))
%     f = f + 999;
% end

%f_tot = 1/2*(f')*(f);

end

