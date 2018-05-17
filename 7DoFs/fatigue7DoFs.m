function [fat_tot, fat] = fatigue7DoFs(LWR, q, f_ext, duration, capacity, previous_fatigue)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% if nargin == 5
%     fat = zeros(7, 1);
% else
%     fat = previous_fatigue;
% end

if nargin == 5
    previous_fatigue = zeros(7, 1);
end


%% Fatigue model

% gravity compensation torques
tau_grav = LWR.gravload(q)';
% tau_grav = zeros(7,1);

% torque
% tau = LWR.jacob0(q)'* f_ext;
% tau = tau_grav;
lambda = 1;
tau = LWR.jacob0(q)'* f_ext + lambda*tau_grav;
% tau_tot = 0.5*(tau')*(tau);

% initial value
fat_tot = 0;
fat = zeros(7, 1);

% penalty 
% penalty = 999;
% fatigue threshold
% fat_th = 1 * ones(7,1);

% for i=1:7
%     fat(i) = fat(i) + 1 - exp(-abs(tau(i)*duration/capacity(i)));
%     fat_tot = fat_tot + 1/2 *fat(i)^2;
%     if fat(i) > fat_th(i)
%         fat_tot = fat_tot + penalty;
%     end
% end

eps = 0; %1e-3;
for i=1:7
    fat(i) = previous_fatigue(i) + (1 - previous_fatigue(i)) * (1 - exp(-abs(tau(i)*duration/capacity(i))));
    fat_tot = fat_tot - log(1 + eps - (fat(i))^2);
    % fat_tot = fat_tot + (1/(100*(1 - fat(i))));
    % fat_tot = fat_tot + 1/2 *(1/(1 - fat(i)));
end

%f = 0.5*([f1;f2]'*[f1;f2]);
% if (f1 > f_th(1))
%     f = f + 999;
% elseif (f2 > f_th(2))
%     f = f + 999;
% end

%f_tot = 1/2*(f')*(f);


end

