function f = scalarFatigue2DoFs(q2)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% Model
l1 = Link('d', 0, 'a', 1, 'alpha', 0);
l2 = Link('d', 0, 'a', 1, 'alpha', 0);
planar_arm = SerialLink([l1 l2], 'name', '2 DoFs Planar Arm');

% External Force
f_ext = zeros(6,1);
f_ext(1) = 0;
f_ext(2) = 1;

% Joint configuration (q1 is FIXED)
q1 = 5/12*pi; 
q = [q1 q2];

% Scalar fatigue model
T = 1;
capacity = ones(2,1);
f_th = 1 * ones(2,1);

tau = planar_arm.jacob0(q)'* f_ext;
f1 = 1 - exp(-abs(tau(1))*T/capacity(1));
f2 = 1 - exp(-abs(tau(2))*T/capacity(2));


f = 0.5*([f1;f2]'*[f1;f2]);
% if (f1 > f_th(1))
%     f = f + 999;
% elseif (f2 > f_th(2))
%     f = f + 999;
% end
      
end

