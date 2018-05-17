close all
clear all
clc

%% 

addpath('7DoFs')

syms tau;
TAU = sym('tau', [2 1]);
fatigue = 1 - exp(-abs(tau));
fatigue_vec = 1 - exp(-abs(TAU));
penalty_l2norm = 0.5*(fatigue_vec')*fatigue_vec;

eps = 0;%0.000001;
fatigue_log_a = (log(1+eps-fatigue))^2;
fatigue_log_b = -(log(1+eps-fatigue^2));

% ezplot(tau,fatigue);
% figure
% fplot(tau,fatigue^2);
% grid on
figure
subplot(1,2,1)
fplot(fatigue,fatigue_log_a)
subplot(1,2,2)
fplot(tau,fatigue_log_a, [-100 100])

figure
subplot(1,2,1)
fplot(fatigue,fatigue_log_b)
subplot(1,2,2)
fplot(tau,fatigue_log_b, [-100 100])


%%

% penalty_rational = 1/(1 - fatigue) * 1/(1 - fatigue);
% penalty_logarithmic = 1/log(fatigue) * 1/log(fatigue);

% subplot(2,2,1)
% fplot(fatigue, penalty_rational, [0 0.9])
% subplot(2,2,2) 
% fplot(tau, penalty_rational)
% 
% subplot(2,2,3)
% fplot(fatigue, penalty_logarithmic)
% subplot(2,2,4)
% fplot(tau, penalty_logarithmic)

penalty_rational = 0.5 * ( (1/(1 - fatigue_vec(1)))^2 + (1/(1 -fatigue_vec(2)))^2);
penalty_log_rational = 0.5 * ( (1/log(fatigue_vec(1)))^2 + (1/log(fatigue_vec(2)))^2);
penalty_logarithmic = -log(1-fatigue_vec(1)^2) - log(1-fatigue_vec(2)^2);

%% plots

tau_tmp = -5:0.1:5;
[tau_tmp1, tau_tmp2] = meshgrid(tau_tmp);
tau1 = tau_tmp1;
tau2 = tau_tmp2;

fatigue_tmp = 0:0.01:1;
[fatigue_tmp1, fatigue_tmp2] = meshgrid(fatigue_tmp);
% fatigue_vec(1) = fatigue_tmp1;
% fatigue_vec(2) = fatigue_tmp2;

figure
penalty_l2norm_eval = double(subs(penalty_l2norm));
subplot(1,2,1)
surf(fatigue_tmp1, fatigue_tmp2, penalty_l2norm_eval);
subplot(1,2,2)
surf(tau_tmp1, tau_tmp2, penalty_l2norm_eval);

figure
penalty_rational_eval = double(subs(penalty_rational));
subplot(1,2,1)
surf(fatigue_tmp1, fatigue_tmp2, penalty_rational_eval);
subplot(1,2,2)
surf(tau_tmp1, tau_tmp2, penalty_rational_eval);

figure
penalty_log_rational_eval = double(subs(penalty_log_rational));
subplot(1,2,1)
surf(fatigue_tmp1, fatigue_tmp2, penalty_log_rational_eval);
subplot(1,2,2)
surf(tau_tmp1, tau_tmp2, penalty_log_rational_eval);

figure
penalty_logarithmic_eval = double(subs(penalty_logarithmic));
subplot(1,2,1)
surf(fatigue_tmp1, fatigue_tmp2, penalty_logarithmic_eval);
subplot(1,2,2)
surf(tau_tmp1, tau_tmp2, penalty_logarithmic_eval);



