close all
clear all
clc

%%
% time = 0:0.01:100';
% tau_one = ones(size(time));
% % tau_sin = sin(time);
% fatigue_one = zeros(size(time));
% fatigue_sin = zeros(size(time));
% sum_tau_sin = zeros(size(time));
% sum_tau_one = zeros(size(time));

%% FIRST TRIAL
% for i=2:size(time,2)
%     sum_tau_one(i) = sum(tau_one(1:i));
%     sum_tau_sin(i) = sum(abs(tau_sin(1:i)));
%     f1(i) = fatigue(time(i),100,sum_tau_one(i));
%     f2(i) = fatigue(time(i),100,sum_tau_sin(i));
%     instant_fatigue_one = fatigue(time(i),1,tau_one(i));
%     instant_fatigue_sin = fatigue(time(i),1,tau_sin(i));
%     fatigue_one(i) = fatigue_one(i-1) + instant_fatigue_one;
%     fatigue_sin(i) = fatigue_sin(i-1) + instant_fatigue_sin;
% end

% figure 
% plot(time, sum_tau_one)
% hold on
% grid on
% plot(time, sum_tau_sin)
% legend("sum torque one", "sum torque sin")
% title("Sum of torques")

% figure 
% plot(time, fatigue_one)
% hold on
% grid on
% plot(time, fatigue_sin)
% legend("torque one", "torque sin")
% title("Fatigue")

%% SECOND TRIAL

% fatigue_2 =  fatigue2(time,1);
% 
% figure 
% plot(time, fatigue_2)
% hold on
% grid on
% legend("second trial")
% title("Fatigue")
