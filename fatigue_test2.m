close all
clear all
clc

%%

q = 0:0.01:10;
tau_f = 1.5 * q + 2;
tau_g = (q-4).^2 + 1;
tau = tau_f + tau_g;

figure 
plot(q, tau_f)
hold on
grid on
plot(q, tau_g)
plot(q, tau);
legend("torque force", "torque gravity", "total_torque")
title("Torques")

%%

f = fatigue(1,1,tau);

figure 
plot(tau, f);
hold on
grid on