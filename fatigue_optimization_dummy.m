close all
clear all
clc

%% unconstrained optimization
% objective function
Kg = -4;
Kf = 1.5;
K0 = 3;

capacity = 1;
T = 10;
fatigue = @(q)(1 - exp(-(T/capacity)*((q+Kg)^2) + Kf*q + K0));

% initial condition
q0 = 0;

% optimization
[q_opt, fatigue_opt] = fminsearch(fatigue,q0);

%% constrained optimization
% constraints
lb = 0.2;
ub = 0.5;
A = [];
b = [];
Aeq = [];
beq = [];

% optimization
%q_opt_constr = fmincon(fatigue,q0,A,b,Aeq,beq,f_lb,f_ub);

