function f = fatigue1(time,capacity,tau)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
f = (1 - exp(-(time*tau)/capacity));
end

