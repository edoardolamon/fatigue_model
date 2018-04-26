function f = fatigue2(time, capacity)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
f = zeros(size(time,2));
df = exp(-time/capacity);
for i=1:size(time,2)
    f(i) = sum(df(1:i));
end

end

