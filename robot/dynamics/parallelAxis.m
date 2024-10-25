function [I] = parallelAxis(Ic, m, r)
%PARALLELAXIS Summary of this function goes here
I = Ic + m*(r'*r*eye(3) + r*r');
end