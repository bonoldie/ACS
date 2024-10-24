function [inertia] = prism_inertia(mass, a,b,c)
%PRISM_INERTIA Summary of this function goes here
%   Detailed explanation goes here
inertia = [
    (1/12)*mass*(b^2 + c^2) (1/12)*mass*(a^2 + c^2) (1/12)*mass*(a^2 + b^2) 0 0 0
];
end

