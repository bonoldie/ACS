function [inertia] = cylinderInertia(mass, a, b, h)
%CYLINDER_INERTIA Summary of this function goes here
%   Detailed explanation goes here
inertia = [
    0.5*mass*(a^2+b^2) 0.5*mass*(3*(a^2 + b^2)^2 + h^2) 0.5*mass*(3*(a^2 + b^2)^2 + h^2) 0 0 0
];
end

