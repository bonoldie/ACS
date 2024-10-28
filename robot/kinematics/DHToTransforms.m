function [out] = DHToTransforms(DH_table)
%DHTOTRANSFORMS Summary of this function goes here
%   Detailed explanation goes here
syms DH_a DH_alpha DH_d DH_theta;

% homogeneous transform following the DH convention
homogeneous_transform = [
    cos(DH_theta) -sin(DH_theta)*cos(DH_alpha)  sin(DH_theta)*sin(DH_alpha) DH_a*cos(DH_theta);
    sin(DH_theta)  cos(DH_theta)*cos(DH_alpha) -cos(DH_theta)*sin(DH_alpha) DH_a*sin(DH_theta);
    0              sin(DH_alpha)                cos(DH_alpha)               DH_d;
    0              0                            0                           1
];

% transformation matrices from the DH definition
T_dh = cell(1, size(DH_table, 2));

for i=1:size(DH_table,1)
    T_dh{i} = subs(homogeneous_transform, [DH_a DH_alpha DH_d DH_theta], DH_table(i, :));
end


out = T_dh;

end

