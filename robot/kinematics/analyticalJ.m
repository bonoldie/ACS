function [out] = analyticalJ(kinematics)
%ANALYTICALJ Summary of this function goes here
%   Detailed explanation goes here

% analytical jacobian 
T_cumulative = cumulateTransforms(kinematics.T);

phi_ZYZ = gradient(atan2(T_cumulative{end}(2,3), T_cumulative{end}(1,3)), jointSymbols);
theta_ZYZ = gradient(atan2(sqrt(T_cumulative{end}(1,3)^2 + T_cumulative{end}(2,3)^2), T_cumulative{end}(3,3)), jointSymbols);
psi_ZYZ =  gradient(atan2(T_cumulative{end}(3,2), -T_cumulative{end}(3,1)), jointSymbols);

out = [ 
    phi_ZYZ';
    theta_ZYZ';
    psi_ZYZ';
    gradient(T_cumulative{end}(1,4), jointSymbols)';
    gradient(T_cumulative{end}(2,4), jointSymbols)'; 
    gradient(T_cumulative{end}(3,4), jointSymbols)'; 
];

end

