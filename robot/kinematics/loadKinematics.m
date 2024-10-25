function [out] = loadKinematics(DH_table, EETransform, jointSymbols, jointTypes)
%LOADKINEMATICS Loads the structure of a robot from a DH table and a simple
%description to calculate its kinematics.
%   --

syms DH_a DH_alpha DH_d DH_theta;

% homogeneous transform following the DH convention
homogeneous_transform = [
    cos(DH_theta) -sin(DH_theta)*cos(DH_alpha)  sin(DH_theta)*sin(DH_alpha) DH_a*cos(DH_theta);
    sin(DH_theta)  cos(DH_theta)*cos(DH_alpha) -cos(DH_theta)*sin(DH_alpha) DH_a*sin(DH_theta);
    0              sin(DH_alpha)                cos(DH_alpha)               DH_d;
    0              0                            0                           1
];

DOF = size(jointSymbols, 1);

% transformation matrices from the DH definition
T_dh = cell(1, size(DH_table, 2));

for i=1:size(DH_table,1)
    T_dh{i} = subs(homogeneous_transform, [DH_a DH_alpha DH_d DH_theta], DH_table(i, :));
end

T = [T_dh, EETransform];

T_cumulative = cumulateTransforms(T);


% geometric Jacobian

Jg = sym(zeros(6, DOF));

d_b_ee = T_cumulative{end}(1:3,4);

for i = 1:DOF
    if strcmp(jointTypes{i}, 'Revolute')
        Jg(:, i) = [
            cross(T_cumulative{i}(1:3, 3), d_b_ee - T_cumulative{i}(1:3, 4));
            T_cumulative{i}(1:3, 3);
        ];
    elseif strcmp(jointTypes{i}, 'Prismatic')
        Jg(:, i) = [
            T_cumulative{i}(1:3, 3);    
            0;0;0;
        ];
    end 
end


% analytical jacobian 

phi_ZYZ = gradient(atan2(T_cumulative{end}(2,3), T_cumulative{end}(1,3)), jointSymbols);
theta_ZYZ = gradient(atan2(sqrt(T_cumulative{end}(1,3)^2 + T_cumulative{end}(2,3)^2), T_cumulative{end}(3,3)), jointSymbols);
psi_ZYZ =  gradient(atan2(T_cumulative{end}(3,2), -T_cumulative{end}(3,1)), jointSymbols);

Ja = [ 
    phi_ZYZ';
    theta_ZYZ';
    psi_ZYZ';
    gradient(T_cumulative{end}(1,4), jointSymbols)';
    gradient(T_cumulative{end}(2,4), jointSymbols)'; 
    gradient(T_cumulative{end}(3,4), jointSymbols)'; 
];

out = struct( ...
    'DOF',          DOF, ...
    'T',            {T}, ...
    'T_cumulative', {T_cumulative}, ...
    'Ja',           {Ja}, ...
    'Jg',           {Jg} ...
);

end

