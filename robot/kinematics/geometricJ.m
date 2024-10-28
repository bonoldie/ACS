function [out] = geometricJ(kinematics, from, to)
%GEOMETRICJ Summary of this function goes here

% partial geometric Jacobian matrices
Jg = cell(1, DOF);

T_from_to = transformFromTo(kinematics, from, to);

for i = 1:DOF
    transformIndex = find(strcmp(transformsNames,int2str(i-1)));

    T_0_i_cumulative = cumulateTransforms({transforms{zeroTransformIndex:transformIndex}});

    d_0_i = T_0_i_cumulative{end}(1:3,4);

    Jg{i} = sym(zeros(6, DOF));

    for j=1:i
        if strcmp(jointTypes{j}, 'Revolute')
            Jg{i}(:, j) = [
                cross(T_0_i_cumulative{j}(1:3, 3), d_0_i - T_0_i_cumulative{j}(1:3, 4));
                T_0_i_cumulative{j}(1:3, 3);
            ];
        elseif strcmp(jointTypes{j}, 'Prismatic')
            Jg{i}(:, j) = [
                T_0_i_cumulative{j}(1:3, 3);    
                0;0;0;
            ];
        end 
    end
end



end

