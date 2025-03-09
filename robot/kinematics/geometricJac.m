function [J] = geometricJac(roboticStructure) 
    
    jointsIndex = find(cell2mat(cellfun(@(x) ismember(x, {'Prismatic','Revolute'}), roboticStructure.jointsType, 'UniformOutput', 0)));

    J = sym(zeros(6, roboticStructure.DOF));

    p_ee = roboticStructure.T_b_i{end}(1:3, 4);
    
    for i=1:roboticStructure.DOF
        z_j_minus_1 = roboticStructure.T_b_i{jointsIndex(i)-1}(1:3,3);
        p_j_minus_1 = roboticStructure.T_b_i{jointsIndex(i)-1}(1:3,4);
        
        if strcmp(roboticStructure.jointsType{jointsIndex(i)}, 'Revolute')
            J(:, i) = [
                cross(z_j_minus_1, p_ee - p_j_minus_1); % diff(p_ee, kinematics.joints.symbols(j)); % 
                z_j_minus_1
            ];
        elseif strcmp(roboticStructure.jointsType{jointsIndex(i)}, 'Prismatic')
            J(:, i) = [
                z_j_minus_1;
                0;0;0;
            ];
        end
    end

    J = simplify(J);
end