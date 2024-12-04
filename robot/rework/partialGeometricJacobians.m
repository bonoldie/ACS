function [out] = partialGeometricJacobians(roboticStructure)
    %partialGJ Summary of this function goes here
    %   Detailed explanation goes here

    jointsIndex = find(cell2mat(cellfun(@(x) ismember(x, {'Prismatic','Revolute'}), roboticStructure.jointsType, 'UniformOutput', 0)));

    out = cell(1, size(jointsIndex,2));
    
    for i=1:roboticStructure.DOF
        
        out{i} = sym(zeros(6, roboticStructure.DOF));
    
        p_li = roboticStructure.CoM{jointsIndex(i)};
        
        for j=1:i
            z_j_minus_1 = roboticStructure.T_b_i{jointsIndex(j)-1}(1:3,3);
            p_j_minus_1 = roboticStructure.T_b_i{jointsIndex(j)-1}(1:3,4);
            
            if strcmp(roboticStructure.jointsType{jointsIndex(j)}, 'Revolute')
                out{i}(:, j) = [
                    cross(z_j_minus_1, p_li - p_j_minus_1); % diff(p_li, kinematics.joints.symbols(j)); % 
                    z_j_minus_1
                ];
            elseif strcmp(roboticStructure.jointsType{jointsIndex(j)}, 'Prismatic')
                out{i}(:, j) = [
                    z_j_minus_1;
                    0;0;0;
                ];
            end
        end
    end

end