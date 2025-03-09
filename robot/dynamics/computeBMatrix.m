function [B] = computeBMatrix(roboticStructure)
    %CALCULATEBMATRIX Summary of this function goes here
    %   Detailed explanation goes here
    
B = sym(zeros(3));

jointsIndex = find(cell2mat(cellfun(@(x) ismember(x, {'Prismatic','Revolute'}), roboticStructure.jointsType, 'UniformOutput', 0)));

partialGeometricJs = partialGeometricJacobians(roboticStructure);

for i=1:roboticStructure.DOF
    partialGeometricJ = partialGeometricJs{i};
    Ri = roboticStructure.T_b_i{jointsIndex(i)}(1:3,1:3);

    B = B + (...
        roboticStructure.mass(jointsIndex(i)) * partialGeometricJ(1:3,:)' * partialGeometricJ(1:3,:) + ... 
        partialGeometricJ(4:6, :)' * Ri * roboticStructure.MoI{jointsIndex(i)} * Ri' * partialGeometricJ(4:6, :)...
    );
end

B = simplify(B);

% solve(simplify(eig(B)) > 0, [q3], 'ReturnConditions', true);

end