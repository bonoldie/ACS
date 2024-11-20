function [G] = calculateGMatrix(roboticStructure)
    %CALCULATEGMATRIX Summary of this function goes here
    %   Detailed explanation goes here
    
jointsIndex = find(cell2mat(cellfun(@(x) ismember(x, {'Prismatic','Revolute'}), roboticStructure.jointsType, 'UniformOutput', 0)));

G = sym(zeros(roboticStructure.DOF, roboticStructure.DOF));

partialGeometricJs = partialGJ(roboticStructure);

gVec = [0;0;-9.81];

for i=1:roboticStructure.DOF
    for j=1:roboticStructure.DOF
       G(i, j) = - roboticStructure.mass(jointsIndex(j)) * gVec' * partialGeometricJs{j}(1:3, i);
    end
end

G = simplify(sum(G, 2));
end