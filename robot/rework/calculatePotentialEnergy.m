function [potentialEnergyEquation] = calculatePotentialEnergy(roboticStructure)
    %CALCULATEPOTENTIALENERGY Summary of this function goes here
    %   Detailed explanation goes here

    potentialEnergyEquation = 0;

    jointsIndex = find(cell2mat(cellfun(@(x) ismember(x, {'Prismatic','Revolute'}), roboticStructure.jointsType, 'UniformOutput', 0)));

    for i=1:roboticStructure.DOF
        potentialEnergyEquation = potentialEnergyEquation - roboticStructure.mass(jointsIndex(i)) * roboticStructure.g' * roboticStructure.CoM{jointsIndex(i)} ;
    end
end