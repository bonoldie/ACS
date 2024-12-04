function [evalStructure, torques] = evalRobot(roboticStructure, jointValues)
    evalStructure = roboticStructure;
    
    evalStructure.dynamics.B = subs(evalStructure.dynamics.B, evalStructure.jointsSymbol(:), jointValues(:));
    evalStructure.dynamics.C = subs(evalStructure.dynamics.C, evalStructure.jointsSymbol(:), jointValues(:));
    evalStructure.dynamics.G = subs(evalStructure.dynamics.G, evalStructure.jointsSymbol(:), jointValues(:));

    for i=1:size(evalStructure.T, 2)
        evalStructure.T{i} =  subs(evalStructure.T{i} , evalStructure.jointsSymbol(:), jointValues(:));
    end
    for i=1:size(evalStructure.T_b_i, 2)
        evalStructure.T_b_i{i} =  subs(evalStructure.T_b_i{i} , evalStructure.jointsSymbol(:), jointValues(:));
    end
    for i=1:size(evalStructure.CoM, 2)
        evalStructure.CoM{i} =  subs(evalStructure.CoM{i} , evalStructure.jointsSymbol(:), jointValues(:));
    end

    torques = subs(evalStructure.eq_motion, evalStructure.jointsSymbol(:), jointValues(:));

    evalStructure.energy.kinetic = double(subs(evalStructure.energy.kinetic, evalStructure.jointsSymbol(:), jointValues(:)));
    evalStructure.energy.potential = double(subs(evalStructure.energy.potential, evalStructure.jointsSymbol(:), jointValues(:)));

    
    evalStructure.J = double(subs(evalStructure.J, evalStructure.jointsSymbol(:), jointValues(:)));
end
