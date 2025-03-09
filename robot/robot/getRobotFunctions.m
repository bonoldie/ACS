function [functions] = getRobotFunctions(roboticStructure)

functions.B = matlabFunction(roboticStructure.dynamics.B,Vars=roboticStructure.jointsSymbol(:, 1));
functions.C = matlabFunction(roboticStructure.dynamics.C,Vars=[roboticStructure.jointsSymbol(:, 1); roboticStructure.jointsSymbol(:, 2)]);
functions.G = matlabFunction(roboticStructure.dynamics.G,Vars=roboticStructure.jointsSymbol(:, 1));
functions.J = matlabFunction(roboticStructure.J,Vars=roboticStructure.jointsSymbol(:, 1));
functions.Ja = matlabFunction(roboticStructure.Ja,Vars=roboticStructure.jointsSymbol(:, 1));
functions.dJa = matlabFunction(roboticStructure.dJa,Vars=[roboticStructure.jointsSymbol(:, 1); roboticStructure.jointsSymbol(:, 2)]);
functions.Ta = matlabFunction(roboticStructure.Ta,Vars=roboticStructure.jointsSymbol(:, 1));
functions.T_b_ee = matlabFunction(roboticStructure.T_b_i{end},Vars=roboticStructure.jointsSymbol(:, 1));

functions.tau = matlabFunction(roboticStructure.dynamics.B*roboticStructure.jointsSymbol(:, 3) + roboticStructure.dynamics.C*roboticStructure.jointsSymbol(:, 2) + roboticStructure.dynamics.G, Vars=[roboticStructure.jointsSymbol(:, 1); roboticStructure.jointsSymbol(:, 2); roboticStructure.jointsSymbol(:, 3)]);

end