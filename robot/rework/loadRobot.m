function [roboticStructure] = loadRobot(transforms, framesName, jointsType, linksName, mass, MoI, CoM, jointsSymbol, g)
%LOADROBOT Loads the robot mechanical parameters into a structure 
%   --

roboticStructure = struct( ...
    'DOF',        size(jointsSymbol, 1), ...
    'linksName',  {linksName}, ... 
    'jointsSymbol', {jointsSymbol},... 
    'jointsType', {jointsType},...
    'framesName', {framesName}, ...
    'mass', {mass},...
    'MoI', {MoI},...
    'CoM', {CoM},...
    'T', {transforms},...
    'T_b_i', {cumulateTransforms(transforms)},...
    'g', {g}...
);

roboticStructure.dynamics = struct( ...
    'B', computeBMatrix(roboticStructure)...
);

roboticStructure.dynamics.C = computeCMatrix(roboticStructure);
roboticStructure.dynamics.G = computeGMatrix(roboticStructure);

roboticStructure.eq_motion = roboticStructure.dynamics.B*jointsSymbol(:,3) + roboticStructure.dynamics.C*jointsSymbol(:,2) + roboticStructure.dynamics.G;

roboticStructure.energy.kinetic = 0.5 * jointsSymbol(:, 2)' * roboticStructure.dynamics.B * jointsSymbol(:, 2);
roboticStructure.energy.potential = calculatePotentialEnergy(roboticStructure);

roboticStructure.J = geometricJac(roboticStructure);
[Ja, Ta] = analyticalJac(roboticStructure);
roboticStructure.Ja = Ja;

syms q_t(t) [roboticStructure.DOF 1];
syms t real;

q_t_eval = q_t(t);

roboticStructure.dJa = diff(subs(Ja, jointsSymbol(:, 1), q_t_eval), t);

roboticStructure.dJa = subs(roboticStructure.dJa, [diff(q_t_eval, t)], [jointsSymbol(:, 2)]);
roboticStructure.dJa = subs(roboticStructure.dJa, q_t_eval, jointsSymbol(:, 1));

roboticStructure.Ta = Ta;

roboticStructure.func.B = matlabFunction(roboticStructure.dynamics.B,Vars=roboticStructure.jointsSymbol(:, 1));
roboticStructure.func.C = matlabFunction(roboticStructure.dynamics.C,Vars=[roboticStructure.jointsSymbol(:, 1); roboticStructure.jointsSymbol(:, 2)]);
roboticStructure.func.G = matlabFunction(roboticStructure.dynamics.G,Vars=roboticStructure.jointsSymbol(:, 1));
roboticStructure.func.J = matlabFunction(roboticStructure.J,Vars=roboticStructure.jointsSymbol(:, 1));
roboticStructure.func.Ja = matlabFunction(roboticStructure.Ja,Vars=roboticStructure.jointsSymbol(:, 1));
roboticStructure.func.dJa = matlabFunction(roboticStructure.dJa,Vars=[roboticStructure.jointsSymbol(:, 1); roboticStructure.jointsSymbol(:, 2)]);
roboticStructure.func.Ta = matlabFunction(roboticStructure.Ta,Vars=roboticStructure.jointsSymbol(:, 1));

roboticStructure.func.T_b_ee = matlabFunction(roboticStructure.T_b_i{end},Vars=roboticStructure.jointsSymbol(:, 1));

% matlabFunction(roboticStructure.T_b_i{end}(1:3, 4),roboticStructure.jointsSymbol(:, 1));
% matlabFunction(rotm2eul(roboticStructure.T_b_i{end}(1:3, 1:3), "ZYZ"),roboticStructure.jointsSymbol(:, 1));
% roboticStructure.func.kinematics = @(q) [];
  
end
