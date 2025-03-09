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

roboticStructure.energy.kinetic = simplify(0.5 * jointsSymbol(:, 2)' * roboticStructure.dynamics.B * jointsSymbol(:, 2));
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

% matlabFunction(roboticStructure.T_b_i{end}(1:3, 4),roboticStructure.jointsSymbol(:, 1));
% matlabFunction(rotm2eul(roboticStructure.T_b_i{end}(1:3, 1:3), "ZYZ"),roboticStructure.jointsSymbol(:, 1));
% roboticStructure.func.kinematics = @(q) [];
  
end
