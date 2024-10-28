function [out] = evaluateKinematics(kinematics, jointValues)
%EVALUATEKINEMATICS evaluate the kinematics for ea
%   Detailed explanation goes here

out = kinematics;

for i=1:size(out.T,2)
    out.T{i} = double(subs(out.T{i}, kinematics.joints.symbols', jointValues'));
    % out.T_cumulative{i} = double(subs(out.T_cumulative{i}, jointSymbols', jointValues'));
end

% for i=1:out.DOF
    % out.Jg{i} = double(subs(out.Jg{i}, jointSymbols', jointValues'));
% end

% out.Ja = double(subs(out.Ja, jointSymbols', jointValues'));

end

