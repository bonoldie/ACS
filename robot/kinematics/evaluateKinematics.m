function [out] = evaluateKinematics(kinematics, jointSymbols, jointValues)
%EVALUATEKINEMATICS evaluate the kinematics for ea
%   Detailed explanation goes here

out = kinematics;

for i=1:size(out.T,2)
    out.T{i} = double(subs(out.T{i}, jointSymbols', jointValues'));
    out.T_cumulative{i} = double(subs(out.T_cumulative{i}, jointSymbols', jointValues'));
end

out.Jg = double(subs(out.Jg, jointSymbols', jointValues'));
out.Ja = double(subs(out.Ja, jointSymbols', jointValues'));

end

