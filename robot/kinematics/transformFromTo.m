function [out] = transformsFromTo(kinematics,from, to)
%TRANSFORMSFROMTO Summary of this function goes here

fromIndex = from;

if ischar(from)
    fromIndex = find(strcmp(kinematics.frames, from));
end

toIndex = to;

if ischar(to)
    toIndex = find(strcmp(kinematics.frames, to));
end

if fromIndex == toIndex 
    out = eye(4);
else
    out = cumulateTransforms({kinematics.T{fromIndex:toIndex-1}});
    out = out{end};
end
end
