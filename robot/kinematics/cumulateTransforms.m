function [out] = cumulateTransforms(transformsCellArray)
%CUMULATETRANSFORMS Summary of this function goes here
out = transformsCellArray;

if size(transformsCellArray, 2) < 2
    return;
end

for i = 2:size(transformsCellArray, 2)
    out{i} = out{i-1} * out{i};
end

end

