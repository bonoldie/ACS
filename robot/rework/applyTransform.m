function [out] = applyTransform(T,v)
%APPLYTRANSFORM Summary of this function goes here
%   Detailed explanation goes here
out = T*[v;1];
out = out(1:3);
end