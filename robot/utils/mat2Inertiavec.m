function [inertiaVec] = mat2Inertiavec(m)
inertiaVec = [m(1,1) m(2,2) m(3,3) m(2,3) m(1,2) m(1,3)]
end

