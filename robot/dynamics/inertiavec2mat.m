function [m] = mat2Inertiavec(inertiaVec)
m = [
    inertiaVec(1) inertiaVec(5) inertiaVec(6);
    inertiaVec(5) inertiaVec(2) inertiaVec(4);
    inertiaVec(6) inertiaVec(4) inertiaVec(3);
];
end

