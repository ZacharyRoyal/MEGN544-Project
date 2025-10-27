% multiplyQuat: multiply two quaternions
% 
% [Q] = multiplyQuat(Q_left, Q_right) 
% Takes in two quaternions and multiplies them together
% 
% Q = result of the multiplication
% 
% Q_left = left quaternion of the multiplication
% Q_right = right quaternion of the multiplication
% 
% Zachary Royal
% 10891021
% MEGN 544
% 2025-10-02
function [Q] = multiplyQuat(Q_left, Q_right) 

    p0 = Q_left(1);
    q0 = Q_right(1);

    p = Q_left(2:4);
    q = Q_right(2:4);

    real_comp = p0*q0-q'*p;
    vec_comp = p0*q + q0*p + cpMap(p)*q;

    Q = [real_comp; vec_comp(1); vec_comp(2); vec_comp(3)];

end