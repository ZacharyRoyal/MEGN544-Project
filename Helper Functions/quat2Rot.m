% quat2Rot: converts a quaternion to the corresponding rotation matrix
% 
% [R] = quat2Rot(Q) 
% Takes in a unit quaternion and converts it into the corresponding
% rotation matrix which encodes the same rotation
% 
% R = rotation matrix which encodes the same rotation as the given
% quaternion
% 
% Q = given quaternion
% 
% Zachary Royal
% 10891021
% MEGN 544
% 2025-09-26
function [R] = quat2Rot(Q) 
    q_0 = Q(1);
    q = Q(2:4);

    term1 = (q_0^2-dot(q,q))*eye(3);
    term2 = 2*q_0*cpMap(q);
    term3 = 2*(q*q');

    R = term1 + term2 + term3;
end